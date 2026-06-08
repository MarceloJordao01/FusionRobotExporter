# -*- coding: utf-8 -*-
"""
Mesh editing for the viewer: quadric-decimation simplification plus writing the
chosen visual/collision mesh back into the loaded .xacro / .sdf.

Design decisions (see project notes):
  - Visual and collision are edited independently per link.
  - New (simplified) meshes are written into the package's own meshes/ folder so
    the package stays self-contained and ROS-ready.
  - Before the description file is first edited, a pristine .bak copy is placed
    INSIDE meshes/ (one per file; never overwritten).
  - Simplification always decimates from the link's *original* mesh (the un-
    suffixed basename), so re-simplifying with a new ratio overwrites the
    __vis/__col file instead of compounding the loss.

Decimation backend: trimesh.simplify_quadric_decimation, which on trimesh>=4.5
delegates to the pure-wheel `fast-simplification` package (validated in Docker).
"""

import os
import re
import shutil
import xml.etree.ElementTree as ET

# Decimation backends live in their own registry; re-exported so existing callers
# keep using meshedit.simplify_mesh(...).
from simplifiers import simplify_mesh, list_backends, BACKENDS  # noqa: F401

XACRO_NS = 'http://www.ros.org/wiki/xacro'
MESH_EXTS = ('.stl', '.obj', '.dae', '.ply')


# --------------------------------------------------------------------------- #
# Small filesystem / path helpers
# --------------------------------------------------------------------------- #
def _read_text(path):
    """Read XML tolerant of UTF-8 / Windows-1252; latin-1 never raises."""
    with open(path, 'rb') as f:
        raw = f.read()
    try:
        return raw.decode('utf-8')
    except UnicodeDecodeError:
        return raw.decode('latin-1')


def pkg_root(file_path):
    """Nearest ancestor that holds a meshes/ folder, else the file's dir."""
    d = os.path.dirname(os.path.abspath(file_path))
    probe = d
    for _ in range(4):
        if os.path.isdir(os.path.join(probe, 'meshes')):
            return probe
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    return d


def meshes_dir(file_path):
    """The package's meshes/ folder (created if missing)."""
    d = os.path.join(pkg_root(file_path), 'meshes')
    os.makedirs(d, exist_ok=True)
    return d


def _resolve_abs(ref, pkg_root_dir, file_dir):
    """Resolve a mesh reference (package://, model://, abs, relative) to an
    absolute filesystem path, or None if empty."""
    raw = (ref or '').strip()
    if not raw:
        return None
    if raw.startswith('package://') or raw.startswith('model://'):
        rest = raw.split('://', 1)[1].split('/', 1)
        rel = rest[1] if len(rest) > 1 else rest[0]
        return os.path.join(pkg_root_dir, rel)
    if os.path.isabs(raw):
        return raw
    cand = os.path.join(file_dir, raw)
    if os.path.exists(cand):
        return cand
    return os.path.join(pkg_root_dir, raw)


# --------------------------------------------------------------------------- #
# Simplification
# --------------------------------------------------------------------------- #
def original_basename(basename):
    """Strip a trailing __vis / __col so re-simplifying always starts from the
    pristine mesh rather than an already-decimated one."""
    stem, ext = os.path.splitext(basename)
    for suf in ('__vis', '__col'):
        if stem.endswith(suf):
            stem = stem[:-len(suf)]
            break
    return stem + ext


def simplified_name(src_basename, target):
    """Deterministic output name so re-simplifying overwrites rather than piles
    up: <stem>__<target>.<ext>, e.g. base_link__col.stl / base_link__vis.stl."""
    stem, ext = os.path.splitext(original_basename(src_basename))
    if ext.lower() not in MESH_EXTS:
        ext = '.stl'
    suffix = {'collision': 'col', 'visual': 'vis'}.get(target, target)
    return stem + '__' + suffix + ext


def mesh_info(abs_path):
    """Triangle/vertex counts for a mesh file on disk."""
    import trimesh
    mesh = trimesh.load(abs_path, force='mesh', process=False)
    return {'faces': int(len(mesh.faces)), 'vertices': int(len(mesh.vertices))}


# --------------------------------------------------------------------------- #
# Backup / restore of the description file
# --------------------------------------------------------------------------- #
def backup_once(file_path):
    """Copy the description file into meshes/<name>.bak once (kept pristine)."""
    bak = os.path.join(meshes_dir(file_path), os.path.basename(file_path) + '.bak')
    if not os.path.exists(bak):
        shutil.copy2(file_path, bak)
    return bak


def restore_backup(file_path):
    """Revert the description file to its pristine meshes/<name>.bak."""
    bak = os.path.join(meshes_dir(file_path), os.path.basename(file_path) + '.bak')
    if not os.path.exists(bak):
        raise FileNotFoundError('no backup exists for this file yet')
    shutil.copy2(bak, file_path)
    return bak


# --------------------------------------------------------------------------- #
# Reading the link/visual/collision structure
# --------------------------------------------------------------------------- #
def _find_named(parent, tag, name):
    for el in parent.iter(tag):
        if el.get('name') == name:
            return el
    return None


def _geom_mesh(node):
    """Return the <mesh> under node's <geometry>, raising a clear error if the
    link/visual/collision/geometry/mesh chain is incomplete."""
    if node is None:
        raise KeyError('link has no such visual/collision block')
    geom = node.find('geometry')
    if geom is None:
        raise KeyError('geometry not found')
    mesh = geom.find('mesh')
    if mesh is None:
        raise KeyError('this geometry is not a <mesh> (cannot repoint)')
    return mesh


def _ref_urdf(node):
    try:
        return _geom_mesh(node).get('filename')
    except KeyError:
        return None


def _ref_sdf(node):
    try:
        uri = _geom_mesh(node).find('uri')
    except KeyError:
        return None
    return uri.text if uri is not None else None


def _all_refs_urdf(root):
    return [m.get('filename') for m in root.iter('mesh')]


def _all_refs_sdf(root):
    refs = []
    for m in root.iter('mesh'):
        uri = m.find('uri')
        if uri is not None and uri.text:
            refs.append(uri.text)
    return refs


def _sdf_model(root):
    model = root.find('model')
    return model if model is not None else root


def link_meshes(file_path):
    """List links with their visual/collision mesh reference + resolved absolute
    path. Shape: [{'link': name,
                   'visual': {'ref': str, 'abs': path} | None,
                   'collision': {'ref': str, 'abs': path} | None}, ...]"""
    ext = os.path.splitext(file_path)[1].lower()
    pr = pkg_root(file_path)
    file_dir = os.path.dirname(os.path.abspath(file_path))
    body = re.sub(r'^\s*<\?xml[^>]*\?>', '', _read_text(file_path), count=1)
    root = ET.fromstring(body)

    if ext == '.sdf':
        container, get_ref = _sdf_model(root), _ref_sdf
    else:
        container, get_ref = root, _ref_urdf

    out = []
    for link in container.findall('link'):
        entry = {'link': link.get('name'), 'visual': None, 'collision': None}
        for target in ('visual', 'collision'):
            ref = get_ref(link.find(target))
            if not ref:
                continue
            entry[target] = {'ref': ref, 'abs': _resolve_abs(ref, pr, file_dir)}
        out.append(entry)
    return out


# --------------------------------------------------------------------------- #
# Writing a new mesh reference back into the description
# --------------------------------------------------------------------------- #
def _mesh_reference(existing_refs, pkg_name, new_basename):
    """Build the new mesh reference string. Reuse the prefix style already in the
    document (package:// / model:// / relative) and just swap the basename, so we
    match whatever convention the exporter wrote."""
    for ref in existing_refs:
        ref = (ref or '').strip()
        if not ref:
            continue
        prefix = ref.rsplit('/', 1)[0]
        return prefix + '/' + new_basename
    if pkg_name:
        return 'package://' + pkg_name + '/meshes/' + new_basename
    return 'meshes/' + new_basename


def assign_mesh(file_path, link_name, target, new_basename, scale=None):
    """Point link_name's <visual> or <collision> geometry at meshes/<new_basename>
    and write the file back (backing it up first). `target` is 'visual' or
    'collision'. `scale` (3-list) overrides the scale attribute when given.
    Returns the new reference string written."""
    if target not in ('visual', 'collision'):
        raise ValueError("target must be 'visual' or 'collision'")

    backup_once(file_path)

    ext = os.path.splitext(file_path)[1].lower()
    pkg = os.path.basename(pkg_root(file_path))
    text = _read_text(file_path)

    m = re.match(r'\s*<\?xml[^>]*\?>', text)
    decl = m.group(0).strip() if m else '<?xml version="1.0" ?>'
    body = text[m.end():] if m else text

    ET.register_namespace('xacro', XACRO_NS)
    root = ET.fromstring(body)

    if ext == '.sdf':
        link = _find_named(_sdf_model(root), 'link', link_name)
        mesh = _geom_mesh(link.find(target) if link is not None else None)
        ref = _mesh_reference(_all_refs_sdf(root), pkg, new_basename)
        uri = mesh.find('uri')
        if uri is None:
            uri = ET.SubElement(mesh, 'uri')
        uri.text = ref
        if scale is not None:
            sc = mesh.find('scale')
            if sc is None:
                sc = ET.SubElement(mesh, 'scale')
            sc.text = ' '.join(str(s) for s in scale)
    else:
        link = _find_named(root, 'link', link_name)
        mesh = _geom_mesh(link.find(target) if link is not None else None)
        ref = _mesh_reference(_all_refs_urdf(root), pkg, new_basename)
        mesh.set('filename', ref)
        if scale is not None:
            mesh.set('scale', ' '.join(str(s) for s in scale))

    ET.indent(root, space='  ')
    out = decl + '\n' + ET.tostring(root, encoding='unicode')
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(out)
    return ref
