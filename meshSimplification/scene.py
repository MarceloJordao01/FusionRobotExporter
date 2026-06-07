# -*- coding: utf-8 -*-
"""
Parse a URDF (.urdf / .xacro) or SDF (.sdf) file into a flat "scene":
a list of visual entries, each already placed in world coordinates so the
browser viewer only has to load the mesh and apply a position/quaternion/scale.

Why server-side instead of a JS URDF loader:
  - the exporter writes .xacro (xacro:include of materials/trans/gazebo); the
    body is plain URDF, so we just inline the includes and read the links/joints.
  - SDF has no common browser loader; here both formats produce the same
    scene shape.

Conventions (match the exporter):
  - lengths are meters; rpy is fixed-axis XYZ (R = Rz @ Ry @ Rx).
  - URDF: link world frames come from forward kinematics at the zero joint
    configuration (joint <origin> chained from the root link).
  - SDF: FusionSDF writes each <link>/<visual> <pose> relative to the model
    (world), so poses are used directly.
"""

import os
import re
import math
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_COLOR = [0.7, 0.7, 0.7, 1.0]
MESH_EXTS = ('.stl', '.obj', '.dae')


# --------------------------------------------------------------------------- #
# Small linear-algebra helpers
# --------------------------------------------------------------------------- #
def _rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _xform(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = _rpy_to_matrix(rpy)
    m[:3, 3] = xyz
    return m


def _matrix_to_pos_quat(m):
    pos = m[:3, 3].tolist()
    rot = m[:3, :3]
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return pos, [float(x), float(y), float(z), float(w)]


def _floats(text, n, default=0.0):
    if not text:
        return [default] * n
    vals = [float(v) for v in text.replace(',', ' ').split()]
    while len(vals) < n:
        vals.append(default)
    return vals[:n]


def _local(tag):
    return tag.split('}')[-1]


def _read_text(path):
    """Fusion writes XML as UTF-8 or Windows-1252 depending on content; try
    UTF-8 first and fall back to latin-1 (never raises on byte values)."""
    with open(path, 'rb') as f:
        raw = f.read()
    try:
        return raw.decode('utf-8')
    except UnicodeDecodeError:
        return raw.decode('latin-1')


def _strip_decl(text):
    """Drop the <?xml ...?> declaration so a fixed-up string parses regardless
    of any encoding attribute it may carry."""
    return re.sub(r'^\s*<\?xml[^>]*\?>', '', text, count=1)


# --------------------------------------------------------------------------- #
# Mesh path resolution -> URL served by /mesh
# --------------------------------------------------------------------------- #
def _pkg_root(file_path):
    """
    Package root for a description file: the nearest ancestor that contains a
    'meshes' folder, else the file's own directory. `package://pkg/...` and
    `$(find pkg)` resolve against this.
    """
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


def _resolve_mesh(filename, pkg_root, file_dir, data_dir):
    """Turn a mesh reference into a path relative to data_dir (for /mesh?path=)."""
    raw = filename.strip()
    if raw.startswith('package://') or raw.startswith('model://'):
        rest = raw.split('://', 1)[1].split('/', 1)
        rel = rest[1] if len(rest) > 1 else rest[0]
        abs_path = os.path.join(pkg_root, rel)
    elif os.path.isabs(raw):
        abs_path = raw
    else:
        abs_path = os.path.join(file_dir, raw)
        if not os.path.exists(abs_path):
            abs_path = os.path.join(pkg_root, raw)
    rel_to_data = os.path.relpath(os.path.abspath(abs_path), data_dir)
    return rel_to_data.replace('\\', '/')


# --------------------------------------------------------------------------- #
# URDF / xacro
# --------------------------------------------------------------------------- #
def _load_urdf_elements(path, pkg_root, visited):
    """Read an .xacro/.urdf and return its top-level children, inlining
    xacro:include (the only xacro feature the exporter uses in the body)."""
    text = _read_text(path)
    # Resolve $(find pkg) -> package root so includes can be read from disk.
    text = re.sub(r'\$\(find [^)]+\)', pkg_root.replace('\\', '/'), text)
    root = ET.fromstring(_strip_decl(text))

    elements = []
    for child in root:
        if _local(child.tag) == 'include':
            inc = child.get('filename', '')
            inc_path = inc if os.path.isabs(inc) else os.path.join(os.path.dirname(path), inc)
            inc_path = os.path.normpath(inc_path)
            if inc_path not in visited and os.path.exists(inc_path):
                visited.add(inc_path)
                elements.extend(_load_urdf_elements(inc_path, pkg_root, visited))
        else:
            elements.append(child)
    return elements


def parse_urdf(file_path, data_dir):
    pkg_root = _pkg_root(file_path)
    file_dir = os.path.dirname(os.path.abspath(file_path))
    elements = _load_urdf_elements(file_path, pkg_root, {os.path.normpath(file_path)})

    materials = {}
    links = {}
    joints = []
    robot_name = 'robot'

    for el in elements:
        tag = _local(el.tag)
        if tag == 'material':
            color = el.find('color')
            if color is not None:
                materials[el.get('name')] = _floats(color.get('rgba'), 4, 1.0)
        elif tag == 'link':
            links[el.get('name')] = el
        elif tag == 'joint':
            jtype = el.get('type', 'fixed')
            origin = el.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, 3)
            rpy = _floats(origin.get('rpy') if origin is not None else None, 3)
            parent = el.find('parent')
            child = el.find('child')
            axis_el = el.find('axis')
            # URDF <axis> is expressed in the joint (child) frame; default +X.
            axis = _floats(axis_el.get('xyz'), 3) if axis_el is not None else [1.0, 0.0, 0.0]
            if parent is not None and child is not None:
                joints.append({
                    'name': el.get('name', 'joint'),
                    'type': jtype,
                    'parent': parent.get('link'),
                    'child': child.get('link'),
                    'xform': _xform(xyz, rpy),
                    'axis': axis,
                })

    # Forward kinematics at zero config -> world transform per link.
    child_links = {j['child'] for j in joints}
    world = {name: np.eye(4) for name in links if name not in child_links}
    remaining = list(joints)
    progress = True
    while remaining and progress:
        progress = False
        still = []
        for j in remaining:
            if j['parent'] in world:
                world[j['child']] = world[j['parent']] @ j['xform']
                progress = True
            else:
                still.append(j)
        remaining = still
    for j in remaining:  # disconnected: place at origin
        world.setdefault(j['child'], np.eye(4))

    entries = []
    for name, link_el in links.items():
        link_world = world.get(name, np.eye(4))
        for visual in link_el.findall('visual'):
            geom = visual.find('geometry')
            mesh = geom.find('mesh') if geom is not None else None
            if mesh is None:
                continue
            origin = visual.find('origin')
            v_xyz = _floats(origin.get('xyz') if origin is not None else None, 3)
            v_rpy = _floats(origin.get('rpy') if origin is not None else None, 3)
            m_world = link_world @ _xform(v_xyz, v_rpy)
            pos, quat = _matrix_to_pos_quat(m_world)

            scale = _floats(mesh.get('scale'), 3, 1.0)
            mat = visual.find('material')
            color = DEFAULT_COLOR
            if mat is not None:
                c = mat.find('color')
                if c is not None:
                    color = _floats(c.get('rgba'), 4, 1.0)
                elif mat.get('name') in materials:
                    color = materials[mat.get('name')]

            entries.append({
                'link': name,
                'mesh': _resolve_mesh(mesh.get('filename', ''), pkg_root, file_dir, data_dir),
                'format': os.path.splitext(mesh.get('filename', ''))[1].lower().lstrip('.'),
                'position': pos,
                'quaternion': quat,
                'scale': scale,
                'color': color,
            })

    # Joint markers: the joint frame coincides with the child link frame.
    joints_out = []
    for j in joints:
        cw = world.get(j['child'])
        if cw is None:
            continue
        pos, quat = _matrix_to_pos_quat(cw)
        axis_world = None
        if j['type'] != 'fixed':
            a = cw[:3, :3] @ np.array(j['axis'])
            n = np.linalg.norm(a)
            if n > 1e-9:
                axis_world = (a / n).tolist()
        joints_out.append({'name': j['name'], 'type': j['type'],
                           'position': pos, 'quaternion': quat, 'axis': axis_world})

    robot_name = os.path.splitext(os.path.basename(file_path))[0]
    return {'name': robot_name, 'format': 'urdf', 'entries': entries, 'joints': joints_out}


# --------------------------------------------------------------------------- #
# SDF
# --------------------------------------------------------------------------- #
def parse_sdf(file_path, data_dir):
    pkg_root = _pkg_root(file_path)
    file_dir = os.path.dirname(os.path.abspath(file_path))
    root = ET.fromstring(_strip_decl(_read_text(file_path)))
    model = root.find('model')
    if model is None:
        model = root

    def pose_of(el):
        p = el.find('pose')
        vals = _floats(p.text if p is not None else None, 6)
        return _xform(vals[:3], vals[3:6])

    entries = []
    link_worlds = {}
    name = model.get('name', 'model')
    for link in model.findall('link'):
        link_world = pose_of(link)
        link_worlds[link.get('name', 'link')] = link_world
        for visual in link.findall('visual'):
            geom = visual.find('geometry')
            mesh = geom.find('mesh') if geom is not None else None
            if mesh is None:
                continue
            uri = mesh.find('uri')
            if uri is None or not uri.text:
                continue
            scale_el = mesh.find('scale')
            scale = _floats(scale_el.text if scale_el is not None else None, 3, 1.0)
            m_world = link_world @ pose_of(visual)
            pos, quat = _matrix_to_pos_quat(m_world)
            entries.append({
                'link': link.get('name', 'link'),
                'mesh': _resolve_mesh(uri.text, pkg_root, file_dir, data_dir),
                'format': os.path.splitext(uri.text)[1].lower().lstrip('.'),
                'position': pos,
                'quaternion': quat,
                'scale': scale,
                'color': DEFAULT_COLOR,
            })

    # Joints: SDF <pose> defaults to the child link frame; <axis><xyz> too.
    joints_out = []
    for j in model.findall('joint'):
        child_el = j.find('child')
        child = child_el.text.strip() if child_el is not None and child_el.text else None
        base = link_worlds.get(child, np.eye(4))
        jw = base @ pose_of(j)
        jtype = j.get('type', 'fixed')
        pos, quat = _matrix_to_pos_quat(jw)
        axis_world = None
        axis_el = j.find('axis')
        xyz_el = axis_el.find('xyz') if axis_el is not None else None
        if xyz_el is not None and xyz_el.text and jtype != 'fixed':
            a = jw[:3, :3] @ np.array(_floats(xyz_el.text, 3))
            n = np.linalg.norm(a)
            if n > 1e-9:
                axis_world = (a / n).tolist()
        joints_out.append({'name': j.get('name', 'joint'), 'type': jtype,
                           'position': pos, 'quaternion': quat, 'axis': axis_world})

    return {'name': name, 'format': 'sdf', 'entries': entries, 'joints': joints_out}


def parse_file(file_path, data_dir):
    ext = os.path.splitext(file_path)[1].lower()
    if ext == '.sdf':
        return parse_sdf(file_path, data_dir)
    return parse_urdf(file_path, data_dir)
