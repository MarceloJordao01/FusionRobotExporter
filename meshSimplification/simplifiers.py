# -*- coding: utf-8 -*-
"""
Pluggable mesh-simplification backends.

Each backend is a small function (mesh, target, params) -> trimesh.Trimesh.
`BACKENDS` is the registry the API/UI introspect via `list_backends()`:
every entry advertises a param schema (so the UI can render controls) and an
optional `requires` module (so the UI can grey it out and show an install hint
when the dependency is missing).

Always available (no extra deps beyond trimesh + numpy + fast-simplification):
  - quadric        Quadric Error Metrics edge-collapse (Garland-Heckbert).
  - quadric_agg    Same, exposing fast-simplification's `aggressiveness`.
  - cluster_grid   Vertex clustering on a uniform grid (a different family).

Optional (install to unlock — see requirements-extra.txt):
  - open3d_quadric        Open3D's QEM implementation.
  - open3d_cluster        Open3D vertex clustering (voxel grid).
"""

import os
import sys
import json
import importlib
import subprocess


def _have(module):
    try:
        importlib.import_module(module)
        return True
    except Exception:
        return False


def _target_count(before, ratio, target_faces):
    if target_faces is not None:
        t = int(target_faces)
    elif ratio is not None:
        t = int(round(before * float(ratio)))
    else:
        raise ValueError('provide ratio or target_faces')
    return max(4, min(t, before))


# --------------------------------------------------------------------------- #
# Backend implementations
# --------------------------------------------------------------------------- #
def _quadric(mesh, target, params):
    # trimesh>=4.5 routes this to the fast-simplification backend.
    try:
        return mesh.simplify_quadric_decimation(face_count=target)
    except TypeError:
        return mesh.simplify_quadric_decimation(target)


def _quadric_agg(mesh, target, params):
    import trimesh
    import fast_simplification
    agg = float(params.get('aggressiveness', 7))
    pts, faces = fast_simplification.simplify(
        mesh.vertices, mesh.faces, target_count=int(target), agg=agg)
    return trimesh.Trimesh(vertices=pts, faces=faces, process=False)


def _fit_box(mesh, oriented):
    box = mesh.bounding_box_oriented if oriented else mesh.bounding_box
    return box.vertices.copy(), box.faces.copy()


def _fit_sphere(mesh):
    import trimesh
    import numpy as np
    bs = mesh.bounding_sphere
    center = np.asarray(bs.primitive.center, dtype=float)
    radius = float(bs.primitive.radius)
    sph = trimesh.creation.icosphere(subdivisions=2, radius=radius)  # ~320 tris
    sph.apply_translation(center)
    return sph.vertices, sph.faces


def _fit_cylinder(mesh):
    import trimesh
    import numpy as np
    obb = mesh.bounding_box_oriented
    T = np.asarray(obb.primitive.transform, dtype=float)
    ext = np.asarray(obb.primitive.extents, dtype=float)
    axis = int(np.argmax(ext))                       # cylinder along the longest OBB edge
    height = float(ext[axis])
    radius = max(float(ext[i]) for i in range(3) if i != axis) / 2.0
    cyl = trimesh.creation.cylinder(radius=radius, height=height, sections=24)
    local = np.zeros(3); local[axis] = 1.0
    cyl.apply_transform(T @ trimesh.geometry.align_vectors([0, 0, 1], local))
    return cyl.vertices, cyl.faces


def _fit_geometry(mesh, target, params):
    """Replace the whole shape with one fitted primitive — a very coarse mesh,
    ideal for collision. `shape` picks box / cylinder / sphere / convex hull;
    `oriented` only affects the box (fit to the shape vs axis-aligned)."""
    import trimesh
    shape = params.get('shape', 'box')
    if shape == 'sphere':
        v, f = _fit_sphere(mesh)
    elif shape == 'cylinder':
        v, f = _fit_cylinder(mesh)
    elif shape == 'convex':
        hull = mesh.convex_hull
        v, f = hull.vertices.copy(), hull.faces.copy()
    else:  # 'box'
        v, f = _fit_box(mesh, bool(params.get('oriented', True)))
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def _cluster_grid(mesh, target, params):
    """Snap vertices onto a uniform grid (N cells along the longest bbox edge),
    weld coincident ones, drop the collapsed/degenerate faces. Pure numpy."""
    import numpy as np
    import trimesh
    n = max(float(params.get('grid', 64)), 1.0)
    v = np.asarray(mesh.vertices, dtype=np.float64)
    mn = v.min(axis=0)
    ext = float((v.max(axis=0) - mn).max()) or 1.0
    cell = ext / n
    keys = np.floor((v - mn) / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    counts = np.bincount(inv, minlength=len(uniq)).astype(np.float64)
    newv = np.empty((len(uniq), 3))
    for i in range(3):
        newv[:, i] = np.bincount(inv, weights=v[:, i], minlength=len(uniq)) / counts
    faces = inv[np.asarray(mesh.faces)]
    good = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    faces = np.unique(np.sort(faces[good], axis=1), axis=0)
    return trimesh.Trimesh(vertices=newv, faces=faces, process=False)


def _to_open3d(mesh):
    import open3d as o3d
    import numpy as np
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices, dtype=np.float64))
    m.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces, dtype=np.int32))
    return m


def _from_open3d(m):
    import numpy as np
    import trimesh
    return trimesh.Trimesh(vertices=np.asarray(m.vertices),
                           faces=np.asarray(m.triangles), process=False)


def _open3d_quadric(mesh, target, params):
    return _from_open3d(_to_open3d(mesh).simplify_quadric_decimation(int(target)))


def _open3d_cluster(mesh, target, params):
    import open3d as o3d
    import numpy as np
    n = max(float(params.get('voxel', 64)), 1.0)
    v = np.asarray(mesh.vertices)
    ext = float((v.max(axis=0) - v.min(axis=0)).max()) or 1.0
    s = _to_open3d(mesh).simplify_vertex_clustering(
        voxel_size=ext / n,
        contraction=o3d.geometry.SimplificationContraction.Average)
    return _from_open3d(s)


def _to_pymeshlab(mesh):
    import pymeshlab
    import numpy as np
    # STL meshes are vertex-split (every face has its own vertices), so without
    # welding every edge is a boundary and preserveboundary=True blocks all
    # collapses. Merge coincident vertices first to restore real connectivity.
    m = mesh
    try:
        m = mesh.copy()
        m.merge_vertices()
    except Exception:
        m = mesh
    # Contiguous, correctly-typed arrays — non-contiguous buffers handed to
    # pymeshlab/eigen are a classic cause of heap corruption (munmap_chunk).
    v = np.ascontiguousarray(m.vertices, dtype=np.float64)
    f = np.ascontiguousarray(m.faces, dtype=np.int32)
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(vertex_matrix=v, face_matrix=f))
    # Best-effort cleaning: degenerate / duplicate / unreferenced geometry makes
    # the quadric collapse crash on some real meshes.
    for nm in ('meshing_remove_duplicate_vertices',
               'meshing_remove_unreferenced_vertices',
               'meshing_remove_null_faces',
               'meshing_remove_duplicate_faces'):
        try:
            getattr(ms, nm)()
        except Exception:
            pass
    return ms


def _from_pymeshlab(ms):
    import trimesh
    import numpy as np
    m = ms.current_mesh()
    # Copy out so trimesh owns the buffers (avoid a double-free when the MeshSet
    # is collected).
    v = np.array(m.vertex_matrix(), dtype=np.float64)
    f = np.array(m.face_matrix(), dtype=np.int64)
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def _pml_apply(ms, names, **kw):
    """Run the first filter (called as a MeshSet method) that exists in this
    pymeshlab version. Filter names were renamed across releases, and recent
    pymeshlab exposes filters as methods (apply_filter() is a silent no-op for
    some of them), so we dispatch via getattr."""
    last = None
    for name in names:
        fn = getattr(ms, name, None)
        if fn is None:
            continue
        try:
            fn(**kw)
            return
        except Exception as e:
            last = e
    if last is not None:
        raise last
    raise RuntimeError('no matching pymeshlab filter: ' + ', '.join(names))


def _meshlab_quadric(mesh, target, params):
    ms = _to_pymeshlab(mesh)
    _pml_apply(
        ms,
        ['meshing_decimation_quadric_edge_collapse',
         'simplification_quadric_edge_collapse_decimation'],
        targetfacenum=int(target),
        qualitythr=float(params.get('quality_threshold', 0.3)),
        preserveboundary=bool(params.get('preserve_boundary', True)),
        preservenormal=True, optimalplacement=True, planarquadric=True)
    return _from_pymeshlab(ms)


def _meshlab_cluster(mesh, target, params):
    import pymeshlab
    ms = _to_pymeshlab(mesh)
    pct = float(params.get('cell_percent', 1.0))
    Percent = getattr(pymeshlab, 'PercentageValue', None) or getattr(pymeshlab, 'Percentage')
    _pml_apply(
        ms,
        ['meshing_decimation_clustering',
         'simplification_clustering_decimation'],
        threshold=Percent(pct))
    return _from_pymeshlab(ms)


# --------------------------------------------------------------------------- #
# Registry
# --------------------------------------------------------------------------- #
BACKENDS = {
    'quadric': {
        'label': 'Quadric (QEM, fast)',
        'desc': 'Garland-Heckbert quadric edge-collapse. Best general-purpose; '
                'preserves silhouette. Honors the exact face target.',
        'requires': None,
        'honors_target': True,
        'params': [],
        'fn': _quadric,
    },
    'quadric_agg': {
        'label': 'Quadric + aggressiveness',
        'desc': 'Same QEM, but tune how hard it pushes. Higher = fewer faces '
                'faster, lower = more careful / higher quality.',
        'requires': None,
        'honors_target': True,
        'params': [{'name': 'aggressiveness', 'label': 'Aggressiveness',
                    'min': 1, 'max': 10, 'step': 0.5, 'default': 7}],
        'fn': _quadric_agg,
    },
    'cluster_grid': {
        'label': 'Vertex clustering (grid)',
        'desc': 'Welds vertices onto a uniform grid. Different family — robust '
                'and very fast, coarser look. Face count follows the grid, not '
                'the target.',
        'requires': None,
        'honors_target': False,
        'params': [{'name': 'grid', 'label': 'Grid resolution',
                    'min': 8, 'max': 256, 'step': 1, 'default': 64}],
        'fn': _cluster_grid,
    },
    'fit_geometry': {
        'label': 'Fit geometry',
        'desc': 'Replace the whole shape with one fitted primitive (box, cylinder, '
                'sphere or convex hull). Very coarse — ideal for collision. '
                '“Oriented” only affects the box.',
        'requires': None,
        'honors_target': False,
        'params': [
            {'name': 'shape', 'label': 'Shape', 'type': 'select', 'default': 'box',
             'options': [
                 {'value': 'box', 'label': 'Box'},
                 {'value': 'cylinder', 'label': 'Cylinder'},
                 {'value': 'sphere', 'label': 'Sphere'},
                 {'value': 'convex', 'label': 'Convex hull'},
             ]},
            {'name': 'oriented', 'label': 'Oriented box (fit to shape)',
             'type': 'bool', 'default': True},
        ],
        'fn': _fit_geometry,
    },
    'open3d_quadric': {
        'label': 'Open3D quadric',
        'desc': "Open3D's own QEM implementation, for comparison.",
        'requires': 'open3d',
        'honors_target': True,
        'params': [],
        'fn': _open3d_quadric,
    },
    'open3d_cluster': {
        'label': 'Open3D vertex clustering',
        'desc': 'Open3D voxel-grid clustering. Face count follows the voxel '
                'resolution, not the target.',
        'requires': 'open3d',
        'honors_target': False,
        'params': [{'name': 'voxel', 'label': 'Voxel resolution',
                    'min': 8, 'max': 256, 'step': 1, 'default': 64}],
        'fn': _open3d_cluster,
    },
    'meshlab_quadric': {
        'label': 'MeshLab quadric edge-collapse',
        'desc': "MeshLab's Quadric Edge Collapse Decimation (QEM) with quality "
                'threshold and boundary preservation. Honors the face target.',
        'requires': 'pymeshlab',
        'isolate': True,
        'honors_target': True,
        'params': [
            {'name': 'quality_threshold', 'label': 'Quality threshold',
             'min': 0, 'max': 1, 'step': 0.05, 'default': 0.3},
            {'name': 'preserve_boundary', 'label': 'Preserve boundary',
             'type': 'bool', 'default': True},
        ],
        'fn': _meshlab_quadric,
    },
    'meshlab_cluster': {
        'label': 'MeshLab clustering decimation',
        'desc': "MeshLab's Clustering Decimation. Cell size is a percentage of "
                'the bounding-box diagonal — face count follows the cell size.',
        'requires': 'pymeshlab',
        'isolate': True,
        'honors_target': False,
        'params': [{'name': 'cell_percent', 'label': 'Cell size (% bbox)',
                    'min': 0.1, 'max': 10, 'step': 0.1, 'default': 1}],
        'fn': _meshlab_cluster,
    },
}

DEFAULT = 'quadric'


def list_backends():
    """Serializable view for /api/algorithms (no `fn`, plus an availability flag)."""
    out = []
    for key, spec in BACKENDS.items():
        req = spec['requires']
        out.append({
            'id': key,
            'label': spec['label'],
            'desc': spec['desc'],
            'requires': req,
            'available': req is None or _have(req),
            'honors_target': spec['honors_target'],
            'params': spec['params'],
        })
    return out


def _run_isolated(algorithm, in_abs, out_abs, target, params):
    """Run a backend in a child process so a native crash (pymeshlab can abort
    the whole interpreter with munmap_chunk/segfault on some meshes) becomes a
    catchable error instead of killing the server."""
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    cmd = [sys.executable, os.path.abspath(__file__), '--worker',
           in_abs, out_abs, algorithm, str(int(target)), json.dumps(params or {})]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        lines = (proc.stderr or proc.stdout or '').strip().splitlines()
        tail = lines[-1] if lines else f'exit code {proc.returncode}'
        raise RuntimeError(f"'{algorithm}' crashed: {tail}")


def simplify_mesh(in_abs, out_abs, ratio=None, target_faces=None,
                  algorithm=DEFAULT, params=None):
    """Decimate `in_abs` with the chosen backend, write `out_abs`, return
    (before, after) face counts. Unknown algorithm falls back to the default."""
    import trimesh
    mesh = trimesh.load(in_abs, force='mesh', process=False)
    before = len(mesh.faces)

    spec = BACKENDS.get(algorithm) or BACKENDS[DEFAULT]
    if spec['requires'] and not _have(spec['requires']):
        raise RuntimeError(f"algorithm '{algorithm}' needs `pip install "
                           f"{spec['requires']}`")

    target = _target_count(before, ratio, target_faces)

    if spec.get('isolate'):
        _run_isolated(algorithm, in_abs, out_abs, target, params)
        out = trimesh.load(out_abs, force='mesh', process=False)
        return before, len(out.faces)

    if spec['honors_target'] and target >= before:
        out = mesh  # nothing to do
    else:
        out = spec['fn'](mesh, target, params or {})

    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    out.export(out_abs)
    return before, len(out.faces)


# --------------------------------------------------------------------------- #
# Worker entry point: `python simplifiers.py --worker IN OUT ALGO TARGET PARAMS`
# Used by _run_isolated for crash-prone (isolated) backends.
# --------------------------------------------------------------------------- #
if __name__ == '__main__' and len(sys.argv) >= 2 and sys.argv[1] == '--worker':
    import trimesh
    _in, _out, _algo, _target, _params = sys.argv[2:7]
    _mesh = trimesh.load(_in, force='mesh', process=False)
    _result = BACKENDS[_algo]['fn'](_mesh, int(_target), json.loads(_params))
    os.makedirs(os.path.dirname(_out), exist_ok=True)
    _result.export(_out)
