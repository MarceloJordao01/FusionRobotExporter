# -*- coding: utf-8 -*-
"""
Flask app that renders the URDF/SDF of a package folder mounted at /data.

Endpoints
  GET /                      -> viewer page
  GET /api/files            -> list of .urdf/.xacro/.sdf found under DATA_DIR
  GET /api/scene?file=REL   -> flat scene JSON for that file (see scene.py)
  GET /mesh?path=REL        -> serve a mesh file from under DATA_DIR

Mesh simplification will be added later as further endpoints; for now this is
render-only.
"""

import os
import re

from flask import Flask, jsonify, request, send_file, render_template, abort

import scene as scene_mod
import meshedit

DATA_DIR = os.path.abspath(os.environ.get('DATA_DIR', '/data'))
PORT = int(os.environ.get('PORT', '5000'))

app = Flask(__name__)


def _safe_path(rel):
    """Resolve `rel` under DATA_DIR, blocking path traversal."""
    full = os.path.abspath(os.path.join(DATA_DIR, rel))
    if full != DATA_DIR and not full.startswith(DATA_DIR + os.sep):
        abort(403)
    return full


@app.route('/')
def index():
    return render_template('index.html')


def _rel(full):
    return os.path.relpath(full, DATA_DIR).replace('\\', '/')


@app.route('/api/browse')
def api_browse():
    """List the sub-folders and description files of one directory under
    DATA_DIR (shallow, so a large mounted tree is never walked at once)."""
    rel = request.args.get('dir', '').strip().strip('/')
    full = _safe_path(rel) if rel else DATA_DIR
    if not os.path.isdir(full):
        abort(404)

    dirs, files = [], []
    try:
        for name in sorted(os.listdir(full), key=str.lower):
            p = os.path.join(full, name)
            if os.path.isdir(p):
                dirs.append({'rel': _rel(p), 'name': name})
            elif name.lower().endswith(('.urdf', '.xacro', '.sdf')):
                files.append({'rel': _rel(p), 'name': name})
    except PermissionError:
        pass

    parent = os.path.dirname(rel) if rel else None
    return jsonify({'data_dir': DATA_DIR, 'dir': rel, 'parent': parent,
                    'dirs': dirs, 'files': files})


@app.route('/api/scene')
def api_scene():
    rel = request.args.get('file', '')
    if not rel:
        abort(400)
    full = _safe_path(rel)
    if not os.path.exists(full):
        abort(404)
    try:
        data = scene_mod.parse_file(full, DATA_DIR)
    except Exception as e:  # surface parse errors to the UI instead of 500-ing
        return jsonify({'error': f'{type(e).__name__}: {e}'}), 200
    return jsonify(data)


def _data_rel(abs_path):
    """Path relative to DATA_DIR with forward slashes, for /mesh?path=."""
    if not abs_path:
        return None
    try:
        return os.path.relpath(os.path.abspath(abs_path), DATA_DIR).replace('\\', '/')
    except ValueError:  # different drive on Windows
        return None


@app.route('/api/algorithms')
def api_algorithms():
    """Available simplification backends + their param schema, for the UI."""
    return jsonify({'algorithms': meshedit.list_backends(), 'default': 'quadric'})


@app.route('/api/links')
def api_links():
    """Per-link visual/collision mesh references, with triangle counts. Powers
    the mesh-simplification panel."""
    rel = request.args.get('file', '')
    if not rel:
        abort(400)
    full = _safe_path(rel)
    if not os.path.isfile(full):
        abort(404)
    try:
        links = meshedit.link_meshes(full)
    except Exception as e:
        return jsonify({'error': f'{type(e).__name__}: {e}'}), 200

    out = []
    for L in links:
        item = {'link': L['link'], 'visual': None, 'collision': None}
        for t in ('visual', 'collision'):
            mt = L[t]
            if not mt:
                continue
            abs_path = mt['abs']
            present = bool(abs_path and os.path.isfile(abs_path))
            faces = None
            if present:
                try:
                    faces = meshedit.mesh_info(abs_path)['faces']
                except Exception:
                    faces = None
            # Original (un-suffixed) face count drives the live estimate. When the
            # current mesh IS the original, reuse its count (no extra mesh load).
            orig_faces = faces
            if present:
                orig_base = meshedit.original_basename(os.path.basename(abs_path))
                if orig_base != os.path.basename(abs_path):
                    orig_abs = os.path.join(os.path.dirname(abs_path), orig_base)
                    if os.path.isfile(orig_abs):
                        try:
                            orig_faces = meshedit.mesh_info(orig_abs)['faces']
                        except Exception:
                            orig_faces = faces
            item[t] = {'ref': mt['ref'], 'faces': faces, 'orig_faces': orig_faces,
                       'mesh': _data_rel(abs_path), 'missing': not present}
        out.append(item)
    return jsonify({'file': rel, 'links': out})


def _link_source(full, link, target):
    """Resolve the pristine (un-suffixed) mesh to decimate for a link/target.
    Returns (orig_abs, None) or (None, error_message)."""
    info = next((L for L in meshedit.link_meshes(full) if L['link'] == link), None)
    if info is None or not info.get(target):
        return None, f'link {link!r} has no {target} mesh'
    cur_abs = info[target]['abs']
    if not cur_abs or not os.path.isfile(cur_abs):
        return None, f"source mesh not found: {info[target]['ref']}"
    orig_abs = os.path.join(os.path.dirname(cur_abs),
                            meshedit.original_basename(os.path.basename(cur_abs)))
    if not os.path.isfile(orig_abs):
        orig_abs = cur_abs
    return orig_abs, None


@app.route('/api/simplify', methods=['POST'])
def api_simplify():
    """Decimate one link's visual or collision mesh and repoint the description
    at the new file. Body: {file, link, target, ratio | target_faces}."""
    data = request.get_json(force=True, silent=True) or {}
    rel = data.get('file', '')
    link = data.get('link', '')
    target = data.get('target', '')
    ratio = data.get('ratio')
    target_faces = data.get('target_faces')
    if not rel or not link or target not in ('visual', 'collision'):
        abort(400)
    if ratio is None and target_faces is None:
        return jsonify({'error': 'provide ratio or target_faces'}), 200

    full = _safe_path(rel)
    if not os.path.isfile(full):
        abort(404)

    try:
        orig_abs, err = _link_source(full, link, target)
        if err:
            return jsonify({'error': err}), 200
        new_base = meshedit.simplified_name(os.path.basename(orig_abs), target)
        out_abs = os.path.join(meshedit.meshes_dir(full), new_base)
        before, after = meshedit.simplify_mesh(
            orig_abs, out_abs, ratio=ratio, target_faces=target_faces,
            algorithm=data.get('algorithm', 'quadric'), params=data.get('params'))
        ref = meshedit.assign_mesh(full, link, target, new_base, scale=data.get('scale'))
    except Exception as e:
        return jsonify({'error': f'{type(e).__name__}: {e}'}), 200

    return jsonify({'ref': ref, 'before': before, 'after': after,
                    'mesh': _data_rel(out_abs)})


@app.route('/api/preview', methods=['POST'])
def api_preview():
    """Decimate to a throwaway file under meshes/.preview/ WITHOUT touching the
    description, so the viewer can show the result before committing. Same body
    as /api/simplify."""
    data = request.get_json(force=True, silent=True) or {}
    rel = data.get('file', '')
    link = data.get('link', '')
    target = data.get('target', '')
    ratio = data.get('ratio')
    target_faces = data.get('target_faces')
    if not rel or not link or target not in ('visual', 'collision'):
        abort(400)
    if ratio is None and target_faces is None:
        return jsonify({'error': 'provide ratio or target_faces'}), 200

    full = _safe_path(rel)
    if not os.path.isfile(full):
        abort(404)

    try:
        orig_abs, err = _link_source(full, link, target)
        if err:
            return jsonify({'error': err}), 200
        ext = os.path.splitext(orig_abs)[1].lower() or '.stl'
        pdir = os.path.join(meshedit.meshes_dir(full), '.preview')
        os.makedirs(pdir, exist_ok=True)
        safe_link = re.sub(r'[^A-Za-z0-9_.-]', '_', link)
        out_abs = os.path.join(pdir, f'{safe_link}__{target}{ext}')
        before, after = meshedit.simplify_mesh(
            orig_abs, out_abs, ratio=ratio, target_faces=target_faces,
            algorithm=data.get('algorithm', 'quadric'), params=data.get('params'))
    except Exception as e:
        return jsonify({'error': f'{type(e).__name__}: {e}'}), 200

    return jsonify({'before': before, 'after': after,
                    'mesh': _data_rel(out_abs), 'format': ext.lstrip('.')})


@app.route('/api/revert', methods=['POST'])
def api_revert():
    """Restore the description file from its pristine meshes/<name>.bak."""
    data = request.get_json(force=True, silent=True) or {}
    rel = data.get('file', '')
    if not rel:
        abort(400)
    full = _safe_path(rel)
    try:
        meshedit.restore_backup(full)
    except Exception as e:
        return jsonify({'error': f'{type(e).__name__}: {e}'}), 200
    return jsonify({'ok': True})


@app.route('/mesh')
def mesh():
    rel = request.args.get('path', '')
    if not rel:
        abort(400)
    full = _safe_path(rel)
    if not os.path.isfile(full):
        abort(404)
    return send_file(full)


if __name__ == '__main__':
    print(f'[meshSimplification] serving {DATA_DIR} on http://0.0.0.0:{PORT}')
    app.run(host='0.0.0.0', port=PORT)
