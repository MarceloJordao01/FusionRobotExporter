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

from flask import Flask, jsonify, request, send_file, render_template, abort

import scene as scene_mod

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
