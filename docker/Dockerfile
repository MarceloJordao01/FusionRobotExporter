# Mesh viewer / simplifier container for FusionRobotExporter.
# Serves a small Flask app that renders the URDF/SDF of a mounted package folder
# (mounted at /data) and runs mesh simplification on it.
#
# This is the Dockerfile used by build.bat (image: fusion-mesh-viewer).
# Build context is the FusionRobotExporter/ folder.
FROM python:3.11-slim

WORKDIR /app

# System libs needed by the optional simplification backends:
#   open3d   -> libgl1 libgomp1
#   pymeshlab (MeshLab/Qt) -> the rest of this list
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libgomp1 libegl1 libopengl0 libxkbcommon0 libdbus-1-3 \
        libglib2.0-0 libxrender1 libxcb1 libx11-6 libfontconfig1 libfreetype6 \
        libcom-err2 libgssapi-krb5-2 libkrb5-3 libp11-kit0 \
    && rm -rf /var/lib/apt/lists/*

# Base deps + the optional backends (open3d, pymeshlab) so every algorithm in the
# viewer's dropdown is available out of the box.
COPY meshSimplification/requirements.txt meshSimplification/requirements-extra.txt /app/
RUN pip install --no-cache-dir -r /app/requirements.txt -r /app/requirements-extra.txt

COPY meshSimplification/ /app/

# The user-chosen package folder is bind-mounted here by run.bat.
ENV DATA_DIR=/data
ENV PORT=5000

EXPOSE 5000

CMD ["python", "app.py"]
