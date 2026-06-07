# Mesh viewer / simplifier container for FusionRobotExporter.
# Serves a small Flask app that renders the URDF/SDF of a mounted package
# folder (mounted at /data) and -- later -- runs mesh simplification on it.
FROM python:3.11-slim

WORKDIR /app

# System libs that the mesh tooling (trimesh / scipy, added later for
# simplification) tends to need. Kept minimal for the render-only stage.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY meshSimplification/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt

COPY meshSimplification/ /app/

# The user-chosen package folder is bind-mounted here by run.bat.
ENV DATA_DIR=/data
ENV PORT=5000

EXPOSE 5000

CMD ["python", "app.py"]
