FROM python:3.11-slim

WORKDIR /app

# Optional backends (open3d + pymeshlab) and their system libs are installed by
# default. For a lean image without them, build with --build-arg EXTRAS=0.
ARG EXTRAS=1

COPY requirements.txt requirements-extra.txt ./
RUN pip install --no-cache-dir -r requirements.txt && \
    if [ "$EXTRAS" = "1" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            libgl1 libgomp1 libegl1 libopengl0 libxkbcommon0 libdbus-1-3 \
            libglib2.0-0 libxrender1 libxcb1 libx11-6 libfontconfig1 libfreetype6 \
            libcom-err2 libgssapi-krb5-2 libkrb5-3 libp11-kit0 && \
        rm -rf /var/lib/apt/lists/* && \
        pip install --no-cache-dir -r requirements-extra.txt ; \
    fi

COPY . .

ENV DATA_DIR=/data \
    PORT=5000
EXPOSE 5000

# Mount your exported ROS/SDF package at /data (extras included by default):
#   docker build -t fusion-viewer .
#   docker run --rm -p 5000:5000 -v /path/to/package:/data fusion-viewer
CMD ["python", "app.py"]
