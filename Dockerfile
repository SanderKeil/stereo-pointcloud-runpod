FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.11 python3.11-venv python3-pip git ffmpeg libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/bin/python3.11 /usr/bin/python

WORKDIR /app

# Model source, pinned to the exact commit this project has been validated against.
RUN git clone https://github.com/NVlabs/FoundationStereo.git /app/FoundationStereo && \
    cd /app/FoundationStereo && git checkout 6e88068
# Local fix already applied upstream-equivalent: make device selection cuda/cpu-aware
# instead of hardcoding .cuda(), matching stereo_pointcloud's patched run_demo.py.
COPY foundation_stereo_device.patch /app/foundation_stereo_device.patch
RUN cd /app/FoundationStereo && git apply /app/foundation_stereo_device.patch

COPY requirements.txt /app/requirements.txt
RUN python -m pip install --no-cache-dir --upgrade pip && \
    python -m pip install --no-cache-dir torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu124 && \
    python -m pip install --no-cache-dir -r /app/requirements.txt

COPY handler.py /app/handler.py

ENV PYTHONPATH=/app/FoundationStereo
# Populated from a RunPod Network Volume mounted at /runpod-volume, not baked
# into the image (the checkpoint is 750MB+ and Google Drive, where the
# official weights are hosted, rate-limits automated downloads).
ENV CKPT_DIR=/runpod-volume/pretrained_models/11-33-40/model_best_bp2.pth

CMD ["python", "/app/handler.py"]
