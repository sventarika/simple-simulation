FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    # Python and build tools
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    # python3-numpy \
    # Mathematical libraries for numpy, scipy, etc.
    libblas3 \
    libblas-dev \
    liblapack3 \
    liblapack-dev \
    gfortran \
    # OpenCV dependencies and build requirements (based on https://rockyshikoku.medium.com/use-h264-codec-with-cv2-videowriter-e00145ded181)
    libopencv-dev \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgtk-3-dev \
    # Image format libraries
    libpng-dev \
    libjpeg-dev \
    libopenexr-dev \
    libtiff-dev \
    libwebp-dev \
    # Video codec libraries
    x264 \
    libx264-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    ffmpeg \
    # GStreamer for video processing
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    # SSL and security
    libssl-dev \
    # Git and utilities
    git \
    wget \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install uv for Python package management
RUN pip3 install --no-cache-dir uv --break-system-packages

# Note: python3-opencv is already installed via apt above (system OpenCV)
# If you need pip opencv-python, uncomment below (but it may conflict with system OpenCV):
# RUN pip3 uninstall -y opencv-python opencv-contrib-python || true
RUN pip3 install --no-cache-dir --no-binary opencv-python opencv-python --break-system-packages

# Set up environment variables
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
ENV OMP_NUM_THREADS=8
ENV PYTHONPATH=/work/simple-simulation

# Create working directory
RUN mkdir -p /work/simple-simulation
WORKDIR /work/simple-simulation

# Default command
CMD ["/bin/bash"]
