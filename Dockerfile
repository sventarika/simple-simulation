FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
    # Python and build tools
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    # Mathematical libraries for numpy, scipy, etc.
    libblas3 \
    libblas-dev \
    liblapack3 \
    liblapack-dev \
    gfortran \
    # OpenCV dependencies
    libopencv-dev \
    python3-opencv \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    # Video codec libraries
    libx264-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    # Git and utilities
    git \
    wget \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install uv for Python package management
RUN pip3 install --no-cache-dir uv --break-system-packages

# Set up environment variables
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
ENV OMP_NUM_THREADS=8
ENV PYTHONPATH=/work/simple-simulation

# Create working directory
RUN mkdir -p /work/simple-simulation
WORKDIR /work/simple-simulation

# Default command
CMD ["/bin/bash"]
