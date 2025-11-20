FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    # Python and build tools
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    git \
    build-essential \
    # OpenCV dependencies
    libgl1 \
    libglib2.0-0

# Install uv for Python package management
RUN pip3 install --no-cache-dir uv --break-system-packages

# Set up environment variables
ENV PYTHONPATH=/work/simple-simulation

# Create working directory
RUN mkdir -p /work/simple-simulation
WORKDIR /work/simple-simulation
COPY . /work/simple-simulation/
RUN cd /work/simple-simulation && uv sync

# Default command
CMD ["/bin/bash"]
