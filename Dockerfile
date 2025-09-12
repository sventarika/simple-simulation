FROM gitlab.ika.rwth-aachen.de:5050/fb-fi/misc/opencv-docker/ubuntu-20.04-opencv-4.8.0.76

RUN mkdir -p /work/simple-simulation
WORKDIR /work/simple-simulation

# Install CoinHSL, see: https://github.com/coin-or-tools/ThirdParty-HSL
RUN apt-get update && \
    apt-get install -y libblas3 libblas-dev liblapack3 liblapack-dev gfortran
RUN apt-get update && \
    apt-get install -y wget
RUN mkdir /work/build
WORKDIR /work/build
COPY coinhsl/ coinhsl
COPY install_hsl.sh .
RUN ./install_hsl.sh > debug.txt
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
ENV OMP_NUM_THREADS=8

WORKDIR /work/simulation-manager

# Install requirements
COPY requirements.txt .
RUN python -m pip install -r requirements.txt

# Add all modules to pythonpath
ENV PYTHONPATH=$PYTHONPATH:/work/simple-simulation
