FROM gitlab.ika.rwth-aachen.de:5050/fb-fi/misc/opencv-docker/ubuntu-20.04-opencv-4.8.0.76

RUN mkdir -p /work/simulation-manager
WORKDIR /work/simulation-manager

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
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager/submodules/pilots
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager/submodules/lanelet-network-wrapper
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager/submodules/mpc-controller
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager/submodules/simulation-core
ENV PYTHONPATH=$PYTHONPATH:/work/simulation-manager/submodules/simple-scenario
