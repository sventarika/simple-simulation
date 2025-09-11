#!/bin/bash

# DO NOT USE YOURSELF (IS ONLY USED DURING DOCKER BUILD)!

# Check whether the coinhsl-archive-2022.12.02.tar.gz exists
if [ -e coinhsl/coinhsl-archive-2022.12.02.tar.gz ]
then
    echo "coinhsl/coinhsl-archive-2022.12.02.tar.gz exists. Proceed with building."
else
    echo "coinhsl/coinhsl-archive-2022.12.02.tar.gz does not exist. Will not use MA27 linear solver."
    exit 0
fi

git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd /work/build/ThirdParty-HSL
mv ../coinhsl/coinhsl-archive-2022.12.02.tar.gz .
tar -xvf coinhsl-archive-2022.12.02.tar.gz
ln -s coinhsl-archive-2022.12.02 coinhsl
./configure # LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O2 -fopenmp" FCFLAGS="-g -O2 -fopenmp" CFLAGS="-g -O2 -fopenmp"
make
make install
ln -s /usr/local/lib/libcoinhsl.so /usr/local/lib/libhsl.so
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
# export OMP_NUM_THREADS=8
touch /work/build/.success
