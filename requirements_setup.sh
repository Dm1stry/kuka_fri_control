#!/bin/bash

sudo apt update

echo "Install Eigen3 and its dependence"

sudo apt install cmake libboost-dev libblas-dev liblapack-dev
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
gunzip eigen-3.4.0.tar.gz && tar xf eigen-3.4.0.tar

cd eigen-3.4.0/ && mkdir build && cd build/

cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo make install

cd ../..

rm -f eigen-3.4.0.tar
rm -rf eigen-3.4.0

echo "Install pinocchio"

git clone https://github.com/stack-of-tasks/pinocchio.git

cd pinocchio && mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF
cmake --build .
sudo make install

cd ../..
rm -rf pinocchio

