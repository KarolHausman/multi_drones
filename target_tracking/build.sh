#!/bin/sh

mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=.. ../src
make -j5
make install
