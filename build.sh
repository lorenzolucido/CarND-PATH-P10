#!/bin/bash

# cd `dirname $0`

# Remove the dedicated output directories
rm -rf build

# Compile code.
mkdir -p build
cd build
cmake ..
make -j `nproc` $*
