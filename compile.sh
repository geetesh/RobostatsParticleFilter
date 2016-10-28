#!/bin/bash
echo "Compiling!"
mkdir build;
cd build;
cmake ..
make
echo "Done!"
date
