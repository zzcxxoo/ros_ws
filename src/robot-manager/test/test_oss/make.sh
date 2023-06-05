#! /usr/bin/bash

cmake --build build
cd build
sudo make install
cd ..