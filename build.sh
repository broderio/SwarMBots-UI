#!/bin/bash

# If the build directory exists, delete it
if [ -d "build" ]; then
    rm -rf build
fi

# Create the build directory
mkdir build

# Run cmake commands
cmake -B build/ -S . -DCMAKE_TOOLCHAIN_FILE=lib/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build/