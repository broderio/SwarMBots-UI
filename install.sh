#!/bin/bash

# Initialize and update the vcpkg submodule
git submodule update --init --recursive

# Build vcpkg
./lib/vcpkg/bootstrap-vcpkg.sh

# Install websocketpp
./lib/vcpkg/vcpkg install websocketpp