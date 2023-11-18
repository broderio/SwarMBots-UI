cmake_minimum_required(VERSION 3.5)
project(SwarMBots-UI)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(mbot)
add_subdirectory(examples)