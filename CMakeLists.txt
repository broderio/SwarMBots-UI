cmake_minimum_required(VERSION 3.5)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/lib/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

project(SwarMBots-UI)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(mbot)
add_subdirectory(examples)