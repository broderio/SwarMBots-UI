# Check if build directory exists, if not make it
if [ ! -d "build" ]; then
  mkdir build
fi

# If the first argument is "clean", remove all content within the build directory
if [ "$1" = "clean" ]; then
  rm -rf build/*
  shift  # Shift the arguments to the left
fi

# Run cmake to generate makefiles
cmake -S . -B build/

# Compile and link
if [ -z "$1" ]; then
  cmake --build build/ --target all -- -j 4
else
  for target in "$@"; do
    cmake --build build/ --target $target -- -j 4
  done
fi