# Check if build directory exists, if not make it
if [ ! -d "build" ]; then
  mkdir build
fi

# Run cmake to generate makefiles
cmake -S . -B build/

# Compile and link
if [ -z "$1" ]; then
  cmake --build build/ --target all -- -j 4
else
  cmake --build build/ --target $1 -- -j 4
fi