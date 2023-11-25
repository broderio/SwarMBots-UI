# Check if build directory exists, if not make it
if [ ! -d "build" ]; then
  mkdir build
fi

# Run cmake to generate makefiles
cmake -S . -B build/

# Compile and link
cmake --build build/ --target all -- -j 4