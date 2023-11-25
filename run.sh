# Check if build directory exists, if not return
if [ ! -d "build" ]; then
  echo "Build directory does not exist. Please run build.sh first."
  exit 1
fi

# Run the example specifed by input, add each input argument to the execute command
./build/examples/$1/$1 ${@:2}