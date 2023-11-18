# SwarMBots-UI

## Installation

This project requires the `websocketpp` package. In order to build the package, we must also install /`vcpkg`. Go to a local directory, pull the repository

```bash
cd [DIRECTORY]
git clone https://github.com/Microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh
vcpkg install websocketpp
```

## Building and Compiling

This project uses `cmake` for building and compiling. Follow the steps below to compile:

```bash
cmake -B build/ -S . -DCMAKE_TOOLCHAIN_FILE=[DIRECTORY]/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build/
```

To execute an example:
```bash
./build/examples/[EXAMPLE]/[EXAMPLE]
```

## Pairing a Robot

To pair a robot, use the `pair.py` script as follows:

```bash
python3 pair.py [SERIAL PORT]
```

Follow the prompts to complete the pairing process.

## Using Pilot Mode

To use pilot mode, use the `pilot.py` script as follows:

```bash
python3 pilot.py [SERIAL PORT]
```

## Starting the GUI

This is an example program that starts the server, spins the robots for 5 seconds, and stops. Open `gui.html` in your browser before running this code. Press connect in the browser on the top right after the code has begun executing to begin receiving data from the robots.

```cpp
#include <vector>
#include <unistd.h>

#include "mbot.h"

int main() {
    mbot::port = "serial/port/for/host"; 
    mbot::start_server();
    std::vector<mbot> mbots = mbot::init_from_file("path/to/macs.txt");

    sleep(5);

    for (auto &m : mbots) {
        m.set_robot_vel_goal(0, 0, 5);
    }

    sleep(5);

    for (auto &m : mbots) {
        m.set_robot_vel_goal(0, 0, 0);
    }
}
```

## License

This project is licensed under the Polyform Noncommercial License - see the [LICENSE.md](LICENSE.md) file for details