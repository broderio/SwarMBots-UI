# SwarMBots-UI

## Installation

This project requires the `websocketpp` package to start the server for the GUI. Install the prerequisite packages with the install script.

```bash
chmod +x install.sh
./install.sh
```

## Pairing a Robot

To pair a robot, use the `pair.py` script as follows:

```bash
python3 python/pair.py [SERIAL PORT]
```

Follow the prompts to complete the pairing process.

## Using Pilot Mode

To use pilot mode, use the `pilot.py` script as follows:

```bash
python3 python/pilot.py [SERIAL PORT]
```

## Building and Compiling Programs

This project uses `cmake` for building and compiling. Use the build script to create the build directory and compile the examples.

```bash
chmod +x build.sh
./build.sh
```

After the build directory is created, if changes are made to the examples, you can recompile them with the following command:

```bash
cmake --build build/ --target [EXAMPLE]
```

To execute an example:
```bash
./build/examples/[EXAMPLE]/[EXAMPLE]
```

## Starting the GUI

This is an example program that starts the server, spins the robots for 5 seconds, and stops. Open `gui.html` in your browser before running this code. Press connect in the browser on the top right after the code has begun executing to begin receiving data from the robots.

```cpp
#include <vector>
#include <unistd.h>

#include "mbot.h"

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string file_path = argv[2];

    mbot::port = port;
    std::vector<mbot> mbots = mbot::init_from_file(file_path);
    mbot::start_server();

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