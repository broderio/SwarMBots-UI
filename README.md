# SwarMBots-UI

## Installation

This project requires the `websocketpp` package to start the server for the GUI. Install the prerequisite packages with the install script.

```bash
chmod +x scripts/install.sh
./scripts/install.sh
```

## Pairing a Robot

To pair a robot, use the `pair.py` script. Plug the client board of the MBot that you wish to pair into your laptop. Run the script and press the reset button on the client board. A MAC address should appear in the terminal if the program was executed successfully.

```bash
python3 python/pair.py [SERIAL PORT]
```

## Using Pilot Mode

To use pilot mode, use the `pilot.py` script. The host board must be in serial mode when the script is run. It will then send a timesync message to all robots whose MAC addresses are in `macs.txt`. You can then switch the host board to pilot mode and drive the paired MBots.

```bash
python3 python/pilot.py [SERIAL PORT]
```

## Building and Compiling Programs

This project uses `cmake` for building and compiling. To compile the examples, run the following commands.

```bash
chmod +x scripts/build.sh
./scripts/build.sh
```

To save on compile time, you can specify the example you want to compile, rather than all of them.

```bash
./scripts/build.sh [EXAMPLE]
```

After compilation, all of the executables are located in `build/examples/*`. To execute an example, run the following command:
```bash
chmod +x scripts/run.sh
./scripts/run.sh [EXAMPLE] [ARG1] [ARG2] ... [ARGN]
```

## Starting the GUI

This is an example program that starts the server, spins the robots for 5 seconds, and stops. Open `gui.html` in your browser before running this code. Press connect in the browser on the top right after the code has begun executing to begin receiving data from the robots.

```cpp
#include <vector>
#include <unistd.h>

#include "mbot.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    mbot::port = port;
    std::vector<mbot> mbots = mbot::init_from_file();
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

## Interfacing with ROS2

Follow the instructions below to interface with ROS2:

**Terminal 1:**
```bash
. scripts/docker_run.sh
. scripts/docker_exec.sh
cd root
git clone https://github.com/broderio/SwarMBots-ROS2.git
cd SwarMBots-ROS2
source /opt/ros/humble/setup.sh
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libncurses-dev
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
colcon build
source install/setup.sh
```

**Terminal 2:**
```bash
. scripts/docker_exec.sh
cd root/SwarMBots-ROS2
source /opt/ros/humble/setup.sh
source install/setup.sh
```

**Terminal 3:**
```bash
python3 python/uartSocket.py [SERIAL PORT]
```

**Terminal 2:**
```bash
ros2 run mbot mbot
```

**Terminal 1:**
```bash
ros2 run mbot teleop
```

**Optionally in Terminal 4:**
```bash
cd root/SwarMBots-ROS2
source /opt/ros/humble/setup.sh
source install/setup.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## License

This project is licensed under the Polyform Noncommercial License - see the [LICENSE.md](LICENSE.md) file for details
