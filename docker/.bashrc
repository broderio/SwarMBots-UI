REPO_NAME="SwarMBots-ROS2"
REPO_URL="https://github.com/broderio/$REPO_NAME.git"

if [ ! -d "/root/$REPO_NAME" ]; then
    echo "Cloning the repository..."
    git clone $REPO_URL /root/$REPO_NAME
fi

if [ ! -f "/etc/flag" ]; then
    echo "Installing dependencies..."
    sudo apt-get update && sudo apt-get upgrade
    sudo apt-get install libncurses-dev
    sudo apt install ros-$ROS_DISTRO-foxglove-bridge

    # Create a file to indicate that the container has been created before
    touch /etc/flag
fi

source /opt/ros/humble/setup.sh
cd /root/$REPO_NAME
if [ ! -d "/root/$REPO_NAME/install" ]; then
    colcon build
fi
source /root/$REPO_NAME/install/setup.sh