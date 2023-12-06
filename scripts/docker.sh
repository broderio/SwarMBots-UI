IMAGE="osrf/ros:humble-desktop"
CONTAINER_NAME="swarmbots"

if [ "$1" = "stop" ]; then
    # Check if the container is running
    CONTAINER_ID=$(docker ps -q -f name=$CONTAINER_NAME)

    # Check if a container was found
    if [ -z "$CONTAINER_ID" ]; then
        echo "No running container named $CONTAINER_NAME found"
        exit 1
    fi

    # Execute a bash shell in the container
    docker stop $CONTAINER_NAME
    exit 0
fi

# Check if the Docker image exists locally
if [ -z "$(docker images -q $IMAGE)" ]; then
    echo "Docker image $IMAGE not found locally"
    echo "Pulling the Docker image..."
    docker pull $IMAGE
fi

# Check if the Docker container exists
if [ -z "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    echo "Docker container $CONTAINER_NAME not found"
    echo "Running the Docker container..."

    docker run --hostname=a7be075b748b \
        --mac-address=02:42:ac:11:00:02 \
        --env=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        --env=LANG=C.UTF-8 \
        --env=LC_ALL=C.UTF-8 \
        --env=ROS_DISTRO=humble \
        -p 9002:9002 \
        -p 8765:8765 \
        -v macs.txt:/etc/macs.txt:ro \
        --restart=no \
        --label='org.opencontainers.image.ref.name=ubuntu' \
        --label='org.opencontainers.image.version=22.04' \
        --runtime=runc \
        --name $CONTAINER_NAME \
        -it \
        -d $image
fi

# Check if the container is running
CONTAINER_ID=$(docker ps -q -f name=$CONTAINER_NAME)

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
    echo "No running container named $CONTAINER_NAME found"
    echo "Starting the container..."
    docker start $CONTAINER_NAME
fi

# Execute a bash shell in the container
docker exec -it $CONTAINER_NAME bash