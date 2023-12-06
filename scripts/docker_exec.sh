#!/bin/bash

# Specify the container name
CONTAINER_NAME="swarmbots"

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