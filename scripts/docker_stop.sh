#!/bin/bash

# Specify the container name
CONTAINER_NAME="swarmbots"

# Check if the container is running
CONTAINER_ID=$(docker ps -q -f name=$CONTAINER_NAME)

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
    echo "No running container named $CONTAINER_NAME found"
    exit(1)
fi

# Execute a bash shell in the container
docker stop $CONTAINER_NAME