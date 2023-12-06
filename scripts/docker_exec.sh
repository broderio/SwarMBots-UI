#!/bin/bash

# Specify the port
PORT=9002

# Find the container ID of the container using the specified port
CONTAINER_ID=$(docker ps --format '{{.ID}}\t{{.Ports}}' | grep $PORT | awk '{print $1}')

# Check if a container was found
if [ -z "$CONTAINER_ID" ]; then
    echo "No running container is using port $PORT"
    exit 1
fi

# Execute a bash shell in the container
docker exec -it $CONTAINER_ID bash