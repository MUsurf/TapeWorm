#!/bin/bash

# Check if the container name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

# Copy catkin_ws directory to the specified container
docker cp catkin_ws "$1":/root
