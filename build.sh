#!/bin/bash

# Check if container name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

# Determine if running on Jetson
IS_JETSON=$(grep -q 'NVIDIA' /proc/device-tree/model && echo "yes" || echo "no")

# Check if container exists
if docker ps -a --format '{{.Names}}' | grep -q "^$1$"; then
    # Ask for confirmation before removing the container
    read -p "Container '$1' already exists. Do you want to stop and remove it? [y/n] " -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Stop and remove the container
        echo "Stopping and removing existing container '$1'..."
        if docker stop "$1" > /dev/null && docker rm "$1" > /dev/null; then
            echo "Container '$1' removed successfully."
        else
            echo "Failed to remove container '$1'. Please check permissions."
            exit 1
        fi
    else
        echo "Exiting without removing container."
        exit 0
    fi
fi

# Build Docker image from Dockerfile
docker build -t jelly2 .

# Conditional parameters for Jetson GPIO access
if [ "$IS_JETSON" == "yes" ]; then
    RUN_OPTS="--privileged --device /dev/gpiochip0:/dev/gpiochip0 --device /dev/gpiochip1:/dev/gpiochip1"
else
    RUN_OPTS=""
fi

# Create a container from the built image
docker run -d -it --name "$1" $RUN_OPTS jelly2
