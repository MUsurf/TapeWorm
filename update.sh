#!/bin/bash
# Use this file to update 

# Check if container name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

container_name=$1


########### Remove all existing screen sessions ###########

# Get a list of all screen sessions
sessions=$(screen -ls | grep -o '[0-9]*\.[^ ]*')

# Check if there are any sessions to kill
if [ -z "$sessions" ]; then
    echo "No screen sessions to kill."
else
    # Iterate over each session and kill it
    for session in $sessions; do
        screen -S "$session" -X quit
        echo "Killed screen session: $session"
    done
fi




# Remove existing workspace
docker exec $container_name rm -rf /root/catkin_ws

# Copy in the new workspace
docker cp catkin_ws $container_name:/root/catkin_ws

# Run catkin_make inside the container
echo Running `catkin_make` this may take a moment
docker exec $container_name bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

docker exec $container_name bash -c "cd /root/catkin_ws"