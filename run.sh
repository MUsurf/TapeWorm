#!/bin/bash

# Check if the container name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

# Start roscore
docker exec $1 sudo screen -dmS roscore bash -c "cd ~ && source /opt/ros/noetic/setup.bash && roscore"

# Start my node
docker exec $1 sudo screen -dmS ros bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch imu_bno055 imu.launch"
