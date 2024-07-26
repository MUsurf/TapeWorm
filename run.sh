#!/bin/bash

# Check if the container name is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

container_name=$1

# Function to check if a screen session exists
check_screen_session_exists() {
    session_name=$1
    if docker exec $container_name sudo screen -ls | grep -q "$session_name"; then
        return 0
    else
        return 1
    fi
}

# Function to check and log screen session status
check_screen_status() {
    echo "Checking screen sessions..."
    docker exec $container_name sudo screen -ls
    echo "--------------------------"
}

# Remove any dead screen sessions from a restarted container
echo "Wiping dead screen sessions..."
docker exec $container_name sudo screen -wipe
sleep 2
echo "--------------------------"

############################################################
# Start desired screen sessions here

# # Start the ROS node in another screen session
# screen_name="imu"
# if check_screen_session_exists $screen_name; then
#     echo "$screen_name already exists. Quitting existing session..."
#     docker exec $container_name sudo screen -XS $screen_name quit
#     sleep 2
# fi

# echo "Starting ROS node with screen '$screen_name'..."
# docker exec $container_name sudo screen -dmS $screen_name bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch imu_bno055 imu.launch" && echo "ROS node started" || echo "Failed to start ROS node"
# sleep 2
# check_screen_status

# Start the ROS none in another screen session
screen_name="statemachine"
if check_screen_session_exists $screen_name; then
    echo "$screen_name already exists. Quitting existing session..."
    docker exec $container_name sudo screen -XS $screen_name quit
    sleep 2
fi

echo "Starting ROS node with screen '$screen_name'..."
docker exec $container_name sudo screen -dmS $screen_name bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch state_machine statemachine.launch" && echo "ROS node started" || echo "Failed to start ROS node"
sleep 2
check_screen_status

############################################################

# Final check of the running screen sessions
echo "Final check of screen sessions:"
check_screen_status

# Keep the script running for observation (adjust time as needed)
sleep 5
