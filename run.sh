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
    if docker exec $container_name screen -ls | grep -q "$session_name"; then
        return 0
    else
        return 1
    fi
}

# Function to check and log screen session status
check_screen_status() {
    echo "Checking screen sessions..."
    docker exec $container_name screen -ls
    echo "--------------------------"
}

# Remove any dead screen sessions from a restarted container
echo "Wiping dead screen sessions..."
docker exec $container_name sudo screen -wipe
sleep 2
echo "--------------------------"

# Ask user for the desired launch file
echo "Which ROS launch file would you like to start?"
echo "1) sm.launch"
echo "2) sm_nomotor.launch"
read -p "Enter the number of your choice: " choice

# Determine the launch file based on user input
case $choice in
    1)
        launch_file="sm.launch"
        ;;
    2)
        launch_file="sm_nomotor.launch"
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

############################################################
# Start desired screen sessions here

# Start the ROS node in another screen session
screen_name="Jelly2"
if check_screen_session_exists $screen_name; then
    echo "$screen_name already exists. Quitting existing session..."
    docker exec $container_name sudo screen -XS $screen_name quit
    sleep 2
fi

echo "Starting ROS node with screen '$screen_name' and launch file '$launch_file'..."
docker exec $container_name sudo screen -dmS $screen_name bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch main main.launch launch_file:=$launch_file" && echo "ROS node started" || echo "Failed to start ROS node"
sleep 2
check_screen_status

############################################################

# Final check of the running screen sessions
echo "Final check of screen sessions:"
check_screen_status

# Keep the script running for observation (adjust time as needed)
sleep 5
