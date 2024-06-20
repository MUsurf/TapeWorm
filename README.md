# JellyRos2
JellyRos2 is formerly Jelly_ROS_23-24
This repository contains the code for the second iteration of the submersible robot Jelly.

To run you must have the Docker Engine installed an NVIDIA Jetson. This can run on any device so long as the path to the GPIO is changed for RUN_OPTS in build.sh
To run you must use a bash terminal. For Windows you can use GitBash

`sudo bash build.sh <name>` or `./build.sh <name>`
Replace `<name>` with the desired name of the container. The file may ask you if you would like to remove the existing container if one currently exists under that name.

---
### During Development
Place desired commands to run the code in run.sh. catkin_ws is located in in the ~ directory which is /root for this container.
Do not develop within the container as your changes will not be saved. To install a dependency please add it to the Dockerfile