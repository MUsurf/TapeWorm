FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# The base image already has ROS installed with its dependencies, 
# but you might still need to install specific utilities.
RUN apt-get update && apt-get install -y \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    python3 \
    python3-pip \
    screen \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Ensure Python 3 is the default python version
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install ROS dependencies, including catkin
RUN apt-get update && apt-get install -y \
    ros-noetic-catkin \
    ros-noetic-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# This step may not be necessary as the setup should already be in the bashrc of the base image
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install numpy using pip
RUN python3 -m pip install numpy

# Using bash to ensure the ROS environment is properly sourced
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    apt-get update && \
    rosdep install --from-paths /root/catkin_ws/src --ignore-src --rosdistro=noetic -y && \
    rm -rf /var/lib/apt/lists/*'

CMD ["bash"]
