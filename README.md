# JellyRos2

formerly Jelly_ROS_23-24

This repository contains the code for the second iteration of the submersible robot Jelly.

## Development

Place desired commands to run the code in run.sh. catkin_ws is located in the ~ directory which is /root for this container.

**Do not develop within the container** as your changes will not be saved. To install a dependency please add it to the Dockerfile.

## Docker

To run you must have the Docker Engine installed an NVIDIA Jetson. This can run on any device so long as the path to the GPIO is changed for RUN_OPTS in build.sh

To run you must use a bash terminal. For Windows you can use GitBash.

`sudo bash build.sh <name>` or `./build.sh <name>`
Replace `<name>` with the desired name of the container. The script will prompt to delete a container if an existing one is found.

## Running with ROS

Before you run anything with ros you need to have sourced ros this is usually done with the command `source XXXX`

Steps to run ROS

1. Source ROS into system
2. cd catkin_workspace directory
3. catkin_make
4. source devel/setup.bash
5. rosrun 'package_name' 'python_file'.py

Common Problems

+ ROS is not sourced roscore will not run
+ catkin_make fails because you are missing a dependency
+ catkin_make fails because you did not configure CMakeLists
+ Python files are not executables

+ Check that I2C address is correct
+ All files should be run with python3 as the interpreter
+ Types for launch files are **Very important**

At step 2 you should see a directory with the following structure

    catkin_workspace
    |___
        src
        |___
            CMakeLists.txt
            'package_name'/
            |___
                CMakeLists
                package.xml
                src/
                |___
                    **python files**
                scripts/
                |___
                    **python files**

## Documentation Practices

### Catkin Modules

All nodes in Catkin Workspace must have the following documentation added at the head of the file. White Space is important here!

    '''

    "Control System: eg. ROS or ROS2"

    Node: "Name"
    Publishes: 
        "Topics each on own line"
    Subscribes:
        "Topics each on own line"
    
    Maintainer: "Name"

    '''

An optional but highly encouraged additional piece of documentation to add to each file is a short addendum to the above segment explaining what and why the node exists.

    '''
    
    This Node functions to do X

    This Node is here to break out the behavoirs of X system into multiple nodes.

    '''

### Documentation Builder

Documentation will be built with [Sphinx](https://www.sphinx-doc.org/en/master/) this requires that there be a second file system 'Documentation' which is compiled into docs.

All code in 'Generated/' is automatically grabbed by 'Generate_docs.sh' **do not edit here all changes can be overwritten**.
