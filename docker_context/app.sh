#!/bin/bash

# Navigate to the workspace directory inside the Docker container
cd /root/workspace

# Check if the workspace is valid (has a colcon workspace)
if [ ! -d "src" ]; then
    echo "Workspace directory (src) not found. Exiting..."
    exit 1
fi

# Build the ROS 2 workspace
echo "Building the ROS 2 workspace..."
colcon build

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "ROS 2 workspace build failed. Exiting..."
    exit 1
fi

# Source the ROS 2 setup script to set up the environment
echo "Sourcing the ROS 2 setup script..."
source /root/workspace/install/setup.bash

# Launch the simulation with the ROS 2 launch file
echo "Launching the limo simulation..."
ros2 launch limo_simulation limo.launch.py
