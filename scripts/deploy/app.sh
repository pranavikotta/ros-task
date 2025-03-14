# #!/bin/bash

# # Get base.sh funcs
# source "$(dirname "$0")/base.sh"

# stop_docker

# # Declare mode, use GPU by default
# mode="gpu"

# # Declare sim, use sim by default
# sim="True"

# while getopts 'ch' opt; do
#     case "$opt" in
#         c)
#             mode="cpu"
#             ;;
#         ?|h)
#             echo "Usage: $(basename $0) [-c]"
#             exit 1
#             ;;
#     esac
# done
# shift "$(($OPTIND -1))"

# # Define the container name
# CONTAINER_NAME="limo_bot_container"

# # Start the Docker container based on the mode
# if [ "$mode" == "gpu" ]; then
#     run_docker --runtime=nvidia \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v $(dirname "$0")/../../workspace/:/root/workspace/src \
#     --name "$CONTAINER_NAME" \
#     limo_bot:sim bash -c "/root/workspace/scripts/deploy/app.sh"
# else
#     run_docker \
#     -v $(dirname "$0")/../../workspace/:/root/workspace/src \
#     --name "$CONTAINER_NAME" \
#     limo_bot:sim bash -c "/root/workspace/scripts/deploy/app.sh"
# fi

# # Navigate to the workspace directory inside the Docker container
# cd /root/workspace

# # Check if the workspace is valid (has a colcon workspace)
# if [ ! -d "src" ]; then
#     echo "Workspace directory (src) not found. Exiting..."
#     exit 1
# fi

# # Build the ROS 2 workspace
# echo "Building the ROS 2 workspace..."
# colcon build

# # Check if build was successful
# if [ $? -ne 0 ]; then
#     echo "ROS 2 workspace build failed. Exiting..."
#     exit 1
# fi

# # Source the ROS 2 setup script to set up the environment
# echo "Sourcing the ROS 2 setup script..."
# source /root/workspace/install/setup.bash

# # Launch the simulation with the ROS 2 launch file
# echo "Launching the limo simulation..."
# ros2 launch limo_simulation limo.launch.py


# #!/bin/bash

# # Get base.sh funcs
# source "$(dirname "$0")/base.sh"

# stop_docker

# # Declare mode, use GPU by default
# mode="gpu"

# while getopts 'ch' opt; do
#     case "$opt" in
#         c)
#             mode="cpu"
#             ;;
#         ?|h)
#             echo "Usage: $(basename $0) [-c]"
#             exit 1
#             ;;
#     esac
# done
# shift "$(($OPTIND -1))"

# # Define the container name
# CONTAINER_NAME="limo_bot_container"

# # Define the absolute path to your workspace
# WORKSPACE_PATH="$HOME/ros2_ws/src/ros-task/workspace"  # Change this to your actual path

# # Start the Docker container based on the mode
# if [ "$mode" == "gpu" ]; then
#     run_docker --runtime=nvidia \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v "$WORKSPACE_PATH":/root/workspace/src \
#     --name "$CONTAINER_NAME" \
#     -d limo_bot:sim  # Start the container in detached mode
# else
#     run_docker \
#     -v "$WORKSPACE_PATH":/root/workspace/src \
#     --name "$CONTAINER_NAME" \
#     -d limo_bot:sim  # Start the container in detached mode
# fi

# # Wait for the container to be ready
# sleep 5  # Adjust the sleep time as necessary

# # Execute commands inside the running container
# docker exec "$CONTAINER_NAME" bash -c "
#     # Navigate to the workspace directory
#     cd /root/workspace || exit 1

#     # Check if the workspace is valid (has a colcon workspace)
#     if [ ! -d 'src' ]; then
#         echo 'Workspace directory (src) not found. Exiting...'
#         exit 1
#     fi

#     # Build the ROS 2 workspace
#     echo 'Building the ROS 2 workspace...'
#     colcon build

#     # Check if build was successful
#     if [ \$? -ne 0 ]; then
#         echo 'ROS 2 workspace build failed. Exiting...'
#         exit 1
#     fi

#     # Source the ROS 2 setup script to set up the environment
#     echo 'Sourcing the ROS 2 setup script...'
#     source /root/workspace/install/setup.bash

#     # Launch the simulation with the ROS 2 launch file
#     echo 'Launching the limo simulation...'
#     ros2 launch limo_simulation limo.launch.py


# #!/bin/bash

# # Navigate to the workspace directory
# cd /root/workspace || { echo "Failed to change directory to /root/workspace"; exit 1; }

# # Build the ROS 2 workspace
# echo "Building the ROS 2 workspace..."
# colcon build

# # Check if the build was successful
# if [ $? -ne 0 ]; then
#     echo "ROS 2 workspace build failed. Exiting..."
#     exit 1
# fi

# # Source the ROS 2 setup script to set up the environment
# echo "Sourcing the ROS 2 setup script..."
# source /root/workspace/install/setup.bash

# # Launch the simulation with the ROS 2 launch file
# echo "Launching the limo simulation..."
# ros2 launch limo_simulation limo.launch.py


source /opt/ros/humble/setup.bash
colcon build
source /root/workspace/install/setup.bash

ros2 launch limo_simulation limo.launch.py