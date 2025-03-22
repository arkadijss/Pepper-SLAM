#!/bin/bash

if [ ! -d "/workspace/catkin_ws" ]; then
# Build the packages for the Pepper robot
cd /workspace
mkdir -p catkin_ws/src
cp -r ./src/perception catkin_ws/src/perception
cp -r ./src/mapping/pepper_mapping catkin_ws/src/pepper_mapping
cd catkin_ws
rosdep update && rosdep install --from-paths src/pepper_mapping -y
source /opt/ros/$ROS_DISTRO/setup.sh
catkin_make
fi

# Add the setup files to bashrc so they are sourced in every new shell
echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> "/home/$(whoami)/.bashrc"
echo "source /workspace/catkin_ws/devel/setup.bash" >> "/home/$(whoami)/.bashrc"

# Start the default command or shell to keep the container running
exec "$@"