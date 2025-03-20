FROM ros:noetic-ros-base-focal

# NON ROOT USER ACCESS
ARG USERNAME=uzer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \  
    python3 \ 
    python3-pip \
    g++ \ 
    libgl1 \ 
    libglib2.0-0 \  
    libsm6 \ 
    libxext6 \ 
    net-tools

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-moveit-ros-visualization 

# Pepper gazebo dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros \ 
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-state-controller

# Pepper mapping dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-depth-image-proc \
    ros-$ROS_DISTRO-pointcloud-to-laserscan

# SLAM
RUN apt-get install -y \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-map-server \

# Meshes for viz
RUN apt-get install -y ros-$ROS_DISTRO-pepper-meshes

# Visualization
RUN apt-get install -y ros-$ROS_DISTRO-rqt-image-view

# X terminal emulator for multiple bashes
RUN apt-get install -y xterm

# Create a symlink for python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

# rqt_robot_steering for controlling the robot
RUN apt-get install -y ros-$ROS_DISTRO-rqt-robot-steering

# hector_trajectory_server for obtaining trajectory data
RUN apt-get install -y ros-$ROS_DISTRO-hector-trajectory-server

# Bridge
RUN apt-get install -y ros-$ROS_DISTRO-naoqi-driver

USER $USERNAME

RUN pip install torch==2.2.1
RUN pip install torchvision==0.17.1
RUN pip install --upgrade numpy
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Copy the entrypoint script into the container
COPY ./entrypoint.sh /home/$USERNAME/entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/home/uzer/entrypoint.sh"]