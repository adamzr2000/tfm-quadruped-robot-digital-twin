# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=melodic

# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="adamzr2000@gmail.com"

# Setup the working directory in the container
WORKDIR /root

# Create a new user 'go1' with sudo privileges and set a password
RUN useradd -m go1 && \
    echo "go1:go1" | chpasswd && adduser go1 sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Update the GPG key for ROS melodic repository to avoid key expiration issues
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys AD19BAB3CBF125EA

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    git \
    wget \
    screen \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-teleop-twist-joy \    
    && rm -rf /var/lib/apt/lists/*

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1


# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/go1/catkin_ws/src

# Switch back to root to change file permissions
USER root

# Change permissions to make scripts executable
RUN chmod a+x catkin_ws/src/unitree_ros_to_real/unitree_legged_real/scripts/*

# Switch back to user go1
USER go1

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make 

# Add ROS environment setup to bashrc of user 'go1'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/go1/scripts
COPY scripts/* /home/go1/scripts/
WORKDIR /home/go1/scripts/

USER root

# Make all scripts executable
RUN chmod +755 *.sh

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

USER go1

CMD "./wait-for-ros-nodes.sh"
