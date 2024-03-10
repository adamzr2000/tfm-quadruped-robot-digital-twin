#!/bin/bash

# Function to display the menu
display_menu() {
    echo "==========================="
    echo "   Select ROS MASTER IP    "
    echo "==========================="
    echo "  1 -> localhost"
    echo "  2 -> custom"
    echo "==========================="
}

# Function to get user choice
get_choice() {
    read -p "Enter your choice (1/2): " choice
    case $choice in
        1) ros_master_ip="127.0.0.1"; return;;
        2) read -p "Enter custom ROS MASTER IP: " custom_ip; ros_master_ip="$custom_ip"; return;;
        *) echo "Invalid choice. Please enter 1 or 2."; get_choice;;
    esac
}

# Prompt the user to select ROS MASTER IP
display_menu
get_choice

# Construct ROS MASTER URI
ros_master_uri="http://${ros_master_ip}:11311"

# Run docker container with selected ROS MASTER URI and ROS_IP
echo 'Running go1-robot docker image.'

docker run \
    -itd \
    --name roscore \
    --hostname roscore \
    --rm \
    --net host \
    -e ROS_MASTER_URI="$ros_master_uri" \
    -e ROS_IP="$ros_master_ip" \
    --privileged \
    adamzr2000/roscore:latest \

echo "Done."