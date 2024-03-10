#!/bin/bash



# Function to clean up and terminate all ROS nodes
cleanup() {
    echo "Cleaning up ROS nodes..."

    kill "$keyboard_control"
    rosnode kill go1_twist_sub
    rosnode kill go1_joint_states_publisher

    echo "Cleanup finished."
    exit 0
}

# Trap Ctrl+C signal (SIGINT) and call the cleanup function
trap cleanup SIGINT

source /home/go1/catkin_ws/devel/setup.bash

# Start go1 twist sub in the background
screen -S ros-go1-twist-sub -dm rosrun unitree_legged_real twist_sub_wifi6 __name:=go1_twist_sub --wait

# Start joint states publisher in the background
screen -S ros-joint-states-publisher -dm rosrun ros_state_helper high_level_joint_states_publisher.py --wait

source /opt/ros/$ROS_DISTRO/setup.bash

# Start the keyboard control in the foreground
rosrun teleop_twist_keyboard teleop_twist_keyboard.py