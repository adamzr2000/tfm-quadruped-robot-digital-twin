# Run the package
You can control your real Go1 robot from ROS by this package.

Before you run expamle program, please run command

```
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```
or
```
roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
```

It depends which control mode you want to use.

Then, if you want to run high-level control mode, you can run example_walk node like this
```
rosrun unitree_legged_real ros_example_walk
```

If you want to run low-level control mode, you can run example_position program node like this
```
rosrun unitree_legged_real ros_example_postion
```

You can also run the node state_sub to subscribe the feedback information from Go1 robot
```
rosrun unitree_legged_real state_sub
```

You can also run the launch file that enables you control robot via keyboard like you can do in turtlesim package
```
roslaunch unitree_legged_real keyboard_control.launch
```

And before you do the low-level control, please press L2+A to sit the robot down and then press L1+L2+start to make the robot into
mode in which you can do joint-level control, finally make sure you hang the robot up before you run low-level control.

# Additional scripts
```
roslaunch go1-math-motion circle.launch
```


# ROS in robot's rpi
```
export ROS_MASTER_URI=http://192.168.123.161:11311
```


# UDP connectivity test
Server:
```
nc -u -l 12345
```

```
echo "Testing UDP connectivity" | nc -u 127.0.0.1 12345
echo "Testing UDP connectivity" | nc -u 192.168.123.161 12345
```



