# ROS

This is a playground to learn more about ROS. The first project is with a SCARA robotic arm using inverse kinematics to obtain desired positions for the end-effector.
Put the 'catkin_inverse_kinematics' in 'catkin_ws/src' and compile*:

```bash
catkin_make && source devel/setup.bash
roslaunch kinematics_lib scara_launch.launch
```

*Assuming the user in this case has taken all the beginner tutorials on the official ROS-website.
