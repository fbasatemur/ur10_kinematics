# ur10_kinematics

## Project Description

In the ROS1 Noetic environment, an application has been implemented for the forward kinematics of a UR10 robotic arm with 6 degrees of freedom (6DOF) in the Gazebo simulation environment. The position and orientation information of the end-effector is obtained based on the angle values provided for the 6 joints of the UR10 arm.

The project utilizes the following ROS packages: geometry_msgs, roscpp, nav_msgs, std_msgs.

To install the simulation world:
```bash
git clone https://github.com/fbasatemur/ur10_kinematics.git
mkdir -p gazebo_plugins_rtg/src
cd ur10_kinematics*
mv gazebo_plugins_rtg ../gazebo_plugins_rtg/src
cd ../gazebo_plugins_rtg
rosdep install -a
catkin_make
source ~/.bashrc
cp -r src/gazebo_plugins_rtg/models/ur10 ~/.gazebo/models
```

To start the simulation world (UR10)
```bash
roslaunch gazebo_plugins_rtg ur10.launch
```

To install ur10_kinematics:
```bash
cd ur10_kinematics*
catkin_make
source devel/setup.bash
```

For observing the end-effector pose using forward kinematics:
```bash
rosrun ur10_kinematics forward_kinematics
```

## Forward Kinematic
To obtain the end-effector position and orientation information of the UR10 robot arm, the calculation of the homogeneous matrix between joints based on the given joint points on the left is expressed below:

<p float="left">
  <img src="https://github.com/fbasatemur/ur10_kinematics/blob/main/doc/quest1_0.png?ref_type=heads" width="390" height="600"/>
  <img src="https://github.com/fbasatemur/ur10_kinematics/blob/main/doc/quest1_1.png?ref_type=heads" width="390" height="600"/> 
</p>

The appearance of the robot arm for the given joint angles { 0.5, -0.2, 0.6, -0.6, -0.4, 0.5 }, along with the end-effector position calculated from the odometry message, is provided below.

<p float="left">
  <img src="https://github.com/fbasatemur/ur10_kinematics/blob/main/doc/quest1_2.png?ref_type=heads" width="390" height="600"/>
  <img src="https://github.com/fbasatemur/ur10_kinematics/blob/main/doc/quest1_3.png?ref_type=heads" width="400" height="277"/> 
</p>

The <3,3> region of the resulting homogeneous matrix is used for orientation, and the <3,1> region is used for position extraction.
