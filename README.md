# Adele-ROS
The ROS packages for running the Adele 6DOF robotic arm.

## Prerequisite packages
1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. [Moveit!](https://moveit.ros.org/install/)
3. [ros_control](http://wiki.ros.org/ros_control#Install)
4. [ros-control-boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate)

To check whether you have the prerequisites installed, first run:
> source path_to_your_workspace/devel/setup.bash
>
> roscore

Then in a separate window, run:
> rospack list

If you can find all of the following in the list, you're good to go:
- TBC

## Installing prerequisites

## Documentation
The Adele-ROS workspace has 3 main packages:
1. adele_control_2, the RobotHW package (for ros_control use)
2. adele_moveit_config, the moveit package (for moveit to access the URDF)
3. adele_urdf_1, the URDF package (provides the robot model and data)

### adele_control_2
There are two critical scripts for adele_control_2:
1. AdeleHWInterface.cpp
2. [FollowJointTrajectoryServer.cpp](#### FollowJointTrajectoryServer)
3. AdeleControlLoop.cpp

#### AdeleHWInterface
Serves to define the AdeleHW class.
##### AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model)
Constructor, accepts 2 parameters.
- & nh: accepts the calling script's node handle by reference
- \* urdf_model: a pointer to the urdf model object 
  - can also be provided as NULL, in which case, the script will try to read the urdf from the ROS parameter server
#####

#### FollowJointTrajectoryServer
Defines the action server running the FollowJointTrajectory action, the FollowAction object. This serves as the main means of accepting plotted trajectories from moveit!.
##### FollowAction(onst ros::NodeHandle& nh, std::string name)
Constructor, accepts 2 parameters and starts the action server

