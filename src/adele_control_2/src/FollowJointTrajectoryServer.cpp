#include <adele_control_2/FollowJointTrajectoryServer.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace adele_control_2{

    FollowAction::FollowAction(){
        //Default constructor, do not use
    }

    FollowAction::FollowAction(const ros::NodeHandle& nh, std::string name):
        name_(name), nh_(nh)
    {
        internalServer.start();
    }

    FollowAction::~FollowAction(){

    }

    void FollowAction::orderSixtySix(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){
        ros::Rate r(1);
        
        /*
        Pseudocode here:
        1. Search all the trajectory points in the goal->trajectory.points list and store them
        2. Load the trajectories into another list
        3. Publish to leonardo topic, sleep until time for next trajectory
        4. check feedback from leonardo
        5. Publish accordingly
        */
    }
}