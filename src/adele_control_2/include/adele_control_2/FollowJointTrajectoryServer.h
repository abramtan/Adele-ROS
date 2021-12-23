
#ifndef ADELE_CONTROL_2_FOLLOWJOINTTRAJECTORYSERVER
#define ADELE_CONTROL_2_FOLLOWJOINTTRAJECTORYSERVER

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include<array>
#include<vector>
#include<boost/shared_ptr.hpp>
#include<string>

namespace adele_control_2{

    class FollowAction{
        protected:

            ros::NodeHandle nh_;
            std::string name_;

            actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> internalServer;
            

            std::string serverErrors;
            control_msgs::FollowJointTrajectoryActionFeedback feedbackReporting;
            control_msgs::FollowJointTrajectoryActionResult errorReporting;

        public:

            FollowAction();
            ~FollowAction();

            FollowAction(const ros::NodeHandle& nh, std::string name);

            void orderSixtySix(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
            
    }

}


#endif 