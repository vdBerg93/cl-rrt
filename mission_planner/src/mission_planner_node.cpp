using namespace std;

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>

// Include messages for state and goal
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
// Include motion planner message
#include "car_msgs/MotionRequest.h"
#include "car_msgs/State.h"
#include "car_msgs/MotionResponse.h"
#include "car_msgs/resetplanner.h"
#include "car_msgs/LaneDet.h"
#include "PolynomialRegression.h"
//reset msgs
#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

int simCount = 0;
#include "functions.h"
#include "functions.cpp"

// Check if goal is reached
bool goalReachedCheck(const vector<double>& carState, const vector<double>& goalPose){
    const double goal_radius = 5;
    return ( sqrt( pow(carState[0]-goalPose[0],2) + pow(carState[1]-goalPose[1],2) ) <= goal_radius);
}

// Reset Gazebo, the localization and the motion planner
void resetSimulation(ros::ServiceClient& reset_simulation_client_, ros::ServiceClient& reset_ekf_client_, ros::ServiceClient& reset_planner_client_){
    // Empty reset messages
    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;
    car_msgs::resetplanner reset_planner_msg_;
    // Call reset clients
    reset_simulation_client_.call(reset_msg_);
    reset_ekf_client_.call(reset_pose_msg_);
    reset_planner_client_.call(reset_planner_msg_);
}


int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "mission_planner_node");
	ros::NodeHandle nh;

    // Initialize communication class
    MsgManager msgManager;

    // Subscribe to car state message. Used in motion request. State = [x,y,theta,delta,v,a]
    ros::Subscriber subState = nh.subscribe("/carstate",0,&MsgManager::stateCallback, &msgManager);
    // Subscribe to Rviz goal
    ros::Subscriber subRviz = nh.subscribe("/move_base_simple/goal", 0, &MsgManager::goalCallback, &msgManager);

    // Initialize the publisher that sends a request to the motion planner
    ros::Publisher pubMP = nh.advertise<car_msgs::MotionRequest>("/motionplanner/request",0);
    msgManager.ptrPubMP = &pubMP;
    ros::Rate rate(5);  // Used to configure motion planning frequency (Also adjust time limit in planner)

    // These service clients are used for automatically resetting Gazebo and the motion planner
    ros::ServiceClient reset_planner_client_    = nh.serviceClient<car_msgs::resetplanner>("motionplanner/reset");
    ros::ServiceClient reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    ros::ServiceClient reset_ekf_client_= nh.serviceClient<robot_localization::SetPose>("/set_pose");

    // Get parameter from server. Replanning means that the path is updated on the go.
    // Disabling replanning means motion is planned a single time only.
    bool doReplanning;
    ros::param::get("/motionplanner/replan",doReplanning);
    
    // msgManager.goalReceived = true;
    while (ros::ok()){
        if (msgManager.goalReceived){
            msgManager.sendMotionRequest();

            if (goalReachedCheck(msgManager.carPose, msgManager.goalW)){
                ROS_WARN_STREAM("Goal reached! Resetting simulation...");
                resetSimulation(reset_simulation_client_, reset_ekf_client_, reset_planner_client_);
                ROS_ERROR_STREAM("Simulation count= "<<(++simCount));
            }
        }else{
            ROS_INFO_STREAM_THROTTLE(1,"Waiting for goal pose from Rviz...");
        }
        ros::spinOnce();
        rate.sleep();
    }
}



