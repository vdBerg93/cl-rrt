using namespace std;

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <limits>
#include <cmath>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include "prius_msgs/Control.h"
#include "car_msgs/State.h"
#include "car_msgs/MotionResponse.h"
#include "state_estimator/observer.h"

double convertQuaternionToEuler(geometry_msgs::Quaternion input);

int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "state_estimator_node");
	ros::NodeHandle nh;
    ros::Rate r(50);
    // publisher init
    ros::Publisher pubControl = nh.advertise<car_msgs::State>("/carstate",1);
    // Communication class init
    Observer observer(&pubControl);
    // Initialize message subscribers
    ros::Subscriber subState2 = nh.subscribe("/joint_states",1,&Observer::callbackJoints, &observer);
    // ros::Subscriber subSteer = nh.subscribe("/prius",1,&Observer::callbackSteer, &observer);
    ros::Subscriber subOdometry = nh.subscribe("/base_pose_ground_truth",1,&Observer::callbackOdometry, &observer);
    while (ros::ok()){
        ros::spinOnce();
        observer.publishStates();
        r.sleep();
    }
}


void Observer::callbackOdometry(const nav_msgs::Odometry& msg){
    // UPdate vehicle pose and longitudinal velocity
    carState[0] = msg.pose.pose.position.x;
    carState[1] = msg.pose.pose.position.y;
    carState[2] = convertQuaternionToEuler(msg.pose.pose.orientation);

    carState[4] = sqrt( pow(msg.twist.twist.linear.x,2) + pow(msg.twist.twist.linear.y,2));
    if (isnan(carState[4])){
        carState[4] = 0;
    }
}

void Observer::callbackJoints(const sensor_msgs::JointState& msg){
    // Get the steer angle from joint_states topic
    // carState[3] = msg.position.back();  // Update steer angle
    carState[3] = msg.position[4]; // 4 = front right , 5 = front left
//    cout<<"Steer angle="<<carState[3]<<" rad"<<endl;
}

void Observer::publishStates(){
    car_msgs::State msg;
    msg.state.insert(msg.state.begin(), carState.begin(), carState.end());
    assert(!isnan(msg.state[0])); assert(!isnan(msg.state[1])); assert(!isnan(msg.state[2])); assert(!isnan(msg.state[3])); assert(!isnan(msg.state[4]));
    ptrPub->publish(msg);
}

double convertQuaternionToEuler(geometry_msgs::Quaternion input){
    // Return the yaw angle from the input message
    double siny_cosp = 2 * (input.w * input.z + input.x * input.y);
    double cosy_cosp = 1 - 2 * (input.y * input.y + input.z * input.z);
    return atan2(siny_cosp, cosy_cosp);
}
