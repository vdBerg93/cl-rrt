//#######################################################
//## Object detection for Prius Automated Vehicle #######
//#######################################################
#include <vector>
using namespace std;
bool DEBUG = 0;

void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void rotateVelocityVector(double& Vx, double& Vy, const vector<double>& carPose);
template<class T>
void rotateVector2D(T& X, T& Y, const T& angle){
	T Xnew =  cos(angle)*X + sin(angle)*Y;
	T Ynew = -sin(angle)*X + cos(angle)*Y;
	X = Xnew; Y = Ynew;
}

// Global variables
double Kalman_gain_pos{0.1};
double Kalman_gain_vel{0.01};
double Kalman_gain_meas{0.5};
double OBB_size{1.5};

const double Rate = 20;


// Include header files
#include <headers.h>
#include <dynamic_reconfigure/server.h>
#include <obstacle_tracker/TrackerConfig.h>

#include <functions.h>
#include <kalman.cpp>
#include "observer.cpp"

void testKalman();

//#### MAIN FUNCTION ####################################
int main (int argc, char** argv)
{
	// testKalman();
  // std::setprecision(3);
  
	// Initialize ROS
	ros::init (argc, argv, "obstacle_tracker_node"); 	// Initialize ROS system
	ros::NodeHandle nh;				// Create nodehandle
	// car_msgs::getobstacles::Response test;
	Observer ObserveObject; ptrObs = &ObserveObject;
	for(int i = 0; i!=6; i++){
		ObserveObject.carState.push_back(0);
	}

	// Rviz publisher
	ros::Publisher pubR =nh.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 100);
	ObserveObject.pubRviz = &pubR;

	// Create service server
	ros::ServiceServer server = nh.advertiseService("getobstacles", &Observer::callbackService,&ObserveObject);

	// MPC publisher
	ros::Publisher pubMPC = nh.advertise<vision_msgs::Detection2DArray>("/detection_2D",100);
	ObserveObject.pubMPC = &pubMPC;

	// Car state subscriber
	ros::Subscriber sub = nh.subscribe("/carstate",1,&Observer::callbackState, &ObserveObject);


	// TF listener
	tf::TransformListener listener;
	ObserveObject.tfListener = &listener;

	// // Dynamic reconfiguration
	dynamic_reconfigure::Server<obstacle_tracker_node::TrackerConfig> parserver;
  	dynamic_reconfigure::Server<obstacle_tracker_node::TrackerConfig>::CallbackType f;
  	f = boost::bind(&callbackParameter, _1, _2);
  	parserver.setCallback(f);

	ros::Rate r(Rate);
	int startcount = 0;
	while( ros::ok() ){
		// Give TF some time to initialize
		if (startcount<Rate){
			startcount++;
		}else{
			tf::StampedTransform tfPed1, tfPed2;
			// Lookup transformation of pedestrian link
    		ros::Time now = ros::Time::now();
			listener.waitForTransform("map","ped_link_1", now, ros::Duration(3));
    		listener.lookupTransform("map", "ped_link_1", now, tfPed1);
			listener.waitForTransform("map","ped_link_2", now, ros::Duration(3));
    		listener.lookupTransform("map", "ped_link_2", now, tfPed2);
			// Generate OBB
			car_msgs::Obstacle2D pedObs1;
			pedObs1.obb.center.x = tfPed1.getOrigin().x();	
			pedObs1.obb.center.y = tfPed1.getOrigin().y();
			pedObs1.obb.center.theta = 0;
			pedObs1.obb.size_x = OBB_size; 
			pedObs1.obb.size_y = OBB_size;
			// Generate OBB
			car_msgs::Obstacle2D pedObs2;
			pedObs2.obb.center.x = tfPed2.getOrigin().x();	
			pedObs2.obb.center.y = tfPed2.getOrigin().y();
			pedObs2.obb.center.theta = 0;
			pedObs2.obb.size_x = OBB_size; 
			pedObs2.obb.size_y = OBB_size;
			// Update obstacle vector
			ObserveObject.Obs.clear();
			ObserveObject.Obs.push_back(pedObs1);
			ObserveObject.Obs.push_back(pedObs2);
			// Update KF
			ObserveObject.updateTrackersKF();

			// Transform from map to car coordinate
			for(int i = 0; i!=ObserveObject.Obs.size(); i++){
				transformPointWorldToCar(ObserveObject.Obs[i].obb.center.x,ObserveObject.Obs[i].obb.center.y,ObserveObject.carState);
				double Gain = 1;
				ObserveObject.Obs[i].vel.linear.x = Gain*ObserveObject.Obs[i].vel.linear.x;
				ObserveObject.Obs[i].vel.linear.y = Gain*ObserveObject.Obs[i].vel.linear.y;
				rotateVelocityVector(ObserveObject.Obs[i].vel.linear.x,ObserveObject.Obs[i].vel.linear.y,ObserveObject.carState);
			}
			// Send MPC message
			vision_msgs::Detection2DArray msg = generateMPCmessage(ObserveObject.Obs);
			ObserveObject.pubMPC->publish(msg);
			assert((ObserveObject.Obs.size()>0)&&"Wrong dimension of obstacles");

			ObserveObject.sendMarkerMsg(ObserveObject.Obs);
			// Update velocities
			for(int i = 0; i!=ObserveObject.Obs.size(); i++){
				ROS_INFO_STREAM("Updated. x="<<ObserveObject.Obs[i].obb.center.x<<", y="<<ObserveObject.Obs[i].obb.center.y<<
								", Vx="<<ObserveObject.Obs[i].vel.linear.x<<", Vy="<<ObserveObject.Obs[i].vel.linear.y);
			}
		}
		ros::spinOnce();
		r.sleep();
	}
}
// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}

// Rotate the velocity vector to car frame
void rotateVelocityVector(double& Vx, double& Vy, const vector<double>& carPose){
	double X = cos(carPose[2])*Vx + sin(carPose[2])*Vy;
	double Y = -sin(carPose[2])*Vx + cos(carPose[2])*Vy;
	Vx = X; Vy = Y;
}