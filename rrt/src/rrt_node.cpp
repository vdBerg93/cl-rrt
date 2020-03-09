// Global vars for debugging and plotting
bool draw_tree =1;
bool draw_obs = 0;
bool draw_final_path = 0;
bool debug_mode = 0;
bool debug_reference = 0;
bool debug_velocity = 0;
bool draw_states = 0;
bool debug_sim = 0;
bool commit_path = false;
bool obs_use_pred = true;
double Tcommit {0.25};

// Global variables
double sim_dt;
double ctrl_tla, ctrl_dla, ctrl_mindla, ctrl_dlavmin, ctrl_Kp, ctrl_Ki;
double ref_res, ref_int, ref_mindist, vmax, vgoal;
double ay_road_max;

// Failure counters
int fail_iterlimit{0};
int fail_collision{0};
int fail_acclimit{0};
int sim_count{0};

#include "include.cpp"

void updateParameters(){
	// Get parameters from server
	ros::param::get("ctrl/tla",ctrl_tla);
	ros::param::get("ctrl/mindla",ctrl_mindla);
	ros::param::get("ctrl/dlavmin",ctrl_dlavmin);
	ros::param::get("ctrl/refint",ref_int);
	ros::param::get("ctrl/refmindist",ref_mindist);
	ros::param::get("ctrl/sampleTime",sim_dt);
	ros::param::get("ctrl/Kp",ctrl_Kp);
	ros::param::get("ctrl/Ki",ctrl_Ki);
}

int main( int argc, char** argv ){	
	// Initialize ros node handle
	ros::init(argc, argv, "rrt_node");
	ros::NodeHandle nh; ros::Rate rate(20);
	
	updateParameters(); 			// Get parameters from server
	MotionPlanner motionPlanner;	// Initialize MP object
	// Create marker publisher for Rviz
	ros::Publisher pubMarker = nh.advertise<visualization_msgs::MarkerArray>("tree_markerarray",1);
	motionPlanner.pubPtr = &pubMarker; 	// Initialize global pointer to marker publisher

	// Motion request subscriber
	ros::Subscriber sub  = nh.subscribe("/motionplanner/request",0,&MotionPlanner::planMotion, &motionPlanner);

	// Publisher for best path
	ros::Publisher pubBest = nh.advertise<car_msgs::MotionResponse>("/motionplanner/bestpath",1);
	motionPlanner.pubBest = &pubBest;
	
	// Publisher for committed path
	ros::Publisher pubPlan = nh.advertise<car_msgs::MotionResponse>("/motionplanner/response",1);
	motionPlanner.pubPlan = &pubPlan;

	// Path
	ros::Publisher pubMPC = nh.advertise<car_msgs::Trajectory>("/path_publisher/path",1);
	motionPlanner.pubMPC = &pubMPC;

	// State subscriber
	ros::Subscriber subState = nh.subscribe("carstate",1,&MotionPlanner::updateState, &motionPlanner);

	// Reset service
	ros::ServiceServer server = nh.advertiseService("/motionplanner/reset",&MotionPlanner::resetPlanner,&motionPlanner);

	// Register client for obstacle detector
	ros::ServiceClient client = nh.serviceClient<car_msgs::getobstacles>("getobstacles");
	motionPlanner.clientPtr = &client;

	// Give control to ROS
	cout<<"RRT Node running..."<<endl;
	ros::spin();
}

