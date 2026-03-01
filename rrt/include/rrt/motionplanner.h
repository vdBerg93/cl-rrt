#ifndef MP_H
#define MP_H

struct Path{
	MyReference ref;
	StateArray tra;
};

void predictState(VehicleState& X0, const Vehicle& veh, double t);

// Perhaps unnessecary
void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs);
vector<Path> convertNodesToPath(const vector<Node> &path);

vector<Path> getCommittedPath(vector<Node> bestPath, double& Tc);
car_msgs::Trajectory generateMPCmessage(const vector<Path>& path);
void filterMPCmessage(car_msgs::Trajectory& msg);

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
		vector<Path> motionplan; 		// Current motion plan in global coordinates
		vector<Node> bestNodes;			// Logging best path as nodes
		VehicleState state;
		ros::ServiceClient* clientPtr;			// Pointer to client
		ros::Publisher* pubPtr; 				// Pointer to Rviz markers
		ros::Publisher* pubPlan;
		ros::Publisher* pubMPC;
		ros::Publisher* pubBest;				// Pointer to response publisher
		vector<car_msgs::Obstacle2D> det;		// 2D OBB
		void planMotion(car_msgs::MotionRequest msg);
		bool updateObstacles();
		void updateState(car_msgs::State msg);
		void publishPlan(const vector<Path>& plan);
		void publishBestPath(const vector<Path>& path);
		void storeCommit(const vector<Path>& commit);
		bool resetPlanner(car_msgs::resetplanner::Request& req, car_msgs::resetplanner::Response& resp);
		MotionPlanner() = default;
};

visualization_msgs::Marker clearMessage();
visualization_msgs::Marker generateMessage(const vector<Path>& path);
void publishPathToRviz(const vector<Path>& path, ros::Publisher* ptrPub);

void showPath(const vector<Path>& path);
void showNode(const Node& node);
#endif
