#include "rrt/headers.h"
#include "rrt/globals.h"
using namespace std;


//*****************************************
//    motion request callback function (MAIN)
//*****************************************
void MotionPlanner::planMotion(car_msgs::MotionRequest req){
	fail_acclimit=0; fail_collision=0; fail_iterlimit=0; sim_count = 0;
	ROS_INFO_STREAM("---------"<<endl<<"Received request, processing...");
	if(debug_mode){cout<<"Goal =["<<req.goal[0]<<", "<<req.goal[1]<<", "<<req.goal[2]<<", "<<req.goal[3]<<"]"<<endl;}
	// Update variables
	Vehicle veh; veh.setPrius();											// Initialize vehicle parameters
	VehicleState worldState = state;										// State in world coordinates
	VehicleState carPose = transformStateToLocal(worldState);				// State in car coordinates
	updateLookahead(carPose.v);	updateReferenceResolution(carPose.v); 	// Update planner parameters
	vmax = req.vmax; vgoal = req.goal[3];									// Update globals
	updateObstacles();														// Update obstacles

	transformNodesWorldToCar(bestNodes,worldState);			// Transform last path to new coordinate frame

	// When bend=true, plan in straightened road frame so that straight-line
	// references between samples follow the road instead of cutting corners.
	const bool useRoadFrame = req.bend && req.Cxy.size() >= 3 && req.Cxs.size() >= 3;
	vector<double> Cxy_car, Cxs_car;  // originals for back-transform
	if (useRoadFrame) {
		Cxy_car = req.Cxy;
		Cxs_car = req.Cxs;
		transformNodesCarToRoad(bestNodes, carPose, Cxy_car, Cxs_car, veh);
		transformStateCarToRoad(carPose, Cxy_car, Cxs_car, veh);
		// Transform goal to road frame
		double gx = req.goal[0], gy = req.goal[1], gh = req.goal[2];
		transformPoseCarToRoad(gx, gy, gh, Cxy_car, Cxs_car);
		req.goal[0] = gx; req.goal[1] = gy; req.goal[2] = gh;
		// In road frame the road is straight — zero out curvature for planning
		req.Cxy = {0.0, 0.0, 0.0};
	}

	GoalPose goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
	MyRRT RRT(goal,req.laneShifts,req.Cxy, req.bend);		// Initialize RRT planner
	RRT.det = det; RRT.carState = carPose; 					// UPDATE OBSTACLE DETECTIONS and car state

	ROS_INFO_STREAM("Initializing tree...");

	if (!commit_path){
		bestNodes.clear();
	}

	initializeTree(RRT,veh,bestNodes,carPose); assert(RRT.tree.size()>0);

	//*******************************************
	// TREE BUILDING LOOP
	// ******************************************
	Timer timer(200); int iter = 0;						// <---- MOTION PLANNER UPDATE RATE!
	ROS_INFO_STREAM("Starting the tree build...");
	for(iter; timer.Get(); iter++){
		expandTree(veh, RRT, pubPtr, det, req.Cxy);
	};
	ROS_INFO_STREAM("Expansion complete. Tree size is "<<RRT.tree.size()<<" after "<<iter<<" iterations");
	ROS_INFO_STREAM("Fail counters | col: "<<fail_collision<<" iter: "<<fail_iterlimit<<" acc: "<<fail_acclimit<<" sim it: "<<sim_count);

	// ********************************************************
	// Select a best path and tranform it to world coordinates
	// ********************************************************
	if (!commit_path){	bestNodes.clear();	}
	bestNodes = extractBestPath(RRT.tree,pubPtr);	// Select best path
	// Transform back: road → car (if applicable), then car → world
	if (useRoadFrame) {
		transformNodesRoadToCar(bestNodes, carPose, Cxy_car, Cxs_car, veh);
	}
	if(debug_mode){ cout<<"transforming nodes to global..."<<endl;}
	transformNodesCarToworld(bestNodes,worldState);
	// No solution found
	if(bestNodes.size()==0){
		ROS_ERROR_STREAM("No solution found. Returning without response."); return;
	}
	// Print best path to console
	ROS_INFO_STREAM("Printing nodes of best path");
	if(debug_mode){
		for(auto it = bestNodes.begin(); it!= bestNodes.end(); it++){
			showNode(*it);
		}
	}
	vector<Path> plan = convertNodesToPath(bestNodes);

	// Full-resolution trajectory for sim_node
	car_msgs::Trajectory msg = generateMPCmessage(plan);
	if (pubSimTra) { pubSimTra->publish(msg); }

	// Filtered message for MPC controller
	filterMPCmessage(msg);			// Reduce number of waypoints in path message. Space x meters apart.
	if(msg.x.size()>=3){
		pubMPC->publish(msg);
		publishPathToRviz(plan,pubPtr);
	}

	ROS_INFO_STREAM("Replied to request..."<<endl<<"-------------------------");
}

// Get updated obstacles from obstacle detection node
bool MotionPlanner::updateObstacles(){
	ROS_WARN_STREAM_ONCE("In motionplanner: adjust MotionPlanner::updateObstacles() with occupancy grid message");
    car_msgs::getobstacles srv;
    if (clientPtr->exists()) {
        clientPtr->call(srv);
        det = srv.response.obstacles;
    }
    return true;
}

// State callback message
void MotionPlanner::updateState(car_msgs::State msg){
	// state = [x,y,theta,delta,v,a]
	ROS_WARN_STREAM_ONCE("In MotionPlanner::updateState: edit state message to fit Prius");
	assert(msg.state.size()==6);
	state = VehicleState(msg.state[0], msg.state[1], msg.state[2],
	                     msg.state[3], msg.state[4], msg.state[5]);
}


// Clear the path stored in the motion planner
bool MotionPlanner::resetPlanner(car_msgs::resetplanner::Request& req, car_msgs::resetplanner::Response& resp){
	motionplan.clear(); 	return true;
}

/**
 * @brief Generate MPC trajectory message from the planned path.
 *
 * Iterates over path segments and trajectory states, skipping duplicate
 * positions (which occur when v=0 since dx=v*cos=0, dy=v*sin=0).
 */
car_msgs::Trajectory generateMPCmessage(const vector<Path>& path){
	car_msgs::Trajectory tra;
	for(auto it = path.begin(); it!=path.end(); ++it){
		for(int i = 1; i<it->tra.size(); i++){
			// Skip states where position hasn't advanced (occurs when v=0: dx[0]=v*cos=0, dx[1]=v*sin=0)
			if(!tra.x.empty() && tra.x.back()==it->tra[i].x && tra.y.back()==it->tra[i].y){
				continue;
			}
			tra.x.push_back(it->tra[i].x);
			tra.y.push_back(it->tra[i].y);
			tra.theta.push_back(it->tra[i].theta);
			tra.delta.push_back(it->tra[i].delta);
			tra.v.push_back(it->tra[i].v);
			tra.a.push_back(it->tra[i].a);
			tra.a_cmd.push_back(it->tra[i].ref_vel);
			tra.d_cmd.push_back(it->tra[i].steer_cmd);
		}
	}

	return tra;
}

void filterMPCmessage(car_msgs::Trajectory& msg){
	car_msgs::Trajectory msgFiltered;
	double interval = 5; // Distance between waypoints
	double d = 0;
	for(int i = 1; i!=msg.x.size(); i++){
		if (d==0){
			msgFiltered.x.push_back(msg.x[i]);
			msgFiltered.y.push_back(msg.y[i]);
			msgFiltered.theta.push_back(msg.theta[i]);
			msgFiltered.v.push_back(msg.v[i]);
			msgFiltered.a.push_back(msg.a[i]);
			msgFiltered.a_cmd.push_back(msg.a_cmd[i]);
			msgFiltered.d_cmd.push_back(msg.d_cmd[i]);
		}
		d += sqrt( pow(msg.x[i]-msg.x[i-1],2) + pow(msg.y[i]-msg.y[i-1],2));
		if (d>=interval){
			d=0;
		}
	}
	msg = msgFiltered;
}


// Message for clearing all markers
visualization_msgs::Marker clearMessage(){
	visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.ns = "motionplan";
    msg.action = visualization_msgs::Marker::DELETEALL;
}

// Message for publishing a path to Rviz (WORLD COORIDNATES)
visualization_msgs::Marker generateMessage(const vector<Path>& path){
// Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "motionplan";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    msg.id = 0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.5;	// msg/LINE_LIST markers use only the x component of scale, for the line width

    msg.color.r = 0.0;
    msg.color.b = 0.0;
    msg.color.g = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration(3600);

    geometry_msgs::Point p;
	for(auto it = path.begin(); it!=path.end(); ++it){
		for(auto it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
			p.x = it2->x;
			p.y = it2->y;
			p.z = 0;
			msg.points.push_back(p);
		}
	}
    return msg;
}

// Publish a path to Rviz
void publishPathToRviz(const vector<Path>& path, ros::Publisher* ptrPub){
	visualization_msgs::Marker msg = generateMessage(path);
	visualization_msgs::MarkerArray msg2; msg2.markers.push_back(msg);
	ptrPub->publish(msg2);
}

// Commit to a path section
vector<Path> getCommittedPath(vector<Node> bestPath, double& Tp){
	// Find committed reference
	vector<Path> commit;
	for(auto it = bestPath.begin(); it!=bestPath.end(); ++it){ 	// Loop through path
		Path path;		path.ref.dir = it->ref.dir;				// Initialize path
		for(int j = 0; j!=((*it).tra.size()); ++j){				// Start at second entry to avoid double values in path when merging sections
			Tp += sim_dt;										// Update committed time
			path.tra.push_back(it->tra[j]);						// Add state to committed path
			int IDwp = (int)it->tra[j].waypoint_id;			// Add waypoint for committed state
			path.ref.x.push_back(it->ref.x[IDwp]);				// push back waypoint
			path.ref.y.push_back(it->ref.y[IDwp]);				// push back waypoint
			path.ref.v.push_back(it->ref.v[IDwp]);				// push back waypoint
			if (((Tp)>=Tcommit)&&(path.ref.x.size()>=3)){		// Path should be at least three points long for controller to work
				commit.push_back(path);
				return commit;
			}
		}
		commit.push_back(path);
	}
	return commit;
}

/**
 * @brief Predict future vehicle state using simplified kinematic model.
 *
 * Integrates x, y, theta forward in time using Euler steps of dt=0.01s,
 * assuming constant velocity and steering angle.
 */
void predictState(VehicleState& X0, const Vehicle& veh, double t){
	double dt = 0.01;
	for(int i = 0; i!=int(t/dt); i++){
		double dx = X0.v*cos(X0.theta);
		double dy = X0.v*sin(X0.theta);
		double dtheta = (X0.v/veh.L)*tan(X0.delta);
		X0.x     += dt*dx;
		X0.y     += dt*dy;
		X0.theta += dt*dtheta;
	}
}

// Prepare motion response message
car_msgs::MotionResponse preparePathMessage(const vector<Path>& path){
	car_msgs::MotionResponse resp;
	for(auto it = path.begin(); it!=path.end(); ++it){
		// Prepare reference message
		car_msgs::Reference ref;
		ref.dir = it->ref.dir;
		ref.x.insert(ref.x.begin(), it->ref.x.begin(), it->ref.x.end());
		ref.y.insert(ref.y.begin(), it->ref.y.begin(), it->ref.y.end());
		ref.v.insert(ref.v.begin(), it->ref.v.begin(), it->ref.v.end());
		resp.ref.push_back(ref);
		// Prepare trajectory message
		car_msgs::Trajectory tra;
		for(int i = 1; i<it->tra.size(); i++){
			tra.x.push_back(it->tra[i].x);
			tra.y.push_back(it->tra[i].y);
			tra.theta.push_back(it->tra[i].theta);
			tra.delta.push_back(it->tra[i].delta);
			tra.v.push_back(it->tra[i].v);
			tra.a.push_back(it->tra[i].a);
			tra.a_cmd.push_back(it->tra[i].ref_vel);
			tra.d_cmd.push_back(it->tra[i].steer_cmd);
		}
		resp.tra.push_back(tra);
	}
	return resp;
}

// Merge nodes into a path
vector<Path> convertNodesToPath(const vector<Node> &path){
	// Prepare message
	vector<Path> result;
	cout<<"pathsize="<<path.size()<<endl;
	for(auto it = path.begin(); it!=path.end(); ++it){
		Path segment;
		segment.ref = it->ref;
		segment.tra = it->tra;
		result.push_back(segment);
	}
	return result;
}

// Publish the best path
void MotionPlanner::publishBestPath(const vector<Path>& path){
	car_msgs::MotionResponse resp = preparePathMessage(path);
	(*pubBest).publish(resp);
}

void MotionPlanner::storeCommit(const vector<Path>& commit){
	if(commit_path){
		for(auto it = commit.begin(); it!=commit.end(); ++it){
			motionplan.push_back(*it);
		}
	}
}
// Print a path to the terminal
void showPath(const vector<Path>& path){
	for(auto it = path.begin(); it!=path.end(); it++){
		cout<<"Refx = [";
		for(int i = 0; i!=it->ref.x.size(); i++){
			cout<<it->ref.x[i]<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Refy = [";
		for(int i = 0; i!=it->ref.y.size(); i++){
			cout<<it->ref.y[i]<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Refv = [";
		for(int i = 0; i!=it->ref.v.size(); i++){
			cout<<it->ref.v[i]<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Trax = [";
		for(int i = 0; i!=it->tra.size(); i++){
			cout<<it->tra[i].x<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Tray = [";
		for(int i = 0; i!=it->tra.size(); i++){
			cout<<it->tra[i].y<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Trav = [";
		for(int i = 0; i!=it->tra.size(); i++){
			cout<<it->tra[i].v<<", ";
		}
		cout<<"]"<<endl;
	}
}


// Print a node to the terminal
void showNode(const Node& node){
	cout<<"--- Node output ---"<<endl;
	cout<<"Parent= "<<node.parentID<<", Goal reached= "<<node.goalReached<<endl;
	cout<<"Refx = ["<<node.ref.x.front()<<", "<<node.ref.x.back()<<"]"<<endl;
	cout<<"Refy = ["<<node.ref.y.front()<<", "<<node.ref.y.back()<<"]"<<endl;
	cout<<"state= ["<<node.state.x<<", "<<node.state.y<<", "<<node.state.theta<<", "
	    <<node.state.delta<<", "<<node.state.v<<", "<<node.state.a<<"]"<<endl;
}
