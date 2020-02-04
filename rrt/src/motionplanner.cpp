#include "rrt/transformations.h"
#include "transformations.cpp"

// Get updated obstacles from obstacle detection node
bool MotionPlanner::updateObstacles(){
    car_msgs::getobstacles srv;
    (*clientPtr).call(srv);
	det = srv.response.obstacles;
}

// State callback message
void MotionPlanner::updateState(car_msgs::State msg){
	state.clear(); 	state.insert(state.begin(), msg.state.begin(), msg.state.end());
	assert(state.size()==6);
}


bool MotionPlanner::resetPlanner(car_msgs::resetplanner::Request& req, car_msgs::resetplanner::Response& resp){
	motionplan.clear(); 	return true;
}

// Motion planner callback
void MotionPlanner::planMotion(car_msgs::MotionRequest req){
	fail_acclimit=0; fail_collision=0; fail_iterlimit=0; sim_count = 0;
	ROS_INFO_STREAM("---------"<<endl<<"Received request, processing...");
	if(debug_mode){cout<<"Goal =["<<req.goal[0]<<", "<<req.goal[1]<<", "<<req.goal[2]<<", "<<req.goal[3]<<"]"<<endl;}
	// Update variables
	Vehicle veh; veh.setPrius();	
	vector<double> worldState = state;
	vector<double> carPose = transformStateToLocal(worldState);
	updateLookahead(carPose[4]);	updateReferenceResolution(carPose[4]); 

	// debug fixing of ref_res =nan
	assert(!isnan(carPose[0])); assert(!isnan(carPose[1])); assert(!isnan(carPose[2])); assert(!isnan(carPose[3])); assert(!isnan(carPose[4]));
	assert(!isnan(ref_res)); 

	vmax = req.vmax; vgoal = req.goal[3];
	updateObstacles();
	ROS_INFO_STREAM("Considering "<<det.size()<<" obstacles.");
	// Determine an upperbound of the lateral acceleration introduced by bending the path
	if (req.Cxy.size()>0){
		if(debug_mode){cout<<"Cxy=["<<req.Cxy[0]<<", "<<req.Cxy[1]<<", "<<req.Cxy[2]<<endl;}
		ay_road_max = abs(pow(worldState[4],2)*(2*req.Cxy[0]));
	}else{
		ay_road_max = 0;
	}

	// Transform nodes to local coordinates
	transformNodesWorldToCar(bestNodes,worldState);
	if(req.bend){	
		for(auto it = det.begin(); it!=det.end(); ++it){
			// Convert the obstacles
			transformPoseCarToRoad(it->obb.center.x,it->obb.center.y,it->obb.center.theta,req.Cxy,req.Cxs);
			transformVelocityToRoad(it->obb.center.x, it->obb.center.y,it->vel.linear.x, it->vel.linear.y, req.Cxy);
		}
		transformNodesCarToRoad(bestNodes,worldState, req.Cxy, req.Cxs, veh);
		transformPoseCarToRoad(req.goal[0],req.goal[1],req.goal[2],req.Cxy,req.Cxs);
	}
	
	// Initialize RRT planner
	MyRRT RRT(req.goal,req.laneShifts,req.Cxy, req.bend);	
	RRT.det = det; RRT.carState = carPose; 
	ROS_INFO_STREAM("Initializing tree...");
	if (!commit_path){
		bestNodes.clear();
	}

	initializeTree(RRT,veh,bestNodes,carPose);
	for(auto it = RRT.tree.begin(); it!=RRT.tree.end(); it++){
		showNode(*it);
	}
	assert(RRT.tree.size()>0);	

	// Build the tree
	ROS_INFO_STREAM("Starting the tree build...");
	Timer timer(200); int iter = 0;				
	for(iter; timer.Get(); iter++){
		expandTree(veh, RRT, pubPtr, det, req.Cxy); 
	};
	ROS_INFO_STREAM("Expansion complete. Tree size is "<<RRT.tree.size()<<" after "<<iter<<" iterations");
	ROS_INFO_STREAM("Fail counters | col: "<<fail_collision<<" iter: "<<fail_iterlimit<<" acc: "<<fail_acclimit<<" sim it: "<<sim_count);


	// Select best path 
	if (!commit_path){
		bestNodes.clear();	
	}
	bestNodes = extractBestPath(RRT.tree,pubPtr);
	// Transform nodes to world coordinates
	if(req.bend){	
		if(debug_mode){ cout<<"bending nodes..."<<endl;}
		transformNodesRoadToCar(bestNodes,worldState, req.Cxy, req.Cxs, veh);
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
	// publishPlan(plan); 
	// Filtered message for MPC controller
	car_msgs::Trajectory msg = generateMPCmessage(plan);
	filterMPCmessage(msg);
	if(msg.x.size()>=3){
		pubMPC->publish(msg);
		publishPathToRviz(plan,pubPtr);	
	}

	ROS_INFO_STREAM("Replied to request..."<<endl<<"-------------------------");
}

// Prepare motion response message
car_msgs::Trajectory generateMPCmessage(const vector<Path>& path){
	car_msgs::Trajectory tra;
	for(auto it = path.begin(); it!=path.end(); ++it){
		for(int i = 1; i<it->tra.size(); i++){
			tra.x.push_back(it->tra[i][0]);
			tra.y.push_back(it->tra[i][1]);
			tra.theta.push_back(it->tra[i][2]);
			tra.delta.push_back(it->tra[i][3]);
			tra.v.push_back(it->tra[i][4]);
			// tra.v.push_back(6);
			tra.a.push_back(it->tra[i][5]);
			tra.a_cmd.push_back(it->tra[i][8]);
			tra.d_cmd.push_back(it->tra[i][9]);
		}
	}

	// jump:
	// Check for duplicates
	// for(int i = 0; i != (tra.x.size()-1); i++){
	// 	if( ((tra.x[i]-tra.x[i+1])<0.0001)&&((tra.y[i]-tra.y[i+1])<0.0001)){
	// 		tra.x.erase(tra.x.begin()+i);
	// 		tra.y.erase(tra.y.begin()+i);
	// 		tra.theta.erase(tra.theta.begin()+i);
	// 		tra.v.erase(tra.v.begin()+i);
	// 		break;
	// 	}
	// }

	// Double check
	for(int i = 0; i != (tra.x.size()-1); i++){
		if( (tra.x[i]==tra.x[i+1])&&(tra.y[i]==tra.y[i+1])){
			ROS_ERROR_STREAM("Still duplicates in message! Fix this code!");
			break;
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
	// Push back last point
	// bool B1 = msgFiltered.x.back()==msg.x.back();
	// bool B2 = msgFiltered.y.back()==msg.y.back();
	// if ( !(B1&B2)){
	// 	msgFiltered.x.push_back(msg.x.back());
	// 	msgFiltered.y.push_back(msg.y.back());
	// 	msgFiltered.theta.push_back(msg.theta.back());
	// 	msgFiltered.v.push_back(msg.v.back());
	// 	msgFiltered.a.push_back(msg.a.back());
	// 	msgFiltered.a_cmd.push_back(msg.a_cmd.back());
	// 	msgFiltered.d_cmd.push_back(msg.d_cmd.back());
	// }
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
			p.x = (*it2)[0];
			p.y = (*it2)[1];
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
			int IDwp = it->tra[j][7];						// Add waypoint for committed state
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

// Future state prediction
void predictState(vector<double>& X0, const Vehicle& veh, double t){
	double dt = 0.01;
	for(int i = 0; i!=int(t/dt); i++){
		vector<double> dx = {X0[4]*cos(X0[2]), X0[4]*sin(X0[2]), (X0[4]/veh.L)*tan(X0[3])};
		for(int i = 0; i!=dx.size(); i++){
			X0[i] += dt*dx[i];
		}
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
		// for(int i = 0; i<it->tra.size(); i++){
		for(int i = 1; i<it->tra.size(); i++){
			tra.x.push_back(it->tra[i][0]);
			tra.y.push_back(it->tra[i][1]);
			tra.theta.push_back(it->tra[i][2]);
			tra.delta.push_back(it->tra[i][3]);
			tra.v.push_back(it->tra[i][4]);
			tra.a.push_back(it->tra[i][5]);
			tra.a_cmd.push_back(it->tra[i][8]);
			tra.d_cmd.push_back(it->tra[i][9]);
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

// Publish the motion plan
// void MotionPlanner::publishPlan(const vector<Path>& plan){
// 	car_msgs::MotionResponse resp = preparePathMessage(plan);
// 	(*pubPlan).publish(resp);
// }

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
			cout<<it->tra[i][0]<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Tray = [";
		for(int i = 0; i!=it->tra.size(); i++){
			cout<<it->tra[i][1]<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Trav = [";
		for(int i = 0; i!=it->tra.size(); i++){
			cout<<it->tra[i][4]<<", ";
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
	cout<<"state= [";
	for(auto it = node.state.begin(); it!=node.state.end(); ++it){
		cout<<*it<<", ";
	}
	cout<<endl;
}
