//#### Point cloud conversion function ########################
// 1. The message is converted to pcl format
// 2. The point cloud is filterd for the ground
// 3. The outliers are taken
// 4. Clusters are taken from the outliers.
// 5. The size and centerpoint from the clusters are taken
// 6. The size and coordinates are sent 
#include <tracker.h>
#include <car_msgs/State.h>
vision_msgs::Detection2DArray generateMPCmessage(const vector<car_msgs::Obstacle2D>& obstacles);

struct Observer{
	// Obstacle data & Trackers
	vector<double> laneCxy;
	vector<car_msgs::Obstacle2D> Obs;
	vector<Tracker> trackers;
	vector<double> carState;
	void updateTrackers();
	void updateTrackersKF();
	// Callback functions
	void callbackTF(const tf2_msgs::TFMessage& msgIn);
    bool callbackService(car_msgs::getobstacles::Request &req, car_msgs::getobstacles::Response &resp);
	void callbackParameter(obstacle_tracker_node::TrackerConfig &config, uint32_t level);
	void callbackState(const car_msgs::State& msg);
	// Rviz publishing
	void sendMarkerMsg(const vector<car_msgs::Obstacle2D>& det);
	// Publishers
	ros::Publisher* pubPtr;
	ros::Publisher* pubRviz;
	ros::Publisher* pubMPC;
	tf::TransformListener* tfListener;
};

Observer* ptrObs;

void Observer::callbackState(const car_msgs::State& msg){
	carState.clear(); 
	carState.insert(carState.begin(), msg.state.begin(), msg.state.end());
}

bool Observer::callbackService(car_msgs::getobstacles::Request &req, car_msgs::getobstacles::Response &resp){
	for(auto it = Obs.begin(); it!=Obs.end(); ++it){
		resp.obstacles.push_back(*it);
	}
	ROS_INFO_STREAM("Request received. Returned detections: "<<resp.obstacles.size());
	return true;
}

void callbackParameter(obstacle_tracker_node::TrackerConfig &config, uint32_t level){

	Kalman_gain_pos = 		config.Kalman_gain_pos;
	Kalman_gain_vel = 		config.Kalman_gain_vel;
	Kalman_gain_meas = 		config.Kalman_gain_meas;
	OBB_size = config.OBB_size;
	ptrObs->trackers.clear();
	// PCL_cluster_maxit = 	config.PCL_cluster_maxit;
	// PCL_cluster_treshold = 	config.PCL_cluster_treshold;
		ROS_WARN_STREAM("Received reconfigure request. Updated parameters and deleted trackers.");
}

void Observer::callbackTF(const tf2_msgs::TFMessage& msgIn){
	ROS_INFO_STREAM("Calling TF callback...");
	// Find pedestrian
	car_msgs::Obstacle2D pedObs;
	string ped = "ped_link_1";
	// for( int i = 0; i!=msgIn.transforms.size(); i++){
	for(auto it = msgIn.transforms.begin(); it!=msgIn.transforms.end(); ++it){
		// string itString = it->child_frame_id;
		if (ped.compare(it->child_frame_id)){
			ROS_INFO_STREAM("Found ped in TF");
			double x = it->transform.translation.x;
			double y = it->transform.translation.y;
			ROS_INFO_STREAM("Got position");
			pedObs.obb.center.x = x;
			pedObs.obb.center.y = y;
			break;
		}
	}
	pedObs.obb.center.theta = 0;
	pedObs.obb.size_x = 1; pedObs.obb.size_y = 1;
	// Pushback in vector
	Obs.clear();
	Obs.push_back(pedObs);
	assert((Obs.size()!=0)&&"Wrong obstacle vector size!");
	ROS_INFO_STREAM("Pushed in vector");
	// Send MPC message
	vision_msgs::Detection2DArray msg = generateMPCmessage(Obs);
	pubMPC->publish(msg);
	ROS_INFO_STREAM("Sent the MPC message...");
	// Publish to Rviz
	// pubPtr->publish(Obs);
	// ROS_INFO_STREAM("Published to Rviz...");
	// updateTrackers();
	// updateTrackersKF();
	ROS_INFO_STREAM("Updated trackers...");
	sendMarkerMsg(Obs);
	ROS_INFO_STREAM("Sent marker message...");
	ROS_INFO_STREAM("Updated obstacles and trackers.");
}

vector2D getRearMidCord(const car_msgs::Obstacle2D& det){
	vector2D vecYaxis = {-sin(det.obb.center.theta), cos(det.obb.center.theta)};		// normal vector rotated y-axis
	double xr = det.obb.center.x - vecYaxis.dx*det.obb.size_y/2;						// Get x coordinate
	double yr = det.obb.center.y - vecYaxis.dy*det.obb.size_y/2;						// Get y coordinate
	vector2D Prear = {xr,yr};
	return Prear;
}

void Observer::updateTrackersKF(){
// Generate list of tracker IDs
	vector<int> trID;
	for(int j = 0; j!=trackers.size(); j++){
		trID.push_back(j);
	}
	// cout<<"Numer of trackers "<<trID.size()<<endl;
	// Loop through obstacles and match to closest tracker
	for(int i = 0; i!=Obs.size(); i++){
		vector2D Prear = getRearMidCord(Obs[i]);		// Get obstacle rear mid point (closest)
		// Find closest tracker
		double dmin = inf; int idmin = trackers.size()+10;
		for(int j = 0; j!=trID.size(); j++){
			Eigen::VectorXd X = trackers[trID[j]].kf.state();
			double d = pow(Prear.dx-X(0),2) + pow(Prear.dy-X(1),2);
			if (d<dmin){
				dmin = d; idmin = j;
			}
		}
		if (DEBUG){
			cout<<"--- DEBUG INFO ---"<<endl;
			cout<<"trackerlist size "<<trID.size()<<endl;
			cout<<"no# obstacles ="<<Obs.size()<<endl;
			cout<<"now processing obstacle id "<<i<<endl;
			cout<<"------------------"<<endl;
		}

		// cout<<"Looped through trackers."<<endl;
		// 1. A tracker is found. Update it and remove it from the list
		if (idmin<trackers.size()){
			// assert(idmin<trackers.size());
			trackers[trID[idmin]].update(Prear.dx, Prear.dy, Obs[i]);
			if (DEBUG) {
				cout<<"trackeridvecsize="<<trID.size()<<endl;
				cout<<"idmin ="<<idmin<<endl;
			}
			auto it = trID.begin()+idmin;
			trID.erase(it);
		// 2. No tracker was found. Initialize a new one.
		}else{
			Tracker newTracker(Prear.dx,Prear.dy);
			trackers.push_back(newTracker);
			if (DEBUG){
				cout<<"Added a new tracker"<<endl;
			}
		}
	}
	// 3. Increase fail counter for all trackers that were not updated
	for( int i = 0; i!=trID.size(); i++){
		// assert(trID.size()<=0);	// if this ever fails, update the function
		trackers[trID[i]].countLost++;
		auto it = trackers.begin()+trID[i];
		trackers.erase(it);
		if (DEBUG){	cout<<"Deleted a tracker!"<<endl;	}
	}
	return;
}


void Observer::sendMarkerMsg(const vector<car_msgs::Obstacle2D>& obsArray){
	visualization_msgs::MarkerArray msg;

	// GENERATE OBB BOX MARKER LINES
	for(int j = 0; j!=obsArray.size(); j++){
		// Get vertices
		Vertices vertices(obsArray[j]);
		visualization_msgs::Marker marker;
		// Initialize marker message
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();
		marker.ns = "obstacles";
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.id = j;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.scale.x = 0.1;	// msg/LINE_LIST markers use only the x component of scale, for the line width

		// Line strip is red
		marker.color.r = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0.1);
		geometry_msgs::Point p;// int i = 0;
		p.x = vertices.x[3];
		p.y = vertices.y[3];
		p.z = 0;
		marker.points.push_back(p);
		for(int i = 0; i<=3; i++){
			p.x = vertices.x[i];
			p.y = vertices.y[i];
			p.z = 0;
			marker.points.push_back(p);
		}
		msg.markers.push_back(marker);
	}
	// GENERATE MOVEMENT VECTOR
		for(int j = 0; j!=obsArray.size(); j++){
		// Get vertices
		Vertices vertices(obsArray[j]);
		visualization_msgs::Marker marker;
		// Initialize marker message
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();
		marker.ns = "obstacles";
		marker.action = visualization_msgs::Marker::ADD;
		// marker.pose.orientation.w = 1.0;
		marker.id = j+obsArray.size();
		marker.type = visualization_msgs::Marker::ARROW;
		marker.scale.x = 0.1;
		marker.scale.y = 0.2;

		// Line strip is red
		marker.color.r = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0.1);

		// Define pionts
		marker.points.resize(2);
		marker.points[0].x = obsArray[j].obb.center.x;
		marker.points[0].y = obsArray[j].obb.center.y;
		marker.points[0].z = 0;

		// Draw prediction markers
		marker.points[1].x = obsArray[j].obb.center.x + obsArray[j].vel.linear.x;
		marker.points[1].y = obsArray[j].obb.center.y + obsArray[j].vel.linear.y;
		marker.points[1].z = 0;
		msg.markers.push_back(marker);
	}
	pubRviz->publish(msg);
}

vision_msgs::Detection2DArray generateMPCmessage(const vector<car_msgs::Obstacle2D>& obstacles){
	vision_msgs::Detection2DArray msg;
	for(int i = 0; i!=obstacles.size(); i++){
		vision_msgs::Detection2D det;
		det.bbox = obstacles[i].obb;
		det.header.frame_id = "base_link";
		msg.detections.push_back(det);
	}
	msg.header.frame_id = "base_link";
	msg.header.stamp = ros::Time::now();
	return msg;
}
// vision_msgs::Detection2DArray generateMPCmessage(const vector<car_msgs::Obstacle2D>& obstacles){
// 	vision_msgs::Detection2DArray msg;
// 	// for(auto it = obstacles.begin(); it!=obstacles.end(); ++it){
// 	// 	msg.detections.push_back(it->obb);
// 	// }

// 	msg.header.frame_id = "base_link";
// 	return msg;
// }