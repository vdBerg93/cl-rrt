//#### Point cloud conversion function ########################
// 1. The message is converted to pcl format
// 2. The point cloud is filterd for the ground
// 3. The outliers are taken
// 4. Clusters are taken from the outliers.
// 5. The size and centerpoint from the clusters are taken
// 6. The size and coordinates are sent 
#include <tracker.h>
#include "obb.cpp"

vision_msgs::Detection2DArray generateMPCmessage(const vector<car_msgs::Obstacle2D>& obstacles);

struct Observer{
	// Obstacle data & Trackers
	vector<double> laneCxy;
	vector<car_msgs::Obstacle2D> Obs;
	vector<Tracker> trackers;
	void updateTrackers();
	void updateTrackersKF();
	// Callback functions
	void callbackLane(const car_msgs::LaneDet& msg);
	void callbackPointcloud (const sensor_msgs::PointCloud2ConstPtr& input);
    bool callbackService(car_msgs::getobstacles::Request &req, car_msgs::getobstacles::Response &resp);
	// Rviz publishing
	void sendMarkerMsg(const vector<car_msgs::Obstacle2D>& det);
	// Publishers
	ros::Publisher* pubPtr;
	ros::Publisher* pubRviz;
	ros::Publisher* pubMPC;
	tf::TransformListener* tfListener;
};

bool Observer::callbackService(car_msgs::getobstacles::Request &req, car_msgs::getobstacles::Response &resp){
	for(auto it = Obs.begin(); it!=Obs.end(); ++it){
		resp.obstacles.push_back(*it);
	}
	ROS_INFO_STREAM("Request received. Returned detections: "<<resp.obstacles.size());
	return true;
}

void Observer::callbackLane(const car_msgs::LaneDet& msg){
	laneCxy = msg.Cxy;
	return;
}

void Observer::callbackPointcloud (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cout<<"--- Processing new pointcloud ---"<<endl;
  	//#######################################################################
  	//#### Read data and perform segmentation and ground plane removal 
  	//#######################################################################	
	// 1. CLEAR DETECTIONSr
	Obs.clear();

	// Convert ROS message to pcl format 
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg (*input, *cloud);

	pcl::PointCloud<pcl::PointXYZ> pclIn = *cloud;

	bool T = pcl_ros::transformPointCloud("base_link",pclIn, *cloud, *tfListener);
	
	// bool succeed = pcl_ros::transformPointCloud("fr_axle", ros::Time::now(), pclIn, "center_laser_link", *cloud, *tfListener);
	// (target frame, target time, cloud in, fixed frame, cloud out, tf listener)
	// const ros::Time & 	target_time,
	// const pcl::PointCloud< PointT > & 	cloud_in,
	// const std::string & 	fixed_frame,
	// pcl::PointCloud< PointT > & 	cloud_out,
	// const tf::TransformListener & 	tf_listener )	


	//*****************
	// GROUND PLANE FILTERING (DISABLED BECAUSE THIS IS NOT INCLUDED ATM)
	//*****************
  	// Create the segmentation object for the planar model and set all the parameters
 	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);			
  	seg.setDistanceThreshold (0.2);

  	int i=0, nr_points = (int) cloud->points.size ();
  	while (cloud->points.size () > 0.3 * nr_points)
  	{
    	// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud (cloud);
    	seg.segment (*inliers, *coefficients);
    	if (inliers->indices.size () == 0)
    	{
      		cout << "Could not estimate a planar model for the given dataset." << endl;
    	}

    	// Extract the planar inliers from the input cloud
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cloud);
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	// Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

    	// Remove the planar inliers, extract the rest, the ground is filterd out of the point cloud
    	extract.setNegative (true);
    	extract.filter (*cloud_f);
    	*cloud = *cloud_f;
	}

  	//#######################################################################
  	//#### Filter the body
  	//#######################################################################
	float depthThreshold = 0.5;
  	float threshold2 = depthThreshold*depthThreshold;

	for (int p=0; p<cloud->points.size(); ++p)
	{
		// find the squared distance from the origin.
		float pointDepth2 = (cloud->points[p].x * cloud->points[p].x) +
							(cloud->points[p].y * cloud->points[p].y) + 
							(cloud->points[p].z * cloud->points[p].z);
		// remove point if it's within the threshold range
		// if (pointDepth2 < threshold2)
		// {
		// 	cloud->points[p] = cloud->points[cloud->points.size()-1];
		// 	cloud->points.resize(cloud->points.size()-1);
		// 	--p;
		// }
		if ( (std::abs(cloud->points[p].x)<1) && (std::abs(cloud->points[p].y)<1) ){
			cloud->points[p] = cloud->points[cloud->points.size()-1];
			cloud->points.resize(cloud->points.size()-1);
			--p;
		}
	}
  	//#######################################################################
  	//#### Do Euclidean cluster extraction
  	//#######################################################################

  	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud);

  	vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (0.5); // 50cm
  	ec.setMinClusterSize (3);
  	ec.setMaxClusterSize (25000);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);

  	// msgOut.header = input->header;
  	Eigen::Vector4f centroid;

	// Dividing the clusters
  	int j = 0;
  	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    	for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      	cloud_cluster->points.push_back (cloud->points[*pit]); //*
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

		// Extract centroid and min/max
    	pcl::compute3DCentroid (*cloud_cluster, centroid);
    	pcl::PointXYZ min_p, max_p;
    	pcl::getMinMax3D(*cloud_cluster, min_p, max_p);

		// Setup message data
		car_msgs::Obstacle2D obs;

		// Prepare info for 2D message
		obs.obb.center.x = centroid[0];
		obs.obb.center.y = centroid[1];
		// findBestOBB(*cloud_cluster, det);
		getOBB(*cloud_cluster,obs,laneCxy);

		// Add detection to log for service call
		Obs.push_back(obs);
		
		cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
		stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		j++;
  	}
	// For MPC
	vision_msgs::Detection2DArray msg = generateMPCmessage(Obs);
	pubMPC->publish(msg);

	// pubPtr->publish(Obs);
	// updateTrackers();
	updateTrackersKF();
	sendMarkerMsg(Obs);
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
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0.1);

		// Define pionts
		marker.points.resize(2);
		marker.points[0].x = obsArray[j].obb.center.x;
		marker.points[0].y = obsArray[j].obb.center.y;
		marker.points[0].z = 0;
		ROS_WARN_ONCE("IN PCL_CONV PUBLISH: velocity of ego vehicle is fixed");
		// marker.points[1].x = obsArray[j].obb.center.x + 4*(obsArray[j].vel.linear.x+cos(obsArray[j].obb.center.theta)*33); // Add EGO velocity 33m/s
		// marker.points[1].y = obsArray[j].obb.center.y + 4*(obsArray[j].vel.linear.y+sin(obsArray[j].obb.center.theta)*33);
		marker.points[1].x = obsArray[j].obb.center.x + 1*obsArray[j].vel.linear.x; // Add EGO velocity 33m/s
		marker.points[1].y = obsArray[j].obb.center.y + 1*obsArray[j].vel.linear.y;
		// marker.points[1].x = obsArray[j].obb.center.x;
		// marker.points[1].y = obsArray[j].obb.center.y;
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