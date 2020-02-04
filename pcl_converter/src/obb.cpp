
void getOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, car_msgs::Obstacle2D &obs, const vector<double>& Cxy){
	// Generate an OBB of which the heading is aligned with the road
	// Get OBB center position
	double x = obs.obb.center.x;	double y = obs.obb.center.y;
	// Find closest point on arc
	double heading;
	if(Cxy.size()==3){
		vector<double> Plane = findClosestPointOnArc(x,y,Cxy);
		double Xarc = Plane[0];
		// Get local heading of road
		double dydx = 2*Cxy[0]*Xarc + Cxy[1];
		heading = atan2(dydx,1);
	}else{
		heading = 0;
	}

	heading = wrapTo2Pi(heading);
	
	double dxMax{0}, dyMax{0}, dxMin{0}, dyMin{0};
	// Project all cloud points onto the rotated axes
	for(int i = 0; i!=cloud_cluster.points.size(); i++)
	{
		ROS_WARN_ONCE("In getOBB: change box calculations to be more accurate center");
		pcl::PointXYZ pclPoint = cloud_cluster.points[i];
		vector2D A = {pclPoint.x-x,pclPoint.y-y};		// Vector A from c.m. to point
		vector2D vecXaxis = {cos(heading), sin(heading)}; 		// normal vector rotated x-axis
		vector2D vecYaxis = {-sin(heading), cos(heading)};		// normal vector rotated y-axis
		double distX = myDot(A,vecXaxis);					// project A on x-axis
		double distY = myDot(A,vecYaxis); 					// project A on y-axis
		if(distX>dxMax){	dxMax = distX;}		// If distance is larger, update size
		if(distY>dyMax){	dyMax = distY;}		// If distance is larger, update size								
		if(distX<dxMin){	dxMin = distX;}		// If distance is smaller, update size	
		if(distY<dyMin){	dyMin = distY;}		// If distance is smaller, update size	
	}
	// Update box center
	double shiftX = (dxMax+dxMin)/2;
	double shiftY = (dyMax+dyMin)/2;
	obs.obb.center.x +=shiftX*cos(heading);
	obs.obb.center.x +=shiftY*sin(heading);

	// Insert box into detection message
	obs.obb.center.theta = heading;
	obs.obb.size_x = dxMax-dxMin;
	obs.obb.size_y = dyMax-dyMin;

	// Add safety margins
	obs.obb.size_x += 1;
	obs.obb.size_y += 1;
	return;
};

void findBestOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, vision_msgs::Detection2D &det){
	// This function tries to approximate the optimal Oriented Bounding Box for the given cloud cluster.
	// It constructs a bounding box and rotates it between 0-90 degrees, while logging its area.
	// The bounding box with smallest volume is selected as the best OBB and it is put in the detection message.
	vector<OBB> obbVec;
	// Defien bounding box center position
	double c_x = det.bbox.center.x;
	double c_y = det.bbox.center.y;
	// Try multiple Orientations for the Bounding Box
	// for(double theta = 0; theta<(pi/2); theta +=(pi/16)){
		double theta = 0;
		double dxMax{0}, dyMax{0};

		// Project all cloud points onto the rotated axes to get half of size
		for(int i = 0; i!=cloud_cluster.points.size(); i++)
		{
			pcl::PointXYZ pclPoint = cloud_cluster.points[i];
			vector2D A = {pclPoint.x-c_x,pclPoint.y-c_y};		// Vector A from c.m. to point
			vector2D vecXaxis = {cos(theta),-sin(theta)}; 		// normal vector rotated x-axis
			vector2D vecYaxis = {sin(theta), cos(theta)};		// normal vector rotated y-axis
			double distX = myDot(A,vecXaxis);					// project A on x-axis
			double distY = myDot(A,vecYaxis); 					// project A on y-axis
			if(distX>dxMax){
				dxMax = distX;	// If distance is larger, update size
			}
			if(distY>dyMax){
				dyMax = distY;	// If distance is larger, update size								
			}
		}
		double area = dxMax*dyMax;
		//cout<<"theta="<<theta<<" sz_x="<<dxMax<<" sx_y="<<dyMax<<" area="<<area<<endl;
		// Log the data for best box selection
		OBB box = {theta, dxMax, dyMax, area};
		obbVec.push_back(box);
	// }
	// Loop through all boxes and select the smallest one
	int best; double Amin = inf;
	for(int i = 0; i!=obbVec.size(); i++){
		if(obbVec[i].area<Amin){
			Amin = obbVec[i].area;
			best = i;
		}
	}
	// Insert best box into detection message
	det.bbox.center.theta = obbVec[best].theta;
	det.bbox.size_x = obbVec[best].size_x;
	det.bbox.size_y = obbVec[best].size_y;

	return;
};

vector<double> findClosestPointOnArc(const double& Xcar, const double& Ycar, const vector<double>& Cxy){
	    // ***** Find closest point on the road centerline arc *****
    // x = arg min norm([Xcarar,Ycarar]-[xroad,yroad]) (solved symbolically in MATLAB)
	// Results are valid
	float t2 = abs(Cxy[0]);
	float t3 = pow(Cxy[0],3);
	float t4 = pow(Cxy[1],2);
	float t5 = Xcar*Cxy[0]*2.0;
	float t6 = Ycar*Cxy[0]*4.0;
	float t8 = Cxy[0]*Cxy[2]*4.0;
	float t11 = sqrt(3.0);
	float t7 = pow(t2,3);
	float t9 = 1.0/t3;
	float t10 = -t8;
	float t12 = Cxy[1]+t5;
	float t13 = pow(t12,2);
	float t14 = Cxy[1]*t7*9.0;
	float t15 = Xcar*Cxy[0]*t7*18;
	float t17 = t4+t6+t10-2.0;
	float t16 = t13*27;
	float t18 = pow(t17,3);
	float t19 = -t18;
	float t20 = t16+t19;
	float t21 = sqrt(t20);
	float t22 = t3*t11*t21;
	float t23 = t14+t15+t22;
	float t24 = t9*t23;
	double Xarc = ( pow(t24,1.0/3.0) *0.2403749283845681)/t2-Cxy[1]/(Cxy[0]*2.0)+1.0/ pow(Cxy[0],2)*t2*t17*1.0/ pow(t24,1.0/3.0)*0.3466806371753173;
    // y = Cxy[1]*x^2 + Cxy[0]*x + c0
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
	vector<double> result {Xarc,Yarc};
	return result;
}
