
void publishPath(ros::Publisher* ptrPub, int ID, MyReference& ref, Simulation sim){
	// visualization_msgs::Marker msg = createReferenceMsg(ID,ref);
	// ptrPub->publish(msg);	// Publish reference path to Rviz (Yellow)
	visualization_msgs::Marker msg = createEmptyMsg();
	ptrPub->publish(msg);
	msg = createStateMsg(ID,sim.stateArray);
	ptrPub->publish(msg);	// Publish trajectory to Rviz (Green)
}

visualization_msgs::Marker createReferenceMsg(int iD, const MyReference& ref){
    // Initialize marker message
    static visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "reference";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.scale.x = 0.01;	// msg/LINE_LIST markers use only the x component of scale, for the line width

    // Line strip is yellow
    msg.color.r = 1.0;
    msg.color.b = 0.0;
    msg.color.g = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration();
    
    geometry_msgs::Point p;
    for(int i = 0; i<ref.x.size(); i++){
        p.x = ref.x[i];
        p.y = ref.y[i];
        p.z = 0;
        msg.points.push_back(p);
    }
    
    return msg;    
}



visualization_msgs::MarkerArray createEmptyMsg(){
    // Initialize marker message
    visualization_msgs::MarkerArray msg;
    msg.header.frame_id = "center_laser_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "trajectory";
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    return msg;    
}
