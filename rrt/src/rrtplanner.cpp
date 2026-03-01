#include "rrt/headers.h"
#include "rrt/globals.h"
#include <random>
using namespace std;

static mt19937& rng() {
	static mt19937 gen(random_device{}());
	return gen;
}

double getNodeCost(const MyRRT& RRT, const Vehicle& veh, const double& parentCost, const Node& node, const vector<car_msgs::Obstacle2D>& det);

MyRRT::MyRRT(const GoalPose& _goalPose, const vector<double>& _laneShifts, const vector<double>& _Cxy, const bool& _bend):
	bend(_bend), goalReached(0), sortLimit(10), direction(1), goalPose(_goalPose), laneShifts(_laneShifts), Cxy(_Cxy){
		ros::param::get("motionplanner/weight_distance",Wcost[0]);
		ros::param::get("motionplanner/weight_curvature",Wcost[1]);
		ros::param::get("motionplanner/weight_obstacle_gain",Wcost[2]);
		ros::param::get("motionplanner/weight_obstacle_slope",Wcost[3]);
		ros::param::get("motionplanner/weight_lanedeviation",Wcost[4]);
	}

void MyRRT::addInitialNode(const VehicleState& state){
	// Set the first node in the tree at the current preview point of the lateral controller
	MyReference ref;
	double xend {1}, yend{0}, res{0.1};
	int N = floor(sqrt( pow(xend,2) + pow(yend,2))/res);
    ref.x = LinearSpacedVector(0,xend,N);								// Reference (x)
    ref.y = LinearSpacedVector(0,yend,N);								// Reference (y)
    for(int i = 0; i!=N; i++){													// Reference (v)
        ref.v.push_back(state.v);
    }
    ref.dir = 1;																// fwd driving only
    // Initialize tree
    StateArray T; T.push_back(state);
    Node initialNode(state,-1,ref,T,0,0,0);
	tree.push_back(initialNode);
}

void initializeTree(MyRRT& RRT, const Vehicle& veh, vector<Node>& nodes, VehicleState& carState){
	// VehicleState already has all 10 fields zero-initialized; no padding needed.

	// If committed path is empty, initialize tree with single point at (x,y) = (Dla,0)
	if (nodes.size()==0){
		RRT.addInitialNode(carState);
		ROS_INFO_STREAM("Initialized empty tree!");
		return;
	}

	// Erase nodes behind the vehicle
	for(auto it = nodes.begin(); it!=nodes.end(); ){
		it->goalReached = 0;
		if ((it->tra.back().x)<0){
			it = nodes.erase(it);
			ROS_INFO_STREAM("Erased a node from initialization!");
		} else {
			++it;
		}
	}

	// Check if goal is reached
	for(auto it = nodes.begin(); it!=nodes.end(); ++it){
		for(int i = 0; i!=it->tra.size(); i++){
			double Dgoal = sqrt(pow(it->tra[i].x-RRT.goalPose.x,2) + pow(it->tra[i].y,2) );
			double Hgoal = abs(it->tra[i].theta - RRT.goalPose.theta);
			double dVgoal = abs(it->tra[i].v-RRT.goalPose.v);
			if ( (Dgoal<=1) && (Hgoal<=0.05) && (dVgoal<=0.1)){
				it->goalReached = 1;
			}
		}
	}

	// Check collisions, if collision initialize with empty tree
	for(auto it = nodes.begin(); it!=nodes.end(); ++it){
		for(int i = 0; i!=it->tra.size(); i++){
			double Dobs = checkObsDistance(RRT.carState);
			ROS_WARN_STREAM_ONCE(" in initializeTree: update Dobs function!");
			if(Dobs==0){
				RRT.addInitialNode(carState);
				ROS_INFO_STREAM("Initialized empty tree (collision in prior nodes)!");
				return;
			}
		}
	}

	// Update cost estimates
	nodes.front().costS = getNodeCost(RRT, veh, 0, nodes.front(), RRT.det);
	for(int i = 1; i != nodes.size(); i++){
		nodes[i].costS = getNodeCost(RRT,veh,nodes[i-1].costS, nodes[i], RRT.det);
	}

	// No collisions, add all nodes to tree
	for(int i = 0; i!=nodes.size(); i++){
		nodes[i].parentID = i-1;
		RRT.tree.push_back(nodes[i]);
	}
	ROS_INFO_STREAM("Initialized tree with previous nodes");
}

// Distance to the lane centerline Cxy
double getDistToLane(const double& x, const double& y, double S, const vector<double>& Cxy){
	double Lx = (x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2])/(pow(Cxy[1],2) + 1);
	double Ly = S + Cxy[2] + (Cxy[1]*(x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2]))/(pow(Cxy[1],2) + 1);
	return sqrt( pow(Lx-x,2) + pow(Ly-y,2) );
}

/**
 * @brief Compute cost of a node (used in tree initialization only).
 *
 * cost = parent_cost + sum_over_trajectory_of:
 *   W[0]*v*dt  +  W[1]*|kappa|  +  W[2]*exp(-W[3]*Dobs)  [+ W[4]*Dgoallane]
 */
double getNodeCost(const MyRRT& RRT, const Vehicle& veh, const double& parentCost, const Node& node, const vector<car_msgs::Obstacle2D>& det){
	double cost = parentCost;
	for(auto it = node.tra.begin(); it!=node.tra.end(); it++){
		double Dobs = checkObsDistance(RRT.carState);
		ROS_WARN_STREAM_ONCE("in getNodeCost: update obstacle distance fcn!");
		double kappa = tan(it->delta)/veh.L;								// Vehicle path curvature
		cost += RRT.Wcost[0]*it->v*sim_dt + RRT.Wcost[1]*abs(kappa) + RRT.Wcost[2]*exp(-RRT.Wcost[3]*Dobs);
		if (RRT.bend){
			double Dgoallane = getDistToLane(it->x,it->y,RRT.laneShifts[0],RRT.Cxy);
			cost += RRT.Wcost[4]*Dgoallane;
		}
	}
	return cost;
}


// Perform a tree expansion
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub, const vector<car_msgs::Obstacle2D>& det, const vector<double>& Cxy){
	if(debug_mode){
		cout<<"New iteration."<<endl;
	}

	// #### RANDOM SAMPLING: ####
	geometry_msgs::Point sample;
	if (RRT.bend){ 	// Sample on lane for lane-change scenarios
		double Lmax = RRT.goalPose.x;
		sample = sampleOnLane(Cxy, RRT.laneShifts, Lmax);
	}else{ 			// Sample around vehicle for straight driving
		sample = sampleAroundVehicle(RRT.goalPose);
	}
	int dir = 1; // Driving direction variable
	// #### SORTING THE NODES ####
	vector<int> sortedNodes; bool node_added = false;
	double r = uniform_real_distribution<double>(0.0, 1.0)(rng()); 	// Generate random value [0-1]
	if(r<=((RRT.goalReached*0.3)+(!RRT.goalReached*0.7))){								// Select a heuristic (shifts after goal is reached)
		sortedNodes = sortNodesExplore(RRT,sample,veh); 		// Sort nodes in increasing Dubins distance to sample
	}else{
		sortedNodes = sortNodesOptimize(RRT,sample,veh); 		// Sort nodes on total cost (time) to reach sample
	}
	// #### NODE EXPANSION ####
	// Loop through the sorted nodes untill expansion succeeds
	for(vector<int>::iterator it = sortedNodes.begin(); it != sortedNodes.end(); ++it){
		MyReference ref = getReference(sample, RRT.tree[*it], dir);	// Generate a reference path
		Simulation sim(RRT,RRT.tree[*it].state,ref,veh,false,true,RRT.tree[*it].ref.v.back());					// Do closed-loop prediction
		// If trajectory is admissible and collisionfree, add it to the tree
		if(sim.endReached||sim.goalReached){
				Node node(sim.stateArray.back(), *it, ref,sim.stateArray, sim.costE + RRT.tree[*it].costE, sim.costS + RRT.tree[*it].costS, sim.goalReached);
				RRT.addNode(node); 	node_added = true;
				break;
		}
	};
	// #### GOAL BIASED EXPANSION ####
	// Loop through the added nodes and try a goal expansion
	if ( node_added && feasibleGoalBias(RRT, veh) ) {
		ROS_DEBUG_STREAM("Doing goal expansion...");
		MyReference ref_goal = getGoalReference(veh, RRT.tree.back(), RRT.goalPose);
		Simulation sim_goal(RRT,RRT.tree.back().state, ref_goal,veh,true,true,RRT.tree.back().ref.v.back());

		// If trajectory is admissible and collision free, add it to the tree
		if(sim_goal.endReached||sim_goal.goalReached){
			Node node_goal(sim_goal.stateArray.back(), RRT.tree.size()-1, ref_goal, sim_goal.stateArray,sim_goal.costE + RRT.tree.back().costE, sim_goal.costS + RRT.tree.back().costS, sim_goal.goalReached);
			RRT.addNode(node_goal);
		}
	}
};

// Uniform sampling around the vehicle. Sampling is done in a rectangle aligned with the line that connects (x,y)_car and (x,y)_goal
geometry_msgs::Point sampleAroundVehicle(const GoalPose& goalPose){
	double dGoal = sqrt( pow(goalPose.x,2) + pow(goalPose.y,2) );		// Distance to the goal
	double goalHeading = atan2( goalPose.y, goalPose.x );				// Heading (x,y)_car to (x,y)_world
	double latMin {-7}, latMax{7};										// Width of the box

	geometry_msgs::Point sample;
	double rLong = uniform_real_distribution<double>(0.0, dGoal+10)(rng());				// Random long. coordinate
	double rLat = uniform_real_distribution<double>(latMin, latMax)(rng()); // Random lat. coordinate

	sample.x = rLong*cos(goalHeading) + rLat*cos(goalHeading+pi/2);		// Rotate the coordinates to align with goal heading
	sample.y = rLong*sin(goalHeading) + rLat*sin(goalHeading+pi/2);

	if(debug_mode){cout<<"Generated sample: x="<<sample.x<<" y="<<sample.y<<endl;}
	return sample;
}

// Sample on the given lane center lines
geometry_msgs::Point sampleOnLane(const vector<double>& Cxy, vector<double> laneShifts, double Lmax){
	// sample w.r.t. the straightened road y(x) = c1*x + c0;
	// 1. Sample length coordinate (S) on reference road centerline
	double S = uniform_real_distribution<double>(ctrl_dla, Lmax)(rng());
	// 2. Sample coordinate (rho) from lane shifts
	// Select a random lane
	int laneIndex = uniform_int_distribution<int>(0, (int)laneShifts.size()-1)(rng());
	double rho = laneShifts[laneIndex];
	assert(laneIndex >= 0 && laneIndex <= (int)(laneShifts.size()-1));
    // Rotate (S,rho) with slope, translate with C0
	double theta = atan2(Cxy[1], 1);
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];
	// Prepare sample
	geometry_msgs::Point sample;
	sample.x = Xstraight;
	sample.y = Ystraight;
	if(debug_mode){cout<<"Generated sample: x="<<sample.x<<" y="<<sample.y<<endl;}
	return sample;
}

/**
 * @brief Pre-screen tree nodes by squared Euclidean distance to sample.
 *
 * Returns indices of the closest nCandidates nodes, avoiding expensive
 * Dubins evaluations on distant nodes. Uses partial_sort for O(n) average.
 */
static vector<int> euclideanPrescreen(const MyRRT& rrt, const geometry_msgs::Point& sample, int nCandidates){
	vector<pair<int,float>> eucDist;
	eucDist.reserve(rrt.tree.size());
	for(int i = 0; i != (int)rrt.tree.size(); i++){
		float dx = sample.x - rrt.tree[i].state.x;
		float dy = sample.y - rrt.tree[i].state.y;
		eucDist.push_back(make_pair(i, dx*dx + dy*dy));
	}
	nCandidates = std::min(nCandidates, (int)rrt.tree.size());
	std::partial_sort(eucDist.begin(), eucDist.begin()+nCandidates, eucDist.end(),
		[](const pair<int,float>& a, const pair<int,float>& b){ return a.second < b.second; });
	vector<int> result;
	result.reserve(nCandidates);
	for(int i = 0; i < nCandidates; i++){
		result.push_back(eucDist[i].first);
	}
	return result;
}

/**
 * @brief Sort nodes by exploration heuristic (Dubins distance to sample).
 *
 * Pre-screens with Euclidean distance, then computes Dubins distance only
 * for the closest candidates. Returns up to sortLimit feasible node indices.
 */
vector<int> sortNodesExplore(const MyRRT& rrt, const geometry_msgs::Point& sample, const Vehicle& veh){
	// Pre-screen with cheap Euclidean distance before computing Dubins (O(n·log n) → O(n))
	int nCandidates = std::max(4*(int)rrt.sortLimit, 20);
	vector<int> candidates = euclideanPrescreen(rrt, sample, nCandidates);

	vector<pair<int,float>> dVector;
	dVector.reserve(candidates.size());
	for(int nodeid : candidates){
		dVector.push_back(make_pair(nodeid, dubinsDistance(sample, rrt.tree[nodeid], rrt.direction, veh)));
	}
	// Sort the pairs from shortest to longest distance
	sort(dVector.begin(),dVector.end(),[](const pair<int,float>& a, const pair<int,float>& b){return a.second< b.second;});
	// Extract feasible connections until maximum size is reached
	vector<int> sortedList;
	for(vector<pair<int,float>>::iterator it = dVector.begin(); it != dVector.end(); ++it){
		if(feasibleNode(rrt,rrt.tree[it->first],sample))
		{
			sortedList.push_back(it->first);
		}
		if (sortedList.size()==rrt.sortLimit){break;}
	}

	if(debug_mode){cout<<"sorted nodes to exploring heuristic."<<endl;}
	return sortedList;
}

/**
 * @brief Sort nodes by optimization heuristic (parent cost + Dubins distance).
 *
 * Combines accumulated travel cost with Dubins distance to the sample.
 * Returns up to sortLimit feasible node indices sorted by total cost.
 */
vector<int> sortNodesOptimize(const MyRRT& rrt, const geometry_msgs::Point& sample, const Vehicle& veh){
	// Pre-screen with cheap Euclidean distance before computing Dubins (O(n·log n) → O(n))
	int nCandidates = std::max(4*(int)rrt.sortLimit, 20);
	vector<int> candidates = euclideanPrescreen(rrt, sample, nCandidates);

	vector<pair<int,float>> dVector;
	dVector.reserve(candidates.size());
	for(int index : candidates){
		// Cost = cost_parent + dubins distance
		dVector.push_back(make_pair(index, rrt.tree[index].costE + dubinsDistance(sample, rrt.tree[index], rrt.direction, veh)));
	}
	sort(dVector.begin(),dVector.end(),[](const pair<int,float>& a, const pair<int,float>& b){return a.second< b.second;});
	vector<int> sortedList;
	for(vector<pair<int,float>>::iterator it = dVector.begin(); it != dVector.end(); ++it){
		if(feasibleNode(rrt,rrt.tree[it->first],sample))
		{
			sortedList.push_back(it->first);
		}
		if (sortedList.size()==rrt.sortLimit){break;}
	}

	if(debug_mode){cout<<"Sorted nodes with optimization heuristic."<<endl;}
	return sortedList;
}

// Check if node connection is feasible
bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample){
	// Calculate reference heading
	double angPar = atan2(node.ref.y.back()-node.ref.y.front(),node.ref.x.back()-node.ref.x.front());
	double angNew = atan2(sample.y-node.ref.y.back(),sample.x-node.ref.x.back());
	// Calculate length of new reference
	double Lref = sqrt( pow(node.ref.x.back()-sample.x,2) + pow(node.ref.y.back()-sample.y,2));
	// Reject when heading difference exceeds limit
	if (abs(angleDiff(angNew,angPar))>(pi/4)){
        return false;
    }
	// Reject when new reference would be too short (at least 3 data points)
    else if(Lref<(2.1*ref_res)){
		return false;
    }
	else{
		return true;
	}
}

/**
 * @brief Check if a goal-biased expansion is feasible based on turning radius.
 *
 * Defines circles of minimum turning radius (veh.rho) left and right of the
 * goal. If the last node lies within either circle, the goal cannot be reached
 * due to the vehicle's minimum turning radius. Also checks that the heading
 * angle to the goal is within limits (pi/8).
 */
bool feasibleGoalBias(const MyRRT& rrt, const Vehicle& veh){
	// Define circles of minimum turning radius left and right of the vehicle
	double R1 = veh.rho; double R2{R1-0.3};
	geometry_msgs::Point center_l, center_r;
	center_l.x = rrt.goalPose.x+R1*cos(rrt.goalPose.theta-M_PI_2);
	center_l.y = rrt.goalPose.y+R1*sin(rrt.goalPose.theta-M_PI_2);
	center_r.x = rrt.goalPose.x+R1*cos(rrt.goalPose.theta+M_PI_2);
	center_r.y = rrt.goalPose.y+R1*sin(rrt.goalPose.theta+M_PI_2);
	// If the node state lies within either one of these circles, the goal bias is not feasible due to the vehicle' minimum turning radius
	const Node& node = rrt.tree.back();
	bool outside_left_circle = sqrt( pow(node.state.x-center_l.x,2) + pow(node.state.y-center_l.y,2) ) > R2;
	bool outside_right_circle = sqrt( pow(node.state.x-center_r.x,2) + pow(node.state.y-center_r.y,2) ) > R2;
	// Determine the angle of the reference to the goal heading
	double angleRef = atan2( rrt.goalPose.y-node.ref.y.back(), rrt.goalPose.x-node.ref.x.back());
	double dHead1 = abs(wrapToPi(rrt.goalPose.theta-angleRef));
	double dHead2 = abs(wrapToPi(rrt.goalPose.theta+pi-angleRef));
	double minAngleDiff = min(dHead1,dHead2);
	double sgn = sign(cos(rrt.goalPose.theta+M_PI_2-angleRef));
	double angle = sgn*minAngleDiff;
	// Check if angle lies is within limits
	bool angle_within_limits = abs(angle)<(M_PI_4/2);

	return outside_left_circle*outside_right_circle*angle_within_limits;
}

// Extract the best path from the tree with backtracking
vector<Node> extractBestPath(const vector<Node>& tree, ros::Publisher* ptrPub){
	vector<Node> bestPath; 					// Initialize returned vector
	vector<pair<int,double>> pair_vector;	// Initialize pair. 1: NodeID, 2: Cost
	// Loop through the tree. When node reached goal, add it to the pair vector
	visualization_msgs::MarkerArray msgArray;
	visualization_msgs::Marker msgClear = createEmptyMsg();
	msgArray.markers.push_back(msgClear);
	for(int nodeid = 0; nodeid !=tree.size(); nodeid++){
		if(tree[nodeid].goalReached){
				pair_vector.push_back(make_pair(nodeid,tree[nodeid].costS));
				if(draw_tree){
					visualization_msgs::Marker msg = createStateMsg(nodeid,tree[nodeid].tra,1);
					msgArray.markers.push_back(msg);
				}
		}else{
			if (draw_tree){
				visualization_msgs::Marker msg = createStateMsg(nodeid,tree[nodeid].tra,0);
				msgArray.markers.push_back(msg);
			}
		}
	}
	if(draw_tree){
		ptrPub->publish(msgArray);
	}
	if(pair_vector.size()==0){
		ROS_WARN("No solution was found!");
	}else{
		cout<<pair_vector.size()<<" paths to the goal found!"<<endl;
		// Sort the pair vector in ascending cost
		sort(pair_vector.begin(),pair_vector.end(),[](const pair<int,double>& a, const pair<int,double>& b){return a.second< b.second;});
		// Add lowest cost solution to best path vector
		bestPath.push_back(tree[pair_vector.front().first]);
		// Backtracking: use push_back + reverse to avoid O(k²) front-insert cost
		int parent = bestPath.back().parentID;
		ROS_WARN_STREAM("Best path cost = "<<tree[pair_vector.front().first].costS);
		while (parent!=-1){
			bestPath.push_back(tree[parent]);
			parent = bestPath.back().parentID;
		}
		std::reverse(bestPath.begin(), bestPath.end());
	}
	if(debug_mode){
		cout<<"Returned best path!"<<endl;
	}
	return bestPath;
}

/**
 * @brief Dubins distance from a sample point to a tree node.
 *
 * Computes a lower-bound path length using the Dubins metric (shortest
 * path through tangent arcs and straight segments) with the vehicle's
 * minimum turning radius veh.rho.
 *
 * @param S    Target sample point
 * @param N    Source tree node (uses N.state.x, .y, .theta)
 * @param dir  Driving direction (+1 forward)
 * @param veh  Vehicle parameters (uses veh.rho)
 */
float dubinsDistance(geometry_msgs::Point S, const Node& N, int dir, const Vehicle& veh){
    float rho = veh.rho;
    // 1. Subtract node location
    float qw_x = S.x - N.state.x;
    float qw_y = S.y - N.state.y;
    // 2. Rotate to 0 rotation
    float ang = -N.state.theta-M_PI*(dir!=1);
    float tmp = cos(ang)*qw_x - sin(ang)*qw_y;
    qw_y = abs(sin(ang)*qw_x + cos(ang)*qw_y);
    qw_x = tmp;

    // Parts of the solution
    float dc = sqrt( qw_x*qw_x + (qw_y-rho)*(qw_y-rho) );
    float thetac = atan2(qw_x,rho-qw_y);
    while(thetac<0){
        thetac = thetac + 2*M_PI;
    }

    float df = sqrt( qw_x*qw_x + (qw_y+rho)*(qw_y+rho) );
    float alpha =  2*M_PI - acos( std::max(-1.0f, std::min(1.0f, (5*rho*rho - df*df)/(4*rho*rho))));

    // Check if qw lies within circles
    bool q_in_Dp = 0;
    if ((qw_x*qw_x+(qw_y+rho)*(qw_y+rho)<=rho*rho)|(qw_x*qw_x+(qw_y-rho)*(qw_y-rho)<=rho*rho)){
        q_in_Dp = 1;
    }

    // Choose solution
    if(!q_in_Dp){
        return sqrt(dc*dc - rho*rho) + rho*(thetac - acos(rho/dc));
    }
    else{
        return rho*(alpha + asin(qw_x/df) - asin(rho*sin(alpha)/df));
    }
}

/******************************************
 **** RVIZ PUBLISHING *********************
 *****************************************/

// Create a message for publishing a trajectory
visualization_msgs::Marker createStateMsg(int ID, const StateArray& T, bool goalReached){
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "tree";
	msg.id = ID;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.025;	// msg/LINE_LIST markers use only the x component of scale, for the line width
	if (goalReached){
		msg.color.g = 1;
		msg.color.r = 1;
	}else{
		msg.color.r = 1.0;
	}
	msg.color.a = 1.0;
	msg.lifetime = ros::Duration();

    geometry_msgs::Point p;
    for(int i = 0; i<T.size(); i++){
        p.x = T[i].x;
        p.y = T[i].y;
        p.z = 0;
        msg.points.push_back(p);
    }
    return msg;
}

// Create message for deleting all Rviz markers
visualization_msgs::Marker createEmptyMsg(){
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "trajectory";
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    return msg;
}
