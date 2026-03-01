#ifndef RRT_H
#define RRT_H

// Configuration
#include "rrt/vehicle.h"
#include "datatypes.h"
#include "vision_msgs/Detection2DArray.h"

// Tree build timer with limit set in ms
struct Timer{
	clock_t tstart, tnow;
	double diff, timeLimit;
	Timer(double _timeLimit): tstart(clock()), timeLimit(_timeLimit){}
	bool Get(){
		tnow = clock();
		diff = diffclock(tnow,tstart);
		return 0 + (diff<timeLimit);
	}
	double diffclock(clock_t clock1, clock_t clock2){
		double diffticks = clock1 - clock2;
		double diffms = (diffticks)/(CLOCKS_PER_SEC/1000);
		return diffms;
	}
};

struct MyReference{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> v;
    int dir;
    double aend;
};

struct Node{
    VehicleState state;         // Node state
    int parentID;               // Parent ID
    MyReference ref;            // Reference to reach node
    float costE;                // Costfunction for exploration
    float costS;                // Costfunction for selecting the best path in structured driving
    bool goalReached;           // Boolean stating whether goal has been reached
    StateArray tra;
    Node(){};
    Node( VehicleState _state, int _parentID, MyReference _ref, StateArray _tra, double _costE, double _costS, bool _goal) : state(_state), parentID(_parentID), ref(std::move(_ref)), tra(std::move(_tra)),costE(_costE), costS(_costS), goalReached(_goal){};
};

#include <car_msgs/Obstacle2D.h>

class MyRRT{
    public:
        int sortLimit;
        bool reverseAllowed;
        bool goalReached;
        bool bend;
        GoalPose goalPose;
        int direction;
        std::vector<double> laneShifts;   // Lane shifts. 1st element is goal lane. 2nd element is other lane
        std::vector<double> Cxy;

        // For other functions only (sim. etc)
        std::vector<car_msgs::Obstacle2D> det;
        VehicleState carState;
        double Wcost[5];

        // Tree iniitalization
        MyRRT(const GoalPose& _goalPose, const std::vector<double>& _laneShifts, const std::vector<double>& _Cxy, const bool& _bend);
        void addInitialNode(const VehicleState& state);
        // Tree operations
        void addNode(Node node);
        Node getNode(int ID);
        void addNodes(const std::vector<Node>& nodes);
        void getBestPath();
        std::vector<Node> tree;
    private:



};
void initializeTree(MyRRT& RRT, const Vehicle& veh, std::vector<Node>& nodes, VehicleState& carState);
double getDistToLane(const double& x, const double& y, double S, const std::vector<double>& Cxy);

geometry_msgs::Point sampleAroundVehicle(const GoalPose& goalPose);
geometry_msgs::Point sampleOnLane(const std::vector<double>& Cxy, std::vector<double> laneShifts, double Lmax);
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub, const std::vector<car_msgs::Obstacle2D>& det, const std::vector<double>& Cxy);
std::vector<int> sortNodesExplore(const MyRRT& rrt, const geometry_msgs::Point& sample, const Vehicle& veh);
std::vector<int> sortNodesOptimize(const MyRRT& rrt, const geometry_msgs::Point& sample, const Vehicle& veh);
bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample);
bool feasibleGoalBias(const MyRRT& rrt, const Vehicle& veh);
float dubinsDistance(geometry_msgs::Point S, const Node& N, int dir, const Vehicle& veh);
visualization_msgs::Marker createStateMsg(int ID, const StateArray& T, bool goalReached);
visualization_msgs::Marker createEmptyMsg();
std::vector<Node> extractBestPath(const std::vector<Node>& tree, ros::Publisher* ptrPub);


// Reference generation functions
void generateVelocityProfile(MyReference& ref, const double& a0, const int& IDwp, const double& v0, const double& vmax, const GoalPose& goal, bool GB);
MyReference getReference(geometry_msgs::Point sample, const Node& node, int dir);
MyReference getGoalReference(const Vehicle& veh, const Node& node, const GoalPose& goalPose);
std::vector<double> getCoefficients(const double& Sf, const double& v0, const double& vf, const double& a0, const double& af);
double getVelocity(const double& v0, const std::vector<double>& coef, const double& t);
std::vector<double> getVelocityVector(const double& v0, const std::vector<double>& coef, const std::vector<double>& Tpath);
std::vector<double> getTimeVector(const std::vector<double>& coef, const double& t0, const double& v0, const double& a0, const double& af, const double& Sf, const int& N);
void showVelocityProfile(const MyReference& ref);


/* ----------------------------------------
        SIMPLE DATA OPERATIONS
-----------------------------------------*/
// Add a node to the tree
inline void MyRRT::addNode(Node node){
	tree.push_back(std::move(node));
}

// Add multiple nodes to the tree
inline void MyRRT::addNodes(const std::vector<Node>& nodes){
	tree.insert(tree.end(), nodes.begin(), nodes.end());
}

#endif
