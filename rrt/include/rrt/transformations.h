#ifndef TRANSFORM_H
#define TRANSFORM_H

// Goal & state transformations
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs);

// State transformation
void transformStateWorldToCar(VehicleState& state, const VehicleState& carPose);
void transformStateRoadToCar(VehicleState& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToRoad(VehicleState& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToWorld(VehicleState& state, const VehicleState& carPose);
VehicleState transformStateToLocal(const VehicleState& worldState);
// Transform point
void transformPointWorldToCar(double& Xw, double& Yw, const VehicleState& carPose);
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointCarToWorld(double& Xc, double& Yc, const VehicleState& carPose);
// Path transformations
void transformPathWorldToCar(vector<Path>& path, const VehicleState& carPose);
void transformPathCarToRoad(vector<Path>& path,const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathRoadToCar(vector<Path>& path, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathCarToWorld(vector<Path>& path, const VehicleState& worldState);
// Node transformations
void transformNodesWorldToCar(vector<Node>& nodes, const VehicleState carState);
void transformNodesCarToworld(vector<Node>& nodes, const VehicleState carState);
void transformNodesRoadToCar(vector<Node>& nodes, const VehicleState carState, const vector<double>& Cxy, const vector<double> Cxs, const Vehicle& veh);
void transformNodesCarToRoad(vector<Node>& nodes, const VehicleState carState, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);

void rotateVelocityVector(double& Vx, double& Vy, const double& angle);
void transformVelocityToRoad(const double& x, const double& y, double& Vx, double& Vy, const vector<double>& Cxy);

#endif
