#ifndef TRANSFORM_H
#define TRANSFORM_H

// Goal & state transformations
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs);
void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh);

// State transformation
void transformStateWorldToCar(state_type& state, const state_type& carPose);							
void transformStateRoadToCar(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToRoad(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToWorld(state_type& state, const state_type& carPose);
vector<double> transformStateToLocal(const vector<double>& worldState);
// Tranformat point
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose);
// Path transformations
void transformPathWorldToCar(vector<Path>& path, const vector<double>& carPose);
void transformPathCarToRoad(vector<Path>& path,const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathRoadToCar(vector<Path>& path, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathCarToWorld(vector<Path>& path, const vector<double>& worldState);

void rotateVelocityVector(double& Vx, double& Vy, const vector<double>& carPose);
void transformVelocityToRoad(const double& x, const double& y, double& Vx, double& Vy, const vector<double>& Cxy);

#endif