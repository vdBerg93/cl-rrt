#ifndef TRANSFORM_H
#define TRANSFORM_H

// Goal & state transformations
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const std::vector<double>& Cxy, const std::vector<double>& Cxs);

// State transformation
void transformStateWorldToCar(VehicleState& state, const VehicleState& carPose);
void transformStateRoadToCar(VehicleState& state, const std::vector<double>& Cxy, const std::vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToRoad(VehicleState& state, const std::vector<double>& Cxy, const std::vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToWorld(VehicleState& state, const VehicleState& carPose);
VehicleState transformStateToLocal(const VehicleState& worldState);
// Transform point
void transformPointWorldToCar(double& Xw, double& Yw, const VehicleState& carPose);
void transformPointCarToRoad(double& Xcar, double& Ycar,const std::vector<double>& Cxy, const std::vector<double>& Cxs);
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const std::vector<double>& Cxy, const std::vector<double>& Cxs);
void transformPointCarToWorld(double& Xc, double& Yc, const VehicleState& carPose);
// Path transformations
void transformPathWorldToCar(std::vector<Path>& path, const VehicleState& carPose);
void transformPathCarToRoad(std::vector<Path>& path,const std::vector<double>& Cxy, const std::vector<double>& Cxs, const Vehicle& veh);
void transformPathRoadToCar(std::vector<Path>& path, const std::vector<double>& Cxy, const std::vector<double>& Cxs, const Vehicle& veh);
void transformPathCarToWorld(std::vector<Path>& path, const VehicleState& worldState);
// Node transformations
void transformNodesWorldToCar(std::vector<Node>& nodes, const VehicleState carState);
void transformNodesCarToworld(std::vector<Node>& nodes, const VehicleState carState);
void transformNodesRoadToCar(std::vector<Node>& nodes, const VehicleState carState, const std::vector<double>& Cxy, const std::vector<double> Cxs, const Vehicle& veh);
void transformNodesCarToRoad(std::vector<Node>& nodes, const VehicleState carState, const std::vector<double>& Cxy, const std::vector<double>& Cxs, const Vehicle& veh);

void rotateVelocityVector(double& Vx, double& Vy, const double& angle);
void transformVelocityToRoad(const double& x, const double& y, double& Vx, double& Vy, const std::vector<double>& Cxy);

// Arc-geometry helper (exposed for testability)
std::vector<double> findClosestPointOnArc(const double& Xcar, const double& Ycar, const std::vector<double>& Cxy);

#endif
