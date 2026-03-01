#ifndef DATATYPES_H
#define DATATYPES_H

#include <vector>

/**
 * @brief Full vehicle state vector.
 *
 * Fields x..time (indices 0-6) are dynamic states integrated by VehicleODE
 * via forward-Euler. Fields waypoint_id..steer_cmd are logging-only slots
 * filled by the caller after each integration step; they are NOT part of
 * the ODE and must never be written by IntegrateEuler.
 */
struct VehicleState {
    // Dynamic states (integrated by VehicleODE)
    double x      = 0;  ///< position x [m]
    double y      = 0;  ///< position y [m]
    double theta  = 0;  ///< heading [rad]
    double delta  = 0;  ///< steering angle [rad]
    double v      = 0;  ///< velocity [m/s]
    double a      = 0;  ///< acceleration [m/s^2]
    double time   = 0;  ///< simulation time [s]
    // Logging-only fields (NOT integrated)
    double waypoint_id = 0;  ///< closest waypoint index (cast to double)
    double ref_vel     = 0;  ///< reference velocity at waypoint [m/s]
    double steer_cmd   = 0;  ///< commanded steering angle [rad]

    VehicleState() = default;

    /// Construct from ROS state callback (6 elements: x,y,theta,delta,v,a)
    VehicleState(double _x, double _y, double _theta, double _delta, double _v, double _a)
        : x(_x), y(_y), theta(_theta), delta(_delta), v(_v), a(_a) {}

    /// Full 10-field constructor
    VehicleState(double _x, double _y, double _theta, double _delta, double _v, double _a,
                 double _time, double _wpid, double _refv, double _scmd)
        : x(_x), y(_y), theta(_theta), delta(_delta), v(_v), a(_a),
          time(_time), waypoint_id(_wpid), ref_vel(_refv), steer_cmd(_scmd) {}
};

/**
 * @brief State derivative returned by VehicleODE (7 elements).
 *
 * Separates the derivative from the 10-field VehicleState so that
 * IntegrateEuler cannot accidentally write to logging-only fields.
 */
struct StateDeriv {
    double dx     = 0;
    double dy     = 0;
    double dtheta = 0;
    double ddelta = 0;
    double dv     = 0;
    double da     = 0;
    double dt     = 0;
};

/**
 * @brief Goal pose with target velocity.
 *
 * Replaces the previous `vector<double> goalPose` where
 * [0]=x, [1]=y, [2]=theta, [3]=v.
 */
struct GoalPose {
    double x     = 0;
    double y     = 0;
    double theta = 0;
    double v     = 0;

    GoalPose() = default;
    GoalPose(double _x, double _y, double _theta, double _v)
        : x(_x), y(_y), theta(_theta), v(_v) {}
};

typedef std::vector<VehicleState> StateArray;

struct point2D{
    double x,y;
    point2D(double _x, double _y){
        x = _x; y = _y;
    }
};

struct R2S{
    float x,y,theta;
    R2S(float _x, float _y, float _theta):x(_x), y(_y), theta(_theta){};
};

struct poly{
    float c2,c1,c0;
};

#endif
