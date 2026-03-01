#ifndef CTRL_H
#define CTRL_H

#include "rrt/datatypes.h"

struct ControlCommand{
    double dc, ac;
    ControlCommand(double _dc, double _ac):dc(_dc),ac(_ac){};
};

class Controller{
    public:
        // Waypoint
        int IDwp;
        geometry_msgs::Point Ppreview;
        // Lateral control
        double ym;
        // Longitudinal control
        double E, iE;
        // Globals
        bool endreached;

        Controller(const MyReference& ref, const VehicleState& x);
        ControlCommand getControls(const MyReference& ref, const Vehicle& veh, const VehicleState& x);
    private:
        double getAccelerationCommand(const Vehicle& veh, const MyReference& ref, const VehicleState& x);
        double getSteerCommand(const MyReference& ref, const VehicleState& x, const Vehicle& veh);
        void updateWaypoint(const MyReference& ref, const VehicleState& x);
};


double getLateralError(const MyReference &ref, const VehicleState &x, const int& IDwp,const geometry_msgs::Point& Ppreview);
int findClosestPoint(const MyReference& ref, const geometry_msgs::Point& point, int ID);
void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&pose)[3]);
double interpolate(const double (&Txval)[3], const double (&Tyval)[3]);
void updateLookahead(double v);

void updateReferenceResolution(double v);

// Longitudinal lookahead index (defined in controller.cpp)
extern int LAlong;

#endif
