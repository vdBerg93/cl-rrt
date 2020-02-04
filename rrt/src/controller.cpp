// Vehicle controller
#include "rrt/simulation.h"
#include "rrt/controller.h"
#include "rrt/rrtplanner.h"
#include "rrt/vehicle.h"
#include "rrt/datatypes.h"
#include <boost/range/irange.hpp>


//*******************************
// CONTROLLER CLASS FUNCTIONS 
//*******************************
void updateLookahead(double v){
	double dla_c = ctrl_mindla - ctrl_tla*ctrl_dlavmin; 
	ctrl_dla = std::max(ctrl_mindla,dla_c+ctrl_tla*std::abs(v));
}

void updateReferenceResolution(double v){
    // double ref_int{0.2}, ref_mindist{0.2};
    ref_res = std::max(abs(v)*ref_int,ref_mindist);
}

Controller::Controller(const MyReference& ref, const state_type& x){
    updateLookahead(x[4]);      // Update the lookahead distance (velocity dependent)
    IDwp = 0; endreached = 0;   // Lateral control initialization
    iE = 0;                     // Longitudinal control error integral
    updateWaypoint(ref,x);      // Initialize the first waypoint
}

ControlCommand Controller::getControls(const MyReference& ref, const Vehicle& veh, const state_type& x){
    updateWaypoint(ref, x); // Update the closest waypoint with the new preview point
    ControlCommand C {getSteerCommand(ref, x, veh),getAccelerationCommand(veh, ref, x)};
    return C;
}
int LAlong = 2;

double Controller::getAccelerationCommand(const Vehicle& veh, const MyReference& ref, const state_type& x){
    // int LAlong = 2;                         // Look additional x points in front of preview point (else velocity error could be zero)
    double E = ref.v[IDwp+LAlong]-x[4];            // Error
    iE = iE + E*sim_dt;                     // Integral error

    // Calculate acceleration command and constrain it
    double aCmd = checkSaturation(veh.amin,veh.amax,ctrl_Kp*E+ctrl_Ki*iE);
    return aCmd;
};

double Controller::getSteerCommand(const MyReference& ref, const state_type& x, const Vehicle& veh){
    ym = getLateralError(ref,x,IDwp,Ppreview);                          // Get the lateral error at preview point, perpendicular to vehicle 
    double cmdDelta = 2*((veh.L+veh.Kus*x[4]*x[4])/pow(ctrl_dla,2))*ym; // Single preview point control (Schmeitz, 2017, "Towards a Generic Lateral Control Concept ...")
    return checkSaturation(-veh.dmax,veh.dmax,cmdDelta);;               // Constrain with actuator saturation limits
};

void Controller::updateWaypoint(const MyReference& ref, const state_type& x){
    updateLookahead(x[4]);  // Update the lookahead distance
    // Use lookahead distance to update the preview point
    Ppreview.x = x[0] + ctrl_dla*ref.dir*std::cos(x[2]);
    Ppreview.y = x[1] + ctrl_dla*ref.dir*std::sin(x[2]);
    // Update the waypoint ID
    IDwp = findClosestPoint(ref, Ppreview, IDwp);

    // if (IDwp>=(ref.x.size()-2)){
    if (IDwp>=ref.x.size()-1-LAlong){
        endreached = 1;
    };
    if ((ref.x[IDwp]==ref.x.back())&&(ref.y[IDwp]==ref.y.back())){
        endreached = 1;
    }
};

double getLateralError(const MyReference &ref, const state_type &x, const int& IDwp,const geometry_msgs::Point& Ppreview){
    // Determine the ID's of the reference that will be used for lateral error calculation
    int IDmin, IDmax;
    if (IDwp==0){
        IDmin = IDwp; IDmax = IDwp+2;
    }
    else if (IDwp==ref.x.size()){
        IDmin = IDwp-2; IDmax = IDwp;
    }
    else{
        IDmin = IDwp-1; IDmax = IDwp+1;
    }
    // Extract points
    double xval[3] {ref.x[IDmin],ref.x[IDmin+1],ref.x[IDmax]};
    double yval[3] {ref.y[IDmin],ref.y[IDmin+1],ref.y[IDmax]};
    // Extend preview point with vehicle heading
    double Xpreview[3] {Ppreview.x,Ppreview.y,x[2]};
    // Transform the extracted reference into local coordinates of the preview point
    double Txval[3], Tyval[3];
    transformToVehicle(xval,yval,Txval,Tyval,Xpreview);
    // Find the coordinate of local x-axis intersection to get the lateral error 
    double ym = interpolate(Txval,Tyval);
    return ym;
}


int findClosestPoint(const MyReference& ref, const geometry_msgs::Point& point, int ID){
    // Find the point along the reference that is closest to the preview point
    double dmin{inf}, di;
    int idmin = 0;
    for(int i = ID; i<ref.x.size(); i++){
        di = (ref.x[i]-point.x)*(ref.x[i]-point.x) + (ref.y[i]-point.y)*(ref.y[i]-point.y);
        // If next point is closer, update minimum
        if(di<dmin){ 
            dmin = di;
            idmin = i;
        }
        // else the points are increasing in distance (linear reference)
        // else{
        //     return idmin;
        // }
    }
    return idmin;
};

void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&x)[3]){
    // Transform to vehicle coordinates of preview point with a homogenous transformation.
    //      H = [R,d;zeros(1,2),1];
    // 1. Define the rotation matrix and position vector
    //      R = [cos(X3),-sin(X3);sin(X3),cos(X3)];
    //      d = [X1;X2];
    // 2. Define the inverse of the homogenous transformation matrix
    //      Hinv = [R',-R'*d;zeros(1,2),1];
    // 3. Loop through the points and transform them
    //      pointTransformed = Hinv*[Rx;Ry;1];
    for(int i = 0; i<=2; i++){
        Txval[i] = xval[i]*cos(x[2]) - x[0]*cos(x[2]) - yval[i]*sin(x[2])+ x[1]*sin(x[2]);
        Tyval[i] = yval[i]*cos(x[2]) - x[1]*cos(x[2]) + xval[i]*sin(x[2])- x[0]*sin(x[2]);
        	// double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
            // double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
    }
    return;
}

double interpolate(const double (&Txval)[3], const double (&Tyval)[3]){
    // Do a second order Lagrange interpolation around three closest data points
    // The lateral error is equal to the y-coordinate of x-axis intersection
    double y{0}, L;
    for(int i = 0; i<=2; i++){
        L = 1;
        for(int j = 0; j<=2; j++){
            if (i!=j){
                L = L*(Txval[j])/(Txval[i]-Txval[j]);
            }
        }
        y = y + Tyval[i]*L;
    }
    return y;
}
