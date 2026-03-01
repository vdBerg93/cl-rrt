// Vehicle controller
#include "rrt/headers.h"
#include "rrt/globals.h"


//*******************************
// CONTROLLER CLASS FUNCTIONS
//*******************************

/**
 * @brief Update velocity-dependent lookahead distance.
 *
 * dla = max(dla_min, c + tla * |v|)  where c = dla_min - tla * dlavmin
 * This ensures the preview point moves further ahead at higher speeds.
 */
void updateLookahead(double v){
	double dla_c = ctrl_mindla - ctrl_tla*ctrl_dlavmin;
	ctrl_dla = std::max(ctrl_mindla,dla_c+ctrl_tla*std::abs(v));
}

void updateReferenceResolution(double v){
    ref_res = std::max(abs(v)*ref_int,ref_mindist);
}

Controller::Controller(const MyReference& ref, const VehicleState& x){
    updateLookahead(x.v);       // Update the lookahead distance (velocity dependent)
    IDwp = 0; endreached = 0;   // Lateral control initialization
    iE = 0;                     // Longitudinal control error integral
    updateWaypoint(ref,x);      // Initialize the first waypoint
}

ControlCommand Controller::getControls(const MyReference& ref, const Vehicle& veh, const VehicleState& x){
    updateWaypoint(ref, x); // Update the closest waypoint with the new preview point
    ControlCommand C {getSteerCommand(ref, x, veh),getAccelerationCommand(veh, ref, x)};
    return C;
}
int LAlong = 2;

/**
 * @brief PI longitudinal controller with back-calculation anti-windup.
 *
 * aCmd = Kp * e + Ki * integral(e)
 * The integral is only accumulated when the output is unsaturated,
 * preventing integrator windup against the actuator limits.
 */
double Controller::getAccelerationCommand(const Vehicle& veh, const MyReference& ref, const VehicleState& x){
    double E = ref.v[IDwp+LAlong]-x.v;            // Error

    // Calculate raw command before saturation
    double aRaw = ctrl_Kp*E + ctrl_Ki*iE;
    double aCmd = checkSaturation(veh.amin, veh.amax, aRaw);

    // Anti-windup: only accumulate integral when output is unsaturated
    if (aRaw >= veh.amin && aRaw <= veh.amax){
        iE += E * sim_dt;
    }
    return aCmd;
};

/**
 * @brief Schmeitz single-preview-point lateral controller.
 *
 * delta_cmd = 2 * (L + Kus*v^2) / dla^2 * ym
 * where ym is the lateral error at the preview point, computed via
 * Lagrange interpolation of three reference points transformed to
 * the preview point's local frame.
 *
 * Reference: Schmeitz, 2017, "Towards a Generic Lateral Control Concept ..."
 */
double Controller::getSteerCommand(const MyReference& ref, const VehicleState& x, const Vehicle& veh){
    ym = getLateralError(ref,x,IDwp,Ppreview);                          // Get the lateral error at preview point, perpendicular to vehicle
    double cmdDelta = 2*((veh.L+veh.Kus*x.v*x.v)/pow(ctrl_dla,2))*ym; // Single preview point control (Schmeitz, 2017, "Towards a Generic Lateral Control Concept ...")
    return checkSaturation(-veh.dmax,veh.dmax,cmdDelta);;               // Constrain with actuator saturation limits
};

void Controller::updateWaypoint(const MyReference& ref, const VehicleState& x){
    updateLookahead(x.v);  // Update the lookahead distance
    // Use lookahead distance to update the preview point
    Ppreview.x = x.x + ctrl_dla*ref.dir*std::cos(x.theta);
    Ppreview.y = x.y + ctrl_dla*ref.dir*std::sin(x.theta);
    // Update the waypoint ID
    IDwp = findClosestPoint(ref, Ppreview, IDwp);

    if (IDwp>=ref.x.size()-1-LAlong){
        endreached = 1;
    };
    if ((ref.x[IDwp]==ref.x.back())&&(ref.y[IDwp]==ref.y.back())){
        endreached = 1;
    }
};

/**
 * @brief Compute lateral error at the preview point via Lagrange interpolation.
 *
 * 1. Select 3 reference points around the closest waypoint
 * 2. Transform them to the preview point's local frame (homogeneous transform)
 * 3. Perform 2nd-order Lagrange interpolation to find the y-intercept (lateral error)
 */
double getLateralError(const MyReference &ref, const VehicleState &x, const int& IDwp,const geometry_msgs::Point& Ppreview){
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
    double Xpreview[3] {Ppreview.x,Ppreview.y,x.theta};
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
    }
    return idmin;
};

/**
 * @brief Homogeneous transformation of reference points to the preview point's local frame.
 *
 * H = [R, d; 0 0 1],  Hinv = [R', -R'd; 0 0 1]
 * where R = [cos(theta), -sin(theta); sin(theta), cos(theta)]
 * and d = [pose[0]; pose[1]].
 *
 * @param pose  3-element array {preview.x, preview.y, heading} — NOT a VehicleState
 */
void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&pose)[3]){
    for(int i = 0; i<=2; i++){
        Txval[i] = xval[i]*cos(pose[2]) - pose[0]*cos(pose[2]) - yval[i]*sin(pose[2]) + pose[1]*sin(pose[2]);
        Tyval[i] = yval[i]*cos(pose[2]) - pose[1]*cos(pose[2]) - xval[i]*sin(pose[2]) + pose[0]*sin(pose[2]);  // yL = -sin(θ)*(xW-x0) + cos(θ)*(yW-y0)
    }
    return;
}

/** Second order Lagrange interpolation
 * @param Txval x-coordinates of the three points around the preview point, transformed to the preview point's local coordinates
 * @param Tyval y-coordinates of the three points around the preview point, transformed to the preview point's local coordinates
 * @return y-coordinate of the x-axis intersection, which is the lateral error
 */
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
