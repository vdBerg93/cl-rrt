#include "rrt/headers.h"
#include "rrt/globals.h"
using namespace std;

void enforceConstraints(const double& min, const double& max, double& val){
    val = std::max(std::min(val,max),min);
}

/**
 * @brief Bicycle-model vehicle ODE.
 *
 * Computes the 7-element state derivative for the kinematic bicycle model
 * with first-order actuator dynamics:
 *   dx/dt     = v * cos(theta)
 *   dy/dt     = v * sin(theta)
 *   dtheta/dt = (v/L) * tan(delta) * Gss       (Gss = sideslip TF)
 *   ddelta/dt = (1/Td) * (dc - delta)           (first-order steer lag)
 *   dv/dt     = a
 *   da/dt     = (1/Ta) * (ac - a)               (first-order accel lag)
 *   dt/dt     = 1                                (time)
 *
 * @param ctrl   Steering (dc) and acceleration (ac) commands
 * @param x      Current vehicle state
 * @param veh    Vehicle parameters (L, Td, Ta, Vch, limits)
 * @return StateDeriv with 7 derivative fields
 */
StateDeriv VehicleODE(ControlCommand& ctrl, const VehicleState& x, const Vehicle& veh){
    StateDeriv dx;
	double Gss = 1/( 1 + pow((x.v/veh.Vch),2)); 	// Sideslip transfer function
	dx.dx     = x.v*cos(x.theta);
    dx.dy     = x.v*sin(x.theta);
    dx.dtheta = (x.v/veh.L)*tan(x.delta)*Gss;
    dx.ddelta = (1/veh.Td)*(ctrl.dc-x.delta);
    dx.dv     = x.a;
    dx.da     = (1/veh.Ta)*(ctrl.ac-x.a);
    dx.dt     = 1;
	// Constraints
	enforceConstraints(-veh.ddmax, veh.ddmax, dx.ddelta);
	return dx;
};

/**
 * @brief Forward-Euler integration of the 7 dynamic states.
 *
 * Integrates only the dynamic fields of VehicleState (x..time).
 * Logging fields (waypoint_id, ref_vel, steer_cmd) are untouched.
 * Post-integration constraints enforce actuator limits on delta and acceleration.
 */
void IntegrateEuler(VehicleState& x, const StateDeriv& dx, double& dt, const Vehicle& veh){
	x.x     += dx.dx * dt;
	x.y     += dx.dy * dt;
	x.theta += dx.dtheta * dt;
	x.delta += dx.ddelta * dt;
	x.v     += dx.dv * dt;
	x.a     += dx.da * dt;
	x.time  += dx.dt * dt;
	// Constraints
	enforceConstraints(-veh.dmax,veh.dmax,x.delta);
	enforceConstraints(veh.amin, veh.amax, x.a);   // clamp acceleration state, not jerk
};

Simulation::Simulation(	const MyRRT& RRT, const VehicleState& state, MyReference& ref, const Vehicle& veh,
					   	const bool& GoalBiased, const bool& genProfile, const double& Vstart):
					   	costE(0), costS(0), goalReached(false), endReached(false){
	stateArray.push_back(state); 				// Push initial state into statearray
	Controller control(ref,state);				// Initialize controller
	stateArray.back().waypoint_id = control.IDwp;		// Add waypoint ID in stateArray
	if (genProfile){
		generateVelocityProfile(ref,0,control.IDwp,Vstart,vmax,RRT.goalPose,GoalBiased);

	}
	propagate(RRT, control,ref,veh);			// Predict vehicle trajectory
};

/**
 * @brief Closed-loop simulation loop.
 *
 * Iterates up to 20s/sim_dt steps. Each iteration:
 *   1. Get controller commands for current state
 *   2. Compute state derivative (VehicleODE)
 *   3. Forward-Euler integration
 *   4. Fill logging fields (waypoint_id, ref_vel, steer_cmd)
 *   5. Check collision, update costs, check termination
 *
 * Termination conditions:
 *   - Collision (Dobs==0)
 *   - Lateral acceleration exceeds 3 m/s^2
 *   - Goal reached (dist<1m, heading<0.05rad)
 *   - End of reference reached with velocity error < 0.1 m/s
 *   - Iteration limit (20s horizon)
 */
void Simulation::propagate(const MyRRT& RRT, Controller& control, const MyReference& ref, const Vehicle& veh){
	bool wasNearGoal = false;
	int endreachedStep = -1;
	const int graceSteps = (int)(2.0 / sim_dt); // 2s grace period after end of reference

	for(int i = 0; i<(20/sim_dt); i++){
		sim_count++;
		VehicleState x = stateArray[i];								// Set x as last vehicle state
		ControlCommand ctrlCmd = control.getControls(ref,veh,x);	// Get controls for state
		StateDeriv dx = VehicleODE(ctrlCmd, x, veh);				// Get vehicle state transition
		IntegrateEuler(x, dx, sim_dt, veh);							// Get new state
		x.waypoint_id = control.IDwp;								// Add waypoint ID to vehicle state
		x.ref_vel = ref.v[std::min(control.IDwp+LAlong, (int)ref.v.size()-1)];
		x.steer_cmd = ctrlCmd.dc;									// Control logging
		stateArray.push_back(x);									// Add state to statearray
		// ****** CHECK COLLISION *****
		ROS_WARN_STREAM_ONCE("In simulation.cpp -> Simulation::propagate: Adjust the collision check");
		double Dobs = checkObsDistance(x); // Feed grid
		if (Dobs==0){
			endReached = false; fail_collision++; return;
		}

		// ****** UPDATE COSTS ********
		costE += x.v*sim_dt;
		double kappa = tan(x.delta)/veh.L;								// Vehicle path curvature
		costS += RRT.Wcost[0]*x.v*sim_dt + RRT.Wcost[1]*abs(kappa) + RRT.Wcost[2]*exp(-RRT.Wcost[3]*Dobs);
		if (RRT.bend){
			double Dgoallane = getDistToLane(x.x,x.y,RRT.laneShifts[0],RRT.Cxy);
			costS += RRT.Wcost[4]*Dgoallane;
		}

		// Check acceleration limits
		double ay = abs(x.v*dx.dtheta);
		double ay_road = 0;
		if (RRT.Cxy.size() >= 3) {
			double kappa_road = (2*RRT.Cxy[0]) / pow(1 + pow(RRT.Cxy[1] + 2*RRT.Cxy[0]*x.x, 2), 1.5);
			ay_road = x.v*x.v*abs(kappa_road);
		}
		if ( ay + ay_road > 3){
			endReached = false; fail_acclimit++;
			return;
		}
		if (draw_states){
			// Print the states
		}

		// Stop simulation if goal is reached
		double dist_to_goal = sqrt( pow(x.x-RRT.goalPose.x,2) + pow(x.y-RRT.goalPose.y,2));
		double goal_heading_error = abs(angleDiff(x.theta,RRT.goalPose.theta));

		// Stop simulation when end of reference is reached and velocity has settled
		double Verror = (x.v-ref.v.back());
		if (control.endreached) {
			if (endreachedStep < 0) endreachedStep = i;
			if (abs(Verror) < 0.1) {
				if(wasNearGoal&&debug_sim){
					ROS_WARN_STREAM("Was near goal but did not reach! Egoalvel= "<<Verror<<", Eprofile="<<(x.v-ref.v[control.IDwp]));
					ROS_WARN_STREAM("Dist2goal= "<<dist_to_goal<<" head error= "<<goal_heading_error<<" dla= "<<ctrl_dla);
					showVelocityProfile(ref);
				}
				endReached = true; return;
			}
			if (i - endreachedStep >= graceSteps) {
				endReached = true; return;
			}
		}

		// Goal reached check
		if ((dist_to_goal<=1)&&(goal_heading_error<0.05)){
			double Verror = abs(x.v-RRT.goalPose.v);
			wasNearGoal = true;
				if(debug_sim){	ROS_INFO_STREAM("goal reached");}
				ROS_INFO_STREAM_THROTTLE(1,"Goal reached in "<<stateArray.back().time<<" seconds!");
				goalReached = true; return;
		}
		if (draw_states){
			// cout<<"x="<<x.x<<", y="<<x.y<<", ac="<<ctrlCmd.ac<<", a="<<x.a<<", v="<<x.v<<endl;
		}
	}
	if(wasNearGoal&&debug_sim){
		ROS_WARN_STREAM("Was near goal but end not reached");
		showVelocityProfile(ref);
	}
	fail_iterlimit++;
};
