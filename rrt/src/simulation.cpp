#include <cmath>

#include "rrt/controller.h"
#include "rrt/simulation.h"
#include "rrt/controller.h"

void enforceConstraints(const double& min, const double& max, double& val){
    val = std::max(std::min(val,max),min);
}

state_type VehicleODE(ControlCommand& ctrl, state_type& x, const Vehicle& veh){
    state_type dx(7);
	double Gss = 1/( 1 + pow((x[4]/veh.Vch),2)); 	// Sideslip transfer function
	dx[0] = x[4]*cos(x[2]);            			// xdot
    dx[1] = x[4]*sin(x[2]);            			// ydot
    dx[2] = (x[4]/veh.L)*tan(x[3])*Gss;			// thetadot
    dx[3] = (1/veh.Td)*(ctrl.dc-x[3]);  			// deltadot
    dx[4] = x[5];                      			// vdot
    dx[5] = (1/veh.Ta)*(ctrl.ac-x[5]);  			// adot
    dx[6] = 1;                          		// dt
	// Constraints
	enforceConstraints(veh.amin, veh.amax, dx[4]);
	enforceConstraints(-veh.ddmax, veh.ddmax, dx[3]);
	return dx;
};

void IntegrateEuler(ControlCommand& ctrl, state_type& x, state_type& dx, double& dt, const Vehicle& veh){
	for(int i = 0; i<= x.size(); i++){
		x[i] = x[i] + dx[i]*dt;
	};
	// Constraints
	enforceConstraints(-veh.dmax,veh.dmax,x[3]);
	enforceConstraints(veh.amin, veh.amax, dx[5]);
};

Simulation::Simulation(	const MyRRT& RRT, const vector<double>& state, MyReference& ref, const Vehicle& veh, 
					   	const bool& GoalBiased, const bool& genProfile, const double& Vstart): 
					   	costE(0), costS(0), goalReached(false), endReached(false){
	stateArray.push_back(state); 				// Push initial state into statearray
	Controller control(ref,state);				// Initialize controller
	stateArray.back()[7] = control.IDwp;		// Add waypoint ID in stateArray
	if (genProfile){
		generateVelocityProfile(ref,0,control.IDwp,Vstart,vmax,RRT.goalPose,GoalBiased);

	}
	propagate(RRT, control,ref,veh);			// Predict vehicle trajectory
};

double getDistToLane(const double& x, const double& y, double S, const vector<double>& Cxy){
	double Lx = (x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2])/(pow(Cxy[1],2) + 1);
	double Ly = S + Cxy[2] + (Cxy[1]*(x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2]))/(pow(Cxy[1],2) + 1);
	return sqrt( pow(Lx-x,2) + pow(Ly-y,2) );
}

void Simulation::propagate(const MyRRT& RRT, Controller control, const MyReference& ref, const Vehicle& veh){
	bool wasNearGoal = false;

	for(int i = 0; i<(20/sim_dt); i++){
		sim_count++;
		state_type x = stateArray[i];								// Set x as last vehicle state
		ControlCommand ctrlCmd = control.getControls(ref,veh,x);	// Get controls for state
		state_type dx = VehicleODE(ctrlCmd, x, veh);				// Get vehicle state transition
		IntegrateEuler(ctrlCmd, x, dx, sim_dt, veh);				// Get new state
		x[7] = control.IDwp;									// Add waypoint ID to vehicle state
		// x[8] = ctrlCmd.ac;		// Control logging
		x[8] = ref.v[control.IDwp+LAlong];
		x[9] = ctrlCmd.dc;		// Control logging
		stateArray.push_back(x);									// Add state to statearray
		// ****** CHECK COLLISION *****
		ROS_WARN_STREAM("In simulation.cpp -> Simulation::propagate: Adjust the collision check");
		/********************************
		HOW TO ADJUST COLLISION CHECK
		*********************************
		1. Make sure to feed occupancy grid to simulation as follows: 
			- Make callback function to occupancy grid in motionplanner object
			- Feed the obstacle grid to expand_tree
			- Feed the obstacle grid to Simulation::sim(....)
			- Feed the obstacle grid to Simulation::propagate(...)
			- Feed the grid to myCollisionCheck(...) below
		2. Adjust myCollisionCheck() in collisioncheck.cpp to use obstacle grid
		*/
		// bool COL = myCollisionCheck(x);	
		double Dobs = checkObsDistance(x); // Feed grid
		if (Dobs==0){
			endReached = false; fail_collision++; return;
		}

		// ****** UPDATE COSTS ********
		costE += x[4]*sim_dt;
		double kappa = tan(x[3])/veh.L;								// Vehicle path curvature
		costS += RRT.Wcost[0]*x[4]*sim_dt + RRT.Wcost[1]*abs(kappa) + RRT.Wcost[2]*exp(-RRT.Wcost[3]*Dobs);
		if (RRT.bend){
			double Dgoallane = getDistToLane(x[0],x[1],RRT.laneShifts[0],RRT.Cxy);
			costS += RRT.Wcost[4]*Dgoallane;
		}

		// Check acceleration limits
		double ay = abs(x[4]*dx[2]);
		// ROS_INFO_STREAM_THROTTLE(2,"Max road lateral acceleration: "<<ay_road_max);
		if ( ay + ay_road_max> 3){
			// if(debug_sim){	ROS_WARN_STREAM("Acceleration exceeded! "<<ay<<" m/s2 , delta="<<x[3]);}
			endReached = false; fail_acclimit++;
			return;
		}
		if (draw_states){
			// Print the states
		}

		// Stop simulation if goal is reached
		double dist_to_goal = sqrt( pow(x[0]-RRT.goalPose[0],2) + pow(x[1]-RRT.goalPose[1],2));
		double goal_heading_error = abs(angleDiff(x[2],RRT.goalPose[2]));

		// Stop simulation when end of reference is reached and velocity < terminate velocity
		double Verror = (x[4]-ref.v.back());
		if (control.endreached&&abs(Verror<0.1)){
			if(wasNearGoal&&debug_sim){
				ROS_WARN_STREAM("Was near goal but did not reach! Egoalvel= "<<Verror<<", Eprofile="<<(x[4]-ref.v[control.IDwp]));
				ROS_WARN_STREAM("Dist2goal= "<<dist_to_goal<<" head error= "<<goal_heading_error<<" dla= "<<ctrl_dla);
				showVelocityProfile(ref);
			}
			endReached = true; return;
		}

		// Goal reached check
		if ((dist_to_goal<=1)&&(goal_heading_error<0.05)){
			double Verror = abs(x[4]-RRT.goalPose[3]);
			wasNearGoal = true;
			// if (Verror<0.1){
				if(debug_sim){	ROS_INFO_STREAM("goal reached");}
				ROS_INFO_STREAM_THROTTLE(1,"Goal reached in "<<stateArray.back()[6]<<" seconds!");
				goalReached = true; return;
			// }
		}
		if (draw_states){
			// cout<<"x="<<x[0]<<", y="<<x[1]<<", ac="<<ctrlCmd.ac<<", a="<<x[5]<<", v="<<x[4]<<endl;
		}
	}	
	if(wasNearGoal&&debug_sim){
		ROS_WARN_STREAM("Was near goal but end not reached");
		showVelocityProfile(ref);
	}
	fail_iterlimit++;
};
