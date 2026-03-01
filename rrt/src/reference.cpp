
/* --------------------------------------
	REFERENCE GENERATION
---------------------------------------*/
#include "rrt/headers.h"
#include "rrt/globals.h"

/**
 * @brief Generate a linear reference path from the last node reference point to the sample.
 *
 * The reference is a straight line from the end of the parent node's reference
 * to the sample point, discretized at ref_res spacing.
 */
MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir){
	if(debug_mode){
		cout<<"Generating reference..."<<endl;
	}
	MyReference ref;
	double L = sqrt( pow(sample.x-node.ref.x.back(),2) + pow(sample.y-node.ref.y.back(),2) );
	int N = round(L/ref_res)+1;
	ref.x = LinearSpacedVector(node.ref.x.back(),sample.x,N);
	ref.y = LinearSpacedVector(node.ref.y.back(),sample.y,N);
	ref.dir = dir;
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated reference."<<endl;}
	return ref;
};

/**
 * @brief Generate a two-segment goal-biased reference.
 *
 * Segment 1: from the parent node's reference end to a point near the goal
 * (aligned with goal heading). Segment 2: extends beyond the goal by the
 * lookahead distance to allow the controller to track through the goal.
 */
MyReference getGoalReference(const Vehicle& veh, Node node, const GoalPose& goalPose){;

	double dla_c = ctrl_mindla - ctrl_tla*ctrl_dlavmin;
	double dla_end = std::max(ctrl_mindla,dla_c+ctrl_tla*std::abs(goalPose.v));

	double Dextend = dla_end; assert(abs(dla_end-3.2)<0.01);
	double Dalign = 1;

	MyReference ref;

	// Alignment
	geometry_msgs::Point P1, P2, Pclose, Pfar;
	P1.x = goalPose.x+Dalign*cos(goalPose.theta); P1.y = goalPose.y+Dalign*sin(goalPose.theta);
	P2.x = goalPose.x-Dalign*cos(goalPose.theta); P2.y = goalPose.y-Dalign*sin(goalPose.theta);
	double H = atan2(P2.y-P1.y,P2.x-P1.x);

	// Select closest point
	if( sqrt( pow(P1.x-node.ref.x.back(),2) + pow(P1.y-node.ref.y.back(),2)) < sqrt( pow(P2.x-node.ref.x.back(),2) + pow(P2.y-node.ref.y.back(),2))){
		Pclose = P1; Pfar = P1;
	}else{
		Pclose = P2; Pfar = P2;
	}
	// Extend to account for lookahead distance
	Pfar.x += (Dextend+Dalign)*cos(goalPose.theta);
	Pfar.y += (Dextend+Dalign)*sin(goalPose.theta);

	// Segment lengths
	double N1 = round(sqrt( pow(Pclose.x-node.ref.x.back(),2) + pow(Pclose.y-node.ref.y.back(),2))/ref_res)+1;
	double N2 = round(sqrt( pow(Pfar.x-Pclose.x,2) + pow(Pfar.y-Pclose.y,2))/ref_res)+1;
	// Generate reference
	vector<double> Refx  = LinearSpacedVector(node.ref.x.back(),Pclose.x,N1);
	vector<double> Refxa = LinearSpacedVector(Pclose.x,Pfar.x,N2);
	vector<double> Refy  = LinearSpacedVector(node.ref.y.back(),Pclose.y,N1);
	vector<double> Refya = LinearSpacedVector(Pclose.y,Pfar.y,N2);
	ref.x.insert(ref.x.end(),Refx.begin(),Refx.end());
	ref.x.insert(ref.x.end(),Refxa.begin(),Refxa.end());
	ref.y.insert(ref.y.end(),Refy.begin(),Refy.end());
	ref.y.insert(ref.y.end(),Refya.begin(),Refya.end());

	// For debugging
	assert(ref.x.size()==ref.y.size());
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated goal reference."<<endl;}
	return ref;
};

/**
 * @brief Generate a trapezoidal velocity profile along the reference.
 *
 * Computes a three-phase velocity profile:
 *   1. Acceleration phase:  v0 -> Vcoast  (constant a_acc)
 *   2. Coasting phase:      Vcoast         (constant velocity)
 *   3. Braking phase:       Vcoast -> vend (constant a_dec)
 *
 * Vcoast is chosen so that the total distance matches the path length Lp.
 * If vmax can be reached for at least tmin seconds, Vcoast = vmax.
 * Otherwise, Vcoast is computed from the quadratic distance equation.
 */
void generateVelocityProfile(MyReference& ref, const double& a0, const int& IDwp, const double& v0, const double& vmax, const GoalPose& goal, bool GB){
	double vend = goal.v;
	// Slope shape configuration
	double a_acc = 1;	double a_dec = -1; 	double tmin = 1;

	double Lp, res;
	if(GB){
		double Dgoal = sqrt( pow(goal.x-ref.x.front(),2) + pow(goal.y-ref.y.front(),2));
		Lp = Dgoal + ctrl_mindla;
		res = Lp/(ref.x.size()-1);
	}else{
		double Dgoal = sqrt( pow(goal.x-ref.x.back(),2) + pow(goal.y-ref.y.back(),2));
		double Lref = sqrt( pow(ref.x.front()-ref.x.back(),2) + pow(ref.y.front()-ref.y.back(),2));
		res = Lref/(ref.x.size()-1);
		Lp = Lref + Dgoal + ctrl_mindla;
	}
	// Check if the maximum coasting velocity can be reached for minimum time tmin
	double Daccel = (pow(vmax,2)- pow(v0,2))/(2*a_acc);
	double Dcoast = vmax*tmin;
	double Dbrake = (pow(vend,2)-pow(vmax,2))/(2*a_dec);
	bool D_vmax_bool = (Daccel + Dcoast + Dbrake)<Lp;

    // Check what kind of velocity profile must be generated
	double Vcoast;
	if (vend>(v0+0.1)){
		Vcoast = vend;		 	// if end velocity greater than v0
	}else if (D_vmax_bool){
		Vcoast = vmax;			// If Vmax can be reached, Vcoast = Vmax
	}else{		// Else define as Dacc+Dcoast+Dbrake = D (solved in MATLAB for Vcoast)
        double D = Lp;
		double v1 =  ( sqrt(pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) - 2*D*pow(a_acc,2)*a_dec + pow(a_acc,2)*pow(vend,2) + 2*D*a_acc*pow(a_dec,2) - a_acc*a_dec*pow(v0,2) - a_acc*a_dec*pow(vend,2) + pow(a_dec,2)*pow(v0,2)) + a_acc*a_dec*tmin)/(a_acc - a_dec);

		Vcoast = v1;
    }

	// Update profile distances with new Vcoast
	Daccel = ( pow(Vcoast,2)- pow(v0,2))/(2*a_acc);
	if(Daccel<0){		Daccel = 0; Vcoast = v0;    }
	double dv = Vcoast-vend;
	Dbrake = max(double(0), ( pow(vend,2)-pow(Vcoast,2))/(2*a_dec));
	Dcoast = max(double(0),Lp-Daccel-Dbrake);

	/***************************************
	Linear interpolation
	****************************************/
    double tacc = (Vcoast-v0)/a_acc;
    double tcoast = Dcoast/Vcoast;
    double tbrake = (vend-Vcoast)/a_dec;
	for(int i = 0; i!=ref.x.size(); i++){
        double D = i*res;
        if(D<Daccel){
            double t1 = -(v0 - sqrt(pow(v0,2) + 2*a_acc*D))/a_acc;
            double t2 = -(v0 + sqrt(pow(v0,2) + 2*a_acc*D))/a_acc;
            double t = (t1>=0)*t1+(t2>=0)*t2;
            ref.v.push_back(v0+a_acc*t);
			assert((0<=ref.v.back())&&(ref.v.back()<=10)&&"Error in acceleration part of profile (v>10)");
        }else if (D<=(Daccel+Dcoast)){
            ref.v.push_back(Vcoast);
			assert((0<=ref.v.back())&&(ref.v.back()<=10)&&"Error in coasting part of profile (v>10)");
		}else{
            double t1 = -(Vcoast + sqrt(pow(Vcoast,2) + 2*D*a_dec - 2*Daccel*a_dec - 2*Dcoast*a_dec))/a_dec;
            double t2 = -(Vcoast - sqrt(pow(Vcoast,2) + 2*D*a_dec - 2*Daccel*a_dec - 2*Dcoast*a_dec))/a_dec;
            double dt = (t1!=tbrake)*(t1>=0)*(t1<=tbrake)*t1+(t2>=0)*(t2<=tbrake)*t2;
            ref.v.push_back( max(double(0),Vcoast+a_dec*dt));
			assert((0<=ref.v.back())&&(ref.v.back()<=10)&&"Error in braking part of profile (v>10)");
        }
    }

	if(debug_velocity){
		cout<<"velocity = [";
		for(auto it = ref.v.begin(); it!=ref.v.end(); it++){
			cout<<*it<<", ";
		}
		cout<<"]"<<endl;
	}
	assert(ref.v.size()==ref.x.size());
	assert( (Vcoast<=10)&&"Wrong coast velocity");
}

void showVelocityProfile(const MyReference& ref){
	cout<<endl<<"---Reference---"<<endl;
	cout<<"Velocity profile: "<<endl;
	auto it = ref.v.begin();
	for(it; it!=ref.v.end(); it++){
		cout<<*it<<", ";
	}
	cout<<endl<<endl;
}
