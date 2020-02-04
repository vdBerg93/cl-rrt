/* --------------------------------------
	REFERENCE GENERATION
---------------------------------------*/
#include "rrt/rrtplanner.h"
#include "ros/ros.h"

// Generate a linear reference paths
MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir){
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

// Generate a goal biased reference
MyReference getGoalReference(const Vehicle& veh, Node node, vector<double> goalPose){;
	double Dextend = ctrl_mindla;//+1.2;
	double Dalign = 1;
	
	MyReference ref;

	// Alignment
	geometry_msgs::Point P1, P2, Pclose, Pfar;
	P1.x = goalPose[0]+Dalign*cos(goalPose[2]); P1.y = goalPose[1]+Dalign*sin(goalPose[2]);
	P2.x = goalPose[0]-Dalign*cos(goalPose[2]); P2.y = goalPose[1]-Dalign*sin(goalPose[2]);
	double H = atan2(P2.y-P1.y,P2.x-P1.x);

	// Select closest point
	if( sqrt( pow(P1.x-node.ref.x.back(),2) + pow(P1.y-node.ref.y.back(),2)) < sqrt( pow(P2.x-node.ref.x.back(),2) + pow(P2.y-node.ref.y.back(),2))){
		Pclose = P1; Pfar = P1; 
	}else{
		Pclose = P2; Pfar = P2;
	}
	// Extend to account for lookahead distance
	Pfar.x += (Dalign+Dextend)*cos(goalPose[2]);
	Pfar.y += (Dalign+Dextend)*sin(goalPose[2]);
	
	// Segment lengths
	double N1 = round(sqrt( pow(Pclose.x-node.ref.x.back(),2) + pow(Pclose.y-node.ref.y.back(),2))/ref_res)+1;
	double N2 = round(sqrt( pow(Pfar.x-Pclose.x,2) + pow(Pfar.y-Pclose.y,2))/ref_res)+1;
	// Generate reference
	vector<double> Refx  = LinearSpacedVector(node.ref.x.back(),Pclose.x,N1);
	vector<double> Refxa = LinearSpacedVector(Pclose.x,Pfar.x,N2);
	vector<double> Refy  = LinearSpacedVector(node.ref.y.back(),Pclose.y,N1);
	vector<double> Refya = LinearSpacedVector(Pclose.y,Pfar.y,N2);
	ref.x.insert(ref.x.end(),Refx.begin(),Refx.end());
	ref.x.insert(ref.x.end(),Refxa.begin()+1,Refxa.end());
	ref.y.insert(ref.y.end(),Refy.begin(),Refy.end());
	ref.y.insert(ref.y.end(),Refya.begin()+1,Refya.end());
	
	// For debugging
	assert(ref.x.size()==ref.y.size());
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated goal reference."<<endl;}
	return ref;
};

// Generate a trapezoidal velocity profile
void generateVelocityProfile(MyReference& ref, const double& a0, const int& IDwp, const double& v0, const double& vmax, const vector<double>& goal, bool GB){
	double vend = goal[3];
	// Slope shape configuration
	double a_acc = 1;	double a_dec = -1; 	double tmin = 1;  

	double Lp, res;
	if(GB){
		double Dgoal = sqrt( pow(goal[0]-ref.x.front(),2) + pow(goal[1]-ref.y.front(),2));
		Lp = Dgoal + ctrl_mindla;
		res = Lp/(ref.x.size()-1);
	}else{
		double Dgoal = sqrt( pow(goal[0]-ref.x.back(),2) + pow(goal[1]-ref.y.back(),2));
		double Lref = sqrt( pow(ref.x.front()-ref.x.back(),2) + pow(ref.y.front()-ref.y.back(),2));
		res = Lref/(ref.x.size()-1);
		Lp = Lref + Dgoal + ctrl_mindla;
	}

     
	// Check if the maximum coasting velocity can be reached for minimum time tmin
	double Daccel = (pow(vmax,2)- pow(v0,2))/(2*a_acc);
	double Dcoast = vmax*tmin;
	double Dbrake = (pow(vend,2)-pow(vmax,2))/(2*a_dec);
	double D_vmax_bool = (Daccel + Dcoast + Dbrake)<Lp;
    
    // Check what kind of velocity profile must be generated
	double Vcoast;
	if (vend>v0){
		Vcoast = vend;		 	// if end velocity greater than v0
	}else if (D_vmax_bool){
		Vcoast = vmax;			// If Vmax can be reached, Vcoast = Vmax
	}else{		// Else define as Dacc+Dcoast+Dbrake = D (solved in MATLAB for Vcoast)
        double D = Lp;
		// Vcoast calculation without compensation for tracking error
		double v1 =  ( sqrt(pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) - 2*D*pow(a_acc,2)*a_dec + pow(a_acc,2)*pow(vend,2) + 2*D*a_acc*pow(a_dec,2) - a_acc*a_dec*pow(v0,2) - a_acc*a_dec*pow(vend,2) + pow(a_dec,2)*pow(v0,2)) + a_acc*a_dec*tmin)/(a_acc - a_dec);       

		// Vcoast calculation with compensation for tracking error
		// double v = Vcoast;
		// double v1 = ( sqrt(pow(a_dec,2)*pow(v0,2) + pow(a_acc,2)*pow(vend,2) + pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) + 2*D*a_acc*pow(a_dec,2) - 2*D*pow(a_acc,2)*a_dec - 2*a0*a_acc*pow(a_dec,2) + 2*a0*pow(a_acc,2)*a_dec - a_acc*a_dec*pow(v0,2) - a_acc*a_dec*pow(vend,2) - 2*a1*a_acc*pow(a_dec,2)*v + 2*a1*pow(a_acc,2)*a_dec*v - 2*a2*a_acc*pow(a_dec,2)*pow(v,2) + 2*a2*pow(a_acc,2)*a_dec*pow(v,2)) + a_acc*a_dec*tmin)/(a_acc - a_dec);
		Vcoast = v1;
    }
    
	// Update profile distances with new Vcoast
	Daccel = ( pow(Vcoast,2)- pow(v0,2))/(2*a_acc);
	if(Daccel<0){		Daccel = 0; Vcoast = v0;    }
	Dbrake = max(double(0), ( pow(vend,2)-pow(Vcoast,2))/(2*a_dec));
	Dcoast = max(double(0),Lp-Daccel-Dbrake);

	// Cubic polynomial interpolation
	// double astart = 0;
	double astart = max(double(0),a0);
	ROS_WARN_STREAM("In vel profile: fix negative velocity due to negative acceleration ");
	double vstart = v0+(v0<0.01)*0.1;
	// if (v0<=0.5){
	// 	astart = 5;
	// }
	ROS_WARN_STREAM_THROTTLE(1,"in ref: make acceleration sim dependent");

	// Get amount of points in parts of profile
	int Nacc = (Daccel/res)+1; double t_acc = 0; double dt_acc = double(1)/double(Nacc-1);
	int Nbrake = (Dbrake/res)+1; double t_brake = 0; double dt_brake = double(1)/double(Nbrake-1);
	// Get the profile
	vector<double> Vacc, Vbrake;
	if (Nacc>1){
		// cout<<"Generating acceleration ramp"<<endl;
		vector<double> coef_acc = getCoefficients(Daccel,vstart,Vcoast,astart,0);
		vector<double> Tacc = getTimeVector(coef_acc,0,vstart,0,0,Daccel,Nacc);
		Vacc = getVelocityVector(vstart,coef_acc,Tacc);
	}else{
		 Vacc.push_back(v0);
	}
	if (Nbrake>1){
		vector<double> coef_brake = getCoefficients(Dbrake,Vcoast,goal[3],0,0);
		vector<double> Tbrake = getTimeVector(coef_brake,0,Vcoast,0,0,Dbrake,Nbrake);
		Vbrake = getVelocityVector(Vcoast,coef_brake,Tbrake);
	}else{
		Vbrake.push_back(Vcoast);
	}
	

	auto it_acc = Vacc.begin(); auto it_brake = Vbrake.begin();

	for(int i = 0; i!=ref.x.size(); i++){
        double D = i*res;
        if(D<Daccel){
			ref.v.push_back(*it_acc); it_acc++;
        }else if (D<(Daccel+Dcoast)){
            ref.v.push_back(Vcoast);
		}else{
			ref.v.push_back(*it_brake); it_brake++;
        }
    }

	// if(debug_velocity&&GB){
	// 	// Show profile
	// 	cout<<"--- REFERENCE DEBUG INFO ---"<<endl;
	// 	cout<<"profile = [";
	// 	for(auto it = ref.v.begin(); it!=ref.v.end(); it++){
	// 		cout<<*it<<", ";
	// 	}
	// 	cout<<"]"<<endl<<endl;
	// 	// show other information
	// 	double Dtotal = Daccel+Dbrake+Dcoast;
	// 	cout<<"IDwp="<<IDwp<<endl;
	// 	cout<<"Dacc = "<<Daccel<<". Dcoast = "<<Dcoast<<", Dbrake= "<<Dbrake<<endl;
	// 	cout<<"Lp="<<Lp<<", "<<Dtotal<<endl;
	// 	cout<<"Vcoast = "<<Vcoast<<endl;
	// }
	
	// Check contents

	for(auto it = Vacc.begin(); it!=Vacc.end(); ++it){
		cout<<"v="<<*it<<endl;
		assert( (-1<=(*it))&&( (*it)<= 10)&&"Error in acceleration ramp vector");
	}
	for(auto it = Vbrake.begin(); it!=Vbrake.end(); ++it){
		cout<<"v="<<*it<<endl;
		assert( (-1<=(*it))&&( (*it)<= 10)&&"Error in brake ramp vector");
	}
	assert(ref.v.size()==ref.x.size());
	assert( (Vcoast<=10)&&"Wrong coast velocity");
	assert(Vacc.size()==Nacc); assert(Vbrake.size()==Nbrake);
}

// Print velocity profile in terminal
void showVelocityProfile(const MyReference& ref){
	cout<<endl<<"---Reference---"<<endl;
	cout<<"x = ["<<ref.x.front()<<", "<<ref.x.back()<<"]"<<endl;
	cout<<"y = ["<<ref.y.front()<<", "<<ref.y.back()<<"]"<<endl;
	cout<<"Velocity profile: "<<endl;
	auto it = ref.v.begin();
	for(it; it!=ref.v.end(); it++){
		cout<<*it<<", ";
	}
	cout<<endl<<endl;
}

// Get coefficients of cubic polynomial for velocity interpolation
vector<double> getCoefficients(const double& Sf, const double& v0, const double& vf, const double& a0, const double& af){
	if(debug_velocity){cout<<"---- IN COEFFICIENT CALCULATION ----"<<endl;}
	double a,b,c,tf;
	if (abs(a0)<0.05){
	 	// Use solution solved for a0 = 0;
		a = 0;
	 	tf = (2*Sf)/(v0 + vf);
		c = (2*v0 - 2*vf + a0*tf + af*tf)/(pow(tf,3));
		b = -(a0 - af + (3*(2*v0 - 2*vf + a0*tf + af*tf))/tf)/(2*tf);
	}else{
		a = a0;
        // Trying first solution of second order root
		double tf1 = -(3*v0 + 3*vf - sqrt(3)*sqrt(3*pow(v0,2) + 6*v0*vf + 3*pow(vf,2) + 4*a0*Sf))/a0;
		double c1 = (2*v0 - 2*vf + a0*tf1 + af*tf1)/(pow(tf1,3));
		double b1 = -(a0 - af + (3*(2*v0 - 2*vf + a0*tf1 + af*tf1))/tf1)/(2*tf1);
		// Trying second solution 
		double tf2 = -(3*v0 + 3*vf + sqrt(3)*sqrt(3*pow(v0,2) + 6*v0*vf + 3*pow(vf,2) + 4*a0*Sf))/a0;
		double c2 = (2*v0 - 2*vf + a0*tf2 + af*tf2)/(pow(tf2,3));
		double b2 = -(a0 - af + (3*(2*v0 - 2*vf + a0*tf2 + af*tf2))/tf2)/(2*tf2);
		// Select best solution
		double S1 = v0*tf1 + 0.5*a*tf1*tf1 + 0.333333333333333333*b1*tf1*tf1*tf1 + 0.25*c1*tf1*tf1*tf1*tf1;
		double S2 = v0*tf2 + 0.5*a*tf2*tf2 + 0.333333333333333333*b2*tf2*tf2*tf2 + 0.25*c2*tf2*tf2*tf2*tf2;
		if(debug_velocity){		
			cout<<"S1="<<S1<<"; S2="<<S2<<endl;		
		}
		
		if( abs(Sf-S1)<abs(Sf-S2) ){
			b = b1; c = c1; tf = tf1;
		}else{
			b = b2; c = c2; tf = tf2;
		}
	}
	vector<double> coef{a,b,c,tf};

	// Check results
	double Send =  v0*tf + 0.5*coef[0]*tf*tf + 0.333333333333333333*coef[1]*tf*tf*tf + 0.25*coef[2]*tf*tf*tf*tf;
	double Vend = v0 + coef[0]*tf + coef[1]*tf*tf + coef[2]*tf*tf*tf;

	// DEBUG INFO
	if(debug_velocity){
		
		cout<<"v0="<<v0<<"; vf="<<vf<<"; Sf="<<Sf<<"; a0="<<a0<<"; af="<<af<<endl;
		cout<<"coef = ["<<coef[0]<<", "<<coef[1]<<", "<<coef[2]<<", "<<coef[3]<<"]"<<endl;
		cout<<"S0="<<0<<", Send="<<Send<<endl;
		cout<<"V0="<<0<<", Vend="<<Vend<<endl<<endl;
	}

	cout<<"vend="<<Vend<<", vf="<<vf<<endl;
	assert((abs(Send-Sf)<0.05)&&"Error in spline coefficient calculation!");
	assert((abs(Vend-vf)<0.05)&&"Error in spline coefficient calculation!");
	return coef;
}

// Evaluate cubic polynomial representing the velocity profile
double getVelocity(const double& v0, const vector<double>& coef, const double& t){
	double v = v0 + coef[0]*t + coef[1]*pow(t,2) + coef[2]*pow(t,3);
	return v;
}

// Evaluate cubic polynomial representing the velocity profile
vector<double> getVelocityVector(const double& v0, const vector<double>& coef, const vector<double>& Tpath){
	vector<double> Vpath;
	for(auto it = Tpath.begin(); it!=Tpath.end(); ++it){
		double V = getVelocity(v0,coef,*it);
		Vpath.push_back(V);
	}
	return Vpath;
}


vector<double> getTimeVector(const vector<double>& coef, const double& t0, const double& v0, const double& a0, const double& af, const double& Sf, const int& N){
	double tf = coef[3];
    double res = Sf/N;
	// Generate a lookup table time<->distance that has much higher resolution than the path vector
	vector<double>	 Theap = LinearSpacedVector(t0,tf,5*N);
	vector<double> Sheap;
	for(auto it = Theap.begin(); it!=Theap.end(); ++it){
		double S = v0*(*it) + 0.5*coef[0]*(*it)*(*it) + 0.333333333333333333*coef[1]*(*it)*(*it)*(*it) + 0.25*coef[2]*(*it)*(*it)*(*it)*(*it);
		Sheap.push_back(S);
	}
	double Send =  v0*tf + 0.5*coef[0]*tf*tf + 0.333333333333333333*coef[1]*tf*tf*tf + 0.25*coef[2]*tf*tf*tf*tf;
	cout<<"---- IN TIME VECTOR GENERATION ----"<<endl;
	cout<<"coef = ["<<coef[0]<<", "<<coef[1]<<", "<<coef[2]<<", "<<coef[3]<<"]"<<endl;
	cout<<"Theap.front()="<<Theap.front()<<", back()="<<Theap.back()<<", Sheap.front()="<<Sheap.front()<<", back()="<<Sheap.back()<<endl;
	assert( (abs(Sheap.back()-Sf)<0.05) && "Error in spline coefficient calculation");
	
	// Get the path vector and match it to a time of the lookup table
	vector<double> Spath = LinearSpacedVector(0,Sf,N);
	vector<double> Tpath;
	assert( (Spath.size()==N) && "Wrong path dimension");
	for(auto it = Spath.begin(); it!=Spath.end(); ++it){
		for(int j = 0; j!=Sheap.size(); j++){
			if (Sheap[j]>=(*it)){
				Tpath.push_back(Theap[j]);
				break;
			}else if (j==(Sheap.size()-1)){
				ROS_WARN_STREAM("Sheap.back="<<Sheap.back()<<", Spath.back()="<<Spath.back());
				ROS_WARN_STREAM("Error in last point! Pushing back last element!");
				Tpath.push_back(Theap.back());
			}
		}
    }

	cout<<"Tpath=[";
	for(auto it = Tpath.begin(); it!=Tpath.end(); ++it){
		cout<<*it<<", ";
	}
	cout<<"]"<<endl;
	// for(int i = 0; i!=Spath.size(); i++){
	// 	cout<<"Spath="<<Spath[i]<<endl;
	// 	for(int j = 0; j!=Sheap.size(); j++){
	// 	 	if (abs(Sheap[j]-Spath[i])<res/8){
	// 			Tpath.push_back(Theap[j]);
    //             break;
	// 		}else if (j==(Sheap.size()-1)){
	// 			cout<<"Sp="<<Spath[j]<<"; Sh="<<Sheap.back()<<endl;

	// 			// assert(j!=(Sheap.size()-1));
	// 		}
			
    //     }
    // }
	cout<<"Tpath.size()="<<Tpath.size()<<"; N="<<N<<endl;
	assert(Tpath.size()==N);
	return Tpath;
}


// vector<double> getTimeVector(const vector<double>& coef, const double& t0, const double& v0, const double& a0, const double& af, const double& Sf, const int& N){
// 	double tf = coef[3];
// 	double res = Sf/(N-1);
// 	// Generate a lookup table time<->distance that has much higher resolution than the path vector
// 	vector<double> Theap = LinearSpacedVector(t0,tf,10*N);
// 	vector<double> Sheap;
// 	for(auto it = Theap.begin(); it!=Theap.end(); ++it){
// 		double S = v0*(*it) + 0.5*coef[0]*pow(*it,2) + (1/3)*coef[1]*pow(*it,3) + (1/4)*coef[2]*pow(*it,4);
// 		Sheap.push_back(S);
// 	}
// 	// Get the path vector and match it to a time of the lookup table
// 	vector<double> Spath = LinearSpacedVector(0,Sf,N);
// 	vector<double> Tpath;
// 	for(auto itp = Spath.begin(); itp!=Spath.end(); ++itp){
// 		for(int i = 0; i!=Sheap.size(); i++){
// 			if (abs((*itp)-Sheap[i])<(res/4)){
// 				Tpath.push_back(Theap[i]);
// 				break;
// 			}else if (i==(Sheap.size()-1)){
// 				cout<<"No solution found!"<<endl;
// 				cout<<"coef= ["<<coef[0]<<", "<<coef[1]<<", "<<coef[2]<<"]"<<endl;
// 				cout<<"tf="<<tf<<"; Sf="<<Sf<<endl;
// 				bool B1 = abs(Sheap.back()-Spath.back())<0.01;
// 				bool B2 = abs(Theap.back()-tf)<0.01;
// 				if (B1||B2){
// 					cout<<"B1="<<B1<<"B2="<<B2<<endl;
					
// 				}
// 				sleep(10);
// 			}
// 		}
// 	}
// 	if(Tpath.size()!=N){
// 		cout<<"Path size="<<Tpath.size()<<" N="<<N<<endl;
// 		cout<<"Sheap= ["<<Sheap.front()<<", "<<Sheap.back()<<endl;
// 		cout<<"Spath= ["<<Spath.front()<<", "<<Spath.back()<<endl;
// 	}
// 	assert(Tpath.size()==N);
// 	return Tpath;
// }