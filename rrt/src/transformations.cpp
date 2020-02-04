/**************************************
 **** TRANSFORMATIONS OF 2D POINTS ****
 *************************************/

// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}

// Homogenous transformation from car to world
void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose){
	double Xw = cos(carPose[2])*Xc - sin(carPose[2])*Yc + carPose[0];
	double Yw = sin(carPose[2])*Xc + cos(carPose[2])*Yc + carPose[1];
	Xc = Xw; Yc = Yw;
}

// Find closest point on the road centerline arc
vector<double> findClosestPointOnArc(const double& Xcar, const double& Ycar, const vector<double>& Cxy){
    // x = arg min norm([Xcarar,Ycarar]-[xroad,yroad]) (solved symbolically in MATLAB)
	// Results are valid
	float t2 = abs(Cxy[0]);
	float t3 = pow(Cxy[0],3);
	float t4 = pow(Cxy[1],2);
	float t5 = Xcar*Cxy[0]*2.0;
	float t6 = Ycar*Cxy[0]*4.0;
	float t8 = Cxy[0]*Cxy[2]*4.0;
	float t11 = sqrt(3.0);
	float t7 = pow(t2,3);
	float t9 = 1.0/t3;
	float t10 = -t8;
	float t12 = Cxy[1]+t5;
	float t13 = pow(t12,2);
	float t14 = Cxy[1]*t7*9.0;
	float t15 = Xcar*Cxy[0]*t7*18;
	float t17 = t4+t6+t10-2.0;
	float t16 = t13*27;
	float t18 = pow(t17,3);
	float t19 = -t18;
	float t20 = t16+t19;
	float t21 = sqrt(t20);
	float t22 = t3*t11*t21;
	float t23 = t14+t15+t22;
	float t24 = t9*t23;
	double Xarc = ( pow(t24,1.0/3.0) *0.2403749283845681)/t2-Cxy[1]/(Cxy[0]*2.0)+1.0/ pow(Cxy[0],2)*t2*t17*1.0/ pow(t24,1.0/3.0)*0.3466806371753173;
    // y = Cxy[1]*x^2 + Cxy[0]*x + c0
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
	vector<double> result {Xarc,Yarc};
	return result;
}

// Transform a point from curved-frame to straight-frame
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs){
	vector<double> Parc = findClosestPointOnArc(Xcar,Ycar,Cxy);
	double Xarc{Parc[0]}, Yarc{Parc[1]};
    // ***** Use the previous point to calculate (S,rho) *****
    // Use continuous arc-length parametrization obtained by polynomial fitting
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));
    // half-plane test to determine sign
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    // if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
	if(Ycar<(Yarc - Xarc*(Cxy[1] + 2*Xarc*Cxy[0]) + Xcar*(Cxy[1] + 2*Xarc*Cxy[0]))){
		rho = -rho;
	}

    // ***** Calculate (x,y)_straight *****
    // Calculate heading of slope at (x=0)
    double theta = atan2(Cxy[1], 1);
    // Rotate (S,rho) with slope, translate with C0
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];
    // Pstraight = [cos(theta),-sin(theta);sin(theta),cos(theta)]*[S;rho] + [0;Cxy[2]];

	// ***** Update coordinates *****
	Xcar = Xstraight; Ycar = Ystraight;
}

// Transform a point from the curved-frame to straight-frame
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs){
    //##### Find point on straightened road #####
    double Xroads = (Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight)/(pow(Cxy[1],2) + 1);
    double Yroads = Cxy[2] + (Cxy[1]*(Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight))/(pow(Cxy[1],2) + 1);
    //##### Get (S,rho) ######
    double S = sqrt( pow(Xroads,2) + pow(Yroads-Cxy[2],2) );
    double rho = sqrt( pow(Xroads-Xstraight,2) + pow(Yroads-Ystraight,2) );
    if(Ystraight<(Cxy[1]*Xstraight + Cxy[2])){
        rho = -rho;
    }
    //##### Bend back to car coordinates #####
	double Xarc = -(Cxs[1] - sqrt(pow(Cxs[1],2) - 4*Cxs[0]*Cxs[2] + 4*Cxs[0]*S))/(2*Cxs[0]);
    // double Xarc = Csx[0]*pow(S,2) + Csx[1]*S + Csx[2];
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    // Calculate normal vector at point on arc
    double vx = 1;
    double vy = dydx;
    double L = sqrt ( pow(vx,2) + pow(vy,2) );
    double nx = -(1/L)*dydx;
    double ny = (1/L);
    // Calculate point in car coordinates
    double Xroadc = Xarc + nx*rho;
    double Yroadc = Yarc + ny*rho;
	// Update coordinates
	Xstraight = Xroadc; Ystraight = Yroadc;
}

/**************************************
 **** TRANSFORMATIONS OF STATES    ****
 *************************************/

void transformStateWorldToCar(state_type& state, const state_type& carPose){
	transformPointWorldToCar(state[0],state[1],carPose);
	state[2] -= carPose[2];
}
void transformStateCarToWorld(state_type& state, const state_type& carPose){
	transformPointCarToWorld(state[0],state[1],carPose);
	state[2] += carPose[2];
}

void transformStateCarToRoad(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	vector<double> Parc = findClosestPointOnArc(state[0],state[1],Cxy);
	double curvature = (2*Cxy[0])/ pow( ( pow(Cxy[1] + 2*Cxy[0]*Parc[0],2) + 1),(3/2));
	transformPoseCarToRoad(state[0],state[1],state[2],Cxy,Cxs);
	double delta = atan(curvature*veh.L); 						// Required steer angle to follow road curvature at x=0
	state[3] -= delta;											// Subtract steer angle to straighten states
}

void transformStateRoadToCar(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	transformPoseRoadToCar(state[0],state[1],state[2],Cxy,Cxs);
	vector<double> Parc = findClosestPointOnArc(state[0],state[1],Cxy);
	// Update steer angle	
	double curvature = (2*Cxy[0])/ pow( ( pow(Cxy[1] + 2*Cxy[0]*Parc[0],2) + 1),(3/2));
	double delta = atan(curvature*veh.L); 						// Required steer angle to follow road curvature at x=0
	state[3] += delta;											// Subtract steer angle to straighten states
}

/*******************************************************
 ****** POSE TRANSFORMATIONS ***************************
 ******************************************************/

vector<double> transformStateToLocal(const vector<double>& worldState){
	vector<double> carPose = worldState;
	carPose[0] = 0; carPose[1] = 0; carPose[2]=0;
	return carPose;
}

void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs){
	// Transform obstacle to straightened road
	vector<double> Parc = findClosestPointOnArc(Xcar,Ycar,Cxy);
	double Xarc{Parc[0]}, Yarc{Parc[1]};
    // ***** Use the previous point to calculate (S,rho) *****
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));

    // half-plane test to determine sign
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
		rho = -rho;
	}
	double theta = atan2(Cxy[1], 1); 	// Heading of straightened road
	double Hroad = atan2(dydx,1);		// Heading of road at arc point
	double Hstraight = wrapTo2Pi( (Hcar-Hroad) + theta); // Calculate straightened heading

    // ***** Calculate (x,y)_straight *****
    // Rotate (S,rho) with slope, translate with C0
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];

	// ***** Update coordinates *****
	Xcar = Xstraight; Ycar = Ystraight; Hcar = Hstraight;
	assert((0<=Hcar)&(Hcar<=(2*pi)));
}

void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs){
    //##### Find closest point on straightened road #####
	double Xroads = (Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight)/(pow(Cxy[1],2) + 1);
    double Yroads = Cxy[2] + (Cxy[1]*(Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight))/(pow(Cxy[1],2) + 1);
	//##### Get (S,rho) for straightened road ######
    double S = sqrt( pow(Xroads,2) + pow(Yroads-Cxy[2],2) );
    double rho = sqrt( pow(Xroads-Xstraight,2) + pow(Yroads-Ystraight,2) );
    // Do half-plane test to determine sign of rho
	if(Ystraight<(Cxy[1]*Xstraight + Cxy[2])){
        rho = -rho;
    }
    //##### Bend back to car coordinates #####
	double Xarc = -(Cxs[1] - sqrt(pow(Cxs[1],2) - 4*Cxs[0]*Cxs[2] + 4*Cxs[0]*S))/(2*Cxs[0]);	// Inverse of polyfit Cxs
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];									// Evaluate poly Cxy
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];	// Slope of curved road
	double HroadC = atan2(dydx,1);			// Heading of curved road
	double HroadS = atan2(Cxy[1],1); 		// Heading of straight road
	double Hcar = wrapTo2Pi( (Hstraight - HroadS) + HroadC);
    // Calculate normal vector at point on arc
    double vx = 1;
    double vy = dydx;
    double L = sqrt ( pow(vx,2) + pow(vy,2) );
    double nx = -(1/L)*dydx;
    double ny = (1/L);
    // Calculate point in car coordinates
    double Xroadc = Xarc + nx*rho;
    double Yroadc = Yarc + ny*rho;
	// Update coordinates
	Xstraight = Xroadc; Ystraight = Yroadc; Hstraight = Hcar;
}

/*******************************************
 ***** TRANSFORMATION OF PATHS (T,Ref) *****
 ******************************************/
void transformPathRoadToCar(vector<Path>& path, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	int iter{0};
	for(auto it = path.begin(); it!= path.end(); it++){
		// Transform the reference
		for(int i = 0; i != it->ref.x.size(); i++){
			transformPointRoadToCar(it->ref.x[i],it->ref.y[i],Cxy,Cxs);
		// Transform the trajectory
		}for(int i = 0; i != it->tra.size(); i++){
			transformStateRoadToCar(it->tra[i],Cxy,Cxs,veh);
		}
		iter++;
	}
}
void transformPathCarToWorld(vector<Path>& path, const vector<double>& worldState){
	for(auto it = path.begin(); it!= path.end(); it++){
		// Transform the reference
		for(int i = 0; i != it->ref.x.size(); i++){
			transformPointCarToWorld(it->ref.x[i],it->ref.y[i],worldState);
		// Transform the trajectory
		}for(int i = 0; i != it->tra.size(); i++){
			transformStateCarToWorld(it->tra[i],worldState);
		}
	}
}

void transformPathWorldToCar(vector<Path>& path, const vector<double>& carPose){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		// Transform the reference
		for(int i = 0; i!=(*itP).ref.x.size(); ++i){
			transformPointWorldToCar((*itP).ref.x[i], (*itP).ref.y[i], carPose);
		// Transform the state
		}for(int i = 0; i!=(*itP).tra.size(); ++i){
			transformStateWorldToCar((*itP).tra[i], carPose);
		}
	}
}

void transformPathCarToRoad(vector<Path>& path,const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		// Transform the reference
		for(int i = 0; i!=(*itP).ref.x.size(); ++i){
			transformPointCarToRoad((*itP).ref.x[i], (*itP).ref.y[i],Cxy, Cxs);
		// Transform the state
		}for(int i = 0; i!=(*itP).tra.size(); ++i){
			transformStateCarToRoad((*itP).tra[i], Cxy, Cxs, veh);
		}
	}
}

/* NODE TRANSFORMATIONS */
void transformNodesRoadToCar(vector<Node>& nodes, const vector<double> carState, const vector<double>& Cxy, const vector<double> Cxs, const Vehicle& veh){
	for(auto it = nodes.begin(); it!= nodes.end(); it++){
		// if(debug_mode){cout<<"Transforming node state..."<<endl;}
		transformStateRoadToCar(it->state,Cxy, Cxs, veh);
		for(int i = 0; i!= it->ref.x.size(); i++){
			// if(debug_mode){cout<<"Transforming reference point "<<i<<"/"<<(it->ref.x.size()-1)<<endl;}
			transformPointRoadToCar(it->ref.x[i], it->ref.y[i], Cxy, Cxs);
		}
		for(int j = 0; j!=it->tra.size(); j++){
			// if(debug_mode){cout<<"Transforming trajectory point "<<j<<"/"<<(it->tra.size()-1)<<endl;}
			transformStateRoadToCar(it->tra[j],Cxy,Cxs,veh);
		}
	}
}

void transformNodesCarToRoad(vector<Node>& nodes, const vector<double> carState, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	for(auto it = nodes.begin(); it!= nodes.end(); it++){
			transformStateCarToRoad(it->state,Cxy, Cxs, veh);
		for(int i = 0; i!= it->ref.x.size(); i++){
			transformPointCarToRoad(it->ref.x[i], it->ref.y[i], Cxy, Cxs);
		}
		for(int j = 0; j!=it->tra.size(); j++){
			// if(debug_mode){cout<<"Transforming trajectory point "<<j<<"/"<<(it->tra.size()-1)<<endl;}
			transformStateCarToRoad(it->tra[j],Cxy,Cxs,veh);
		}
					
	}
}

void transformNodesCarToworld(vector<Node>& nodes, const vector<double> carState){
	for(auto it = nodes.begin(); it!= nodes.end(); it++){
		transformStateCarToWorld(it->state,carState);
		// Loop through reference and transform
		for(int i = 0; i!=it->ref.x.size(); i++){
			transformPointCarToWorld(it->ref.x[i],it->ref.y[i],carState);
		}
		// Loop through trajectory and transform
		for(int j = 0; j!=it->tra.size(); j++){
			transformPointCarToWorld(it->tra[j][0], it->tra[j][1], carState);
		}
	}
}

void transformNodesWorldToCar(vector<Node>& nodes, const vector<double> carState){
	for(auto it = nodes.begin(); it!= nodes.end(); it++){
		transformStateWorldToCar(it->state,carState);
		// Loop through reference and transform
		for(int i = 0; i!=it->ref.x.size(); i++){
			transformPointWorldToCar(it->ref.x[i],it->ref.y[i],carState);
		}
		// Loop through trajectory and transform
		for(int j = 0; j!=it->tra.size(); j++){
			transformPointWorldToCar(it->tra[j][0], it->tra[j][1], carState);
		}
	}
}

// Rotate the velocity vector to car frame
void rotateVelocityVector(double& Vx, double& Vy, const double& angle){
	double X = cos(angle)*Vx + sin(angle)*Vy;
	double Y = -sin(angle)*Vx + cos(angle)*Vy;
	Vx = X; Vy = Y;
}

void transformVelocityToRoad(const double& x, const double& y, double& Vx, double& Vy, const vector<double>& Cxy){
	vector<double> Pclose = findClosestPointOnArc(x,y,Cxy);
	double dydx_road = 2*Cxy[0]*Pclose[0] + Cxy[1];
	double H_road = atan2(dydx_road,1);
	rotateVelocityVector(Vx, Vy, H_road);
}