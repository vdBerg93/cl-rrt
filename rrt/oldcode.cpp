/*************************************
 **** RRT NODE (MAIN) ****************
 ************************************/
// // Testing new functions
// double Cxy[3] = {0.002,0.002,0.5};
// double Cxs[3] = {0.000392853825889,0.984741064715002,0.122939454240854};
// // double Csx[3] = {-0.000365738078703,1.013799256597537,-0.110077811210761};
// double X{95}, Y{16.5}; 
// double X0{95}, Y0{16.5}; 
// cout<<"X0="<<X0<<" Y0="<<Y0<<endl;
// transformCarToRoad(X,Y,Cxy,Cxs);
// cout<<"After straightening..."<<endl;
// cout<<"Xstraight="<<X<<" Ystraight="<<Y<<endl;
// transformRoadToCar(X,Y,Cxy,Cxs);
// cout<<"After bending..."<<endl;
// cout<<"Xend="<<X<<" Yend="<<Y<<endl;
// cout<<"Error="<<(sqrt( (X-X0)*(X-X0) + (Y-Y0)*(Y-Y0) ))<<endl;


/*************************************
 **** VELOCITY PROFILE GENERATION ****
 ************************************/

	// Loop through points and generate profile
	//L = 0;
	///***************************************
	//Cubic Bezier curve interpolation
	//***************************************/
	// ROS_WARN_STREAM("Still an error in this part!");
	// // Acceleration interpolation
	// vector<> t_a = LinearSpacedVector(0,1,(Daccel/res)+1);
	// vector<> cp_a{ v0, v0, Vcoast, Vcoast};
	// Vacc = bezierCurveInterpolation(cp_a,t_a);
	// // Deceleration interpolation
	// vector<> t_d = LinearSpacedVector(0,1,(Dbrake/res)+1);
	// vector<> cp_d{ Vcoast, Vcoast, vend, vend};
	// vector<> Vdec = bezierCurveInterpolation(cp_d,t_d);

	// FOR DEBUGGING ONLY
	// MaxAccel = 1.5*Vcoast/(1.5*Daccel-1.5*c1*Daccel);
	// MaxDecel = 1.5*-Vcoast/(1.5*Daccel-1.5*c1*Daccel);

	// SHORTENING (NOT REQUIRED?)
	// while (((vec_acc.size() + vec_coast.size()+vec_brake.size()) > Np)&&(!vec_coast.empty)){
	// 	vec_coast.pop_back();
	// }
	// while( ref.v.size()!=ref.x.size()){
	// 	if(ref.v.size()<Np){
	// 		// Append untill brake vector is empty
	// 		if (!vec_brake.empty()){
	// 			ref.v.push_back(vec_brake.front());		vec_brake.erase(vec_brake.begin()); continue;
	// 		// Append untill coast vector is empty
	// 		}else if (!vec_coast.empty()){
	// 			ref.v.insert(ref.v.begin(), vec_coast.back()); vec_coast.pop_back(); continue;
	// 		// Append until accelerate vector is empty
	// 		}else if (!vec_acc.empty()){
	// 			ref.v.insert(ref.v.begin(), vec_acc.back()); vec_acc.pop_back(); continue;
	// 		// Append the rest with v0 (not used)s
	// 		}else{
	// 			ref.v.insert(ref.v.begin(), v0); vec_acc.pop_back(); continue;
	// 		}	
	// 	}else{
	// 		ref.v.insert(ref.v.begin(), v0);
	// 	}
	// }



//while( length(ref.v)~=length(ref.x) )
    //if length(ref.v)<Np
      //  if ~isempty(vec_brake)
        //    ref.v = [ref.v,vec_brake(1)]; vec_brake(1)=[]; continue;
        //elseif ~isempty(vec_coast)
        //    ref.v = [vec_coast(end),ref.v]; vec_coast(end)=[]; continue;
       // elseif ~isempty(vec_acc)
       //     ref.v = [vec_acc(end),ref.v]; vec_acc(end)=[]; continue;
//         else
//             ref.v = [v0,ref.v]; continue;
//         end
//     else
//         ref.v = [v0,ref.v]; continue;
//     end
// end

// void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend){
// 	//ROS_ERROR_STREAM("FIX VEL PROFILE GENERAATION!");
// 	//ROS_WARN_STREAM("vmax="<<vmax<<" vend="<<vend);
// 	assert(ref.v.size()==0);
// 	// Slope shape configuration
// 	double a_acc = 0.75;	double a_dec = 0.75;	double tmin = 1; double v0 = node.state[4];
// 	// Total reference length
// 	double Lref = sqrt( pow(ref.x.back()-ref.x[0],2) + pow(ref.y.back()-ref.y[0],2) );
// 	// Total shape length from the first waypoint until end of reference
// 	double D = sqrt( pow(ref.x.back()-ref.x[IDwp],2) + pow(ref.y.back()-ref.y[IDwp],2) );
// 	// Check whether the maximum velocity can be reached for minimum time tmin
// 	double Da = (pow(vmax,2)-pow(v0,2))/(2*a_acc); 		// acceleration
// 	double Dc = vmax*tmin; 								// coasting
// 	double Db = (pow(vmax,2)-pow(vend,2))/(2*a_dec);	// braking
// 	//Db += -0.0252*pow(vmax,2) + 1.2344*vmax -0.5347; // compensate for overshoot
// 	// Boolean that states whether Vmax can be reached as coasting velocity
// 	bool BOOL = (Da+Dc+Db)<D;	double Vcoast;
// 	// If Vmax can be reached, Vcoast = Vmax
// 	if(BOOL){ 	
// 		Vcoast = vmax;
// 	}// Else, set Vcoast such that a minimum coasting time tmin is achieved
// 	else{		
// 			ROS_WARN_STREAM("in vel profile: CHeck this for validity!"); // implemented vend check if eq. is correct
// 			Vcoast = -(a_dec*(a_acc*tmin - sqrt((a_dec*pow(a_acc,2)*pow(tmin,2) + 2*D*pow(a_acc,2) + a_acc*pow(v0,2) + 2*a_dec*D*a_acc + a_dec*pow(v0,2) )/a_dec)))/(a_acc + a_dec);
// 	}	
// 	// Calculate profile distances etc
// 	double Daccel = (pow(Vcoast,2)-pow(v0,2))/(2*a_acc); 			// Distance to accelerate to Vcoast
// 	if(Daccel<0){Daccel = 0; Vcoast = v0;}; 						// Distance check
// 	double Dbrake = (pow(Vcoast,2)-pow(vend,2))/(2*a_dec);			// Braking distance
// 	//Dbrake = -0.0252*pow(Vcoast,2) + 1.2344*Vcoast -0.5347; 	// Additional braking distance to reduce controller overshoot
// 	double Dcoast = max(double(0),D-Daccel-Dbrake);					// Coasting distance
// 	double res = Lref/ref.x.size(); 								// ref_res of the reference path
// 	// Control points of the velocity profile
// 	double p3 = Lref-Dbrake;	// Start braking
// 	double p2 = p3-Dcoast; 		// Start coasting
// 	double p1 = p2-Daccel;	 	// Start accelerating
// 	//p1 = std::max(double(0),p1-2*res);
// 	// Loop through points and generate profile
// 	double L = 0;

// 	while(ref.v.size()!=ref.x.size()){
// 		L = L+res;
// 		if (L<p1)
// 		{
// 			// Before ramp up point, constant initial velocity.
// 			// Value is not really relevant since this part is skipped due to the lookahead distance
// 			ref.v.push_back(v0);	continue;
// 		}
// 		if (Daccel!=0){ // If an acceleration part exists, use this equation
// 			ref.v.push_back(v0 + ((p1<=L)&&(L<=p2))*(((L-p1)/Daccel)*(Vcoast-v0)) + (p2<L)*(Vcoast-v0) + (p3<L)*((L-p3)/Dbrake)*(-(Vcoast-vend)));
// 		}
// 		else if ((Daccel==0)&&(Dcoast>0)){ 	// If no acceleration but coasting for time t, use this equation
// 			//Dcoast = D-Dbrake;
// 			if(Dbrake>0){
// 				ref.v.push_back( v0 + (L>=p3)*((L-p3)/Dbrake)*(-v0) );
// 			}else{
// 				ref.v.push_back( Vcoast);
// 			}
// 		}
// 		else{ // Only braking
// 			ref.v.push_back(v0 - (Vcoast-vend)*L/Dbrake );
// 		}
// 	}
// //	cout<<"L(//)="<<(L/Lref)<<endl;

// 	if(debug_reference)ref length="<<ref.x.size();
// 		cout<<"IDwp="<<IDwp<<",\t Daccel="<<Daccel<<"\tDcoast="<<Dcoast<<"\tDbrake="<<Dbrake<<"\t D="<<D<<endl;
// 		cout<<"Reference path:"<<endl;
// 		for(vector<double>::iterator it = ref.x.begin(); it!=ref.x.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 		cout<<endl;
// 		for(vector<double>::iterator it = ref.y.begin(); it!=ref.y.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 		cout<<endl<<"Reference velocity profile:"<<endl;
// 		// For debugging
// 		for(vector<double>::iterator it = ref.v.begin(); it!=ref.v.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 		cout<<"lengths: "<<"D="<<D<<" Daccel="<<Daccel<<" Dbrake="<<Dbrake<<" Dcoast="<<Dcoast<<endl;
// 	}{		
// 		cout<<"
// 	assert(ref.v.size()==ref.x.size());
// 	if(debug_mode){
// 		cout<<"Generated velocity profile. "<<endl;
// 		}
// //	return;
// }


// void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend){
// 	// Slope shape configuration
// 	double a_acc = 0.75;	double a_dec = 0.75;	double tmin = 1; double v0 = node.state[4];
// 	// Total reference length
// 	double Lref = sqrt( pow(ref.x.back()-ref.x[0],2) + pow(ref.y.back()-ref.y[0],2) );
// 	// Total shape length from the first waypoint until end of reference
// 	double D = sqrt( pow(ref.x.back()-ref.x[IDwp],2) + pow(ref.y.back()-ref.y[IDwp],2) );
// 	// Check whether the maximum velocity can be reached for minimum time tmin
// 	double Da = (pow(vmax,2)-pow(v0,2))/(2*a_acc);
// 	double Dc = vmax*tmin;
// 	double Db = (pow(vmax,2)-pow(v0,2))/(2*a_dec) -0.0252*pow(vmax,2) + 1.2344*vmax -0.5347;
// 	bool BOOL = (Da+Dc+Db)<D;
	
// 	double Vcoast;
// 	if(BOOL){ 	// If Vcoast can be reached, Vcoast = Vmax
// 		Vcoast = vmax;
// 	}else{		// Else, Vcoast = ?
// 			Vcoast = -(a_dec*(a_acc*tmin - sqrt((a_dec*pow(a_acc,2)*pow(tmin,2) + 2*D*pow(a_acc,2) + a_acc*pow(v0,2) + 2*a_dec*D*a_acc + a_dec*pow(v0,2) )/a_dec)))/(a_acc + a_dec);
// 	}	

// 	ROS_INFO_STREAM("Vcoast="<<Vcoast);
// 	assert(!isnan(Vcoast));

// 	double Daccel = (pow(Vcoast,2)-pow(v0,2))/(2*a_acc); 		
// 	if(Daccel<0){Daccel = 0; Vcoast = v0;}; 				
// 	double Dbrake = (pow(Vcoast,2)-pow(v0,2))/(2*a_dec);
// 	Dbrake = Dbrake -0.0252*pow(Vcoast,2) + 1.2344*Vcoast -0.5347;
// 	double Dcoast = max(double(0),D-Daccel-Dbrake);
// 	ROS_WARN_STREAM("D="<<D<<" Daccel="<<Daccel<<" Dbrake="<<Dbrake<<" Dcoast="<<Dcoast);
// 	// Ramp up/down generation
// 	double res = Lref/ref.x.size();
// 	/***************************************
// 	Cubic Bezier curve interpolation
// 	***************************************/
// 	// ROS_WARN_STREAM("Still an error in this part!");
// 	// // Acceleration interpolation
// 	// vector<double> t_a = LinearSpacedVector(0,1,(Daccel/res)+1);
// 	// vector<double> cp_a{ v0, v0, Vcoast, Vcoast};
// 	// Vacc = bezierCurveInterpolation(cp_a,t_a);
// 	// // Deceleration interpolation
// 	// vector<double> t_d = LinearSpacedVector(0,1,(Dbrake/res)+1);
// 	// vector<double> cp_d{ Vcoast, Vcoast, vend, vend};
// 	// vector<double> Vdec = bezierCurveInterpolation(cp_d,t_d);

// 	/***************************************
// 	Linear interpolation
// 	****************************************/
// 	vector<double> Vacc;
// 	if(Daccel!=0)
// 	{
// 		vector<double> Vacc = LinearSpacedVector(v0,Vcoast,(Daccel/res)+1);
// 	}
// 	vector<double> Vdec = LinearSpacedVector(Vcoast,vend,(Dbrake/res)+1);
// 	assert(Dbrake<1000);
// 	assert(Vdec.size()<1000);
	
// 	// FOR DEBUGGING ONLY
// 	//double MaxAccel = 1.5*Vcoast/(1.5*Daccel-1.5*c1*Daccel);
// 	//double MaxDecel = 1.5*-Vcoast/(1.5*Daccel-1.5*c1*Daccel);

// 	//assert(Vacc.size()>0); assert(Vdec.size()>0);
// 	int ID_start = ref.x.size()-((D/res));
// 	int ID_accelerate = ref.x.size()-((D/res));
// 	int ID_coast = ID_accelerate+Vacc.size();
// 	int ID_brake = ref.x.size()-Vdec.size();
	
// 	 ROS_WARN_STREAM("IDa="<<ID_accelerate<<" IDc="<<ID_coast<<" IDb="<<ID_brake);
// 	 ROS_WARN_STREAM("sztotal="<<ref.x.size()<<" sza="<<Vacc.size()<<" szb="<<Vdec.size());

// 	cout<<endl;
// 	while(ref.v.size()<ID_start){
// 		ref.v.push_back(v0);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("Finished start part...");
// 	while(Vacc.size()!=0){
// 		ref.v.push_back(Vacc.front());
// 		Vacc.erase(Vacc.begin());
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("Finished acceleration");
// 	while((ref.v.size()-ID_brake)>0){
// 		ref.v.push_back(Vcoast);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("FInished coasting");
// 	for(int i = 0; i<=Vdec.size(); i++){
// 		ref.v.push_back(Vdec[i]);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	// while(Vdec.size()!=0){
// 	// 	ref.v.push_back(Vdec.front());
// 	// 	Vdec.erase(Vdec.begin());
// 	// 	cout<<ref.v.back()<<', ';
// 	// }
// 	ROS_INFO_STREAM("Finished braking");

// 	// // Loop through points and generate profile
// 	// for(int i = 0; i<ref.x.size(); i++){
// 	// 	//ROS_INFO_STREAM("Loop at i="<<i);
// 	// 	if(((i<(ID_accelerate))&&(Daccel!=0))){
// 	// 		//ROS_INFO_STREAM("Skipping...");	
// 	// 		ref.v.push_back(0); continue;
// 	// 	}
// 	// 	//else if(L<=p2){
// 	// 	else if((i<(ID_coast))&&(Dcoast!=0))
// 	// 	{
// 	// 		ref.v.push_back(Vacc[0]);
// 	// 		Vacc.erase(Vacc.begin());
// 	// 		//ROS_INFO_STREAM("Accelerating...");
// 	// 	}
// 	// 	//else if(L<p3){
// 	// 	else if(i<=(ID_brake)){
// 	// 		ref.v.push_back(Vcoast);
// 	// 		//ROS_INFO_STREAM("Coasting...");
// 	// 	}
// 	// 	else{
// 	// 		ref.v.push_back(Vdec[0]);
// 	// 		Vdec.erase(Vdec.begin());
// 	// 		//ROS_INFO_STREAM("Decellerating...");
// 	// 	}
		
// 	// }

// 	// For debugging
// 	// for(int i = 0; i<=ref.v.size();i++){
// 	// 	std::cout<<ref.v[i]<<" ,";
// 	// }
// 	ROS_ERROR_STREAM("Error inside this function!");
// 	//ROS_WARN_STREAM("Vacc sz="<<Vacc.size()<<" Vdec sz="<<Vdec.size());
// 	assert(ref.v.size()==ref.x.size());
// 	ROS_INFO_STREAM("TEST");
// 	return;
// }

//  */




/*********************************
 ******* MOTION PLANNER ***********
 ********************************/
/*
/////////////// MAIN WITH SERVICE /////////////////////
bool MotionPlanner::planMotion(car_msgs::planmotion::Request& req, car_msgs::planmotion::Response& resp){
	cout<<"----------------------------------"<<endl;
	cout<<"Received request, processing..."<<endl;
	// Update global variables
	cout<<"test0"<<endl;
	Vehicle veh; veh.setTalos();	
	cout<<"size state="<<req.state.size()<<endl;
	updateLookahead(req.state[3]);	updateReferenceResolution(req.state[3]); 
	vmax = req.vmax; vgoal = req.goal[3];

	// Actual motion plan querie as in pseudocode
	vector<double> state = getReqState(req);	vector<double> goal = getReqGoal(req); updateObstacles();

	// Build the tree
	vector<Node> tree = buildTree(veh, ptrPub,state,goal);
	vector<Node> bestPath = extractBestPath(tree,1);
	publishPlan(bestPath, resp);
	cout<<"Replying to request..."<<endl;
	cout<<"----------------------------------"<<endl;
	return true;
}

// Get state data from service message
vector<double> getReqState(const car_msgs::MotionRequest& req){
	vector<double> state;
	for(int i =0; i!=5; i++){
		state.push_back(req.state[i]);
	}
	return state;
}
void publishPlan(vector<Node> &path, car_msgs::planmotion::Response &resp){
	// Prepare message
	for(vector<Node>::iterator it = path.begin(); it!=path.end(); ++it){
		car_msgs::Reference ref; 	car_msgs::Trajectory tra;
		ref.dir = it->ref.dir;
		ref.x = it->ref.x;
		ref.y = it-> ref.y;
		ref.v = it->ref.v;
		resp.ref.push_back(ref);
		for(vector<vector<double>>::iterator it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
			tra.x.push_back( (*it2)[0]);
			tra.y.push_back( (*it2)[1]);
			tra.th.push_back( (*it2)[2]);
			tra.d.push_back( (*it2)[3]);
			tra.v.push_back( (*it2)[4]);
		}
		resp.tra.push_back(tra);
	}
	assert(resp.ref.size()==path.size());
}
*/

/*********************************************
 ****** RRT PLANNER **************************
 ********************************************/

// vector<Node> buildTree(Vehicle& veh, ros::Publisher* ptrPub, vector<double> startState, vector<double> goalPose, const vision_msgs::Detection2DArray& det){
// 	// MyRRT RRT(startState, goalPose);	// Initialize tree with first node
// 	Timer timer(100); 				// Initialize timer class with time in ms
	
// 	if(debug_mode){std::cout<< "Building tree..."<<endl;}
// 	for(int iter = 0; timer.Get(); iter++){
// 		expandTree(veh, RRT, ptrPub, det); 	// expand the tree
		
// 		if(debug_mode){	
// 			ROS_INFO_STREAM("Tree is size "<<RRT.tree.size()<<" after " <<iter<<" iterations...");
// 			cout<<"Press key to continue..."<<endl; getchar();
// 		}
// 	};
// 	cout<<"Build complete, final tree size: "<<RRT.tree.size()<<endl;
// 	return RRT.tree;
// };

// MyReference mergeNodes(vector<Node> Nodes){
// 	MyReference merged;
// 	for(vector<Node>::iterator it = Nodes.end(); it!=Nodes.begin(); --it){
// 		cout<<Nodes[0].ref.x.size()<<endl;
// 		//cout<<"size: "<<it->ref.x.size();
// 		for(int i = 0; i!= (it->ref.x.size()-1); i++){
// 			merged.x.push_back(it->ref.x[i]);
// 			merged.y.push_back(it->ref.x[i]);
// 			merged.v.push_back(it->ref.v[i]);
// 		}
// 		cout<<"pushed back ..."<<endl;
// 	}
// 	merged.x.push_back(Nodes.front().ref.x.back());
// 	merged.y.push_back(Nodes.front().ref.y.back());
// 	merged.v.push_back(Nodes.front().ref.v.back());
// 	return merged;
// 	//vector<vector<double>> mergedPath;
// 	// for(it; it!=Nodes.end(); ++it){
// 	// 	mergedPath[0].insert(mergedPath[0].end(), it->ref.x.begin(), it->ref.x.end());
// 	// 	mergedPath[1].insert(mergedPath[1].end(), it->ref.y.begin(), it->ref.y.end());
// 	// 	mergedPath[2].insert(mergedPath[2].end(), it->ref.v.begin(), it->ref.v.end());
// 	// 	if (it!=Nodes.end()){
// 	// 		mergedPath[0].pop_back();
// 	// 		mergedPath[1].pop_back();
// 	// 		mergedPath[2].pop_back();
// 	// 	}
// 	// }
// 	// return mergedPath;
// }