// // void testSample(){
// //     Vehicle veh;
// //     veh.setTalos();
// //     geometry_msgs::Point sample = sampleAroundVehicle(veh);
// //     cout<<"x= "<<sample.x<<" y= "<<sample.y<<endl;
// // }

// // void testSorting(){
// //     Vehicle veh; veh.setTalos();
// //     vector<double> goalPose = {0,10,0,0};
// // 	vector<double> v1{0,0,0,0,0,0,0};
// // 	MyRRT RRT(v1,goalPose);
	
// // 	Node N2;
// // 	vector<double> v2{10,0,0,0,0,0,0};
// // 	N2.state = v2;
// //     geometry_msgs::Point sample = sampleAroundVehicle(veh);
// //     sample.x = 5; sample.y = 0;
// //     RRT.addNode(N2); // id=0, D = ~20
// //     RRT.addNode(N1); // id=1, D = 5
// //     RRT.addNode(N2); // id=2, D = ~20
// //     RRT.addNode(N1); // id=3, D = 5
// //     cout<<"size of tree: "<<RRT.tree.size()<<endl;
// // 	RRT.direction = 1;
// //     vector<int> sortedNodes = sortNodesExplore(RRT,sample);
// //     cout<<"sorted order (ID): ";
// //     for(int index = 0; index !=sortedNodes.size(); index++){
// //         cout<<sortedNodes[index]<<", ";
// //     }
// // }

// void testDubins(){
// 	Node N1;
// 	vector<double> v1{0,0,0,0,0,0,0};
// 	N1.state = v1;
// 	geometry_msgs::Point S;
// 	S.x = 5; S.y = 5; S.z = 0;
	
// 	// Timing variables
// 	clock_t tstart = clock();	double diff;

// 	float Dist;
// 	for(int i = 0; i<1000000; i++){
// 		Dist = dubinsDistance(S,N1,-1);
// 	}

// 	// Calculate timing
// 	clock_t tend = clock();
// 	double diffticks = tend-tstart;
// 	double diffms = (diffticks)/(CLOCKS_PER_SEC);///(1000));
// 	std::cout<<"time elapsed: "<<diffms<<endl;
// 	std::cout<<"Distance: "<<Dist<<endl;
// }

// MyReference testReference(){
// 	geometry_msgs::Point sample;
// 	sample.x = 10;
// 	sample.y = 3;
// 	Node N1;
// 	vector<double> S1 {0,0,0,0,0,0,0};
// 	vector<double> ref {0,0};
// 	N1.state = S1;
// 	N1.ref.x = ref;
// 	N1.ref.y = ref;
// 	signed int direction {1};
// 	MyReference reference = getReference(sample, N1, direction);
// 	cout<<"size: "<<reference.x.size()<<endl;
// 	int imax = reference.x.size();
// 	for(int index = 0; index != reference.x.size(); index++){
// 		cout<<"x="<<(reference.x[index])<<" y="<<(reference.y[index])<<endl;
// 	}
// 	return reference;
// }

// // void testSimulation(ros::Publisher* ptrPub){
// // 	geometry_msgs::Point sample;
// // 	sample.x = 20;
// // 	sample.y = 6;
// // 	Node N1;
// // 	//vector<double> S1 {0,0,0.2915,0,2,0,0};
// // 	vector<double> S1 {0,0,0,0,5,0,0};
// // 	vector<double> ref {0,0};
// // 	Vehicle veh; veh.setTalos();
// // 	N1.state = S1;
// // 	N1.ref.x = ref;
// // 	N1.ref.y = ref;
// // 	signed int direction {1};
// // 	MyReference reference = getReference(sample, N1, direction);

// // 	// ROS_INFO_STREAM("ref.x="<<reference.x.back()<<"ref.size()="<<reference.x.size());
// // 	// ROS_INFO_STREAM("sample.x="<<sample.x);

// // 	cout<<"Reference path generated..."<<endl;
// // 	Simulation sim(RRT,N1,reference,veh);
// // 	// visualize in Rviz
// // 	visualization_msgs::Marker msg = createReferenceMsg(0,reference);
// // 	ptrPub->publish(msg);// cout<<"published reference!"<<endl;
// // 	msg = createStateMsg(0,sim);
// // 	ptrPub->publish(msg);// cout<<"published states!"<<endl;
// // }

// bool test_angle(){
// 	Node node;
// 	node.ref.x.push_back(0); node.ref.y.push_back(0);
// 	node.ref.x.push_back(5); node.ref.y.push_back(5);
// 	node.ref.dir = 1;
// 	geometry_msgs::Point sample; sample.x = 7.5; sample.y = 5;
// 	// When this fails, use the vehicle heading
// 	double angPar = atan2(node.ref.y.back()-node.ref.y.front(),node.ref.y.back()-node.ref.x.front());
// 	double angNew = atan2(sample.y-node.ref.y.back(),sample.x-node.ref.x.back());
// 	//ROS_INFO_STREAM("In try: dy="<<dy<<", dx="<<dx<<", angNew="<<angNew);
// 	// }catch(int e){
// 	// 	ROS_INFO_STREAM("catch");
// 	// 	angPar = node.state[2]+(rrt.direction<0)*pi;
// 	// 	double angNew = atan2(sample.y-node.state[1],sample.x-node.state[0]);
// 	// }
// 	//ROS_INFO_STREAM("angNew="<<angNew);
// 	// Reverse direction when required
// 	angPar = angPar + (node.ref.dir*1<0)*pi;
// 	// Wrap to interval [0,2pi]
// 	angPar = wrapTo2Pi(angPar);
//     angNew = wrapTo2Pi(angNew);

// 	double D = angleDiff(angPar,angNew);
// 	bool pass = D<(pi/4);
// 	cout<<"angpar="<<angPar<<" angnew="<<angNew<<" diff="<<D<<" passed?"<<pass<<endl;
// 	getchar();
// }
// void testCollision(){
// 	cout<<"starting collision testing..."<<endl;
// 	//OBB2D OBS1, OBS2;
// 	OBB obs1(Vector2D(0,0),2,5,0);
// 	OBB obs2(Vector2D(3.55,0),2,5,1.57);
// 	OBB obs3(Vector2D(3.55,0),2,5,2);
// 	cout<<"\n Obstacle 1"<<endl;
// 	for(int i = 0; i<4; i++){	cout<<obs1.vertices[i].x<<","<<obs1.vertices[i].y<<endl;}
// 	cout<<"\n Obstacle 2"<<endl;
// 	for(int i = 0; i<4; i++){	cout<<obs2.vertices[i].x<<","<<obs2.vertices[i].y<<endl;}
// 	cout<<"\n Obstacle 3"<<endl;
// 	for(int i = 0; i<4; i++){	cout<<obs3.vertices[i].x<<","<<obs3.vertices[i].y<<endl;}
// 	// Timing variables
// 	clock_t tstart = clock();	double diff;
// 	bool should_be_col_free, should_be_col;
// 	// insert intersection check
// 	for(int i = 0; i<100000; i++){
// 		should_be_col_free = intersects(obs1,obs2);
// 		should_be_col = intersects(obs1,obs3);
// 	}
// 	// Calculate timing
// 	clock_t tend = clock();
// 	double diffticks = tend-tstart;
// 	double diffms = (diffticks)/(CLOCKS_PER_SEC);///(1000));
	
// 	// Print results
// 	std::cout<<"\n time elapsed: "<<diffms<<endl;
// 	cout<<"Collision free configuration. Collision occured? "<<should_be_col_free<<endl;
// 	cout<<"Collision configuration. Collision occured? "<<should_be_col<<endl;
// 	assert(should_be_col_free==false);
// 	assert(should_be_col==true);
// }

// void testController(const MyReference& ref){
// 	state_type x{0,0,0,0,2,0};
// 	Vehicle veh; veh.setTalos();
// 	Controller control(ref,x);
// 	ControlCommand ctrlCmd = control.getControls(ref,veh,x);
// 	ROS_INFO_STREAM("YM final"<<control.ym);
// 	assert(0.8<=control.ym); assert(1.1>=std::abs(control.ym));

// }