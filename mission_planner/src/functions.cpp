double convertQuaternionToEuler(geometry_msgs::Quaternion input);
double getGoalVelocity();
double getSpeedLimit();
bool waitForConfirmation();
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void transformPoseWorldToCar(vector<double>& state, const vector<double>& carPose);
// void sendMotionRequest(const ros::Publisher* ptrPubMP, const vector<double>& goal, const double& Vmax);
void generateArclengthParametrization(const vector<double>& Cxy, vector<double>& Cxs);

struct MsgManager{
    void initializeGoal(){
        goalW = initialGoal;
        Vgoal = getGoalVelocity();
        ros::param::get("max_velocity",Vmax);
        cout<<"Maximum velocity = "<<Vmax<<" m/s"<<endl;
        assert(Vmax<=8.33); // Speed limit
        goalReceived =true;
    }
    MsgManager(): confirmed(0), goalReceived(0){
        initializeGoal();
        carPose.push_back(0); carPose.push_back(0); carPose.push_back(0); carPose.push_back(0); 
    }
    vector<double> carPose;
    double Vgoal, Vmax;
    vector<double> goalW, goalC;
    bool confirmed, goalReceived;
    ros::Publisher* ptrPubMP;
    void stateCallback(const car_msgs::State& input);
    void goalCallback(geometry_msgs::PoseStamped input);
    void laneCallback(const car_msgs::LaneDet& input);
    vector<double> laneCxy, laneCxs;
    // void motionCallback(car_msgs::MotionResponse resp);
    void sendMotionRequest();
    void updateGoalCar();
};

void MsgManager::laneCallback(const car_msgs::LaneDet& input){
    laneCxy.clear();    laneCxy = input.Cxy;
    laneCxs.clear();    laneCxs = input.Cxs;
    ROS_INFO_STREAM("Updated lane markings.");
    ROS_INFO_STREAM("Cxy=["<<laneCxy[0]<<", "<<laneCxy[1]<<","<<laneCxy[2]<<"]");
    ROS_INFO_STREAM("Cxs=["<<laneCxs[0]<<", "<<laneCxs[1]<<","<<laneCxs[2]<<"]");
}



void MsgManager::stateCallback(const car_msgs::State& input){
    // vector<double> state = {input.pose.pose.position.x,input.pose.pose.position.y,convertQuaternionToEuler(input.pose.pose.orientation)};
    carPose=input.state;
}

void MsgManager::goalCallback(geometry_msgs::PoseStamped input){
    cout<<"Goal pose received! (x,y,theta,vel)"<<endl;
    Vgoal = getGoalVelocity();
    goalW = {input.pose.position.x,input.pose.position.y,convertQuaternionToEuler(input.pose.orientation), Vgoal};
    
    ros::param::get("max_velocity",Vmax);
    cout<<"Maximum velocity = "<<Vmax<<" m/s"<<endl;
    assert(Vmax<=8.33); // Speed limit
    goalReceived =true;
    // Vmax = getSpeedLimit();
    // sendMotionRequest(ptrPubMP, goalC, Vmax);
    // confirmed = waitForConfirmation();
}

// void MsgManager::motionCallback(car_msgs::MotionResponse resp){
    
// }

void MsgManager::updateGoalCar(){
    goalC = goalW;
    transformPoseWorldToCar(goalC,carPose);
    // goalC = {goalW[0]-carPose[0],goalW[1]-carPose[1],goalW[2]-carPose[2],goalW[3]};

}

void MsgManager::sendMotionRequest(){
    car_msgs::MotionRequest req;
    // updateGoalCar();
    if ( (laneCxy.size()==0) ){
        return;
    }
    double x_goal = 30;
    double y_goal = laneCxy[0]*pow(x_goal,2) + laneCxy[1]*x_goal + laneCxy[2];
    double dydx = 2*laneCxy[0]*x_goal + laneCxy[1];
    double h_goal = atan2(dydx,1);

    vector<double> goalC = {x_goal, y_goal, h_goal, 0};
    for(auto it = goalC.begin(); it!=goalC.end(); it++){
        req.goal.push_back(*it);
    }
    if(Vmax<0.01){
        ROS_ERROR_STREAM("Vmax parameter not set! Setting to 5...");
        Vmax = 5;
    }
    req.vmax = Vmax;
    req.bend = true;
    req.Cxy = laneCxy;
    req.Cxs = laneCxs;
    req.laneShifts.push_back(0);
    assert( (req.Cxy.size()==3) && (req.Cxs.size()==3) && "Error in dimensions");
    ptrPubMP->publish(req);
}

bool waitForConfirmation(){
    string userInput;
    string Y = "yes";
    string N = "no";
    while (1){
        cout<<"Start performing the motion plan? (yes/no) "<<endl;
        cin >> userInput;
        if (userInput==Y){
            return true;
        }
        if (userInput==N){
            return false;
        }
    }
}

double getGoalVelocity(){
    double Vgoal = 0;
    return Vgoal;
}
double getSpeedLimit(){
    double Vmax = 100000;
    double Vlimit = 30/3.6;
    while (Vmax>Vlimit){
        cout<<"Please enter the speed limit (max "<<Vlimit<<" m/s): ";
        cin >> Vmax;
    }
    return Vmax;
}

double convertQuaternionToEuler(geometry_msgs::Quaternion input){
    // Return the yaw angle from the input message
    double siny_cosp = 2 * (input.w * input.z + input.x * input.y);
    double cosy_cosp = 1 - 2 * (input.y * input.y + input.z * input.z);
    return atan2(siny_cosp, cosy_cosp);
}



void transformPoseWorldToCar(vector<double>& state, const vector<double>& carPose){
	transformPointWorldToCar(state[0],state[1],carPose);
	state[2] -= carPose[2];
}

// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}