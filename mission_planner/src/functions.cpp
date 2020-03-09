double convertQuaternionToEuler(geometry_msgs::Quaternion input);
double getGoalVelocity();
double getSpeedLimit();
bool waitForConfirmation();
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void transformPoseWorldToCar(vector<double>& state, const vector<double>& carPose);

struct MsgManager{
    void initializeGoal(){
        Vgoal = getGoalVelocity();
        ros::param::get("max_velocity",Vmax);
        cout<<"Maximum velocity = "<<Vmax<<" m/s"<<endl;
        assert(Vmax<=8.33); // Speed limit
        goalReceived =true;
    }
    MsgManager(): confirmed(0), goalReceived(0){
        //initializeGoal();
        carPose.push_back(0); carPose.push_back(0); carPose.push_back(0); carPose.push_back(0); 
    }
    vector<double> carPose;
    double Vgoal, Vmax;
    vector<double> goalW, goalC;
    bool confirmed, goalReceived;
    ros::Publisher* ptrPubMP;
    void stateCallback(const car_msgs::State& input);
    void goalCallback(geometry_msgs::PoseStamped input);
    void sendMotionRequest();
    void updateGoalCar();
};

void MsgManager::stateCallback(const car_msgs::State& input){
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
}

void MsgManager::updateGoalCar(){
    goalC = goalW;
    transformPoseWorldToCar(goalC,carPose);
    // goalC = {goalW[0]-carPose[0],goalW[1]-carPose[1],goalW[2]-carPose[2],goalW[3]};

}

void MsgManager::sendMotionRequest(){
    car_msgs::MotionRequest req;
    updateGoalCar();

    for(auto it = goalC.begin(); it!=goalC.end(); it++){
        req.goal.push_back(*it);
    }
    if(Vmax<0.01){
        ROS_ERROR_STREAM("Vmax parameter not set! Setting to 5...");
        Vmax = 5;
    }
    req.vmax = Vmax;
    req.bend = false;
    req.laneShifts.push_back(0);
    // assert( (req.Cxy.size()==3) && (req.Cxs.size()==3) && "Error in dimensions");
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