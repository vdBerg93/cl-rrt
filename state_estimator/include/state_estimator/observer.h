#ifndef CONTROL_H
#define CONTROL_H

class Observer{
    private:
        ros::Publisher* ptrPub;
        vector<double> carState;
    public:
        Observer(ros::Publisher* pub): ptrPub(pub){
            vector<double> initState{0,0,0,0,0,0};
            carState = initState;
        }   
        void callbackOdometry(const nav_msgs::Odometry& msg);
        void callbackJoints(const sensor_msgs::JointState& msg);
        // void callbackSteer(const prius_msgs::Control& msg);
        void publishStates();
};

#endif
