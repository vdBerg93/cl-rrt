# *** ALSO BRANCH AVAILABLE WITH PERCEPTION AND CAR SIMULATOR ***

# CL-RRT
Closed-Loop Rapidly-exploring Random Tree, a ROS C++ implementation.
[Kuwata et al., 2009, "Real-time Motion Planning with Applications to Autonomous Urban Driving"](http://acl.mit.edu/papers/KuwataTCST09.pdf)

## Getting Started
This package contains:
* cl-rrt: Generate a motion plan using RRT with closed-loop prediction
* mission_planner: generates local goals for the planner
* state_estimator: estimate state of Prius vehicle
* car_msgs: contains all messages used by the package

### Prerequisites

* ROS Melodic with visualization stack
* vision_msgs
* robot_localization

### Installing Motion planner

```
mkdir ~/catkin_ws/src && cd ~/catkin_ws/src
git clone git@github.com:vdBerg93/cl-rrt.git
cd .. && catkin_make
source devel/setup.bash
```

## Description of the algorithm structure
A mission planner is required to define the motion queries for the planner. The mission planner must receive the vehicle state (to be written by the user) and a global goal (can be drawn in Rviz interface). 

The main function of the motion planner is located in motionplanner.cpp and is called MotionPlanner::planMotion. A brief summary of this function:
1. Update the planner inputs and convert them to car coordinate frame
2. Initialize the tree with the previous path
3. Expand the tree while t<t_max (see article hyperlink at top of readme for pseudocode)
4. Extract the best path
5. Do some post processing and publish the planned path


### Adding collision detection
Collision detection is done *during* closed-loop prediction. Collision is checked after every simulation step. When the collision detection fails, closed-loop prediction is aborted and returns fail.

The motion planner object keeps track of the obstacles. They are updated with a service at the start of each motion query with the updateObstacles() function. To enable collision detection, make sure to feed the obstacles through the following pipeline:
Motionplanner::planmotion(...)->expand_tree(...)-> Simulation sim(...) --> Simulation::Propagate(...)
Within the Propagate function, there is a function called myCollisionCheck(). Adjust the content of this function in collisioncheck.cpp to use your collision detection method.

The distance to the closest obstacle is used in the cost function. The cost function is evaluated in Simulation::Propagate(...) and initializeTree(...). Make sure to feed Dobs or alternatively, just feed Dobs=0 into the cost function.


### Adjusting the planning frequency
Both the mission- and motion planners are configured to use a 5 Hz update rate by default. 
If you want to change the update rate, make sure to adjust the following:
a) The rate of the mission planner node
b) The time limit in the expand_tree(...) loop in RRT->motionplanner

### Adjusting the vehicle model
The vehicle is modeled with a Kinematic bicycle model that is extended with understeer and actuator dynamics. The vehicle parameters can be adjusted in include/rrt/vehicle.h

### Improving performance
To improve the performance of the planner, make sure to compile in release mode.
This will increase the number of nodes explored per second.
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Running the planner
```
roslaunch rrt rrt.launch
```
This will apply all parameters, start rviz with a marker array on /tree_markerarray and a pose on /move_base_simple/goal, start the mission planner and finally start the RRT planner.


## Authors

* **Berend van den Berg** - [vdBerg93](https://github.com/vdBerg93)

