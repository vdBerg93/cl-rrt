# CL-RRT
Curvature-Aware Closed-Loop Rapidly-exploring Random Tree, a ROS C++ implementation.
Package contains the motion planner with obstacle detection and pedestrian tracking.
The world contains moving pedestrians, which can be controlled by publishing the positions to the ped_link# topic.
This can be done with the pedsim_ros package, or the spencer people simulations package.

## Getting Started
This package contains:
* ca-cl-rrt: Generate a motion plan using RRT with closed-loop prediction
* mission_planner: generates local goals for the planner
* obstacle_tracker: track pedestrian position
* pcl_converter: use pointcloud detections instead of Gazebo tracking of 'obstacle_tracker'
* state_estimator: estimate state of Prius vehicle
* road_perception_simulator: generate lane centerline detections
* car_msgs: contains all messages used by the package

### Prerequisites

* ROS Melodic with visualization stack
* prius_simulator package
* (optional) LMPCC, lmpcc_obstacle_feed
* (optional) pedsim_ros
* vision_msgs
* robot_localization

### Installing Motion planner

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src

git clone git@github.com:vdBerg93/prius_simulator.git
git clone git@github.com:vdBerg93/ca-cl-rrt.git

catkin_make
```
### Installing LMPCC & moving pedestrians

```
git clone -b four_persons https://github.com/vdBerg93/pedsim_ros.git
git clone https://github.com/bbrito/lmpcc_msgs.git
git clone -b rrt https://gitlab.tudelft.nl/bdebrito/lmpcc_obstacle_feed.git 
git clone -b rrt_integration https://gitlab.tudelft.nl/bdebrito/lmpcc.git

catkin_make
```
## Running the tests
1. start the world, roslaunch rrt twopedcurved.launch
2. start the rrt, roslaunch rrt rrt.launch
3. (optional) start the lmpcc, roslaunch lmpcc lmpcc.launch
4. (optional) start lmpcc with rqt_reconfigure, rosrun rqt_reconfigure rqt_reconfigure


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
Obstacles are modeled as 2-D Oriented Bounding Boxes. Velocities are estimated and provided to the planner.
They are updated with a service at the start of each motion query with the updateObstacles() function. 


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
## Authors

* **Berend van den Berg** - [vdBerg93](https://github.com/vdBerg93)

