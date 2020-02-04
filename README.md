# CA-CL-RRT
Curvature-Aware Closed-Loop Rapidly-exploring Random Tree, a ROS C++ implementation.
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


## Authors

* **Berend van den Berg** - [vdBerg93](https://github.com/vdBerg93)

