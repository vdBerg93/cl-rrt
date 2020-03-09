# CA-CL-RRT
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

### Installing Motion planner

```
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:vdBerg93/cl-rrt.git
cd .. && catkin_make
source devel/setup.bash
```
### Improving performance
To improve the performance of the planner, make sure to compile in release mode.
This will increase the number of nodes explored per second.
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Running the tests
1. roslaunch rrt rrt.launch


## Authors

* **Berend van den Berg** - [vdBerg93](https://github.com/vdBerg93)

