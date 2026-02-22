#ifndef HEADERS_H
#define HEADERS_H

// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vision_msgs/Detection2DArray.h>


// Include messages
#include "car_msgs/getobstacles.h"
#include "car_msgs/MotionRequest.h"
#include "car_msgs/MotionResponse.h"
#include "car_msgs/resetplanner.h"
#include "car_msgs/State.h"
#include "car_msgs/Trajectory.h"
#include "car_msgs/MotionPlan.h"
#include "car_msgs/Obstacle2D.h"
#include "car_msgs/Reference.h"

// Include header files (order matters: each header depends on those above it)
#include "rrt/functions.h"       // no rrt deps
#include "rrt/vehicle.h"         // no rrt deps
#include "rrt/datatypes.h"       // no rrt deps
#include "rrt/collision.h"       // no rrt deps
#include "rrt/rrtplanner.h"      // defines MyReference, Node, MyRRT
#include "rrt/controller.h"      // needs MyReference
#include "rrt/simulation.h"      // needs Controller, MyRRT
#include "rrt/motionplanner.h"   // needs MyReference, Node -> defines Path
#include "rrt/transformations.h" // needs Path, Node, MyReference

#endif
