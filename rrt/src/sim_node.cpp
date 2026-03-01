#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <car_msgs/State.h>
#include <car_msgs/MotionRequest.h>
#include <car_msgs/Trajectory.h>
#include <cmath>
#include <vector>

static const double ROAD_RADIUS = 50.0;
static const double LANE_HALF_WIDTH = 1.8;
static const double LOOKAHEAD_ANGLE = 0.8; // rad (~25m ahead on circle)
static const double TARGET_SPEED = 5.0;

struct CarState {
    double x, y, theta, delta, v, a;
};

class SimNode {
public:
    SimNode(ros::NodeHandle& nh) {
        pub_carstate_ = nh.advertise<car_msgs::State>("/carstate", 1);
        pub_request_  = nh.advertise<car_msgs::MotionRequest>("/motionplanner/request", 1);
        pub_road_     = nh.advertise<visualization_msgs::MarkerArray>("/sim/road_markers", 1, true);
        pub_car_      = nh.advertise<visualization_msgs::Marker>("/sim/car_marker", 1);

        sub_trajectory_ = nh.subscribe("/sim/trajectory", 1, &SimNode::trajectoryCallback, this);

        // Start on circle at (R, 0), heading tangent (north)
        state_.x = ROAD_RADIUS;
        state_.y = 0.0;
        state_.theta = M_PI / 2.0;
        state_.delta = 0.0;
        state_.v = 0.0;
        state_.a = 0.0;

        traj_step_ = 0;
        has_trajectory_ = false;

        publishRoadMarkers();

        sim_timer_    = nh.createTimer(ros::Duration(0.04), &SimNode::simTimerCallback, this);    // 25 Hz
        replan_timer_ = nh.createTimer(ros::Duration(0.5),  &SimNode::replanTimerCallback, this); // 2 Hz
    }

private:
    ros::Publisher pub_carstate_, pub_request_, pub_road_, pub_car_;
    ros::Subscriber sub_trajectory_;
    ros::Timer sim_timer_, replan_timer_;
    tf::TransformBroadcaster tf_broadcaster_;

    CarState state_;
    car_msgs::Trajectory trajectory_;
    int traj_step_;
    bool has_trajectory_;

    void trajectoryCallback(const car_msgs::Trajectory::ConstPtr& msg) {
        trajectory_ = *msg;
        // Find closest point to current car position to avoid jumping back
        double bestDist = std::numeric_limits<double>::max();
        traj_step_ = 0;
        for (int i = 0; i < (int)trajectory_.x.size(); i++) {
            double d = std::pow(trajectory_.x[i] - state_.x, 2)
                     + std::pow(trajectory_.y[i] - state_.y, 2);
            if (d < bestDist) {
                bestDist = d;
                traj_step_ = i;
            }
        }
        has_trajectory_ = true;
        ROS_INFO("Received trajectory with %lu points, starting at step %d",
                 trajectory_.x.size(), traj_step_);
    }

    void simTimerCallback(const ros::TimerEvent&) {
        // Step through planned trajectory
        if (has_trajectory_ && traj_step_ < (int)trajectory_.x.size()) {
            state_.x     = trajectory_.x[traj_step_];
            state_.y     = trajectory_.y[traj_step_];
            state_.theta = trajectory_.theta[traj_step_];
            state_.delta = trajectory_.delta[traj_step_];
            state_.v     = trajectory_.v[traj_step_];
            state_.a     = trajectory_.a[traj_step_];
            traj_step_++;
        }
        // else: hold current state (coast until replan)

        publishCarState();
        broadcastTF();
        publishCarMarker();
    }

    void replanTimerCallback(const ros::TimerEvent&) {
        // Compute goal ahead on circle, in car-local frame
        double car_angle = std::atan2(state_.y, state_.x);
        double goal_angle = car_angle + LOOKAHEAD_ANGLE;

        // Goal in world frame (on circle, tangent heading)
        double gx_world = ROAD_RADIUS * std::cos(goal_angle);
        double gy_world = ROAD_RADIUS * std::sin(goal_angle);
        double gtheta_world = goal_angle + M_PI / 2.0; // tangent to circle

        // Transform to car-local frame
        double dx = gx_world - state_.x;
        double dy = gy_world - state_.y;
        double c = std::cos(-state_.theta);
        double s = std::sin(-state_.theta);
        double gx_local = c * dx - s * dy;
        double gy_local = s * dx + c * dy;
        double gtheta_local = gtheta_world - state_.theta;
        // Normalize angle to [-pi, pi]
        while (gtheta_local >  M_PI) gtheta_local -= 2.0 * M_PI;
        while (gtheta_local < -M_PI) gtheta_local += 2.0 * M_PI;

        car_msgs::MotionRequest req;
        req.goal = {gx_local, gy_local, gtheta_local, TARGET_SPEED};
        req.vmax = TARGET_SPEED;
        req.bend = true;
        req.Cxy = {1.0 / (2.0 * ROAD_RADIUS), 0.0, 0.0};
        req.Cxs = {1.0 / (4.0 * ROAD_RADIUS * ROAD_RADIUS), 1.0, 0.0};
        req.laneShifts = {0.0};

        pub_request_.publish(req);
        publishRoadMarkers();
    }

    void publishCarState() {
        car_msgs::State msg;
        msg.state = {state_.x, state_.y, state_.theta, state_.delta, state_.v, state_.a};
        pub_carstate_.publish(msg);
    }

    void broadcastTF() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(state_.x, state_.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, state_.theta);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

    void publishCarMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "car";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = state_.x;
        marker.pose.position.y = state_.y;
        marker.pose.position.z = 0.75;
        tf::Quaternion q;
        q.setRPY(0, 0, state_.theta);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 4.5; // length
        marker.scale.y = 1.8; // width
        marker.scale.z = 1.5; // height

        marker.color.r = 0.2;
        marker.color.g = 0.4;
        marker.color.b = 0.9;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.0);
        pub_car_.publish(marker);
    }

    void publishRoadMarkers() {
        visualization_msgs::MarkerArray markers;
        const double radii[] = {ROAD_RADIUS, ROAD_RADIUS - LANE_HALF_WIDTH, ROAD_RADIUS + LANE_HALF_WIDTH};
        const float colors[][3] = {{0.5, 0.5, 0.5}, {0.7, 0.7, 0.7}, {0.7, 0.7, 0.7}};

        for (int m = 0; m < 3; m++) {
            visualization_msgs::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = ros::Time::now();
            line.ns = "road";
            line.id = m;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = (m == 0) ? 0.15 : 0.08;
            line.color.r = colors[m][0];
            line.color.g = colors[m][1];
            line.color.b = colors[m][2];
            line.color.a = 1.0;
            line.lifetime = ros::Duration(0.0);

            for (int deg = 0; deg <= 360; deg++) {
                double angle = deg * M_PI / 180.0;
                geometry_msgs::Point p;
                p.x = radii[m] * std::cos(angle);
                p.y = radii[m] * std::sin(angle);
                p.z = 0.0;
                line.points.push_back(p);
            }
            markers.markers.push_back(line);
        }
        pub_road_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_node");
    ros::NodeHandle nh;
    SimNode sim(nh);
    ROS_INFO("Sim node running...");
    ros::spin();
    return 0;
}
