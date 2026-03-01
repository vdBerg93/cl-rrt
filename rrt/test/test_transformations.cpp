#include <gtest/gtest.h>
#include "rrt/headers.h"
using namespace std;

// Tolerance for floating-point comparisons
static constexpr double kTol = 1e-6;
// Relaxed tolerance for road-frame transforms (arc-length polynomial approximation)
static constexpr double kRoadTol = 1e-3;

// Road coefficients for Car<->Road tests
// Road: y = 0.01*x^2  (gentle parabolic curve; Cxy[0] != 0 required by findClosestPointOnArc)
static const std::vector<double> kCxy = {0.01, 0.0, 0.0};
// Arc-length polynomial: S(x) ~ 0.0001*x^2 + 1.0*x  (Cxs[0] != 0 required by inverse)
static const std::vector<double> kCxs = {0.0001, 1.0, 0.0};

static Vehicle makeTestVehicle() {
    Vehicle veh;
    veh.setPrius();
    return veh;
}

// ============================================================
//  PointWorldCar
// ============================================================

TEST(PointWorldCar, Identity) {
    double x = 3.0, y = 4.0;
    VehicleState car(0, 0, 0, 0, 0, 0);
    transformPointWorldToCar(x, y, car);
    EXPECT_NEAR(x, 3.0, kTol);
    EXPECT_NEAR(y, 4.0, kTol);
}

TEST(PointWorldCar, PureTranslation) {
    double x = 5.0, y = 2.0;
    VehicleState car(1.0, 0.0, 0.0, 0, 0, 0);
    transformPointWorldToCar(x, y, car);
    EXPECT_NEAR(x, 4.0, kTol);
    EXPECT_NEAR(y, 2.0, kTol);
}

TEST(PointWorldCar, Rotation90) {
    double x = 1.0, y = 0.0;
    VehicleState car(0, 0, M_PI / 2, 0, 0, 0);
    transformPointWorldToCar(x, y, car);
    EXPECT_NEAR(x, 0.0, kTol);
    EXPECT_NEAR(y, -1.0, kTol);
}

TEST(PointWorldCar, RoundTrip) {
    double x0 = 7.3, y0 = -2.1;
    double x = x0, y = y0;
    VehicleState car(3.0, -1.0, 0.7, 0, 0, 0);
    transformPointWorldToCar(x, y, car);
    transformPointCarToWorld(x, y, car);
    EXPECT_NEAR(x, x0, kTol);
    EXPECT_NEAR(y, y0, kTol);
}

// ============================================================
//  PointCarWorld
// ============================================================

TEST(PointCarWorld, PureTranslation) {
    double x = 4.0, y = 2.0;
    VehicleState car(1.0, 0.0, 0.0, 0, 0, 0);
    transformPointCarToWorld(x, y, car);
    EXPECT_NEAR(x, 5.0, kTol);
    EXPECT_NEAR(y, 2.0, kTol);
}

// ============================================================
//  StateWorldCar
// ============================================================

TEST(StateWorldCar, HeadingSubtracted) {
    VehicleState state(5.0, 3.0, 1.0, 0.1, 10.0, 0.0);
    VehicleState car(0, 0, 0.3, 0, 0, 0);
    transformStateWorldToCar(state, car);
    EXPECT_NEAR(state.theta, 0.7, kTol);
}

TEST(StateWorldCar, RoundTrip) {
    VehicleState orig(5.0, 3.0, 1.0, 0.1, 10.0, 0.5);
    VehicleState state = orig;
    VehicleState car(2.0, -1.0, 0.5, 0, 0, 0);
    transformStateWorldToCar(state, car);
    transformStateCarToWorld(state, car);
    EXPECT_NEAR(state.x, orig.x, kTol);
    EXPECT_NEAR(state.y, orig.y, kTol);
    EXPECT_NEAR(state.theta, orig.theta, kTol);
}

// ============================================================
//  PointCarRoad  (curved road, round-trip)
// ============================================================

TEST(PointCarRoad, RoundTrip) {
    double x0 = 5.0, y0 = 0.5;
    double x = x0, y = y0;
    transformPointCarToRoad(x, y, kCxy, kCxs);
    transformPointRoadToCar(x, y, kCxy, kCxs);
    EXPECT_NEAR(x, x0, kRoadTol);
    EXPECT_NEAR(y, y0, kRoadTol);
}

// ============================================================
//  PoseCarRoad  (curved road, round-trip)
// ============================================================

TEST(PoseCarRoad, RoundTrip) {
    double x0 = 5.0, y0 = 0.5, h0 = 0.1;
    double x = x0, y = y0, h = h0;
    transformPoseCarToRoad(x, y, h, kCxy, kCxs);
    transformPoseRoadToCar(x, y, h, kCxy, kCxs);
    EXPECT_NEAR(x, x0, kRoadTol);
    EXPECT_NEAR(y, y0, kRoadTol);
    EXPECT_NEAR(h, h0, kRoadTol);
}

// ============================================================
//  StateCarRoad  (round-trip)
// ============================================================

TEST(StateCarRoad, RoundTrip) {
    Vehicle veh = makeTestVehicle();
    VehicleState orig(5.0, 0.5, 0.1, 0.02, 10.0, 0.0);
    VehicleState state = orig;
    transformStateCarToRoad(state, kCxy, kCxs, veh);
    transformStateRoadToCar(state, kCxy, kCxs, veh);
    EXPECT_NEAR(state.x, orig.x, kRoadTol);
    EXPECT_NEAR(state.y, orig.y, kRoadTol);
    EXPECT_NEAR(state.theta, orig.theta, kRoadTol);
    EXPECT_NEAR(state.delta, orig.delta, kRoadTol);
}

// ============================================================
//  PointVsPose — proves Point can safely delegate to Pose
// ============================================================

TEST(PointVsPose, CarToRoadSameXY) {
    double xp = 5.0, yp = 0.5;
    double xq = 5.0, yq = 0.5, hq = 0.0;
    transformPointCarToRoad(xp, yp, kCxy, kCxs);
    transformPoseCarToRoad(xq, yq, hq, kCxy, kCxs);
    EXPECT_NEAR(xp, xq, kTol);
    EXPECT_NEAR(yp, yq, kTol);
}

TEST(PointVsPose, RoadToCarSameXY) {
    // Get a valid road-frame point by transforming from car frame first
    double xp = 5.0, yp = 0.5;
    transformPointCarToRoad(xp, yp, kCxy, kCxs);

    // Now transform back via both Point and Pose variants
    double xr = xp, yr = yp;
    double xq = xp, yq = yp, hq = 0.3;
    transformPointRoadToCar(xr, yr, kCxy, kCxs);
    transformPoseRoadToCar(xq, yq, hq, kCxy, kCxs);
    EXPECT_NEAR(xr, xq, kTol);
    EXPECT_NEAR(yr, yq, kTol);
}

// ============================================================
//  PathWorldCar  (round-trip)
// ============================================================

TEST(PathWorldCar, RoundTrip) {
    VehicleState car(2.0, -1.0, 0.5, 0, 0, 0);

    Path seg;
    seg.ref.x = {1.0, 2.0, 3.0};
    seg.ref.y = {0.5, 1.0, 1.5};
    seg.ref.v = {10.0, 10.0, 10.0};
    seg.ref.dir = 1;
    seg.ref.aend = 0.0;
    seg.tra = {
        VehicleState(1.0, 0.5, 0.2, 0.0, 10.0, 0.0),
        VehicleState(2.0, 1.0, 0.3, 0.0, 10.0, 0.0)
    };
    std::vector<Path> path = {seg};
    auto orig = path;

    transformPathWorldToCar(path, car);
    transformPathCarToWorld(path, car);

    EXPECT_NEAR(path[0].ref.x[0], orig[0].ref.x[0], kTol);
    EXPECT_NEAR(path[0].ref.x[2], orig[0].ref.x[2], kTol);
    EXPECT_NEAR(path[0].ref.y[1], orig[0].ref.y[1], kTol);
    EXPECT_NEAR(path[0].tra[0].x, orig[0].tra[0].x, kTol);
    EXPECT_NEAR(path[0].tra[0].y, orig[0].tra[0].y, kTol);
    EXPECT_NEAR(path[0].tra[1].theta, orig[0].tra[1].theta, kTol);
}

// ============================================================
//  NodesWorldCar  (round-trip)
// ============================================================

TEST(NodesWorldCar, RoundTrip) {
    VehicleState car(2.0, -1.0, 0.5, 0, 0, 0);

    Node n;
    n.state = VehicleState(3.0, 1.0, 0.4, 0.01, 8.0, 0.0);
    n.parentID = 0;
    n.ref.x = {1.0, 2.0};
    n.ref.y = {0.5, 1.0};
    n.ref.v = {10.0, 10.0};
    n.ref.dir = 1;
    n.ref.aend = 0.0;
    n.costE = 0;
    n.costS = 0;
    n.goalReached = false;
    n.tra = {VehicleState(1.5, 0.6, 0.3, 0.0, 9.0, 0.0)};
    std::vector<Node> nodes = {n};
    auto orig = nodes;

    transformNodesWorldToCar(nodes, car);
    transformNodesCarToworld(nodes, car);

    EXPECT_NEAR(nodes[0].state.x, orig[0].state.x, kTol);
    EXPECT_NEAR(nodes[0].state.y, orig[0].state.y, kTol);
    EXPECT_NEAR(nodes[0].state.theta, orig[0].state.theta, kTol);
    EXPECT_NEAR(nodes[0].ref.x[0], orig[0].ref.x[0], kTol);
    EXPECT_NEAR(nodes[0].ref.y[1], orig[0].ref.y[1], kTol);
    EXPECT_NEAR(nodes[0].tra[0].x, orig[0].tra[0].x, kTol);
    EXPECT_NEAR(nodes[0].tra[0].theta, orig[0].tra[0].theta, kTol);
}

// ============================================================
//  VelocityRotation
// ============================================================

TEST(VelocityRotation, ZeroAngle) {
    double vx = 10.0, vy = 5.0;
    rotateVelocityVector(vx, vy, 0.0);
    EXPECT_NEAR(vx, 10.0, kTol);
    EXPECT_NEAR(vy, 5.0, kTol);
}

TEST(VelocityRotation, Rotate90) {
    double vx = 1.0, vy = 0.0;
    rotateVelocityVector(vx, vy, M_PI / 2);
    EXPECT_NEAR(vx, 0.0, kTol);
    EXPECT_NEAR(vy, -1.0, kTol);
}

// ============================================================
//  StateToLocal
// ============================================================

TEST(StateToLocal, ZerosPositionAndHeading) {
    VehicleState world(5.0, 3.0, 1.2, 0.05, 15.0, 1.0);
    VehicleState local = transformStateToLocal(world);
    EXPECT_DOUBLE_EQ(local.x, 0.0);
    EXPECT_DOUBLE_EQ(local.y, 0.0);
    EXPECT_DOUBLE_EQ(local.theta, 0.0);
}

TEST(StateToLocal, PreservesDeltaAndVelocity) {
    VehicleState world(5.0, 3.0, 1.2, 0.05, 15.0, 1.0);
    VehicleState local = transformStateToLocal(world);
    EXPECT_DOUBLE_EQ(local.delta, 0.05);
    EXPECT_DOUBLE_EQ(local.v, 15.0);
    EXPECT_DOUBLE_EQ(local.a, 1.0);
}
