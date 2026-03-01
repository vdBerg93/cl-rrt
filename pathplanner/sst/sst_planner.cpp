/**
 * SST (Stable Sparse RRT) — C++ Implementation for Autonomous Driving
 * ====================================================================
 *
 * Algorithm: Li, Littlefield, Bekris (2015)
 * "Asymptotically Optimal Sampling-based Kinodynamic Planning"
 *
 * Features:
 *   - Dynamic bicycle model (6 states) with RK4 integration
 *   - KD-tree nearest neighbor search
 *   - Witness-based sparse tree with dominance pruning
 *   - Road-aware sampling strategy
 *   - PPM visualization output (no external dependencies)
 *
 * Compile:
 *   g++ -O2 -std=c++17 -o sst_planner sst_planner.cpp -lm
 *
 * Run:
 *   ./sst_planner
 *
 * Output:
 *   - Console: planning statistics and solution path
 *   - sst_tree.ppm: visualization of tree and solution
 */

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <unordered_map>
#include <vector>

// ═══════════════════════════════════════════════════════════════
// Configuration
// ═══════════════════════════════════════════════════════════════

namespace config {
    // Integration
    constexpr double DT          = 0.02;   // RK4 timestep (s)

    // Witness grid (asymmetric for road driving)
    constexpr double WITNESS_DX  = 5.0;    // coarse in x (forward)
    constexpr double WITNESS_DY  = 3.0;    // finer in y (lateral / lanes)
    constexpr double WITNESS_DV  = 4.0;    // velocity resolution (m/s)
    constexpr double WITNESS_DTH = 0.5;    // heading resolution (rad)

    // Propagation
    constexpr double PROP_DUR_MIN = 0.3;   // min propagation duration (s)
    constexpr double PROP_DUR_MAX = 0.8;   // max propagation duration (s)

    // Goal tolerance
    constexpr double GOAL_POS_TOL   = 5.0; // position (m)
    constexpr double GOAL_SPEED_TOL = 5.0; // speed (m/s)
    constexpr double GOAL_BIAS      = 0.20; // probability of sampling near goal
    constexpr double ROAD_BIAS      = 0.50; // probability of road-corridor sample

    // Feasibility (relaxed for planning — post-optimization enforces tighter limits)
    constexpr double MAX_LAT_ACCEL = 8.0;  // lateral acceleration limit (m/s^2)

    // Visualization
    constexpr int IMG_W = 1400;
    constexpr int IMG_H = 500;
}

// ═══════════════════════════════════════════════════════════════
// Math Utilities
// ═══════════════════════════════════════════════════════════════

inline double wrap_angle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    return a < 0 ? a + M_PI : a - M_PI;
}

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// ═══════════════════════════════════════════════════════════════
// Vehicle Parameters
// ═══════════════════════════════════════════════════════════════

struct VehicleParams {
    double L           = 2.7;       // wheelbase (m)
    double Lf          = 1.35;      // front axle to CG (m)
    double Lr          = 1.35;      // rear axle to CG (m)
    double m           = 1500.0;    // mass (kg)
    double Iz          = 2500.0;    // yaw inertia (kg·m²)
    double Cf          = 80000.0;   // front cornering stiffness (N/rad)
    double Cr          = 80000.0;   // rear cornering stiffness (N/rad)
    double max_steer   = 0.6109;    // max steering angle (rad) ≈ 35°
    double max_steer_rate = 0.6981; // max steering rate (rad/s) ≈ 40°/s
    double max_accel   = 3.0;       // max acceleration (m/s²)
    double max_decel   = -5.0;      // max deceleration (m/s²)
    double max_speed   = 20.0;      // max speed (m/s) ≈ 72 km/h
    double min_speed   = 0.0;       // min speed (m/s)
};

// ═══════════════════════════════════════════════════════════════
// State: [x, y, theta, v, delta, omega]
// ═══════════════════════════════════════════════════════════════

struct State {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;  // heading (rad)
    double v     = 0.0;  // longitudinal velocity (m/s)
    double delta = 0.0;  // steering angle (rad)
    double omega = 0.0;  // yaw rate (rad/s)

    using Array = std::array<double, 6>;

    Array to_array() const { return {x, y, theta, v, delta, omega}; }

    static State from_array(const Array& a) {
        return {a[0], a[1], a[2], a[3], a[4], a[5]};
    }
};

// Control: [acceleration, steering_rate]
struct Control {
    double a = 0.0;
    double delta_dot = 0.0;
};

// ═══════════════════════════════════════════════════════════════
// Trajectory: sequence of states from a propagation
// ═══════════════════════════════════════════════════════════════

struct Trajectory {
    std::vector<State::Array> states;
    Control control;
    double cost = 0.0;
    bool feasible = true;
};

// ═══════════════════════════════════════════════════════════════
// Dynamic Bicycle Model
// ═══════════════════════════════════════════════════════════════

class DynamicBicycleModel {
public:
    explicit DynamicBicycleModel(const VehicleParams& p) : p_(p) {}

    /**
     * State derivative: ds/dt = f(s, u)
     *
     * Uses kinematic model at low speed (< 2 m/s) for numerical
     * stability, dynamic model with linear tire forces at higher speed.
     */
    State::Array dynamics(const State::Array& s, const Control& u) const {
        const double x = s[0], y = s[1], theta = s[2];
        const double v = s[3], delta = s[4], omega = s[5];
        const double a = clamp(u.a, p_.max_decel, p_.max_accel);
        const double dd = clamp(u.delta_dot, -p_.max_steer_rate, p_.max_steer_rate);

        State::Array ds;

        if (std::abs(v) < 2.0) {
            // Kinematic bicycle model (low speed)
            double beta = std::atan2(p_.Lr * std::tan(delta), p_.L);
            ds[0] = v * std::cos(theta + beta);
            ds[1] = v * std::sin(theta + beta);
            ds[2] = v * std::cos(beta) * std::tan(delta) / p_.L;
            ds[3] = a;
            ds[4] = dd;
            ds[5] = ds[2]; // omega ≈ dtheta/dt
        } else {
            // Dynamic model with lateral tire forces
            double alpha_f = delta - (omega * p_.Lf) / std::max(std::abs(v), 0.1);
            double alpha_r = -(omega * p_.Lr) / std::max(std::abs(v), 0.1);

            double Fyf = p_.Cf * alpha_f;
            double Fyr = p_.Cr * alpha_r;

            ds[0] = v * std::cos(theta);
            ds[1] = v * std::sin(theta);
            ds[2] = omega;
            ds[3] = a;
            ds[4] = dd;
            ds[5] = (p_.Lf * Fyf * std::cos(delta) - p_.Lr * Fyr) / p_.Iz;
        }
        return ds;
    }

    /**
     * Forward-propagate using RK4 integration.
     * Returns trajectory with feasibility check.
     */
    Trajectory propagate(const State& state, const Control& control,
                         double duration) const
    {
        Trajectory traj;
        traj.control = control;
        traj.feasible = true;

        const int n_steps = std::max(1, static_cast<int>(duration / config::DT));
        traj.states.reserve(n_steps + 1);

        State::Array s = state.to_array();
        traj.states.push_back(s);

        double cost = 0.0;
        const Control clamped = {
            clamp(control.a, p_.max_decel, p_.max_accel),
            clamp(control.delta_dot, -p_.max_steer_rate, p_.max_steer_rate)
        };

        for (int i = 0; i < n_steps; ++i) {
            // RK4 integration
            auto k1 = dynamics(s, clamped);

            State::Array s2;
            for (int j = 0; j < 6; ++j) s2[j] = s[j] + 0.5 * config::DT * k1[j];
            auto k2 = dynamics(s2, clamped);

            for (int j = 0; j < 6; ++j) s2[j] = s[j] + 0.5 * config::DT * k2[j];
            auto k3 = dynamics(s2, clamped);

            for (int j = 0; j < 6; ++j) s2[j] = s[j] + config::DT * k3[j];
            auto k4 = dynamics(s2, clamped);

            for (int j = 0; j < 6; ++j) {
                s[j] += (config::DT / 6.0) *
                         (k1[j] + 2.0*k2[j] + 2.0*k3[j] + k4[j]);
            }

            // Enforce state constraints
            s[3] = clamp(s[3], p_.min_speed, p_.max_speed);
            s[4] = clamp(s[4], -p_.max_steer, p_.max_steer);
            s[2] = wrap_angle(s[2]);

            traj.states.push_back(s);

            // Cost: time + lateral acceleration penalty
            double lat_accel = std::abs(s[3] * s[5]);
            cost += config::DT * (1.0 + 0.1 * lat_accel * lat_accel);

            // Feasibility: lateral acceleration limit
            if (lat_accel > config::MAX_LAT_ACCEL) {
                traj.feasible = false;
                break;
            }
        }

        traj.cost = cost;
        return traj;
    }

private:
    VehicleParams p_;
};

// ═══════════════════════════════════════════════════════════════
// Rectangle Obstacle
// ═══════════════════════════════════════════════════════════════

struct AABB {
    double x_min, y_min, x_max, y_max;
};

// ═══════════════════════════════════════════════════════════════
// Environment
// ═══════════════════════════════════════════════════════════════

class Environment {
public:
    double x_min, x_max, y_min, y_max;
    std::vector<AABB> obstacles;
    double vehicle_radius = 1.8; // circle approximation

    Environment(double xlo, double xhi, double ylo, double yhi)
        : x_min(xlo), x_max(xhi), y_min(ylo), y_max(yhi) {}

    void add_obstacle(double x0, double y0, double x1, double y1) {
        obstacles.push_back({x0, y0, x1, y1});
    }

    /**
     * Check trajectory for collisions using swept circle approximation.
     * Returns true if the entire trajectory is collision-free.
     */
    bool is_collision_free(const std::vector<State::Array>& traj) const {
        const double r = vehicle_radius;
        for (const auto& s : traj) {
            double px = s[0], py = s[1];

            // Bounds check
            if (px - r < x_min || px + r > x_max ||
                py - r < y_min || py + r > y_max)
                return false;

            // Obstacle check: circle vs AABB
            for (const auto& obs : obstacles) {
                double cx = clamp(px, obs.x_min, obs.x_max);
                double cy = clamp(py, obs.y_min, obs.y_max);
                double dx = px - cx, dy = py - cy;
                if (dx*dx + dy*dy < r*r)
                    return false;
            }
        }
        return true;
    }
};

// ═══════════════════════════════════════════════════════════════
// SST Node
// ═══════════════════════════════════════════════════════════════

struct SSTNode {
    int id = -1;
    State state;
    double cost = 0.0;             // cost-to-come from root
    int parent_id = -1;
    std::vector<int> children;
    Trajectory trajectory;         // propagated segment leading here
    bool is_active = true;

    SSTNode() = default;
    SSTNode(int id, const State& s, double c, int parent)
        : id(id), state(s), cost(c), parent_id(parent) {}
};

// ═══════════════════════════════════════════════════════════════
// Witness Key (hash for unordered_map)
// ═══════════════════════════════════════════════════════════════

struct WitnessKey {
    int kx, ky, kv, kth;

    bool operator==(const WitnessKey& o) const {
        return kx == o.kx && ky == o.ky && kv == o.kv && kth == o.kth;
    }
};

struct WitnessKeyHash {
    size_t operator()(const WitnessKey& k) const {
        // FNV-1a style hash
        size_t h = 2166136261u;
        h ^= std::hash<int>()(k.kx); h *= 16777619u;
        h ^= std::hash<int>()(k.ky); h *= 16777619u;
        h ^= std::hash<int>()(k.kv); h *= 16777619u;
        h ^= std::hash<int>()(k.kth); h *= 16777619u;
        return h;
    }
};

// ═══════════════════════════════════════════════════════════════
// Simple KD-Tree for nearest-neighbor on active nodes
// ═══════════════════════════════════════════════════════════════
//
// Operates on 4D points: (x, y, theta_scaled, v_scaled)
// Rebuilt periodically for efficiency (SST modifies the active set).
// ═══════════════════════════════════════════════════════════════

class KDTree {
public:
    // Weights for state-space distance
    static constexpr double W_POS   = 1.0;
    static constexpr double W_THETA = 2.0;
    static constexpr double W_V     = 1.0;

    struct Point {
        double coords[4]; // x, y, theta*W_THETA, v*W_V
        int node_id;
    };

    void build(const std::vector<SSTNode>& nodes,
               const std::vector<int>& active_ids)
    {
        points_.clear();
        points_.reserve(active_ids.size());
        for (int id : active_ids) {
            const auto& s = nodes[id].state;
            points_.push_back({
                {s.x * W_POS, s.y * W_POS, s.theta * W_THETA, s.v * W_V},
                id
            });
        }
        if (points_.empty()) return;
        indices_.resize(points_.size());
        std::iota(indices_.begin(), indices_.end(), 0);
        build_recursive(0, static_cast<int>(points_.size()), 0);
    }

    int nearest(const State& query) const {
        if (points_.empty()) return -1;
        double q[4] = {
            query.x * W_POS, query.y * W_POS,
            query.theta * W_THETA, query.v * W_V
        };
        best_dist_ = std::numeric_limits<double>::max();
        best_id_ = -1;
        search_recursive(0, static_cast<int>(points_.size()), 0, q);
        return best_id_;
    }

private:
    std::vector<Point> points_;
    std::vector<int> indices_;
    mutable double best_dist_;
    mutable int best_id_;

    double dist_sq(const double a[4], const double b[4]) const {
        double d = 0;
        for (int i = 0; i < 4; ++i) {
            double di = a[i] - b[i];
            // Wrap theta dimension
            if (i == 2) {
                di = wrap_angle(di / W_THETA) * W_THETA;
            }
            d += di * di;
        }
        return d;
    }

    void build_recursive(int lo, int hi, int depth) {
        if (hi - lo <= 1) return;
        int axis = depth % 4;
        int mid = (lo + hi) / 2;
        std::nth_element(indices_.begin() + lo, indices_.begin() + mid,
                         indices_.begin() + hi,
                         [&](int a, int b) {
                             return points_[a].coords[axis] <
                                    points_[b].coords[axis];
                         });
        // Swap points to match index order
        // (simpler: just use indices for lookup)
        build_recursive(lo, mid, depth + 1);
        build_recursive(mid + 1, hi, depth + 1);
    }

    void search_recursive(int lo, int hi, int depth, const double q[4]) const {
        if (lo >= hi) return;
        int mid = (lo + hi) / 2;
        int idx = indices_[mid];
        double d = dist_sq(q, points_[idx].coords);
        if (d < best_dist_) {
            best_dist_ = d;
            best_id_ = points_[idx].node_id;
        }

        int axis = depth % 4;
        double diff = q[axis] - points_[idx].coords[axis];
        if (axis == 2) diff = wrap_angle(diff / W_THETA) * W_THETA;

        // Search nearer subtree first
        int first_lo  = diff <= 0 ? lo : mid + 1;
        int first_hi  = diff <= 0 ? mid : hi;
        int second_lo = diff <= 0 ? mid + 1 : lo;
        int second_hi = diff <= 0 ? hi : mid;

        search_recursive(first_lo, first_hi, depth + 1, q);

        // Check if farther subtree could contain closer point
        if (diff * diff < best_dist_) {
            search_recursive(second_lo, second_hi, depth + 1, q);
        }
    }
};

// ═══════════════════════════════════════════════════════════════
// SST Planner
// ═══════════════════════════════════════════════════════════════

class SST {
public:
    SST(const DynamicBicycleModel& model, const Environment& env,
        const VehicleParams& params, unsigned seed = 42)
        : model_(model), env_(env), params_(params), rng_(seed) {}

    // ─── Initialization ───
    void set_goal(const State& goal) { goal_ = goal; }

    void initialize(const State& start) {
        nodes_.clear();
        active_ids_.clear();
        witnesses_.clear();
        best_goal_id_ = -1;
        best_goal_cost_ = std::numeric_limits<double>::infinity();
        n_iterations_ = 0;
        n_propagations_ = 0;
        n_pruned_ = 0;
        n_collisions_ = 0;
        n_infeasible_ = 0;
        n_dominated_ = 0;
        goals_found_ = 0;

        // Create root node
        SSTNode root(0, start, 0.0, -1);
        nodes_.push_back(root);
        active_ids_.push_back(0);

        auto key = witness_key(start);
        witnesses_[key] = 0;

        kdtree_dirty_ = true;
    }

    // ─── One SST iteration ───
    bool iterate() {
        ++n_iterations_;

        // Rebuild KD-tree periodically
        if (kdtree_dirty_ || n_iterations_ % 50 == 0) {
            kdtree_.build(nodes_, active_ids_);
            kdtree_dirty_ = false;
        }

        // 1. Sample random target state
        State x_rand = sample_state();

        // 2. Find nearest active node
        int near_id = kdtree_.nearest(x_rand);
        if (near_id < 0) return false;

        // 3. Sample control and duration
        Control u = sample_control();
        double duration = uniform(config::PROP_DUR_MIN, config::PROP_DUR_MAX);

        // 4. Forward propagate
        ++n_propagations_;
        Trajectory traj = model_.propagate(nodes_[near_id].state, u, duration);

        if (!traj.feasible) {
            ++n_infeasible_;
            return false;
        }

        // 5. Collision check
        if (!env_.is_collision_free(traj.states)) {
            ++n_collisions_;
            return false;
        }

        // 6. Compute cost
        double new_cost = nodes_[near_id].cost + traj.cost;
        State new_state = State::from_array(traj.states.back());

        // 7. Witness cell check — CORE OF SST
        auto key = witness_key(new_state);
        auto it = witnesses_.find(key);

        if (it != witnesses_.end()) {
            int existing_id = it->second;
            if (nodes_[existing_id].cost <= new_cost) {
                ++n_dominated_;
                return false;  // existing representative is better
            }
            // New node wins: deactivate existing
            deactivate_node(existing_id);
        }

        // Create new node
        int new_id = static_cast<int>(nodes_.size());
        SSTNode new_node(new_id, new_state, new_cost, near_id);
        new_node.trajectory = std::move(traj);
        nodes_.push_back(std::move(new_node));
        nodes_[near_id].children.push_back(new_id);
        active_ids_.push_back(new_id);
        witnesses_[key] = new_id;
        kdtree_dirty_ = true;

        // 8. Check goal
        if (is_goal(new_state) && new_cost < best_goal_cost_) {
            best_goal_cost_ = new_cost;
            best_goal_id_ = new_id;
            ++goals_found_;
            return true;
        }
        return false;
    }

    // ─── Run planner with time budget ───
    bool plan(const State& start, double time_budget_ms,
              int max_iterations = 1000000)
    {
        initialize(start);

        auto t0 = std::chrono::high_resolution_clock::now();
        double elapsed_ms = 0;

        while (elapsed_ms < time_budget_ms && n_iterations_ < max_iterations) {
            iterate();

            // Check time every 100 iterations (avoid syscall overhead)
            if (n_iterations_ % 100 == 0) {
                auto now = std::chrono::high_resolution_clock::now();
                elapsed_ms = std::chrono::duration<double, std::milli>(now - t0).count();
            }
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        elapsed_ms_ = std::chrono::duration<double, std::milli>(t_end - t0).count();

        return best_goal_id_ >= 0;
    }

    // ─── Extract solution path ───
    std::vector<int> extract_path() const {
        std::vector<int> path;
        if (best_goal_id_ < 0) return path;
        int id = best_goal_id_;
        while (id >= 0) {
            path.push_back(id);
            id = nodes_[id].parent_id;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // ─── Print statistics ───
    void print_stats() const {
        printf("\n");
        printf("════════════════════════════════════════════════════════\n");
        printf("  SST Planning Results (C++)\n");
        printf("════════════════════════════════════════════════════════\n");
        printf("  Time elapsed:       %.1f ms\n", elapsed_ms_);
        printf("  Iterations:         %d\n", n_iterations_);
        printf("  Propagations:       %d\n", n_propagations_);
        printf("  Active nodes:       %zu\n", active_ids_.size());
        printf("  Total nodes:        %zu\n", nodes_.size());
        printf("  Pruned:             %d\n", n_pruned_);
        printf("  Collisions:         %d\n", n_collisions_);
        printf("  Infeasible:         %d\n", n_infeasible_);
        printf("  Dominated:          %d\n", n_dominated_);
        printf("  Witness cells:      %zu\n", witnesses_.size());
        printf("  Goals found:        %d\n", goals_found_);
        if (best_goal_id_ >= 0) {
            printf("  Best goal cost:     %.2f\n", best_goal_cost_);
        }
        printf("  Iterations/sec:     %.0f\n",
               n_iterations_ / (elapsed_ms_ / 1000.0));
        printf("════════════════════════════════════════════════════════\n");
    }

    // ─── Print solution path ───
    void print_path() const {
        auto path = extract_path();
        if (path.empty()) {
            printf("  No solution found.\n");
            return;
        }
        printf("\n  %4s %7s %7s %7s %6s %7s\n",
               "Node", "X", "Y", "θ°", "V", "Cost");
        printf("  ──────────────────────────────────────────────\n");
        for (size_t i = 0; i < path.size(); ++i) {
            const auto& n = nodes_[path[i]];
            printf("  %4zu %7.1f %7.1f %7.1f %6.1f %7.2f\n",
                   i, n.state.x, n.state.y,
                   n.state.theta * 180.0 / M_PI,
                   n.state.v, n.cost);
        }
    }

    // ─── Visualization ───
    void save_ppm(const char* filename) const;

    // Accessors
    const std::vector<SSTNode>& nodes() const { return nodes_; }
    const std::vector<int>& active_ids() const { return active_ids_; }
    int best_goal_id() const { return best_goal_id_; }
    const Environment& env() const { return env_; }

private:
    // ─── Witness key computation ───
    WitnessKey witness_key(const State& s) const {
        return {
            static_cast<int>(std::floor(s.x / config::WITNESS_DX)),
            static_cast<int>(std::floor(s.y / config::WITNESS_DY)),
            static_cast<int>(std::floor(s.v / config::WITNESS_DV)),
            static_cast<int>(std::floor(s.theta / config::WITNESS_DTH))
        };
    }

    // ─── Goal check ───
    bool is_goal(const State& s) const {
        double dx = s.x - goal_.x;
        double dy = s.y - goal_.y;
        return std::sqrt(dx*dx + dy*dy) < config::GOAL_POS_TOL &&
               std::abs(s.v - goal_.v) < config::GOAL_SPEED_TOL;
    }

    // ─── State sampling (road-aware) ───
    State sample_state() {
        double r = uniform(0, 1);

        if (r < config::GOAL_BIAS) {
            // Goal bias
            return {
                goal_.x + normal(0, 5),
                goal_.y + normal(0, 2),
                goal_.theta + normal(0, 0.2),
                goal_.v + normal(0, 2),
                0, 0
            };
        }
        else if (r < config::GOAL_BIAS + config::ROAD_BIAS) {
            // Road corridor bias: sample ahead of tree frontier
            double max_x = 0;
            for (int id : active_ids_) {
                max_x = std::max(max_x, nodes_[id].state.x);
            }
            return {
                uniform(max_x - 5, std::min(max_x + 25, env_.x_max)),
                uniform(env_.y_min + 3, env_.y_max - 3),
                normal(0, 0.3),
                uniform(3, params_.max_speed),
                0, 0
            };
        }
        else {
            // Uniform random
            return {
                uniform(env_.x_min, env_.x_max),
                uniform(env_.y_min, env_.y_max),
                uniform(-M_PI, M_PI),
                uniform(0, params_.max_speed),
                0, 0
            };
        }
    }

    // ─── Control sampling (biased for driving) ───
    Control sample_control() {
        double r = uniform(0, 1);
        Control u;

        if (r < 0.70) {
            // Gentle: near-straight driving
            u.a = normal(0.3, 0.8);
            u.delta_dot = normal(0, 3.0 * M_PI / 180.0);
        } else if (r < 0.90) {
            // Moderate: lane change
            u.a = normal(0, 1.5);
            u.delta_dot = normal(0, 10.0 * M_PI / 180.0);
        } else {
            // Aggressive: emergency
            u.a = uniform(params_.max_decel, params_.max_accel);
            u.delta_dot = uniform(-params_.max_steer_rate, params_.max_steer_rate);
        }

        u.a = clamp(u.a, params_.max_decel, params_.max_accel);
        u.delta_dot = clamp(u.delta_dot, -params_.max_steer_rate,
                            params_.max_steer_rate);
        return u;
    }

    // ─── Deactivate a node (SST dominance) ───
    void deactivate_node(int id) {
        nodes_[id].is_active = false;
        active_ids_.erase(
            std::remove(active_ids_.begin(), active_ids_.end(), id),
            active_ids_.end()
        );
        prune_node(id);
        kdtree_dirty_ = true;
    }

    // ─── Prune inactive leaf nodes (propagates upward) ───
    void prune_node(int id) {
        if (nodes_[id].is_active) return;

        // Check if any children are active or have children themselves
        bool has_active_descendant = false;
        for (int c : nodes_[id].children) {
            if (nodes_[c].is_active || !nodes_[c].children.empty()) {
                has_active_descendant = true;
                break;
            }
        }
        if (has_active_descendant) return;

        // Prune: remove from parent's children, then check parent
        int parent = nodes_[id].parent_id;
        if (parent >= 0) {
            auto& pc = nodes_[parent].children;
            pc.erase(std::remove(pc.begin(), pc.end(), id), pc.end());
            ++n_pruned_;
            prune_node(parent);
        }
    }

    // ─── Random utilities ───
    double uniform(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(rng_);
    }
    double normal(double mean, double stddev) {
        return std::normal_distribution<double>(mean, stddev)(rng_);
    }

    // ─── Members ───
    DynamicBicycleModel model_;
    Environment env_;
    VehicleParams params_;
    State goal_;

    std::vector<SSTNode> nodes_;
    std::vector<int> active_ids_;
    std::unordered_map<WitnessKey, int, WitnessKeyHash> witnesses_;
    KDTree kdtree_;
    bool kdtree_dirty_ = true;

    int best_goal_id_ = -1;
    double best_goal_cost_ = std::numeric_limits<double>::infinity();

    std::mt19937 rng_;
    double elapsed_ms_ = 0;

    // Statistics
    int n_iterations_   = 0;
    int n_propagations_ = 0;
    int n_pruned_       = 0;
    int n_collisions_   = 0;
    int n_infeasible_   = 0;
    int n_dominated_    = 0;
    int goals_found_    = 0;
};

// ═══════════════════════════════════════════════════════════════
// PPM Image Writer (no external dependencies)
// ═══════════════════════════════════════════════════════════════

struct Color {
    uint8_t r, g, b;
};

class Image {
public:
    Image(int w, int h) : w_(w), h_(h), data_(w * h, {240, 240, 240}) {}

    void set(int x, int y, Color c) {
        if (x >= 0 && x < w_ && y >= 0 && y < h_)
            data_[y * w_ + x] = c;
    }

    void fill_rect(int x0, int y0, int x1, int y1, Color c) {
        for (int y = std::max(0, y0); y <= std::min(h_-1, y1); ++y)
            for (int x = std::max(0, x0); x <= std::min(w_-1, x1); ++x)
                data_[y * w_ + x] = c;
    }

    // Bresenham line
    void line(int x0, int y0, int x1, int y1, Color c, int thickness = 1) {
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            for (int ty = -thickness/2; ty <= thickness/2; ++ty)
                for (int tx = -thickness/2; tx <= thickness/2; ++tx)
                    set(x0+tx, y0+ty, c);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    void circle(int cx, int cy, int r, Color c, bool filled = true) {
        for (int y = -r; y <= r; ++y)
            for (int x = -r; x <= r; ++x)
                if (filled ? (x*x + y*y <= r*r) : (std::abs(x*x + y*y - r*r) < 2*r))
                    set(cx + x, cy + y, c);
    }

    void save_ppm(const char* filename) const {
        FILE* f = fopen(filename, "wb");
        fprintf(f, "P6\n%d %d\n255\n", w_, h_);
        fwrite(data_.data(), 3, data_.size(), f);
        fclose(f);
    }

    int w() const { return w_; }
    int h() const { return h_; }

private:
    int w_, h_;
    std::vector<Color> data_;
};

// ─── SST::save_ppm implementation ───

void SST::save_ppm(const char* filename) const {
    const int W = config::IMG_W;
    const int H = config::IMG_H;
    const int margin = 40;

    // Coordinate transform: world → pixel
    double scale_x = (W - 2*margin) / (env_.x_max - env_.x_min);
    double scale_y = (H - 2*margin) / (env_.y_max - env_.y_min);
    double scale = std::min(scale_x, scale_y);

    auto to_px = [&](double wx, double wy) -> std::pair<int, int> {
        int px = margin + static_cast<int>((wx - env_.x_min) * scale);
        int py = H - margin - static_cast<int>((wy - env_.y_min) * scale);
        return {px, py};
    };

    Image img(W, H);

    // Background
    img.fill_rect(0, 0, W-1, H-1, {250, 250, 250});

    // Road surface
    auto [rx0, ry0] = to_px(env_.x_min, env_.y_min);
    auto [rx1, ry1] = to_px(env_.x_max, env_.y_max);
    img.fill_rect(rx0, ry1, rx1, ry0, {220, 220, 220});

    // Obstacles
    for (const auto& obs : env_.obstacles) {
        auto [ox0, oy0] = to_px(obs.x_min, obs.y_max);
        auto [ox1, oy1] = to_px(obs.x_max, obs.y_min);
        img.fill_rect(ox0, oy0, ox1, oy1, {80, 80, 80});
    }

    // Draw tree edges
    for (const auto& node : nodes_) {
        if (node.trajectory.states.size() < 2) continue;
        Color c = node.is_active ? Color{70, 130, 230} : Color{200, 200, 200};
        int thick = node.is_active ? 1 : 1;

        for (size_t i = 1; i < node.trajectory.states.size(); ++i) {
            auto [x0, y0] = to_px(node.trajectory.states[i-1][0],
                                   node.trajectory.states[i-1][1]);
            auto [x1, y1] = to_px(node.trajectory.states[i][0],
                                   node.trajectory.states[i][1]);
            img.line(x0, y0, x1, y1, c, thick);
        }
    }

    // Draw active nodes
    for (int id : active_ids_) {
        auto [px, py] = to_px(nodes_[id].state.x, nodes_[id].state.y);
        img.circle(px, py, 2, {30, 100, 200}, true);
    }

    // Draw solution path
    auto path = extract_path();
    if (!path.empty()) {
        for (int nid : path) {
            const auto& traj = nodes_[nid].trajectory;
            if (traj.states.size() < 2) continue;

            for (size_t i = 1; i < traj.states.size(); ++i) {
                double v = traj.states[i][3];
                // Velocity colormap: red (slow) → yellow → green (fast)
                double t = clamp(v / params_.max_speed, 0, 1);
                uint8_t r = static_cast<uint8_t>(255 * (1 - t));
                uint8_t g = static_cast<uint8_t>(200 * t);
                uint8_t b = 30;

                auto [x0, y0] = to_px(traj.states[i-1][0], traj.states[i-1][1]);
                auto [x1, y1] = to_px(traj.states[i][0], traj.states[i][1]);
                img.line(x0, y0, x1, y1, {r, g, b}, 3);
            }
        }

        // Heading arrows along path
        for (int nid : path) {
            auto [px, py] = to_px(nodes_[nid].state.x, nodes_[nid].state.y);
            double th = nodes_[nid].state.theta;
            int ax = px + static_cast<int>(12 * std::cos(th));
            int ay = py - static_cast<int>(12 * std::sin(th));
            img.line(px, py, ax, ay, {0, 0, 0}, 2);
            img.circle(px, py, 3, {255, 140, 0}, true);
        }
    }

    // Start marker
    auto [sx, sy] = to_px(nodes_[0].state.x, nodes_[0].state.y);
    img.circle(sx, sy, 8, {0, 180, 0}, true);

    // Goal marker
    auto [gx, gy] = to_px(goal_.x, goal_.y);
    // Draw goal circle (unfilled)
    int gr = static_cast<int>(config::GOAL_POS_TOL * scale);
    img.circle(gx, gy, gr, {220, 50, 50}, false);
    img.circle(gx, gy, 6, {220, 50, 50}, true);

    img.save_ppm(filename);
    printf("  Visualization saved to: %s\n", filename);
}

// ═══════════════════════════════════════════════════════════════
// Main: Urban Driving Scenario
// ═══════════════════════════════════════════════════════════════

int main() {
    printf("SST Motion Planner — C++ Autonomous Driving\n");
    printf("════════════════════════════════════════════════════════\n");

    // ── Vehicle ──
    VehicleParams vp;
    DynamicBicycleModel model(vp);

    // ── Environment: 70m road segment, 20m wide ──
    Environment env(0, 70, -10, 10);
    env.add_obstacle(0, 8, 70, 10);       // upper curb
    env.add_obstacle(0, -10, 70, -8);     // lower curb
    env.add_obstacle(22, -6, 26, -3);     // parked car (right lane)
    env.add_obstacle(42, 0, 46, 3);       // stalled vehicle (left lane)
    env.add_obstacle(55, -6, 59, -3);     // parked car (right lane)

    // ── Start and goal ──
    State start = {5.0, -3.0, 0.0, 8.0, 0.0, 0.0};
    State goal  = {65.0, -3.0, 0.0, 8.0, 0.0, 0.0};

    // ════════════════════════════════════════════════════════════
    // Run 1: Realistic planning budget (140ms)
    // ════════════════════════════════════════════════════════════
    printf("\n[Run 1] Planning with 140ms budget (production constraint)...\n");
    {
        SST sst(model, env, vp);
        sst.set_goal(goal);

        bool found = sst.plan(start, 140.0);
        sst.print_stats();

        if (found) {
            printf("\n  ✓ Solution found within 140ms!\n");
            sst.print_path();
        } else {
            printf("\n  ✗ No solution in 140ms.\n");
            printf("    (In production: use tree reuse across cycles)\n");
        }
    }

    // ════════════════════════════════════════════════════════════
    // Run 2: Extended budget for convergence analysis
    // ════════════════════════════════════════════════════════════
    printf("\n[Run 2] Planning with 500ms budget (convergence study)...\n");
    {
        SST sst(model, env, vp);
        sst.set_goal(goal);

        bool found = sst.plan(start, 500.0);
        sst.print_stats();

        if (found) {
            printf("\n  ✓ Solution found!\n");
            sst.print_path();
            sst.save_ppm("sst_tree.ppm");
        } else {
            printf("\n  ✗ No solution found.\n");
            sst.save_ppm("sst_tree.ppm");
        }
    }

    // ════════════════════════════════════════════════════════════
    // Run 3: Multiple seeds for success rate analysis
    // ════════════════════════════════════════════════════════════
    printf("\n[Run 3] Success rate analysis (20 seeds × 140ms)...\n");
    printf("  ────────────────────────────────────────\n");
    int successes = 0;
    double total_iters = 0;
    double total_cost = 0;
    std::vector<double> solve_times;

    for (int seed = 0; seed < 20; ++seed) {
        SST sst(model, env, vp, static_cast<unsigned>(seed * 137 + 7));
        sst.set_goal(goal);
        bool found = sst.plan(start, 140.0);

        int iters = 0;
        // Read iteration count from stats
        // (Hacky: count active nodes as proxy — actually let's just
        //  look at the node count)
        const auto& nodes = sst.nodes();
        int n_active = static_cast<int>(sst.active_ids().size());

        auto t_end = std::chrono::high_resolution_clock::now();

        if (found) {
            ++successes;
            double c = nodes[sst.best_goal_id()].cost;
            total_cost += c;
            printf("  Seed %2d: ✓ FOUND  active=%3d  total=%4zu  cost=%.1f\n",
                   seed, n_active, nodes.size(), c);
        } else {
            printf("  Seed %2d: ✗ miss   active=%3d  total=%4zu\n",
                   seed, n_active, nodes.size());
        }
    }

    printf("  ────────────────────────────────────────\n");
    printf("  Success rate:    %d/20 (%.0f%%)\n", successes, successes * 100.0 / 20);
    if (successes > 0) {
        printf("  Avg solve cost:  %.1f\n", total_cost / successes);
    }

    printf("\nDone.\n");
    return 0;
}
