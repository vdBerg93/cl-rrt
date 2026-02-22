#ifndef GLOBALS_H
#define GLOBALS_H

// Debug and drawing flags
extern bool draw_tree;
extern bool draw_obs;
extern bool draw_final_path;
extern bool debug_mode;
extern bool debug_reference;
extern bool debug_velocity;
extern bool draw_states;
extern bool debug_sim;
extern bool commit_path;
extern bool obs_use_pred;
extern double Tcommit;

// Simulation and control parameters (loaded from ROS param server)
extern double sim_dt;
extern double ctrl_tla, ctrl_dla, ctrl_mindla, ctrl_dlavmin, ctrl_Kp, ctrl_Ki;
extern double ref_res, ref_int, ref_mindist, vmax, vgoal;
extern double ay_road_max;

// Per-query failure counters
extern int fail_iterlimit;
extern int fail_collision;
extern int fail_acclimit;
extern int sim_count;

#endif
