#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_


#include <Eigen/Eigen>
#include <vector>

#include <ros/ros.h>

#include <arena_traj_planner/non_uniform_bspline.h>


struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double local_traj_len_;                // local replanning trajectory length
  double ctrl_pt_dist;                   // distance between adjacient B-spline
                                         // control points
  double clearance_;
  int dynamic_;
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};


struct LocalTrajData {
  /* info of generated traj */

    int traj_id_;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    NonUniformBspline   position_traj_;
    NonUniformBspline   velocity_traj_;
    NonUniformBspline   acceleration_traj_;
    NonUniformBspline   yaw_traj_;
    NonUniformBspline   yawdot_traj_;
    NonUniformBspline   yawdotdot_traj_;

};

















#endif