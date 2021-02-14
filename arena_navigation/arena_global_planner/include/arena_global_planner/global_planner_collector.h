
#ifndef _GLOBAL_PLANNER_COLLECTOR_H
#define _GLOBAL_PLANNER_COLLECTOR_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "arena_mapping/mapping.h"
#include <plan_visualization/planning_visualization.h>

#include "arena_path_search/astar.h"
#include "arena_path_search/kinodynamic_astar.h"

// b-spline
#include "arena_traj_planner/non_uniform_bspline.h"
#include "arena_traj_planner/bspline_optimizer.h"

#include "arena_traj_planner/uniform_bspline.h"
#include "arena_traj_planner/bspline_optimizer_astar.h"
#include "arena_traj_planner/polynomial_traj.h"
//
#include "arena_global_planner/plan_container.hpp"

class GlobalPlanner{
private:
  // ros node
  ros::NodeHandle node_;

  // subscriber
  ros::Subscriber goal_sub_, odom_sub_;

  // publisher
  ros::Publisher global_path_pub_;
  ros::Publisher astar_path_pub_,astar_traj_pub_;
  ros::Publisher kino_astar_path_pub_;
  ros::Publisher sample_path_pub_;
  ros::Publisher traj_pub_;
  ros::Publisher bspline_esdf_pub_,bspline_astar_pub_;

  ros::Publisher global_traj_pub_,global_traj_astar_pub_;
  


  // planner variables
  Eigen::Vector2d current_pt_;                                    // current pt
  Eigen::Quaterniond odom_orient_;                                // orient

  Eigen::Vector2d odom_pos_, odom_vel_;                           // odometry state
  Eigen::Vector2d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector2d end_pt_, end_vel_;                              // target state

  std::vector<Eigen::Vector2d> global_path_;

  // map & enviornment
  GridMap::Ptr grid_map_;     //EDTEnvironment::Ptr edt_environment_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  
  
  // planner objects
  bool use_astar_, use_kino_astar_,use_optimization_;
  Astar::Ptr global_planner_astar_;
  KinodynamicAstar::Ptr global_planner_kino_astar_;

  // traj param
  double ctrl_pt_dist_;
  double max_vel_;
  double max_acc_;

  // bspline 1
  NonUniformBspline position_traj_;
  BsplineOptimizer::Ptr bspline_optimizer_;

  // bspline 2
  BsplineOptimizerAstar::Ptr bspline_optimizer_rebound_; 
  int continous_failures_count_{0};

  void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector2d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

  bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector2d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);


  // flags
  bool have_odom_;

  PlanningVisualization::Ptr visualization_;
  
public:
  GlobalPlanner(){};
  ~GlobalPlanner(){};

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  void init(ros::NodeHandle & nh);

  void goalCallback(const geometry_msgs::PoseStampedPtr& msg);

  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  bool findPath_astar(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

  bool findPath_kino_astar(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc, Eigen::Vector2d end_pt, Eigen::Vector2d end_vel);

  bool planGlobalTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);

  void findPath(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc, Eigen::Vector2d end_pt, Eigen::Vector2d end_vel);

  void optimizePath(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives,std::vector<Eigen::Vector2d> &traj_pts);
  
  void optimizePath_Astar(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives,std::vector<Eigen::Vector2d> &traj_pts);
  
  void visualize_path(std::vector<Eigen::Vector2d> path, const ros::Publisher & pub);
  
  typedef std::shared_ptr<GlobalPlanner> Ptr;

};



#endif
