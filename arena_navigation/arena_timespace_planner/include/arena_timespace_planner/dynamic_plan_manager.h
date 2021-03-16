#ifndef _DYNAMIC_PLAN_MANAGER_H_
#define _DYNAMIC_PLAN_MANAGER_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>

// msgs or srvs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>          // visulization
//#include <arena_plan_msgs/MakeGlobalPlan.h>     // arena plan msg

// mapping
#include <arena_mapping/mapping.h>

// bspline & polynomial
#include <arena_traj_planner/bspline/uniform_bspline.h>
#include <arena_traj_planner/polynomial/polynomial_traj.h>

// global planner
#include <arena_path_search/kinodynamic_astar.h>
#include <arena_traj_planner/bspline_opt/bspline_optimizer_esdf.h>

// mid planner
#//include <arena_dynamic_channel/timed_astar.h>

// local planner/ traj optimizer
#include <arena_traj_planner/bspline_opt/bspline_optimizer.h>

// data container
#include <arena_timespace_planner/dynamic_plan_container.hpp>


class DynPlanManager{
private:
    ros::NodeHandle node_;

    // global planer
    KinodynamicAstar::Ptr global_planner_kino_astar_;

    // mid planner
    
    // global traj optimizer
    BsplineOptimizerESDF::Ptr bspline_optimizer_esdf_;

    // local traj optimizer
    BsplineOptimizer::Ptr bspline_optimizer_;
    int continous_failures_count_{0};

    
    

public: 
    DynPlanManager();
    ~DynPlanManager();

    PlanParameters pp_;
    GridMap::Ptr grid_map_;

    GlobalData global_data_;
    MidData mid_data_;
    
    void initPlanModules(ros::NodeHandle &nh);


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
















#endif