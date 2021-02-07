#include "arena_global_planner/global_planner_collector.h"

void GlobalPlanner::init(ros::NodeHandle & nh){
  node_=nh;

  // map
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  // global planner
  
  nh.param("global_planner/use_astar", use_astar_, false);
  nh.param("global_planner/use_kino_astar", use_kino_astar_, false);
  nh.param("global_planner/use_optimization", use_optimization_, false);

  nh.param("b_spline/control_points_distance", ctrl_pt_dist_, 0.5);
  nh.param("b_spline/max_vel", max_vel_, 3.0);
  nh.param("b_spline/max_acc", max_acc_, 2.0);



  // use_astar, use_kino_astar
  if(use_astar_){
    //global_planner_type_="astar";
    global_planner_astar_.reset(new Astar);
    global_planner_astar_->setParam(nh);
    global_planner_astar_->setEnvironment(edt_environment_);
    global_planner_astar_->init();
    global_planner_astar_->reset();

  }

  if(use_kino_astar_){
    //global_planner_type_="kino_astar";
    global_planner_kino_astar_.reset(new KinodynamicAstar);
    global_planner_kino_astar_->setParam(nh);
    global_planner_kino_astar_->setEnvironment(edt_environment_);
    global_planner_kino_astar_->init();
    global_planner_kino_astar_->reset();
  }
  
  if (use_optimization_) {
      bspline_optimizer_.reset(new BsplineOptimizer);
      bspline_optimizer_->setParam(nh);
      bspline_optimizer_->setEnvironment(edt_environment_);
  }

  // odom
  have_odom_=false;

  // subscriber
  goal_sub_ =nh.subscribe("/goal", 1, &GlobalPlanner::goalCallback,this);
  odom_sub_ = nh.subscribe("/odometry/ground_truth", 1, &GlobalPlanner::odomCallback, this);

  // publisher
  astar_path_pub_ =node_.advertise<nav_msgs::Path>("astar_plan", 1);
  kino_astar_path_pub_ =node_.advertise<nav_msgs::Path>("kino_astar_plan", 1);
  sample_path_pub_ =node_.advertise<nav_msgs::Path>("sample_plan", 1);
  traj_pub_ =node_.advertise<nav_msgs::Path>("traj_plan", 1);
  
  visualization_.reset(new PlanningVisualization(nh));
  

}

void GlobalPlanner::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
  if(have_odom_==false) return;
      
  // start pt
  start_pt_  =  odom_pos_;
  start_vel_ =  odom_vel_;
  start_acc_.setZero();

  // end pt
  end_pt_(0) = msg->pose.position.x;    
  end_pt_(1) = msg->pose.position.y;
  end_vel_.setZero();

  // vis goal
  std::cout << "Goal set!" << std::endl;
  geometry_msgs::PoseStamped p;
  p.header=msg->header;
  p.pose=msg->pose;
  visualization_->drawGoal(p, 0.5, Eigen::Vector4d(1, 1, 1, 1.0));
  std::cout << "Goal vis!" << std::endl;

  // 
  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);

  // find path
  findPath( start_pt_, start_vel_, start_acc_, end_pt_, end_vel_ );
  
}

void GlobalPlanner::findPath( Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,  
                              Eigen::Vector2d end_pt, Eigen::Vector2d end_vel){
  // status
  int status;
  global_path_.clear();

  // astar
  if(use_astar_){
    global_planner_astar_->reset();
    status = global_planner_astar_->search( start_pt, end_pt, false,-1.0);
    //global_path_=global_planner_astar_->getPath();
    if(status==Astar::NO_PATH){
      std::cout << "[Astar replan]: Can't find path." <<std:: endl;
      return;
    }else{
      std::cout << "[Astar replan]: Astar search success."<< std::endl;
      visualize_path(global_planner_astar_->getPath(),astar_path_pub_);
    }
  }

  // kino star
  if(use_kino_astar_){
    // search
    global_planner_kino_astar_->reset();
    status = global_planner_kino_astar_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

    if (status == KinodynamicAstar::NO_PATH) {
      // search again
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

      if (status == KinodynamicAstar::NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return;

      } else {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    // global path
    global_path_ = global_planner_kino_astar_->getKinoTraj(0.01);
    
    // sample from global path
    double      ts = ctrl_pt_dist_ / max_vel_;
    std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
    global_planner_kino_astar_->getSamples(ts, point_set, start_end_derivatives);
    
    /* b-spline */
    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    //NonUniformBspline init(ctrl_pts, 3, ts);


    // bspline trajectory optimization
    ros::Time t1, t2;
    double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

    t1 = ros::Time::now();

    int cost_function = BsplineOptimizer::NORMAL_PHASE;//NORMAL_PHASE;

    if (status != KinodynamicAstar::REACH_END) {
      cost_function |= BsplineOptimizer::ENDPOINT;
    }

    ctrl_pts = bspline_optimizer_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

    t_opt = (ros::Time::now() - t1).toSec();





    // iterative time adjustment

    t1                    = ros::Time::now();
    NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

    double to = pos.getTimeSum();
    pos.setPhysicalLimits(max_vel_, max_acc_);
    bool feasible = pos.checkFeasibility(false);

    int iter_num = 0;
    while (!feasible && ros::ok()) {
      feasible = pos.reallocateTime();
      if (++iter_num >= 3) break;
    }

    double tn = pos.getTimeSum();

    std::cout << "[kino replan]: Reallocate ratio: " << tn / to << std::endl;
    if (tn / to > 3.0) ROS_ERROR("reallocate error.");

    t_adjust = (ros::Time::now() - t1).toSec();

    // save planned results
    position_traj_ = pos;

    // save traj pos
    std::vector<Eigen::Vector2d> traj_pts;
    double  tm, tmp;
    position_traj_.getTimeSpan(tm, tmp);
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::Vector2d pt = position_traj_.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }

    // save traj ctl pts
    // draw the control point

    Eigen::MatrixXd         ctrl_pts_draw = position_traj_.getControlPoint();
    std::vector<Eigen::Vector2d> ctp;
    nav_msgs::Path ctrl_points_path;
    ctrl_points_path.poses.resize(int(ctrl_pts_draw.rows()));
    

    for (int i = 0; i < int(ctrl_pts_draw.rows()); ++i) {
      Eigen::Vector2d pt = ctrl_pts_draw.row(i).transpose();
      ctp.push_back(pt);
      ctrl_points_path.poses[i].pose.position.x=pt(0);
      ctrl_points_path.poses[i].pose.position.y=pt(1);
    }

    
//     gui_path.poses.resize(path.size());
    visualization_->drawGlobalPath(ctrl_points_path,0.2, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));



    // get & visualize path
    visualize_path(global_path_,kino_astar_path_pub_);
    visualize_path(point_set,sample_path_pub_);
    visualize_path(traj_pts,traj_pub_);
    

  }
  
  
  
  

}

void GlobalPlanner::visualize_path(std::vector<Eigen::Vector2d> path, const ros::Publisher & pub){

  //create a path message
  ros::Time plan_time = ros::Time::now();
  std::string global_frame="/map";
  
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = global_frame;
  gui_path.header.stamp = plan_time;
  
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = path[i](0);    //world_x;
      pose.pose.position.y = path[i](1);    //world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      gui_path.poses[i]=pose;               //plan.push_back(pose);

      
      //gui_path.poses[i].header = gui_path.header;
      //gui_path.poses[i].pose.position.x=path[i](0);
      //gui_path.poses[i].pose.position.y=path[i](1);
      //gui_path.poses[i].pose.position.z=0.0;
      //std::cout<< "b="<<gui_path.poses[i].pose.position<<std::endl;
  }

  // draw global path
  //vis->drawGlobalPath(gui_path,0.2, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));
  pub.publish(gui_path);
}

// void GlobalPlanner::visualize_bspline(){
//   auto info = &planner_manager_->local_data_;

//     /* publish traj */
//     plan_manage::Bspline bspline;
//     bspline.order      = 3;
//     bspline.start_time = info->start_time_;
//     bspline.traj_id    = info->traj_id_;

//     Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

//     for (int i = 0; i < pos_pts.rows(); ++i) {
//       geometry_msgs::Point pt;
//       pt.x = pos_pts(i, 0);
//       pt.y = pos_pts(i, 1);
//       pt.z = pos_pts(i, 2);
//       bspline.pos_pts.push_back(pt);
//     }

//     Eigen::VectorXd knots = info->position_traj_.getKnot();
//     for (int i = 0; i < knots.rows(); ++i) {
//       bspline.knots.push_back(knots(i));
//     }

//     Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
//     for (int i = 0; i < yaw_pts.rows(); ++i) {
//       double yaw = yaw_pts(i, 0);
//       bspline.yaw_pts.push_back(yaw);
//     }
//     bspline.yaw_dt = info->yaw_traj_.getInterval();

//     bspline_pub_.publish(bspline);

//     /* visulization */
//     auto plan_data = &planner_manager_->plan_data_;
//     visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
//     visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
//                                 Eigen::Vector4d(1, 0, 0, 1));

//     return true;

//   } else {
//     cout << "generate new traj fail." << endl;
//     return false;
//   }
// }


// //clear the plan, just in case
//     plan.clear();
// geometry_msgs::PoseStamped goal_copy = goal;
//             goal_copy.header.stamp = ros::Time::now();
//             plan.push_back(goal_copy);

// ros::Time plan_time = ros::Time::now();
//     for (int i = path.size() -1; i>=0; i--) {
//         std::pair<float, float> point = path[i];
//         //convert the plan to world coordinates
//         double world_x, world_y;
//         mapToWorld(point.first, point.second, world_x, world_y);

//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = plan_time;
//         pose.header.frame_id = global_frame;
//         pose.pose.position.x = world_x;
//         pose.pose.position.y = world_y;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;
//         pose.pose.orientation.w = 1.0;
//         plan.push_back(pose);



// void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
//     if (!initialized_) {
//         ROS_ERROR(
//                 "This planner has not been initialized yet, but it is being used, please call initialize() before use");
//         return;
//     }

//     //create a message for the plan
//     nav_msgs::Path gui_path;
//     gui_path.poses.resize(path.size());

//     gui_path.header.frame_id = frame_id_;
//     gui_path.header.stamp = ros::Time::now();

//     // Extract the plan in world co-ordinates, we assume the path is all in the same frame
//     for (unsigned int i = 0; i < path.size(); i++) {
//         gui_path.poses[i] = path[i];
//     }

//     plan_pub_.publish(gui_path);
// }

int main(int argc, char **argv){
    
    ros::init(argc, argv, "sdf_map");
    std::cout<<"start"<<std::endl;
    ros::NodeHandle nh("~");

    GlobalPlanner::Ptr gp;

    gp.reset(new GlobalPlanner);
    gp->init(nh);

    ros::spin();
    return 0;
}















