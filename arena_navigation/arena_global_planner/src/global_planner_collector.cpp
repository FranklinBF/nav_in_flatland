#include "arena_global_planner/global_planner_collector.h"

void GlobalPlanner::init(ros::NodeHandle & nh){
  node_=nh;

  // map
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);

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
    global_planner_astar_->setEnvironment(grid_map_);
    global_planner_astar_->init();
    global_planner_astar_->reset();

  }

  if(use_kino_astar_){
    //global_planner_type_="kino_astar";
    global_planner_kino_astar_.reset(new KinodynamicAstar);
    global_planner_kino_astar_->setParam(nh);
    global_planner_kino_astar_->setEnvironment(grid_map_);
    global_planner_kino_astar_->init();
    global_planner_kino_astar_->reset();
  }
  
  if (use_optimization_) {
      bspline_optimizer_.reset(new BsplineOptimizerESDF);
      bspline_optimizer_->setParam(nh);
      bspline_optimizer_->setEnvironment(grid_map_);
  }

  if (use_optimization_) {
      bspline_optimizer_rebound_.reset(new BsplineOptimizerAstar);
      bspline_optimizer_rebound_->setParam(nh);
      bspline_optimizer_rebound_->setEnvironment(grid_map_);
      bspline_optimizer_rebound_->a_star_.reset(new AStar);
      bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));
  }



  // odom
  have_odom_=false;

  // subscriber
  goal_sub_ =nh.subscribe("/goal", 1, &GlobalPlanner::goalCallback,this);
  odom_sub_ = nh.subscribe("/odometry/ground_truth", 1, &GlobalPlanner::odomCallback, this);

  // publisher
  astar_path_pub_ =node_.advertise<nav_msgs::Path>("astar_plan", 1);
  astar_traj_pub_ =node_.advertise<nav_msgs::Path>("astar_traj_plan", 1);
  kino_astar_path_pub_ =node_.advertise<nav_msgs::Path>("kino_astar_plan", 1);
  sample_path_pub_ =node_.advertise<nav_msgs::Path>("sample_plan", 1);
  bspline_esdf_pub_ =node_.advertise<nav_msgs::Path>("bspline_esdf_plan", 1);
  bspline_astar_pub_ =node_.advertise<nav_msgs::Path>("bspline_astar_plan", 1);
  global_traj_pub_ =node_.advertise<nav_msgs::Path>("global_traj_plan", 1);
  global_traj_astar_pub_ =node_.advertise<nav_msgs::Path>("global_traj_astar_plan", 1);



  
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

bool GlobalPlanner::findPath_astar(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt)
{   
    int status;
    global_planner_astar_->reset();
    status = global_planner_astar_->search( start_pt, end_pt, false,-1.0);
    //global_path_=global_planner_astar_->getPath();
    if(status==Astar::NO_PATH){
      std::cout << "[Astar replan]: Can't find path." <<std:: endl;
      return false;
    }else{
      std::cout << "[Astar replan]: Astar search success."<< std::endl;
      visualize_path(global_planner_astar_->getPath(),astar_path_pub_);
      return true;
    }

}

bool GlobalPlanner::findPath_kino_astar(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc, Eigen::Vector2d end_pt, Eigen::Vector2d end_vel)
{   
    int status;
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
        return false;

      } else {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    visualize_path(global_planner_kino_astar_->getKinoTraj(0.01),kino_astar_path_pub_);
    return true;
    
}

void GlobalPlanner::findPath( Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,  
                              Eigen::Vector2d end_pt, Eigen::Vector2d end_vel){
  // status
  int status;
  global_path_.clear();

  //timer
  double dur;
  ros::WallTime t1, t2;

  bool find_astar_success=false, find_kino_astar_success=false;
  // astar
  if(use_astar_){
  
    find_astar_success=findPath_astar(start_pt,end_pt);
    //global_path_=global_planner_astar_->getPath()
    if(find_astar_success){
      double ts = ctrl_pt_dist_ / max_vel_;
      std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
      point_set.clear();
      start_end_derivatives.clear();
      point_set=global_planner_astar_->getPath();

      start_end_derivatives.push_back(start_vel_);
      start_end_derivatives.push_back(end_vel);
      start_end_derivatives.push_back(start_acc);
      start_end_derivatives.push_back(Eigen::Vector2d::Zero());

      std::vector<Eigen::Vector2d> astar_traj;
      optimizePath(ts,point_set,start_end_derivatives,astar_traj);
    
      visualize_path(astar_traj,astar_traj_pub_);
    }
  }

  // kino star
  t1 = ros::WallTime::now();
  if(use_kino_astar_){
    find_kino_astar_success=findPath_kino_astar(start_pt,start_vel,start_acc,end_pt,end_vel);
    //global_path_ = global_planner_kino_astar_->getKinoTraj(0.01);

    // sample from global path
    if(find_kino_astar_success){
      double ts = ctrl_pt_dist_ / max_vel_;
      std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
      global_planner_kino_astar_->getSamples(ts, point_set, start_end_derivatives);

      // optimize path
      std::vector<Eigen::Vector2d> traj_pts_esdf,traj_pts_astar;
      
      std::cout<<"optimize_astar kino start----------------"<<std::endl;
      t1 = ros::WallTime::now();
      optimizePath_Astar(ts,point_set,start_end_derivatives,traj_pts_astar);
      t2 = ros::WallTime::now();
      dur=(t2 - t1).toSec();
      std::cout<<"optimize_astar kino end----------------dur="<<dur<<std::endl;

      std::cout<<"optimize_esdf kino start----------------"<<std::endl;
      t1 = ros::WallTime::now();
      optimizePath(ts,point_set,start_end_derivatives,traj_pts_esdf);
      t2 = ros::WallTime::now();
      dur=(t2 - t1).toSec();
      std::cout<<"optimize_esdf kino end----------------dur="<<dur<<std::endl;

      // get & visualize path
      visualize_path(traj_pts_esdf,bspline_esdf_pub_);
      visualize_path(traj_pts_astar,bspline_astar_pub_);
      visualize_path(point_set,sample_path_pub_);
    }
  }


  

  
  bool find_global_traj_success=false;
  find_global_traj_success = planGlobalTraj(start_pt, start_vel, Eigen::Vector2d::Zero(), end_pt_, end_vel, Eigen::Vector2d::Zero());

  if(find_global_traj_success){
    constexpr double step_size_t = 0.1;
    int i_end = floor(global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector2d> gloabl_traj(i_end);
    std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
    point_set.clear();
    start_end_derivatives.clear();
    for (int i = 0; i < i_end; i++)
    {
        gloabl_traj[i] = global_data_.global_traj_.evaluate(i * step_size_t);
        point_set.push_back(gloabl_traj[i]);
    }
    start_end_derivatives.push_back(start_vel_);
    start_end_derivatives.push_back(end_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(Eigen::Vector2d::Zero());

    //start_end_derivatives.push_back(global_data_.global_traj_.evaluateVel(0));
    //start_end_derivatives.push_back(end_vel);
    //start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(0));
    //start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(i_end*step_size_t));

    double ts = ctrl_pt_dist_ / max_vel_;
    
   

    std::vector<Eigen::Vector2d> gloabl_traj_astar;
    std::cout<<"optimize naive start----------------"<<std::endl;
    
    
    optimizePath_Astar(ts,point_set,start_end_derivatives,gloabl_traj_astar);
    t2 = ros::WallTime::now();
    dur=(t2 - t1).toSec();

    std::cout<<"optimize naive end----------------dur="<<dur<<std::endl;
    visualize_path(gloabl_traj,global_traj_pub_);
    visualize_path(gloabl_traj_astar,global_traj_astar_pub_);


    
  }


  
}

bool GlobalPlanner::planGlobalTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                                         const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc)
{

    // generate global reference trajectory
    std::vector<Eigen::Vector2d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    std::vector<Eigen::Vector2d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector2d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(2, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector2d zero(0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
}

void GlobalPlanner::optimizePath(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives,std::vector<Eigen::Vector2d> &traj_pts)
{   
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    
    // bspline trajectory optimization
    ros::Time t1, t2;
    double t_search, t_opt , t_adjust ;
    t1 = ros::Time::now();
    // define cost function
    int cost_function = BsplineOptimizerESDF::NORMAL_PHASE;//NORMAL_PHASE;
    
    int status;
    if (status != KinodynamicAstar::REACH_END) {
      cost_function |= BsplineOptimizerESDF::ENDPOINT;
    }

    // optimize control points
    ctrl_pts = bspline_optimizer_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

    t_opt = (ros::Time::now() - t1).toSec();

    // iterative time adjustment
    t1  = ros::Time::now();
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);

    double to = pos.getTimeSum();
    double feasibility_tolerance_=0.0;
    pos.setPhysicalLimits(max_vel_, max_acc_,feasibility_tolerance_);
    double ratio;
    bool feasible = pos.checkFeasibility(ratio,false);

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
    
    double  tm, tmp;
    position_traj_.getTimeSpan(tm, tmp);
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::Vector2d pt = position_traj_.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }

    // save traj ctl pts
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
    visualization_->drawGlobalPath(ctrl_points_path,0.2, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));
    
}

void GlobalPlanner::optimizePath_Astar(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives,std::vector<Eigen::Vector2d> &traj_pts){
  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  std::vector<std::vector<Eigen::Vector2d>> a_star_pathes;
  a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

  /*** STEP 2: OPTIMIZE ***/
  bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
  std::cout << "first_optimize_step_success=" << flag_step_1_success << std::endl;

  if (!flag_step_1_success)
  {
      continous_failures_count_++;
      return;
  }

  /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  double feasibility_tolerance_=0.0;
  pos.setPhysicalLimits(max_vel_, max_acc_, feasibility_tolerance_);

  double ratio;
  bool flag_step_2_success = true;
  if (!pos.checkFeasibility(ratio, false))
  {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
  }

  if (!flag_step_2_success)
  { 
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      // save traj pos
    double  tm, tmp;
    pos.getTimeSpan(tm, tmp);
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::Vector2d pt = pos.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }
      return;
  }

  continous_failures_count_ = 0;


  // save traj pos
  double  tm, tmp;
  pos.getTimeSpan(tm, tmp);
  for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::Vector2d pt = pos.evaluateDeBoor(t);
      traj_pts.push_back(pt);
  }
  return;
}

bool GlobalPlanner::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector2d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
{
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
}

void GlobalPlanner::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector2d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
{
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector2d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
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















