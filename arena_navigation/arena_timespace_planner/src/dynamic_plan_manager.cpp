#include <arena_timespace_planner/dynamic_plan_manager.h>


using std::cout;
using std::endl;

void DynamicPlanManager::initPlanModules(ros::NodeHandle &nh)
{
  node_=nh;
  // ros param
  node_.param("plan_manager/max_vel",   pp_.max_vel_, 2.0);
  node_.param("plan_manager/max_acc",   pp_.max_acc_, 3.0);
  node_.param("plan_manager/max_jerk",  pp_.max_jerk_, 4.0);
  node_.param("plan_manager/feasibility_tolerance",   pp_.feasibility_tolerance_, 0.05);
  node_.param("plan_manager/control_points_distance", pp_.ctrl_pt_dist_, 0.4);
  node_.param("plan_manager/use_distinctive_trajs",   pp_.use_distinctive_trajs_, true);
  node_.param("plan_manager/local_time_horizon",   pp_.local_time_horizon_,    2.0);

  

  // init grid_map
  grid_map_.reset(new GridMap);
  grid_map_->initMap(node_);

  // global planner
  global_planner_kino_astar_.reset(new KinodynamicAstar);
  global_planner_kino_astar_->setParam(node_);
  global_planner_kino_astar_->setEnvironment(grid_map_);
  global_planner_kino_astar_->init();
  global_planner_kino_astar_->reset();

  bspline_optimizer_esdf_.reset(new BsplineOptimizerESDF);
  bspline_optimizer_esdf_->setParam(node_);
  bspline_optimizer_esdf_->setEnvironment(grid_map_);

  // MovingObstacleInfo
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  std::string str1="obs_dynamic";
	obs_info_provider_.reserve(100);
  for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
  {
        const ros::master::TopicInfo& info = *it;
        
        if (info.name.find(str1) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			      obs_info_provider_.emplace_back(std::make_shared<DynamicObstacleInfo>(node_,info.name,grid_map_));
        }

  }

  // mid planner: timed_astar
  mid_planner_timed_astar_.reset(new TimedAstarSearch);
  mid_planner_timed_astar_->init(node_,grid_map_,obs_info_provider_);

  bspline_optimizer_.reset(new BsplineOptimizer);
  bspline_optimizer_->setParam(node_);
  bspline_optimizer_->setEnvironment(grid_map_);
  bspline_optimizer_->a_star_.reset(new AStar);
  bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));

}

bool DynamicPlanManager::planGlobalTraj( Eigen::Vector2d &start_pos,  Eigen::Vector2d &end_pos){
    std::cout<<"******************************************************"<<std::endl;
    std::cout<<"[kino replan]start----------------------------------"<<std::endl;
    std::cout<<"******************************************************"<<std::endl;

    // initial reset
    int status;
    global_planner_kino_astar_->reset();
    adjustStartAndTargetPoint(start_pos,end_pos);
    
    // init first search
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel   = Eigen::Vector2d::Zero();
    status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);

    // second search // search again
    if (status == KinodynamicAstar::NO_PATH) {
      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false);

      if (status == KinodynamicAstar::NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;

      } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
      }
    } else 
    {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    // sample points from global intial trajectory
    double sample_step_size = 0.1;   //pp_.ctrl_pt_dist_ / pp_.max_vel_;
    std::vector<Eigen::Vector2d> point_set,start_end_derivatives;
    global_planner_kino_astar_->getSamples(sample_step_size, point_set, start_end_derivatives);

    // get ts, pointset, derivatives
    double ts=sample_step_size;
   
    // optimize global trajectory
    UniformBspline global_traj;
    bool optimize_success;
    optimize_success=optimizeGlobalTraj(ts,point_set,start_end_derivatives,global_traj);
    
    if(!optimize_success){
      ROS_WARN_STREAM("[kino replan]: trajectory optimize failed.");
    }
    
    global_data_.resetData(global_traj);
    return true;

}

bool DynamicPlanManager::optimizeGlobalTraj(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & bspline_traj){
  /* -------------------Optimize ESDF----------------------------------- */
  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  // define cost function
  int cost_function = BsplineOptimizerESDF::NORMAL_PHASE;//NORMAL_PHASE;

  /*** STEP 2: OPTIMIZE ***/
  ctrl_pts = bspline_optimizer_esdf_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

  double to = pos.getTimeSum();
  
  double ratio;
  bool feasible = pos.checkFeasibility(ratio,false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {
      feasible = pos.reallocateTime();
      if (++iter_num >= 3) break;
  }
  double tn = pos.getTimeSum();

  
  //std::cout << "[kino replan]: Reallocate ratio: " << tn / to << std::endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  bspline_traj=pos;
  return true;

}

bool DynamicPlanManager::planMidTraj(const Eigen::Vector2d & start_pos,const Eigen::Vector2d & start_vel, const double & start_dir, const Eigen::Vector2d & end_pos,std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets){
  
  std::vector<Eigen::Vector2d> point_set,start_end_derivatives;
  
  double ts=0.1;
  double local_time_horizon=pp_.local_time_horizon_;
  bool success;
  ROS_WARN_STREAM("mid1------------------------");
  success=mid_planner_timed_astar_->stateTimeAstarSearch(start_pos,start_vel,start_dir,end_pos,ts,local_time_horizon, point_set,start_end_derivatives,line_sets);
  if(!success) return false;
  ROS_WARN_STREAM("mid2------------------------");
  UniformBspline mid_traj;
  ROS_WARN_STREAM("start  mid optimize");
  bool optimize_success=optimizeBsplineTraj(ts,point_set,start_end_derivatives,mid_traj);
  if(!optimize_success){
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    mid_traj = UniformBspline(ctrl_pts, 3, ts);
  }
  ROS_WARN_STREAM("end  mid optimize");
  local_traj_data_.resetData(mid_traj);
  return true;
}

bool DynamicPlanManager::planLocalTraj( Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & target_pos, Eigen::Vector2d & target_vel ){
  //ROS_WARN_STREAM("-----------------------------1");
  // set localtarget for termial cost
  bspline_optimizer_->setLocalTargetPt(target_pos);
  //ROS_WARN_STREAM("-----------------------------2");
  // generate oneshot intial traj
  double dist = (start_pos - target_pos).norm();
  double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;//double time =dist / pp_.max_vel_;

  PolynomialTraj init_traj;
  init_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, Eigen::Vector2d::Zero(), target_pos, target_vel, Eigen::Vector2d::Zero(), time);
  //ROS_WARN_STREAM("-----------------------------3");
  // get pointset and derivatives
  vector<Eigen::Vector2d> first_point_set, point_set, start_end_derivatives;
  double local_time_horizon=pp_.local_time_horizon_;
  double ts = dist > 0.1 ? pp_.ctrl_pt_dist_ / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist_ / pp_.max_vel_ * 5;
  double t_first, t;
  
  bool flag_too_far;
  ts *= 1.5; // ts will be divided by 1.5 in the next
  t_first=0.2;
  do{
    ts /= 1.5;
    point_set.clear();
    flag_too_far = false;
    Eigen::Vector2d last_pt = init_traj.evaluate(t_first);
    for ( t = t_first; t < time; t += ts)
    {
      Eigen::Vector2d pt = init_traj.evaluate(t);
      if ((last_pt - pt).norm() > pp_.ctrl_pt_dist_ * 1.5)
      {
        flag_too_far = true;
        break;
      }

      if(t>local_time_horizon){
        break;
      }
      last_pt = pt;
      point_set.push_back(pt);
    }
  } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
  t -= ts;
  //ROS_WARN_STREAM("-----------------------------4");
  // adjust the last intial control point
  Eigen::Vector2d last_pt=point_set.back();
  adjustStartAndTargetPoint(start_pos,last_pt);
  point_set[point_set.size()-1]=last_pt;


  // // initial rotation part of the traj
  // Eigen::Vector2d init_vel=init_traj.evaluateVel(t/2);
  // Eigen::Vector2d init_rot_pos= start_pos + init_vel/init_vel.norm()*0.01;
  // double new_dir, diff_dir;
  // new_dir=atan2(init_vel(1),init_vel(0));
  // new_dir=new_dir<0?2*PI+new_dir:new_dir;
  // diff_dir = std::abs(new_dir-start_dir);
  // for(double t_rot=0;t_rot<diff_dir/0.5;t_rot+=0.1){
  //   //point_set.insert(point_set.begin(), init_rot_pos);
  // }
  // ROS_WARN_STREAM("new dir="<<new_dir*180/PI);
  // ROS_WARN_STREAM("diff dir="<<diff_dir*180/PI);

  //ROS_WARN_STREAM("point_set size="<<point_set.size());

  if(point_set.size()<3){
    ROS_ERROR("point_set size<3**********************");
    return false;
  }

  start_end_derivatives.push_back(init_traj.evaluateVel(0));
  start_end_derivatives.push_back(init_traj.evaluateVel(t));
  start_end_derivatives.push_back(init_traj.evaluateAcc(0));
  start_end_derivatives.push_back(init_traj.evaluateAcc(t));

  //ROS_WARN_STREAM("-----------------------------5");
  // optimize trajectory
  UniformBspline local_traj;
  bool optimize_success=optimizeBsplineTraj(ts,point_set,start_end_derivatives,local_traj);
  ROS_WARN_STREAM("-----------------------------6");
  if(!optimize_success){
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    local_traj = UniformBspline(ctrl_pts, 3, ts);
    ROS_WARN_STREAM("local traj optimized failed");
    local_traj_data_.resetData(local_traj);
    
    return false;
  }else{
    local_traj_data_.resetData(local_traj);
    return true;
  }
}

bool DynamicPlanManager::optimizeBsplineTraj(double ts,std::vector<Eigen::Vector2d>  point_set, std::vector<Eigen::Vector2d>  start_end_derivatives, UniformBspline & mid_traj){
  

  // init bspline_optimizer local target
  Eigen::Vector2d local_target_pt = point_set.back();
  bspline_optimizer_->setLocalTargetPt(local_target_pt);

  // init bspline
  Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  
  // get collision segments
  std::vector<std::pair<int, int>> segments;
  segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
  
  /*** STEP 2: OPTIMIZE ***/
  bool flag_step_1_success = false;
  std::vector<vector<Eigen::Vector2d>> vis_trajs;

  if (pp_.use_distinctive_trajs_)
  {
    std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
    cout<<"****************************************"<< endl;
    cout << "\033[1;33m"<< "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;
    cout<<"****************************************"<< endl;

    double final_cost, min_cost = 999999.0;
    for (int i = trajs.size() - 1; i >= 0; i--)
    {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          std::vector<Eigen::Vector2d> vis_point_set;
          vis_point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            vis_point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(vis_point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
       
      }

      //visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
  }
  else
  {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      //static int vis_id = 0;
      //visualization_->displayInitPathList(point_set, 0.2, 0);
  }

  cout << "plan_success=" << flag_step_1_success << endl;
  if (!flag_step_1_success)
  {
      //visualization_->displayOptimalList(ctrl_pts, 0);
      //continous_failures_count_++;
      
      mid_traj = UniformBspline(ctrl_pts, 3, ts);
      return false;
  }

  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
 
    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // double ratio;
    // bool flag_step_2_success = true;
    // if (!pos.checkFeasibility(ratio, false))
    // {
    //   cout << "Need to reallocate time." << endl;

    //   Eigen::MatrixXd optimal_control_points;
    //   flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
    //   if (flag_step_2_success)
    //     pos = UniformBspline(optimal_control_points, 3, ts);
    // }

    // if (!flag_step_2_success)
    // {
    //   printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
    //   continous_failures_count_++;
    //   return false;
    // }

    // save planned results
    //updateTrajInfo(pos, ros::Time::now());

    // success. YoY
    //continous_failures_count_ = 0;
  mid_traj=pos;
  return true;
}

bool DynamicPlanManager::adjustStartAndTargetPoint( Eigen::Vector2d & start_pt, Eigen::Vector2d &target_pt)
{   
    double step_size=0.1;

    if(checkCollision(start_pt)){
      ROS_WARN("This start point is insdide an obstacle.");
      do
        {
            start_pt = (start_pt - target_pt).normalized() * step_size + start_pt;
            if (!grid_map_->isInMap(start_pt))
                return false;
        } while (checkCollision(start_pt));
    }

    if(checkCollision(target_pt)){
      ROS_WARN("This target point is insdide an obstacle.");
      do
        {
            target_pt = (target_pt - start_pt).normalized() * step_size + target_pt;
            if (!grid_map_->isInMap(target_pt))
                return false;
        } while (checkCollision(target_pt));
    }
    
    return true;
}

bool DynamicPlanManager::checkCollision(const Eigen::Vector2d &pos){
  bool is_occ= grid_map_->getFusedDynamicInflateOccupancy(pos);
  return is_occ;
}

