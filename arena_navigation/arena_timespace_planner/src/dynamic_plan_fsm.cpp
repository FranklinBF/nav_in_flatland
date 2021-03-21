#include <arena_timespace_planner/dynamic_plan_fsm.h>
using std::cout;
using std::endl;
void DynamicReplanFSM::init(ros::NodeHandle &nh)
{   
    
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_ = false;
    target_traj_data_ = TargetTrajData();

    /*  fsm param  */
    node_=nh;
    node_.param("fsm/goal_tolerance",    goal_tolerance_, 0.5);
    node_.param("fsm/subgoal_tolerance", subgoal_tolerance_, 2.0);
    node_.param("fsm/replan_time_thresh", t_replan_thresh_, 0.5); 

    

    /* initialize main modules */
    planner_manager_.reset(new DynamicPlanManager);
    planner_manager_->initPlanModules(node_);

    /* callback */
    exec_timer_ = node_.createTimer(ros::Duration(0.01), &DynamicReplanFSM::execFSMCallback, this);
    //safety_timer_ = node_.createTimer(ros::Duration(0.05), &DynamicReplanFSM::checkCollisionCallback, this);
    traj_tracker_timer_ = node_.createTimer(ros::Duration(0.01), &DynamicReplanFSM::trackTrajCallback, this);
    //dynamic_occ_map_timer_= node_.createTimer(ros::Duration(1.0), &DynamicReplanFSM::updateDynamicMapCallback,this);
    
    /* ros communication with public node */
    ros::NodeHandle public_nh;
  	goal_sub_ =public_nh.subscribe("goal", 1, &DynamicReplanFSM::goalCallback,this);
  	odom_sub_ = public_nh.subscribe("odom", 1, &DynamicReplanFSM::odomCallback, this);

    cmd_vel_pub_ =public_nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    /* visualization */
    vis_goal_pub_ =	        public_nh.advertise<visualization_msgs::Marker>("vis_goal", 20);
    vis_global_path_pub_ =   public_nh.advertise<nav_msgs::Path>("vis_global_path", 10, true); 
    vis_landmark_pub_ =      public_nh.advertise<visualization_msgs::Marker>("vis_landmarks_", 20);

	vis_triangle_pub_=         public_nh.advertise<visualization_msgs::Marker>("vis_triangle", 20);
    vis_timed_astar_path_pub_= public_nh.advertise<nav_msgs::Path>("vis_path_timed_astar", 20);
    vis_timed_astar_wp_pub_ =  public_nh.advertise<visualization_msgs::Marker>("vis_wps_timed_astar", 20);

    vis_local_path_pub_ =       public_nh.advertise<visualization_msgs::Marker>("vis_local_path", 20);
}

void DynamicReplanFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg){

    odom_pos_=Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
    odom_vel_=Eigen::Vector2d(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    auto euler = odom_orient_.toRotationMatrix().eulerAngles(0, 1, 2); // row,pitch,yaw
    odom_dir_=euler(2); 
    odom_dir_=odom_dir_<0?2*PI+odom_dir_:odom_dir_;
    //cout<<"odom direction"<<odom_dir_*180/3.1415<<endl;
    have_odom_ = true;
}

void DynamicReplanFSM::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
    if(have_odom_==false) return;

    // end pt
    end_pos_=Eigen::Vector2d(msg->pose.position.x,msg->pose.position.y);
    end_vel_=Eigen::Vector2d::Zero();

    have_goal_=true;
    std::cout << "[Plan FSM]Goal set!" << std::endl;

    // change state: to GEN_NEW_GLOBAL
    if (exec_state_ == WAIT_GOAL){
        changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
    } 
    else if (exec_state_ == EXEC_LOCAL){
        changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
    else if (exec_state_ == REPLAN_MID){
        changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
    else if (exec_state_ == INIT){
        changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
  
    // vis goal
    std::vector<Eigen::Vector2d> point_set;
    point_set.push_back(end_pos_);
    visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

}

void DynamicReplanFSM::execFSMCallback(const ros::TimerEvent& e){
    exec_timer_.stop(); // To avoid blockage

    /* init phase wating for odom & goal */
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        if (!have_odom_) cout << "[Plan FSM]no odom." << endl;
        if (!have_goal_) cout << "[Plan FSM]wait for goal." << endl;
        fsm_num = 0;
    }

    /* FSM state-> action */
    switch (exec_state_) {
        /* --------------Case1:INIT ---------------*/
        case INIT: {
            if (!have_odom_) {
                goto force_return;
            }
            if (!have_goal_) {
                goto force_return;
            }

            changeFSMExecState(WAIT_GOAL, "FSM");
            break;
        }

        /* --------------Case2:WAIT_GOAL ---------------*/
        case WAIT_GOAL: {
            if (!have_goal_)
                goto force_return;
            else {
                changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
            }
            break;
        }

        /* --------------Case3:GEN_NEW_GLOBAL ---------------*/
        case GEN_NEW_GLOBAL: {
            if (have_odom_ && have_goal_){
                start_pos_ = odom_pos_;
                bool success= planner_manager_->planGlobalTraj(start_pos_, end_pos_);
                if(success){
                    // set mid_target
                    landmark_wps_.clear();
                    landmark_wps_= planner_manager_->global_data_.getLandmarks();
                    mid_target_=getNextWaypoint();

                    // visualize global path & landmark
                    cout<<"[Plan Manager]GLOBAL_PLAN Success"<<endl;
                    visualizePath(planner_manager_->global_data_.getGlobalPath(),vis_global_path_pub_);
                    visualizePoints(planner_manager_->global_data_.getLandmarks(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),vis_landmark_pub_);
                    changeFSMExecState(REPLAN_MID, "FSM");
                    
                }else{
                    ROS_ERROR("[Plan FSM]Failed to generate the global plan!!!");
                    changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
                }
            }else{
                ROS_ERROR("[Plan FSM]No odom or no target! have_odom_=%d, have_goal_=%d", have_odom_, have_goal_);
            }
            break;
        }

        /* --------------Case4:REPLAN_MID ---------------*/
        case REPLAN_MID: {
            // read mid_target
            double dist_to_goal;
            dist_to_goal=(odom_pos_-end_pos_).norm();

            if(dist_to_goal<goal_tolerance_){
                target_traj_data_ = TargetTrajData();
                have_goal_=false;
                cout<<"[Plan FSM]this global target success, return to wait goal"<<endl;
                changeFSMExecState(WAIT_GOAL, "FSM");
                goto force_return;
            }


            // plan mid trajectory
            Eigen::Vector2d curr_pos = odom_pos_;
            Eigen::Vector2d curr_vel = odom_vel_;
            double          curr_dir = odom_dir_;
            Eigen::Vector2d target_vel = Eigen::Vector2d(0.0,0.0);
            std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> line_sets;
            
            bool success=planner_manager_->planMidTraj(curr_pos,curr_vel,curr_dir,mid_target_,line_sets);
            
            visualizeLines(line_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_);
            
            
            //bool success =planner_manager_->planLocalTraj(curr_pos,curr_vel,curr_dir,mid_target_,target_vel);

            if(success){
                target_traj_data_ = planner_manager_->local_traj_data_;
                visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);

                cout<<"[Plan FSM]MID_REPLAN Success"<<endl;
                changeFSMExecState(EXEC_LOCAL, "FSM");
            }else{
                // for debug
                target_traj_data_ = planner_manager_->local_traj_data_;
                visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);
                
                //target_traj_data_ = TargetTrajData();
                ROS_ERROR("[Plan FSM]Failed to generate the mid trajectory!!!");
                changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
            }
            break;

        }
        /* --------------Case5:EXEC_LOCAL ---------------*/
        case EXEC_LOCAL: {
            // distance to goal
            double dist_to_goal, dist_to_mid_target;
            dist_to_goal=(odom_pos_-end_pos_).norm();

            // reached goal
            if(dist_to_goal<goal_tolerance_){
                target_traj_data_ = TargetTrajData();
                have_goal_=false;
                cout<<"[Plan FSM]this global target success, return to wait goal"<<endl;
                changeFSMExecState(WAIT_GOAL, "FSM");
                goto force_return;
            }

            // reached mid target (not last mid target)
            dist_to_mid_target=(odom_pos_-mid_target_).norm();
            if(dist_to_mid_target<subgoal_tolerance_ && landmark_wps_.size()>1){
                mid_target_=getNextWaypoint();
                cout<<"[Plan FSM]this mid target success, return to wait goal"<<endl;
                changeFSMExecState(REPLAN_MID, "FSM");
            }

            // check time
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - target_traj_data_.start_time_).toSec();
            
            if(t_cur>t_replan_thresh_ || t_cur> target_traj_data_.duration_){
                changeFSMExecState(REPLAN_MID, "FSM");
            }

        }
        break;

    }

    force_return:;
    exec_timer_.start();

}

bool DynamicReplanFSM::planFromCurrentTraj(const int trial_times){
    ros::Time time_now = ros::Time::now();
    double t_curr = (time_now - target_traj_data_.start_time_).toSec();
    Eigen::Vector2d curr_pos = odom_pos_;
    Eigen::Vector2d curr_vel = odom_vel_;
    double          curr_dir = odom_dir_;
    Eigen::Vector2d target_vel =Eigen::Vector2d(0.0,0.0);

    bool success = planner_manager_->planLocalTraj(curr_pos,curr_vel,curr_dir,mid_target_,target_vel);
    if(success){
        target_traj_data_ = planner_manager_->local_traj_data_;
        visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);

    }else{
        // for debug
        target_traj_data_ = planner_manager_->local_traj_data_;
        visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);
        // real
        //target_traj_data_= TargetTrajData();
    }
    
}

void DynamicReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
    auto map = planner_manager_->grid_map_;

    if(have_goal_==false){
        return;
    }

    if (exec_state_ == WAIT_GOAL || exec_state_ == INIT)
      return;
    
    if(target_traj_data_.flag_is_empty){
        return;
    }
    /* ---------- check trajectory ---------- */
    ros::Time time_now = ros::Time::now();
    double t_curr = (time_now- target_traj_data_.start_time_).toSec();
    Eigen::Vector2d p_curr = target_traj_data_.pos_traj_.evaluateDeBoorT(t_curr);
    //const double CLEARANCE = 0.5;
    constexpr double time_step = 0.01;
    double t_2_3 = target_traj_data_.duration_ * 2 / 3;

    bool occ = false;
    for (double t = t_curr; t < target_traj_data_.duration_; t += time_step){
        
        if (t_curr < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
            break;

        occ = map->getFusedInflateOccupancy(target_traj_data_.pos_traj_.evaluateDeBoorT(t));
        
        if(occ==true){
            if(planFromCurrentTraj()){
                changeFSMExecState(EXEC_LOCAL, "SAFETY");
            }else{
                changeFSMExecState(REPLAN_MID, "SAFETY");
            }
            return;
        }
    }
}

void DynamicReplanFSM::trackTrajCallback(const ros::TimerEvent &e){
    geometry_msgs::Twist twist_cmd;

    ros::Time time_now = ros::Time::now();
    double v,w;
    if(target_traj_data_.flag_is_empty){
        v=0.0;
        w=0.0;
    }else{
        double t_cur = (time_now - target_traj_data_.start_time_).toSec();
        v=0.0;
        w=0.0;
        target_traj_data_.getControlInput(t_cur,v,w);
        //ROS_INFO_STREAM("vel: "<<v);
        //ROS_INFO_STREAM("rot vel: "<<w);
    }
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=w;
    twist_cmd.linear.x =v;
    twist_cmd.linear.y =0.0;
    twist_cmd.linear.z =0.0;
    
    cmd_vel_pub_.publish(twist_cmd);
}


/* --------------------helper functions---------------------------- */
void DynamicReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void DynamicReplanFSM::printFSMExecState() {
  string state_str[5] =  {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

Eigen::Vector2d DynamicReplanFSM::getNextWaypoint(){
    
    Eigen::Vector2d next_wp = landmark_wps_.back();
    // if(landmark_wps_.size()>1){
    //     landmark_wps_.erase(landmark_wps_.begin());
    // }

    return next_wp;
}


/* --------------------visualization---------------------------- */
void DynamicReplanFSM::visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = {};//ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  //mk.id              = id;

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = pt_size;
  mk.scale.y = pt_size;
  mk.scale.z = pt_size;

  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < point_set.size(); i++) {
    pt.x = point_set[i](0);
    pt.y = point_set[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void DynamicReplanFSM::visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = pt_size;
    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = color(3);

    geometry_msgs::Point pt1,pt2;
    for (unsigned int i = 0; i < ptr_pair_sets.size(); i++) {
        pt1.x=ptr_pair_sets[i].first(0);
        pt1.y=ptr_pair_sets[i].first(1);
        pt1.z=0.0;

        pt2.x=ptr_pair_sets[i].second(0);
        pt2.y=ptr_pair_sets[i].second(1);
        pt2.z=0.0;

        line_list.points.push_back(pt1);
        line_list.points.push_back(pt2);
    }

    pub.publish(line_list);
    //ROS_INFO("vis once");
}

void DynamicReplanFSM::visualizePath(const std::vector<Eigen::Vector2d> path, const ros::Publisher & pub){

  //create a path message
  ros::Time plan_time = {};//ros::Time::now();
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
  }
  pub.publish(gui_path);
}


int main(int argc, char** argv) 
{
    std::cout<<"Plan manager node start"<<std::endl;
    ros::init(argc, argv, "plan_manager");

    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    //ros::NodeHandle nh("");
    ros::NodeHandle nh("~");

    DynamicReplanFSM dynamic_replan;

    dynamic_replan.init(nh);

    // ros::Duration(1.0).sleep();
    
    DynamicReplanFSM plan_manager;
    //plan_manager.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tPlan manager successfully loaded for namespace\t"<<ns);
    
    ros::spin();
}


















