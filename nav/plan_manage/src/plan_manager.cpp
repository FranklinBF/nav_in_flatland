#include <plan_manage/plan_manager.h>





using namespace std;

void PlanManager::init(ros::NodeHandle& nh) {

    /*  plan param  */

    /* initialize main modules */
    planner_collector_.reset(new PlanCollector);
    planner_collector_->initPlanModules(nh);
    //visualization_.reset(new PlanningVisualization(nh));

    /* init variables */
    exec_state_  = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_   = false;
    cur_state_=new RobotState(Eigen::Vector2d::Zero(),0.0,Eigen::Vector2d::Zero(),0.0);
    
    /* callback */
    exec_timer_   = nh.createTimer(ros::Duration(0.01), &PlanManager::execFSMCallback, this);
    //safety_timer_ = nh.createTimer(ros::Duration(0.05), &PlanManager::checkCollisionCallback, this);

    goal_sub_ =nh.subscribe("/goal", 1, &PlanManager::goalCallback, this);
    odom_sub_ = nh.subscribe("/odom", 1, &PlanManager::odometryCallback, this);

    

    subgoal_pub_  = nh.advertise<geometry_msgs::PoseStamped>("/subgoal",10);

    /* test purpose*/
    
    

}
  
void PlanManager::goalCallback(const geometry_msgs::PoseStampedPtr& msg) {
    // position z must be zero
    if(msg->pose.position.z !=0) return;
    
    // set trigger
    cout << "Triggered!" << endl;

    // set end_state
    end_state_=new RobotState(msg->pose);
    end_state_->vel2d.setZero();
    
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
    
    // set have_goal
    cout << "Goal set!" << endl;
    have_goal_ = true;
    start_time_=ros::Time::now();
}

void PlanManager::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
  cur_state_= new RobotState(*msg);
  have_odom_ = true;
}

void PlanManager::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!have_goal_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!have_goal_) {
        return;
      }
      changeFSMExecState(WAIT_GOAL, "FSM");
      break;
    }

    case WAIT_GOAL: {
      
      if (!have_goal_)
        return;
      else {
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }


    case GEN_NEW_GLOBAL: {
      start_state_=new RobotState(cur_state_->pose2d,cur_state_->theta,cur_state_->vel2d,cur_state_->w);
      bool success=planner_collector_->generate_global_plan(*start_state_,*end_state_);
      if(success){
        changeFSMExecState(REPLAN_MID, "FSM");
      }else{
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }

    case EXEC_LOCAL: {
      /* check env etermine if need to replan */
      
      double dist_to_obstacle;
      
      
      
      random_device rd;
      uniform_real_distribution<double> rand_obstacle;
      uniform_real_distribution<double> rand_goal;
      rand_obstacle = uniform_real_distribution<double>(0.0, 3.0 );
      rand_goal = uniform_real_distribution<double>(0.0, 100.0 );
      default_random_engine eng(rd());

      dist_to_obstacle=rand_obstacle(eng);
   
      
      double dist_to_goal;
      dist_to_goal=(cur_state_->pose2d-end_state_->pose2d).norm();
      
      double dist_to_subgoal;
      dist_to_subgoal=(cur_state_->pose2d-planner_collector_->subgoal_state_->pose2d).norm();
      
      double timeout;
      timeout=1000;
      double time_cost=ros::Time::now().toSec()-start_time_.toSec();
      
      if(dist_to_goal<0.5){
        have_goal_=false;
        cout<<"reached to goal success"<<endl;
        
        changeFSMExecState(WAIT_GOAL, "FSM");
        
      }else if(time_cost>timeout){
        have_goal_=false;
        cout<<"failed to goal"<<endl;
        changeFSMExecState(WAIT_GOAL, "FSM");

      }else if(dist_to_subgoal>2 || dist_to_subgoal<0.2){
        if(cur_state_->vel2d.norm()>0.1){
          changeFSMExecState(REPLAN_MID, "FSM");
        }
      }else{
        //cout<<"Normal:Exec local"<<endl;
        return;
      }

      break;
    }

    case REPLAN_MID: {
      /* get current state info */
      //RobotStatePtr mid_start_state=new RobotState(cur_state_->pose2d,cur_state_->theta,cur_state_->vel2d,cur_state_->w);
      double dist_to_goal=1.0;
      double obstacle_info=1.0;
      double sensor_info=1.0;
      
      /* new waypoint generation*/
   
      bool success = planner_collector_->generate_subgoal(cur_state_,end_state_, planner_collector_->global_path_,obstacle_info,sensor_info);
      if (success) {
        subgoal_pub_.publish(planner_collector_->subgoal_);
        cout<<"MID_REPLAN Success"<<endl;
        changeFSMExecState(EXEC_LOCAL, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }
  }
}

/* helper functions */
void PlanManager::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void PlanManager::printFSMExecState() {
  string state_str[5] =  {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

int main(int argc, char** argv) {
    cout<<"Plan manager node start"<<endl;
    ros::init(argc, argv, "Plan_manager");
    
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle;
    ros::WallRate r(100);

    PlanManager plan_manager;
    plan_manager.init(node_handle);
    ros::spin();
}
