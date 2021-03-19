#include "arena_dynamic_channel/dynamic_obstacle.h"

DynamicObstacleInfo::DynamicObstacleInfo(ros::NodeHandle &nh, std::string topic_name){
    node_=nh;
    topic_name_=topic_name;

    // init subscriber for obstacles state
    ros::NodeHandle public_nh_;
    obs_odom_sub_=public_nh_.subscribe(topic_name_, 1, &DynamicObstacleInfo::updateOdomCallback,this);
        
    // init pos, vel, is_init
    pos_=Eigen::Vector2d::Zero();
    vel_=Eigen::Vector2d::Zero();
    is_init_=true;
}
    
void DynamicObstacleInfo::updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg){
        
    // for nav_msgs::Odometry::ConstPtr msg
    //curr_pos(msg->pose.pose.position.x,msg->pose.pose.position.y);

    Eigen::Vector2d curr_pos(msg->markers[0].pose.position.x,msg->markers[0].pose.position.y); 
        
    double curr_time=msg->markers[0].header.stamp.toSec();
    if(curr_time==last_time_){
            return;
    }
        
    if(is_init_)
    {
        vel_= Eigen::Vector2d(0.0,0.0);
        is_init_=false;
    }else{
        vel_=(curr_pos-pos_)/(curr_time-last_time_);
    }
    
    pos_=curr_pos;
    last_time_=curr_time;

}