#include "arena_dynamic_channel/dynamic_obstacle.h"

DynamicObstacleInfo::DynamicObstacleInfo(ros::NodeHandle &nh, std::string topic_name,GridMap::Ptr grid_map){
    node_=nh;
    topic_name_=topic_name;
    // get  radius param
    node_.param("timed_astar/robot_radius",       obstacle_radius_,    0.3);
    node_.param("timed_astar/inflation_radius",   inflation_radius_,   0.3);
    radius_=obstacle_radius_+inflation_radius_;

    // gridmap
    this->grid_map_=grid_map;
    resolution_=grid_map_->getResolution();

    // init subscriber for obstacles state
    ros::NodeHandle public_nh_;
    obs_odom_sub_=public_nh_.subscribe(topic_name_, 1, &DynamicObstacleInfo::updateOdomCallback,this);
    std::string vel_topic_name=topic_name_+"_vel";
    obs_vel_pub_ =public_nh_.advertise<std_msgs::Float32>(vel_topic_name,1);

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

    if(!is_init_){
        updateDynamicOcc();
    }
    std_msgs::Float32 velocity;
    velocity.data=vel_.norm();
    obs_vel_pub_.publish(velocity);

}

void DynamicObstacleInfo::updateDynamicOcc(){
    // reset old occ
    Eigen::Vector2d vel=vel_;
    Eigen::Vector2d pos=pos_+0.1*vel_;  //pos after 0.1s
    
    for(size_t i=0;i<last_occ_set_.size();++i){
        grid_map_->setDynamicOccupancy(last_occ_set_[i],0);
    }
    // add new occ
    
    last_occ_set_.clear();
    //int i=0;
    //std::cout<<"topic="<<topic_name_<<"  pos="<<pos<<std::endl;

    for(double x=pos(0)-radius_; x<pos(0)+radius_; x+=resolution_){
        for(double y=pos(1)-radius_; y<pos(1)+radius_; y+=resolution_){
            for(double t=0;t<0.2;t+=0.1){
                // curr_pos
                Eigen::Vector2d pt(x,y);
                // future pos at time=t
                pt=pt+vel*t;
                grid_map_->setDynamicOccupancy(pt,1);
                last_occ_set_.push_back(pt);
                //i++;
                //if(i==1){
                //    std::cout<<"topic="<<topic_name_<<"  pt="<<pt<<std::endl;
                //}
                
             
                
            }
        }
    } 
}