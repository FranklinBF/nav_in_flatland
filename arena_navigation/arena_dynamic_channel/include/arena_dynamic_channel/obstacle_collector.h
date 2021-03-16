#ifndef _OBSTACLE_COLLECTOR_H
#define _OBSTACLE_COLLECTOR_H


#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/master.h>
#include"boost/algorithm/string.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>



struct ObstacleNode{
public:
    ros::NodeHandle node_;
    std::string topic_name_;
    ros::Subscriber obs_odom_sub_;

    Eigen::Vector2d pos_;
    Eigen::Vector2d vel_;
    double last_time_;
    bool is_init_;

    Eigen::Vector2d getPosition(){
        return pos_;
    }

    Eigen::Vector2d getVelocity(){
        return vel_;
    }

    ObstacleNode(ros::NodeHandle &nh, std::string topic_name){
        node_=nh;
        topic_name_=topic_name;

        // init subscriber for obstacles state
        ros::NodeHandle public_nh_;
        obs_odom_sub_=public_nh_.subscribe(topic_name_, 1, &ObstacleNode::updateOdomCallback,this);
        
        // init pos, vel, is_init
        pos_=Eigen::Vector2d::Zero();
        vel_=Eigen::Vector2d::Zero();
        is_init_=true;
    }
    
    void updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg){
        
        Eigen::Vector2d curr_pos(msg->markers[0].pose.position.x,msg->markers[0].pose.position.y);
        double curr_time=msg->markers[0].header.stamp.toSec();
        if(curr_time==last_time_){
            return;
        }
        
        if(is_init_){
            vel_= Eigen::Vector2d::Zero();
            is_init_=false;
        }else{
            vel_=(curr_pos-pos_)/(curr_time-last_time_);
        }
        
        pos_=curr_pos;
        last_time_=curr_time;
        //std::cout<<topic_name_<<"vel="<<vel_<<std::endl;
    }

    void updateOdomCallback2(nav_msgs::Odometry::ConstPtr msg){
        
        
        Eigen::Vector2d curr_pos(msg->pose.pose.position.x,msg->pose.pose.position.y);

        double curr_time=msg->header.stamp.toSec();//msg->markers[0].header.stamp.toSec();
        
        if(curr_time==last_time_){
            return;
        }
        
        if(is_init_){
            vel_= Eigen::Vector2d::Zero();
            is_init_=false;
        }else{
            vel_=(curr_pos-pos_)/(curr_time-last_time_);
        }
        
        pos_=curr_pos;
        last_time_=curr_time;
        //std::cout<<topic_name_<<"vel="<<vel_<<std::endl;
    }

    ~ObstacleNode(){std::cout<<"DELTETED obstacle node "<<std::endl;}

    typedef std::shared_ptr<ObstacleNode> Ptr;

};


class ObstacleCollector{
private:
public:
    ObstacleCollector(ros::NodeHandle &nh);
    ~ObstacleCollector(){}; 
    ros::NodeHandle node_;
    std::vector<ObstacleNode::Ptr> obstacles_nodes_;  // please use pointer to push_back into vector
    //std::vector<ObstacleNode> obstacles_nodes_;


    ros::Publisher vis_triangle_pub_;
    ros::Timer vis_timer_; 

    void publish_vis_triangle(const ros::TimerEvent& );
    void visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);


};






















#endif