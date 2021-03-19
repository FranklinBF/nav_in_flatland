#ifndef _DYNAMIC_OBSTACLE_H
#define _DYNAMIC_OBSTACLE_H
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/master.h>

#include"boost/algorithm/string.hpp"

//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>


struct DynamicObstacleInfo{
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

    DynamicObstacleInfo(ros::NodeHandle &nh, std::string topic_name);
    ~DynamicObstacleInfo(){std::cout<<"DELTETED obstacle info object "<<std::endl;}
    
    void updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg);
    
    typedef std::shared_ptr<DynamicObstacleInfo> Ptr;
};










#endif