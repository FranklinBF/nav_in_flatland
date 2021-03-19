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

#include <vector>
#include <arena_mapping/mapping.h>


struct DynamicObstacleInfo{
private:
    GridMap::Ptr grid_map_;
    double resolution_;
public:
    ros::NodeHandle node_;
    std::string topic_name_;
    ros::Subscriber obs_odom_sub_;

    Eigen::Vector2d pos_;
    Eigen::Vector2d vel_;
    double last_time_;
    bool is_init_;

    double radius_=0.5;
    std::vector<Eigen::Vector2d> last_occ_set_;

    Eigen::Vector2d getPosition(){
        return pos_;
    }

    Eigen::Vector2d getVelocity(){
        return vel_;
    }

    DynamicObstacleInfo(ros::NodeHandle &nh, std::string topic_name, GridMap::Ptr grid_map);
    ~DynamicObstacleInfo(){std::cout<<"DELTETED obstacle info object "<<std::endl;}
    
    void updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg);
    
    void updateDynamicOcc();

    typedef std::shared_ptr<DynamicObstacleInfo> Ptr;
};










#endif