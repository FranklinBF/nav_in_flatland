#ifndef _NODE_H_
#define _NODE_H_



#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>

#define inf 1>>20
struct WaypointNode;
typedef WaypointNode* WaypointNodePtr;

struct WaypointNode
{
    int id;
    geometry_msgs::PoseStamped poseStamped;
    double dist_to_goal;
    double dist_to_robot;

    WaypointNode(int index, geometry_msgs::PoseStamped point_pose, geometry_msgs::PoseStamped goal_pose){
        id=index;
        poseStamped=point_pose;
        
        double dx,dy;
        dx=point_pose.pose.position.x-goal_pose.pose.position.x;
        dy=point_pose.pose.position.y-goal_pose.pose.position.y;

        dist_to_goal=sqrt(dx*dx+dy*dy);
        dist_to_robot=inf;
        
    }

    WaypointNode(){};
    ~WaypointNode(){};

};

















#endif