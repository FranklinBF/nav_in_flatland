
#ifndef WP_GENERATOR_H
#define WP_GENERATOR_H
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <flatland_navigation/node.h>
#include <vector>

#include <flatland_navigation/Subgoal.h>
#include <std_srvs/EmptyRequest.h>


//geometry_msgs/PoseStamped
class WaypointGenerator
{
    private:

    protected:
    ros::NodeHandle _nh;
    
    public:
    // parameter for select subgoal
    double _look_ahead_distance;
    double _tolerance;
    int _visted_waypoint_id; 

    // topic publisher
    ros::Publisher _subgoal_pub;
    ros::Publisher _subgoal_vis_pub;
    
    // topic subscriber
    ros::Subscriber _odom_sub;
    ros::Subscriber _goal_sub;

    // service client
    ros::ServiceClient _global_plan_client;
    
    // service server
    ros::ServiceServer _subgoal_generator_server; 

    // global variable
    bool _is_odom_ready;                    // check if odom is availble
    nav_msgs::Odometry _odom;               // save _odom
    nav_msgs::Path _global_plan;            // save_global_plan
    std::vector<WaypointNodePtr> _wpNodes;  // save waypoints, subgoal will be selected from them
    
    geometry_msgs::PoseStamped _subgoal;    // selected _subgoal, which will be published 
    

    // not neccessary
    //ros::Time trigged_time;
    //geometry_msgs::PoseStamped _goal;       // save global goal

    

    WaypointGenerator(const ros::NodeHandle& node_handle,double look_ahead_distance, double tolerance);
	~WaypointGenerator(){};

    

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void goal_callback( const geometry_msgs::PoseStamped msg);

    bool get_next_subgoal();
    bool subgoal_service(flatland_navigation::Subgoal::Request& request, flatland_navigation::Subgoal::Response& response);

    void publish_subgoal();
    void publish_subgoal_vis();
    
    bool get_current_robot_pose(geometry_msgs::PoseStamped &current_pose);
    double metric_dist(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2);

};


#endif /* WP_GENERATOR_H */