#include "flatland_navigation/waypoint_generator.h"

using namespace std;
using bfmt = boost::format;




WaypointGenerator::WaypointGenerator(const ros::NodeHandle& node_handle,double look_ahead_distance, double tolerance)
    : _nh(node_handle),_look_ahead_distance(look_ahead_distance),_tolerance(tolerance)
{
    // Services client
	std::string global_plan_service_name = "/move_base/NavfnROS/make_plan";  
    _global_plan_client= _nh.serviceClient<nav_msgs::GetPlan>(global_plan_service_name);  

    // Service server
    _subgoal_generator_server= _nh.advertiseService("subgoal", &WaypointGenerator::subgoal_service, this);

    // Subscribe
    _odom_sub = _nh.subscribe("odom", 10, &WaypointGenerator::odom_callback,this);
    _goal_sub = _nh.subscribe("goal", 10, &WaypointGenerator::goal_callback,this);

    //Publisher
    _subgoal_pub = _nh.advertise<geometry_msgs::PoseStamped>("subgoal", 1,true);
    _subgoal_vis_pub = _nh.advertise<geometry_msgs::PoseArray>("subgoal_vis", 10);      

}

void WaypointGenerator::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // set odom_ready=true
    _is_odom_ready = true;
    // update _odom
    _odom = *msg;
}

void WaypointGenerator::goal_callback(const geometry_msgs::PoseStamped msg){
    // clear previous waypointNodes 
    _wpNodes.clear();
    //_global_plan=new nav_msgs::Path();
    // reset _visted_waypoint_index
    _visted_waypoint_id=0;
    
    // check if odom/localization is ready
    if (!_is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }

    //trigged_time = ros::Time::now(); //odom.header.stamp;
    
    // if the goal position is within the map
    if (msg.pose.position.z == 0) {
        geometry_msgs::PoseStamped start;
        geometry_msgs::PoseStamped goal;
        // set goal
        goal = msg;
        //_goal=goal; // set_goal
        
        // set start
        bool ret=get_current_robot_pose(start);

        // call get_Plan service from move_base global planner
        if(ret==true){
            nav_msgs::GetPlan srv;
            srv.request.start=start;
            srv.request.goal=goal;
            srv.request.tolerance=_tolerance;
            if (_global_plan_client.call(srv))
            {   
                // set global_plan
                _global_plan=srv.response.plan;
                
                ROS_INFO("[waypoint_generator]:Global Path found");
                
                // get wpNodes from global plan
                int N=_global_plan.poses.size();
                for(int i=0;i<N;i++){
                    WaypointNodePtr wpNodePtr=new WaypointNode(i,_global_plan.poses.at(i),goal);
                    _wpNodes.push_back(wpNodePtr);
                   
                }

                // get first subgoal and publish it
                // if(get_next_subgoal()){
                //     publish_subgoal();
                //     publish_subgoal_vis();
                // }
            }
            else
            {
                ROS_WARN("[waypoint_generator]:Global Path not found");
    
            }
        }
    } else {
        
            ROS_WARN("[waypoint_generator]:invalid goal, out of map range");
    }

}

bool WaypointGenerator::subgoal_service(flatland_navigation::Subgoal::Request& request, flatland_navigation::Subgoal::Response& response) {
  
  if(get_next_subgoal()){

      publish_subgoal();
      publish_subgoal_vis();
      response.subgoal=_subgoal;
      response.success=true;
      response.message="success";
  }else{
      response.success=false;
      response.message="Failed to call subgoal";
  }
  return true;
}

bool WaypointGenerator:: get_next_subgoal(){
    
    //check if the global goal is received or not
    if(_wpNodes.size()==0){
        ROS_WARN("[waypoint_generator][subgoal]: Failed to set subgoal, because no global goal is received");
        return false;
    }
    // get current robot location
    geometry_msgs::PoseStamped current;
    if (bool ret=get_current_robot_pose(current)==false){
        ROS_WARN("[waypoint_generator][subgoal]: Failed to set subgoal, because cannot get current location");
        return false;
    }

    // init the subgoal_id as endgoal_id
    int subgoal_id=_wpNodes.size()-1;
    
    // check if robot is near to the goal & within look_ahead_distance
    double dist_to_goal=metric_dist(_wpNodes.at(subgoal_id)->poseStamped,current);
    if (dist_to_goal<_look_ahead_distance){
        
        // set subgoal as endgoal
        _subgoal=_wpNodes.at(subgoal_id)->poseStamped;
        ROS_INFO("[waypoint_generator][subgoal]: subgoal find, Last subgoal");
        return true;
    }
    
    // get nearst unvisted waypoint along the global path
    double dist_min=2000000000;
    int nearest_id=0;
    for(int i=0;i<_global_plan.poses.size();i++){
        // calculate all the distance between each waypoint to the robot
        _wpNodes.at(i)->dist_to_robot=metric_dist(_wpNodes.at(i)->poseStamped,current);
        
        // update the nearest_id
        if(_wpNodes.at(i)->dist_to_robot < dist_min){
            dist_min=_wpNodes.at(i)->dist_to_robot;
            nearest_id=i;
        }
    }

    // if the robot position is off every global path waypoint with look_head_distance
    // then replan
    if(dist_min>_look_ahead_distance){
        ROS_INFO("[waypoint_generator][subgoal]: replan");
        goal_callback(_wpNodes.at(_wpNodes.size()-1)->poseStamped);
        
        return get_next_subgoal();
    }

    
    // update _visted_waypoint_id, which locate the robot location w.r.t the global path
    if(nearest_id > _visted_waypoint_id){
        // only update the id that have not been visited
        _visted_waypoint_id=nearest_id;
    }
    
    // pick up the next subgoal point with distance at _look_ahead_distance
    for(int i=_visted_waypoint_id;i<_global_plan.poses.size();i++){
        
        // if the waypoint is around the look_ahead_range
        if((_wpNodes.at(i)->dist_to_robot>_look_ahead_distance-_tolerance) 
            && (_wpNodes.at(i)->dist_to_robot<_look_ahead_distance+_tolerance)){
            
            // select the waypoint that with lowest id
            if(i < subgoal_id){
                subgoal_id=i;
            }
        }
    }

    // set subgoal as endgoal
    _subgoal=_wpNodes.at(subgoal_id)->poseStamped;
    ROS_INFO("[waypoint_generator][subgoal]: subgoal found, intermediate subgoal");
    return true;
}

void WaypointGenerator:: publish_subgoal_vis(){
    // init a PoseArray
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = std::string("map");
    poseArray.header.stamp = ros::Time::now();

    // push the subgoal pose into PoseArray 
    geometry_msgs::Pose subgoal_pose;
    subgoal_pose=_subgoal.pose;
    poseArray.poses.push_back(subgoal_pose);

    // publish subgoal_vis
    _subgoal_vis_pub.publish(poseArray);
}

void WaypointGenerator:: publish_subgoal(){
    //publish subgoal pose
    _subgoal.header.stamp=ros::Time::now();
    _subgoal_pub.publish(_subgoal);
}

bool WaypointGenerator::get_current_robot_pose(geometry_msgs::PoseStamped &current){
    // init tf_buffer
    tf2_ros::Buffer tf_buffer;
    
    // init tf_listner and save tf to tf_buffer
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // init a transform as container for output tf
    geometry_msgs::TransformStamped transform_base_to_map;

    // try to get robot_base tf in map
    double start=ros::WallTime::now().toSec();
    while(ros::WallTime::now().toSec()-start<0.2){
        try{
            transform_base_to_map=tf_buffer.lookupTransform("map","base_footprint",ros::Time(0),ros::Duration(1.0));
            current.header.stamp=ros::Time::now();
            current.header.frame_id = "map";
            current.pose.position.x=transform_base_to_map.transform.translation.x;
            current.pose.position.y=transform_base_to_map.transform.translation.y;
            current.pose.position.z=transform_base_to_map.transform.translation.z;
            current.pose.orientation=transform_base_to_map.transform.rotation;
            return true;
        }catch(tf2::TransformException &ex){
            ROS_WARN("[waypoint_generator] Failed to get robot current location error %s\n", ex.what());
        
        }
    }
    return false;
    

}

double WaypointGenerator::metric_dist(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2){
	double dx,dy,dist;
    dx=pose1.pose.position.x-pose2.pose.position.x;
    dy=pose1.pose.position.y-pose2.pose.position.y;
    dist=sqrt(dx*dx+dy*dy);
	return dist; 
}
    
int main(int argc, char** argv) {
    cout<<"waypoint_generator node start"<<endl;
    ros::init(argc, argv, "waypoint_generator");
    
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle;
    ros::WallRate r(100);
    
    // get param
    double look_ahead_distance;// = 3;
    std::string param_look_ahead_distance = ros::this_node::getName() + "/look_ahead_distance";
    node_handle.getParam(param_look_ahead_distance, look_ahead_distance);
    ROS_ERROR("distance %s %.1f",param_look_ahead_distance,look_ahead_distance);
    

    double waypoint_tolerance;//=0.3;
    std::string param_waypoint_tolerance = ros::this_node::getName() + "/waypoint_tolerance";
    node_handle.getParam(param_waypoint_tolerance, waypoint_tolerance);
    ROS_ERROR("param_waypoint_tolerance %s %.1f",param_waypoint_tolerance,waypoint_tolerance);

    WaypointGenerator wg(node_handle,look_ahead_distance,waypoint_tolerance);
    
    ROS_INFO("Ready Subgoal Service server.");
    ros::spin();
    
    //ros::ServiceClient subgoal_client= node.serviceClient<flatland_navigation::Subgoal>("flatland_navigation/Subgoal"); 
    
    /* flatland_navigation::Subgoal srv;
            srv.request={};
            cout<<"Request"<<endl;
            cout<<wg._global_plan.poses.size()<<endl;
            if (subgoal_client.call(srv))
            {   
                cout<<srv.response.success<<endl;
                cout<<srv.response.message<<endl;
            }
            else
            {   
                cout<<srv.response.success<<endl;
                cout<<srv.response.message<<endl;
            }  */
    
    // while (ros::ok()) {
    //     if(q%100==1){
    //         wg.get_next_subgoal();
    //         wg.publish_subgoal();
    //         wg.publish_subgoal_vis();
    //     }
    //     q=q+1;
    //     ros::spinOnce();
    //     r.sleep();
    // }

    return 0;
}
