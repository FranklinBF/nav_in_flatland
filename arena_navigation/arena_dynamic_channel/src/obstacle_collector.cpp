#include "arena_dynamic_channel/obstacle_collector.h"


ObstacleCollector::ObstacleCollector(ros::NodeHandle &nh){
    node_=nh;
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    std::cout<<"init start"<<std::endl;
    std::string str1="obs_dynamic";//"odom_walker";//

    for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;

        if (info.name.find(str1) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
            //ObstacleNode::Ptr obs_node_ptr;
            //obs_node_ptr.reset(new ObstacleNode(node_,info.name));
            //obs_node_ptr=std::make_shared<ObstacleNode>(node_,info.name);
            //printf("%d\n", obs_node_ptr.use_count());
            obstacles_nodes_.emplace_back(std::make_shared<ObstacleNode>(node_,info.name));

            //std::cout << "---------------------------------" << std::endl;
            //printf("%d\n", obstacles_nodes_.back().use_count());
            //std::cout << "---------------------------------" << std::endl;

        }
    }
    
    vis_triangle_pub_= node_.advertise<visualization_msgs::Marker>("vis_triangle", 20);

    //std::cout << "********************--------------" << std::endl;
    //obstacles_nodes_.clear();
    //std::cout << "********************--------------" << std::endl;
    vis_timer_ = node_.createTimer(ros::Duration(0.01), &ObstacleCollector::publish_vis_triangle, this);
}

void ObstacleCollector::visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub){
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

void ObstacleCollector::publish_vis_triangle(const ros::TimerEvent&){

    std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>>  ptr_pair_sets;
    for(size_t i=0;i<obstacles_nodes_.size();++i){
        size_t j=i+1;
        if(j>obstacles_nodes_.size()-1){
            j=0;
        }
        std::pair<Eigen::Vector2d,Eigen::Vector2d> line(obstacles_nodes_[i]->getPosition(),obstacles_nodes_[j]->getPosition());
        ptr_pair_sets.push_back(line);
    }
    visualizeLines(ptr_pair_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_);
}

int main(int argc, char** argv){
    std::cout<<"hhh"<<std::endl;
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle nh("");

    ros::Rate r=ros::Rate(100);
    ObstacleCollector obs=ObstacleCollector(nh);
    // while(ros::ok()){
    //     //obs.publish_vis_triangle();
    //     //ROS_INFO("Done once");
    //     for(size_t i=0;i<obs.test_nodes_.size();i++){
    //         std::cout << "obs:"<< obs.test_nodes_[i]->a<< std::endl;
    //     }
    //     ros::spinOnce();
    //     r.sleep();
    // }

    
    ros::spin();
    

    
}

