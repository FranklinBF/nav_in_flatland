#include <arena_plan_manager/plan_manager.h>




int main(int argc, char** argv) 
{
    std::cout<<"Plan manager node start"<<std::endl;
    ros::init(argc, argv, "plan_manager");

    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle nh("");
    PlanManager plan_manager;
    plan_manager.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tPlan manager successfully loaded for namespace\t"<<ns);
    
    ros::spin();
}