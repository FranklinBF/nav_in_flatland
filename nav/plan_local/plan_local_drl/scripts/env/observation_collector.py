#! /usr/bin/env python
import rospy
import random
import numpy as np

import time # for debuging

# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

# services
from flatland_msgs.srv import StepWorld,StepWorldRequest
from plan_msgs.srv import Subgoal,SubgoalRequest

# message filter
import message_filters

# for transformations
from tf.transformations import *

# #include <message_filters/subscriber.h>
# #include <message_filters/sync_policies/approximate_time.h>
# #include <message_filters/sync_policies/exact_time.h>
# #include <message_filters/time_synchronizer.h>

#   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>SyncPolicyImageOdom;
  
#   shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
#   shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
#   shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
#   SynchronizerImagePose sync_image_pose_;
#   SynchronizerImageOdom sync_image_odom_;

#   ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
#   ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
#   ros::Publisher unknown_pub_, depth_pub_;
#   ros::Timer occ_timer_, esdf_timer_, vis_timer_;

# depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 50));
# pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 25));
# sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
# sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));

# image_sub = message_filters.Subscriber('image', Image)
# info_sub = message_filters.Subscriber('camera_info', CameraInfo)

# ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
# ts.registerCallback(callback)
# rospy.spin()


class ObservationCollector():
    def __init__(self):
        # state_size_t = rospy.get_param("%s/rl_agent/scan_size"% ns)
        # state_size = (state_size_t,2, 1)
        # observation_space = spaces.Box(low=0, high=10, shape=state_size, dtype=np.float)
        
        # state_size_t = rospy.get_param("%s/rl_agent/scan_size"% ns) + rospy.get_param("%s/rl_agent/num_of_wps"%ns)*2
        # state_size = (1, state_size_t, 1)
        
        # [self.static_scan_, self.ped_scan_, self.merged_scan_, self.input_img_, self.wp_, self.twist_, self.__transformed_goal] = self.__state_collector.get_state()
       
        
        # flag of new sensor info
        self._flag_all_received=False

        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._subgoal =  Pose2D()

        # message_filter subscriber: laserscan, robot_pose
        self._scan_sub = message_filters.Subscriber("scan", LaserScan)
        self._robot_pose_sub = message_filters.Subscriber('robot_pose', PoseWithCovarianceStamped)
        
        # message_filters.TimeSynchronizer: call callback only when all sensor info are ready
        self.ts = message_filters.TimeSynchronizer([self._scan_sub, self._robot_pose_sub], 10)
        self.ts.registerCallback(self.callback_observation_received)
        
        # service clients
        self._service_name_step='/step_world'
        self._sim_step_client = rospy.ServiceProxy(self._service_name_step, StepWorld)

        self._service_name_subgoal="/subgoal"
        self._subgoal_client=rospy.ServiceProxy(self._service_name_subgoal, Subgoal)
    
    def get_observations(self):
        # reset flag 
        self._flag_all_received=False
        
        # sim a step forward until all sensor msg uptodate
        while(self._flag_all_received==False):
            self.call_service_takeSimStep()
        
        # collect observations    
        observations={}
        observations["scan"]=self._scan
        observations["robot_pose"]=self._robot_pose
        observations["subgoal"]=self._subgoal
        print("abc"*10)
        print(self._scan.ranges.shape)//362
        return observations
    
    def call_service_takeSimStep(self):
        request=StepWorldRequest()
        rospy.wait_for_service(self._service_name_step)
        try:
            response=self._sim_step_client(request)
            print("step service=",response)
        except rospy.ServiceException as e:
            print("step Service call failed: %s"%e)

    def call_service_askForSubgoal(self):
        request=SubgoalRequest()
        rospy.wait_for_service(self._service_name_subgoal)
        try:
            response=self._subgoal_client(request)
            print("subgoal result=",response.message)
            if(response.success):
                pose2d=self.pose3D_to_pose2D(response.subgoal.pose)
                return pose2d
            else:
                return self._subgoal   
        except rospy.ServiceException as e:
            print("subgoal Service call failed: %s"%e)
            return self._subgoal  
        
    def callback_observation_received(self,msg_LaserScan,msg_PoseWithCovarianceStamped):
        # process sensor msg
        self._scan=self.process_scan_msg(msg_LaserScan)
        self._robot_pose=self.process_pose_msg(msg_PoseWithCovarianceStamped)
        # ask subgoal service
        self._subgoal=self.call_service_askForSubgoal()
        self._flag_all_received=True
        
    def process_scan_msg(self, msg_LaserScan):
        # remove_nans_from_scan
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan
    
    def process_pose_msg(self,msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov=msg_PoseWithCovarianceStamped.pose
        pose=pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)
        
    # utils
    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d=Pose2D()
        pose2d.x=pose3d.position.x
        pose2d.y=pose3d.position.y
        quaternion=(pose3d.orientation.x,pose3d.orientation.y,pose3d.orientation.z,pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta=yaw
        return pose2d
        
   

if __name__ == '__main__':
    
    rospy.init_node('states', anonymous=True)
    print("start")

    state_collector=ObservationCollector()
    i=0
    r=rospy.Rate(100)
    while(i<=3):
        i=i+1
        print("****************************************8")
        obs=state_collector.get_observations()
        print(obs.keys())
        time.sleep(5)
        


    



