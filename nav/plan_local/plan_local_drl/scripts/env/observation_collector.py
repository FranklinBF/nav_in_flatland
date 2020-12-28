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

from gym import spaces



_L = 360  # lidar size
_RS = 3   # robotstate size
_G= 3    # goal size

class ObservationCollector():
    def __init__(self):

        # define observation_space
        self.observation_space = spaces.Tuple((
            spaces.Box(low=0, high=10, shape=(_L,), dtype=np.float32),
            spaces.Box(low=-10, high=10, shape=(_RS,), dtype=np.float32) ,
            spaces.Box(low=-10, high=10, shape=(_G,), dtype=np.float32) 
        ))

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
    
    def get_observation_space(self):
        return self.observation_space

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

        scan=self._scan.ranges.astype(np.float32)
        robot_pose=np.array([self._robot_pose.x,self._robot_pose.y,self._robot_pose.theta]).astype(np.float32)
        subgoal=np.array([self._subgoal.x,self._subgoal.y,self._subgoal.theta])
        obs = (scan, robot_pose,subgoal)  
        print("***"*10)
        print(obs[1])
        return obs
    
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
        obs=state_collector.get_observations()
        time.sleep(5)
        


    



