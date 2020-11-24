#! /usr/bin/env python

from logging import error
import time
import random
import math
import numpy as np
import yaml
from collections import OrderedDict

import rospy
import rospkg

#from tf.transformations import *
import tf
from geometry_msgs.msg import TransformStamped

from flatland_msgs.srv import DeleteModel,DeleteModelRequest
from flatland_msgs.srv import SpawnModel,SpawnModelRequest
from flatland_msgs.srv import MoveModel,MoveModelRequest

from flatland_msgs.msg import Model

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap,GetMapRequest

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped,PoseStamped
from geometry_msgs.msg import Twist, Point 
from std_msgs.msg import Bool
from std_srvs.srv import SetBool,Empty




class TaskGenerator():
    def __init__(self,ns,robot_name,robot_radius):
        self.NS=ns
        self.ROBOT_RADIUS = robot_radius
        self.ROBOT_NAME=robot_name
        
        
        # model files
        self._flatland_models_path = rospkg.RosPack().get_path('flatland_models')

        self._map=OccupancyGrid()
        self._freespace=None
        self._static_obstacles=[]
        self._dynamic_obstacles=[]
        self._peds=[]

        self._path = Path()

        # services client
        self._service_client_get_map=rospy.ServiceProxy("%s/static_map" % self.NS,GetMap)
        self._service_client_move_robot_to = rospy.ServiceProxy('%s/move_model' % self.NS, MoveModel)
        self._service_client_delete_model = rospy.ServiceProxy('%s/delete_model' % self.NS, DeleteModel)
        self._service_client_spawn_model = rospy.ServiceProxy('%s/spawn_model' % self.NS, SpawnModel)

        # topic subscriber
        #self._map_sub=rospy.Subscriber("%s/map" % self.NS, OccupancyGrid, self._map_callback)
        
        
        

        # topic publisher
        self._initialpose_pub=rospy.Publisher('%s/initialpose' % self.NS, PoseWithCovarianceStamped, queue_size=1)
        self._goal_pub = rospy.Publisher('%s/move_base_simple/goal' % self.NS, PoseStamped, queue_size=1)
        
        # tf
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._tf_listener=tf.TransformListener()
        
        # clear world

        
        

    """
    1. load map
    2. freespace 
    3. current position, goal position, robot radius
    4. ask for path
    5. place static obstacle in the path
    6. place dynamic obstacle in the path
    7. place robot in the path
    8. place peds in the path
    """

    """ Static obstacles """
    def remove_all_static_obstacles(self):
        for m in self._static_obstacles:
            srv_request=DeleteModelRequest()
            srv_request.name=m.name
            rospy.wait_for_service('%s/delete_model' % self.NS)
            result=self._service_client_delete_model.call(srv_request)
        self._static_obstacles=[]
    
    def spawn_static_obstacle(self, model_name, index, x,y,theta):
        srv_request = SpawnModelRequest()
        srv_request.yaml_path="%s/obstacles/%s"%(self._flatland_models_path,model_name)
        srv_request.name="stat_obj_%d" % index
        srv_request.ns = self.NS
        
        pose=Pose2D()
        pose.x=x
        pose.y=y
        pose.theta=theta
        srv_request.pose=pose

        rospy.wait_for_service('%s/spawn_model' % self.NS)

        try:
            self._service_client_spawn_model.call(srv_request)
        except (TypeError):
            print('Spawn object: TypeError.')
            return
        except (rospy.ServiceException):
            print('Spawn object: rospy.ServiceException. Closing serivce')
            try:
                self._service_client_spawn_model.close()
            except AttributeError:
                print('Spawn object close(): AttributeError.')
                return
            return
        except AttributeError:
            print('Spawn object: AttributeError.')
            return
        
        obstacle=Model()
        obstacle.yaml_path=srv_request.yaml_path
        obstacle.name=srv_request.name
        obstacle.ns=srv_request.ns
        obstacle.pose=srv_request.pose
        self._static_obstacles.append(obstacle)
        return 

    def generate_random_static_obstacle_yaml(self):
        model_name="random.model.yaml"
        yaml_path="%s/obstacles/%s"%(self._flatland_models_path,model_name)
        
        
        type_list=["circle","polygon"]#
        min_obstacle_radius=0.5
        max_obstacle_radius=1.5
        min_obstacle_vert=3
        max_obstacle_vert=6

        # define body
        body={}
        body["name"]="random"
        body["pose"]=[0,0,0]
        body["type"]="dynamic"
        body["color"]=[0.2, 0.8, 0.2, 0.75]
        body["footprints"]=[]

        # define footprint 
        n=random.randint(0,len(type_list))
        if n==len(type_list):
            n=n-1
        random_type=type_list[n]

        f={}
        f["type"]=random_type
        f["density"]=0
        f["layers"]=["all"]
        f["collision"]='true'
        f["sensor"]="false"

        if f["type"]=="circle":
            f["radius"]=random.uniform(min_obstacle_radius,max_obstacle_radius)
        elif f["type"]=="polygon":
            f["points"]=[]
            random_num_vert=random.randint(min_obstacle_vert,max_obstacle_vert)
            random_length=random.uniform(min_obstacle_radius,max_obstacle_radius)
            #radius_y=random.uniform(min_obstacle_radius,max_obstacle_radius)
            for i in range(random_num_vert):
                angle=2*math.pi*(float(i)/float(random_num_vert))
                vert=[math.cos(angle)*random_length, math.sin(angle)*random_length]
                print(vert)
                print(angle)
                f["points"].append(vert)
        
        body["footprints"].append(f)
        
        # define plugins
        plugin_tween={}
        plugin_tween["name"]="Tween"
        plugin_tween["type"]="Tween"
        plugin_tween["body"]="random"
        plugin_tween["delta"]=[5, 3, 3.141592]
        plugin_tween["duration"]="2.0"
        plugin_tween["mode"]="yoyo"
        plugin_tween["easing"]="quadraticInOut"

        # define dict_file
        dict_file = {'bodies':[body],"plugins":[]}
        with open(yaml_path, 'w') as file:
            documents = yaml.dump(dict_file, file)

    """ map & freespace & valid position check """
    def get_static_map(self):
        srv_request=GetMapRequest()
        rospy.wait_for_service('%s/static_map' % self.NS)
        try:
            self._map=self._service_client_get_map(srv_request).map
            nmap=np.array(self._map.data)
            self._freespace=np.where(nmap == 0)[0]

        except error as e:
            print('Get static map error:',e)
        
        return 

    def get_random_pos_on_map(self,map,freespace):
        def coordinate_pixel_to_meter(index_grid,map):

            # index_grid to column_grid, row_grid
            column=int(index_grid%map.info.width)
            row=int((index_grid-column)/map.info.width)

            # colum_grid, row_grid to x,y in meter
            x=column*map.info.resolution+map.info.origin.position.x   
            y=row*map.info.resolution+map.info.origin.position.y

            return x,y    
           
        # choose random index_grid in freespace
        index=random.randint(0,len(freespace))
        index_grid=freespace[index]
        x,y=coordinate_pixel_to_meter(index_grid,map)
        

        # check the random pos is valid or not
        while(not self.is_pos_valid(x,y,map)):
            index=random.randint(0,len(freespace))
            index_grid=freespace[index]
            x,y=coordinate_pixel_to_meter(index_grid,map)
            
        theta = random.uniform(-math.pi, math.pi) # in radius
        print(" found ")
        return x, y, theta

    def is_pos_valid(self,x,y,map):
        # in pixel
        cell_radius = int((self.ROBOT_RADIUS+0.1)/map.info.resolution)
        x_index = int((x-map.info.origin.position.x)/map.info.resolution)
        y_index = int((y-map.info.origin.position.y)/map.info.resolution)

        # check occupancy around (x_index,y_index) with cell_radius
        for i in range(x_index-cell_radius, x_index+cell_radius, 1):
            for j in range(y_index-cell_radius, y_index+cell_radius, 1):
                index = j * map.info.width + i
                if index>=len(map.data):
                    print("a")
                    return False
                try:
                    value=map.data[index]
                except IndexError:
                    print("IndexError: index: %d, map_length: %d"%(index, len(map.data)))
                    print("b")
                    return False
                if value!=0:
                    print("c=",value)
                    return False
        return True

    """ Robot init position """
    def set_random_robot_pos(self):
        success = False
        x, y, theta=0.0,0.0,0.0
        while not success:
            x, y, theta = self.get_random_pos_on_map(self._map,self._freespace)
            self.set_robot_pos(x, y, theta)
            success=True
        return x, y, theta

    def set_robot_pos(self,x,y,theta):
        # call service move_model
        pose=Pose2D()
        pose.x=x
        pose.y=y
        pose.theta=theta

        srv_request=MoveModelRequest()
        srv_request.name=self.ROBOT_NAME
        srv_request.pose=pose

        
        # call service
        rospy.wait_for_service('%s/move_model'% self.NS)
        self._service_client_move_robot_to(srv_request)
            
        # publish robot position
        self.pub_initial_position(x,y,theta)
   
    def pub_initial_position(self, x, y, theta):
        """
        Publishing new initial position (x, y, theta) --> for localization
        :param x x-position of the robot
        :param y y-position of the robot
        :param theta theta-position of the robot
        """
        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = rospy.get_rostime()
        initpose.header.frame_id = "map"
        initpose.pose.pose.position.x = x
        initpose.pose.pose.position.y = y
        #quaternion = Quaternion(axis=[0, 0, 1], angle=theta)
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta,axes="rzyx")
        initpose.pose.pose.orientation.x = quaternion[0]
        initpose.pose.pose.orientation.y = quaternion[1]
        initpose.pose.pose.orientation.z = quaternion[2]
        initpose.pose.pose.orientation.w = quaternion[3]
        
        initpose.pose.covariance = [0.0000,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.00000,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.00000,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.00000,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.00000,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.00000]
        self._initialpose_pub.publish(initpose)
        return
    
    """ Robot goal position """
    def set_random_robot_goal(self):
        x, y, theta = self.get_random_pos_on_map(self._map,self._freespace)
        self.pub_robot_goal(x, y, theta)
        return x, y, theta

    def pub_robot_goal(self,x,y,theta):
        goal = PoseStamped()
        goal.header.stamp = rospy.get_rostime()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta,axes="rzyx")
        
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        self._goal_pub.publish(goal)
        return 
        
        


    """ Test purpose """
    def tf_listen_map_to_odom(self):
        # listen tf map->odom
        self._tf_listener.waitForTransform("map","odom",rospy.Time(0),rospy.Duration(3.0))
        (trans,rot)=self._tf_listener.lookupTransform("map","odom",rospy.Time(0))
        print(trans)
        
        # publish tf map->odom
        for i in range(1,1):
            translation=(10,2,0)
            rotation=rot
            time=rospy.Time.now()
            child="odom"
            parent="map"
            self._tf_broadcaster.sendTransform(translation, rotation, time, child, parent)
            
    
        


if __name__ == '__main__':
    rospy.init_node('aaa', anonymous=True)
    ns=""
    robot_radius=0.5
    robot_name="myrobot"
    task=TaskGenerator(ns,robot_name,robot_radius)
    task.get_static_map()
    model_name="walker.model.yaml"
    
    '''
    for index in range(0,5):
        x, y, theta=task.get_random_pos_on_map(task._map,task._freespace)
        task.spawn_static_obstacle(model_name,index,x,y,theta)
    
    for i in range(0,5):
        task.set_random_robot_pos()
        task.set_random_robot_goal()
        time.sleep(10)
        
    '''
    for index in range(0,20):
        task.generate_random_static_obstacle_yaml()
        x, y, theta=task.get_random_pos_on_map(task._map,task._freespace)
        model_name="random.model.yaml"
        task.spawn_static_obstacle(model_name,index,x,y,theta)
    #time.sleep(10)
    #task.remove_all_static_obstacles()
    
    
    
    

