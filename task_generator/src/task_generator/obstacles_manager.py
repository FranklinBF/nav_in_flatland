from math import ceil
import math
import random
from typing import Union
import re
import yaml
import os 
from flatland_msgs.srv import DeleteModel, DeleteModelRequest
from flatland_msgs.srv import SpawnModel, SpawnModelRequest
from flatland_msgs.srv import MoveModel, MoveModelRequest
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
import rospy
import rospkg
from .utils import generate_freespace_indices, get_random_pos_on_map


class StaticObstaclesManager:
    """
    A manager class using flatland provided services to spawn, move and delete obstacles.
    """

    def __init__(self, map_: OccupancyGrid):

        # setup proxy to handle  services provided by flatland
        rospy.wait_for_service('move_model', timeout=20)
        rospy.wait_for_service('delete_model', timeout=20)
        rospy.wait_for_service('spawn_model', timeout=20)
        #rospy.wait_for_service('step_world', timeout=20)
        # allow for persistent connections to services
        self._srv_move_model = rospy.ServiceProxy('move_model', MoveModel, persistent=True)
        self._srv_delete_model = rospy.ServiceProxy('delete_model', DeleteModel, persistent=True)
        self._srv_spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel, persistent=True)
        #self._srv_sim_step = rospy.ServiceProxy('step_world', StepWorld, persistent=True)

        self.update_map(map_)
        # model files
        self._flatland_models_path = rospkg.RosPack().get_path('simulator_setup')
        self._static_obstacle_name_list = []

    def update_map(self, new_map: OccupancyGrid):
        self.map = new_map
        # a tuple stores the indices of the non-occupied spaces. format ((y,....),(x,...)
        self._free_space_indices = generate_freespace_indices(self.map)

    def register_obstacles(self, model_yaml_file_name: str, num_instance: int):
        """register the static obstacles and request flatland to respawn the them.

        Args:
            model_yaml_file (string): model file name
            num_instance (string): the number of the obstacle instance to be created.

        Raises:
            Exception:  Exception(f" failed to register obstacles")

        Returns:
            self: support chain creation.
        """
        static_obstacle_name_prefix = "static_obj"
        obstacle_type = model_yaml_file_name.split('.')[0]
        name_prefix = static_obstacle_name_prefix + '_' + obstacle_type
        count_same_type = sum(
            1 if obstacle_name.startswith(name_prefix) else 0
            for obstacle_name in self._static_obstacle_name_list)

        for instance_idx in range(count_same_type, count_same_type + num_instance):
            max_num_try = 2
            i_curr_try = 0
            while i_curr_try < max_num_try:
                spawn_request = SpawnModelRequest()
                spawn_request.yaml_path = os.path.join(self._flatland_models_path,"obstacles",model_yaml_file_name)
                spawn_request.name = f'{name_prefix}_{instance_idx:02d}'
                spawn_request.ns = rospy.get_namespace()
                # x, y, theta = get_random_pos_on_map(self._free_space_indices, self.map,)
                x = self.map.info.origin.position.x - self.map.info.resolution * self.map.info.height
                y = self.map.info.origin.position.y - self.map.info.resolution * self.map.info.width 
                theta = theta = random.uniform(-math.pi, math.pi)
                spawn_request.pose.x = x
                spawn_request.pose.y = y
                spawn_request.pose.theta = theta
                # try to call service
                response = self._srv_spawn_model.call(spawn_request)
                if not response.success:  # if service not succeeds, do something and redo service
                    rospy.logwarn(
                        f"spawn object {spawn_request.name} failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                    rospy.logwarn(response.message)
                    i_curr_try += 1
                else:
                    self._static_obstacle_name_list.append(spawn_request.name)
                    break
            if i_curr_try == max_num_try:
                raise Exception(f" failed to register obstacles")
        return self

    def move_obstacle(self, obstacle_name: str, x: float, y: float, theta: float):
        """move the obstacle to a given position

        Args:
            obstacle_name (str): [description]
            x (float): [description]
            y (float): [description]
            theta (float): [description]
        """

        assert obstacle_name in self._static_obstacle_name_list, "can't move the obstacle because it has not spawned in the flatland"
        # call service move_model

        srv_request = MoveModelRequest()
        srv_request.name = obstacle_name
        srv_request.pose.x = x
        srv_request.pose.y = y
        srv_request.pose.theta = theta

        self._srv_move_model(srv_request)

    def reset_pos_obstacles_random(self, active_obstacle_rate: float = 1, forbidden_zones: Union[list, None] = None):
        """randomly set the position of all the obstacles. In order to dynamically control the number of the obstacles within the
        map while keep the efficiency. we can set the parameter active_obstacle_rate so that the obstacles non-active will moved to the
        outside of the map

        Args:
            active_obstacle_rate (float): a parameter change the number of the obstacles within the map
            forbidden_zones (list): a list of tuples with the format (x,y,r),where the the obstacles should not be reset.
        """
        active_obstacle_names = random.sample(self._static_obstacle_name_list, int(
            len(self._static_obstacle_name_list) * active_obstacle_rate))
        non_active_obstacle_names = set(self._static_obstacle_name_list) - set(active_obstacle_names)

        # non_active obstacles will be moved to outside of the map
        resolution = self.map.info.resolution
        pos_non_active_obstacle = Pose2D()
        pos_non_active_obstacle.x = self.map.info.origin.position.x - resolution * self.map.info.width
        pos_non_active_obstacle.y = self.map.info.origin.position.y - resolution * self.map.info.width

        for obstacle_name in active_obstacle_names:
            move_model_request = MoveModelRequest()
            move_model_request.name = obstacle_name
            # TODO 0.2 is the obstacle radius. it should be set automatically in future.
            move_model_request.pose.x, move_model_request.pose.y, _ = get_random_pos_on_map(
                self._free_space_indices, self.map, 0.2, forbidden_zones)

            self._srv_move_model(move_model_request)

        for non_active_obstacle_name in non_active_obstacle_names:
            move_model_request = MoveModelRequest()
            move_model_request.name = non_active_obstacle_name
            move_model_request.pose = pos_non_active_obstacle
            self._srv_move_model(move_model_request)

    def generate_random_static_obstacle_yaml(self, model_name="random.model.yaml"):
        """generate a flatland model yaml file. The shape of the obstacle is randomly set.

        Args:
            model_name (str, optional): [description]. Defaults to "random.model.yaml".
        """

        yaml_path = "%s/obstacles/%s" % (self._flatland_models_path, model_name)

        type_list = ["circle", "polygon"]
        min_obstacle_radius = 0.5
        max_obstacle_radius = 1.5
        min_obstacle_vert = 3
        max_obstacle_vert = 6

        # define body
        body = {}
        body["name"] = "random"
        body["pose"] = [0, 0, 0]
        body["type"] = "dynamic"
        body["color"] = [1, 0.2, 0.1, 1.0]  # [0.2, 0.8, 0.2, 0.75]
        body["footprints"] = []

        # define footprint
        n = random.randint(0, len(type_list))
        if n == len(type_list):
            n = n - 1
        random_type = type_list[n]

        f = {}
        f["type"] = random_type
        f["density"] = 0
        f["layers"] = ["all"]
        f["collision"] = 'true'
        f["sensor"] = "false"

        if f["type"] == "circle":
            f["radius"] = random.uniform(min_obstacle_radius, max_obstacle_radius)
        elif f["type"] == "polygon":
            f["points"] = []
            random_num_vert = random.randint(min_obstacle_vert, max_obstacle_vert)
            random_length = random.uniform(min_obstacle_radius, max_obstacle_radius)
            # radius_y=random.uniform(min_obstacle_radius,max_obstacle_radius)
            for i in range(random_num_vert):
                angle = 2 * math.pi * (float(i) / float(random_num_vert))
                vert = [math.cos(angle) * random_length, math.sin(angle) * random_length]
                # print(vert)
                # print(angle)
                f["points"].append(vert)

        body["footprints"].append(f)

        # define dict_file
        dict_file = {'bodies': [body], "plugins": []}
        with open(yaml_path, 'w') as file:
            documents = yaml.dump(dict_file, file)

    def remove_obstacle(self, name: str):
        assert name in self._static_obstacle_name_list
        srv_request = DeleteModelRequest()
        srv_request.name = name
        response = self._srv_delete_model.call(srv_request)
        if not response.success:
            raise Exception(f"failed to remove the object with the name: {name}! ")

    def remove_obstacles(self, group_names: Union[list, None] = None):
        """remove all the obstacless belong to specific groups.

        Args:
            group_names (Union[list,None], optional): a list of group names. if it is None then all obstacles will
                be deleted. Defaults to None.
        """
        if group_names is None:
            group_names = '.'
        re_pattern = "^(?:" + '|'.join(group_names) + r')\w*'
        r = re.compile(re_pattern)
        to_be_removed_obstacles_names = list(filter(r.match, self._static_obstacle_name_list))
        for n in to_be_removed_obstacles_names:
            self.remove_obstacle(n)
