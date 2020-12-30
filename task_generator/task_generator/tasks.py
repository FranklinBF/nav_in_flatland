from abc import ABC, abstractmethod
from threading import Condition, Lock
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from nav_msgs.srv import GetMap
from .obstacles_manager import StaticObstaclesManager
from .robot_manager import RobotManager


class ABSTask(ABC):
    """An abstract class, all tasks must implement reset function.

    """

    def __init__(self, obstacles_manager: StaticObstaclesManager, robot_manager: RobotManager):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy("static_map", GetMap)

        rospy.Subscriber("map", OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.
        self._map_lock = Lock()

    @abstractmethod
    def reset(self):
        """
        a funciton to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_: OccupancyGrid):
        with self._map_lock:
            self.obstacles_manager.update_map(map_)
            self.robot_manager.update_map(map_)


class RandomTask(ABSTask):
    """ Evertime the start position and end position of the robot is reset.
    """

    def reset(self):
        """[summary]
        """
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    self.obstacles_manager.reset_pos_obstacles_random(
                        forbidden_zones=[
                            (start_pos.x,
                             start_pos.x,
                             self.robot_manager.ROBOT_RADIUS),
                            (goal_pos.x,
                             goal_pos.y,
                             self.robot_manager.ROBOT_RADIUS)])
                except Exception:
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can mannually set the goal postion of the robot
    """

    def __init__(self, obstacles_manager: StaticObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)
        # subscribe
        rospy.Subscriber("manual_goal", Pose2D, self._set_goal_callback)
        self._goal = Pose2D()
        self._new_goal_received = False
        self._manual_goal_con = Condition()

    def reset(self):
        while True:
            with self._map_lock:
                self.obstacles_manager.reset_pos_obstacles_random()
                self.robot_manager.set_start_pos_random()
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(self._new_goal_received, timeout=60)
                    if not self._new_goal_received:
                        raise Exception("TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager.publish_goal(self._goal.x, self._goal.y, self._goal.theta)
                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: Pose2D):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
        self._manual_goal_con.notify()

def get_predefined_task():
    robot_manager = RobotManager()
    static_obstacles_manager = StaticObstaclesManager()
    static_obstacles_manager.register_obstacles()
