import rospy
from geometry_msgs.msg import Twist
from gym import spaces




self.action = np.array([0.0, 0.0])
        self.v_max_ = 0.8 # ?1.5?
        self.w_max_ = 1.2
        self.__possible_actions = {
            0: [0.0, -self.w_max_],
            1: [self.v_max_, 0.0],
            2: [0.0, self.w_max_],
            3: [self.v_max_, self.w_max_ / 2],
            4: [self.v_max_, -self.w_max_ / 2],
            5: [0.0, 0.0]
            # 6: [0.09, 0.0],
        }
        action_size = len(self.__possible_actions)

        action_space = spaces.Discrete(action_size)

class ActionCollector():
    def __init__(self):
        self.v_max_ = 0.8 
        self.w_max_ = 1.2
        self.possible_actions={
            0: [0.0, -self.w_max_],
            1: [self.v_max_, 0.0],
            2: [0.0, self.w_max_],
            3: [self.v_max_, self.w_max_ / 2],
            4: [self.v_max_, -self.w_max_ / 2],
            5: [0.0, 0.0]
        }
        self.N_DISCRETE_ACTIONS = len(self.possible_actions)
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        
        
        
    def get_cmd_vel_(self, action):
        vel_msg = Twist()
        vel_msg.linear.x = action[0]
        vel_msg.angular.z = action[1]
        return vel_msg