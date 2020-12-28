import gym
from gym import spaces
from plan_local_drl.scripts.env.observation_collector import ObservationCollector
from plan_local_drl.scripts.env.action_collector import ActionCollector



class FlatlandEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self, arg1, arg2, ...):
    super(FlatlandEnv, self).__init__()
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions:
    N_DISCRETE_ACTIONS=4
    self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
    
    # Example for using image as input:
    self.observation_space = spaces.Box(low=0, high=255,
                                        shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

    
    # observation collector
    self.observation_collector = ObservationCollector()
    
    # action collector
    self.action_collector=ActionCollector()
    self.action_space=self.action_collector.get_action_space()
    
    # action agent publisher
    self.agent_action_pub = rospy.Publisher('drl_action_agent' % (self.NS), Twist, queue_size=1)
    
  def step(self, action):
    ...
    return observation, reward, done, info
  def reset(self):
    ...
    return observation  # reward, done, info can't be included
  def render(self, mode='human'):
    ...
  def close (self):
    ...