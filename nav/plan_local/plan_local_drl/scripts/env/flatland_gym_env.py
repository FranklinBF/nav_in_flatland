#! /usr/bin/env python
import gym
from gym import spaces
from stable_baselines.common.env_checker import check_env

from observation_collector import ObservationCollector
from action_collector import ActionCollector
from reward_collector import RewardCollector

import rospy
from geometry_msgs.msg import Twist

import time

class FlatlandEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self):
    super(FlatlandEnv, self).__init__()
    # Define action and observation space
    # They must be gym.spaces objects
  
    # observation collector
    self.observation_collector = ObservationCollector()
    self.observation_space=self.observation_collector.get_observation_space()
    
    # action collector
    self.action_collector=ActionCollector()
    self.action_space=self.action_collector.get_action_space()
    
    # reward collector
    self.reward_collector=RewardCollector()

    # action agent publisher
    #self.agent_action_pub = rospy.Publisher('drl_action_agent', Twist, queue_size=1)
    self.agent_action_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
  def step(self, action):
    # encode action to cmd_vel
    cmd_vel=self.action_collector.get_cmd_vel(action_id=action)
    
    # publish cmd_vel
    self.agent_action_pub.publish(cmd_vel)

    # wait for new observations
    obs=self.observation_collector.get_observations()

    # check if done
    done = self.is_done(obs)

    #calculate reward
    reward=self.reward_collector.get_reward(action)

    # info
    info = {}

    return obs, reward, done, info


  def reset(self):
    # set task
    
    # set goal, start global plan
    
    # get observation
    obs=self.observation_space.sample()
    return obs  # reward, done, info can't be included

  def render(self, mode='human'):
    ...
  
  def close (self):
    ...
  
  def is_done(self,obs):
    done=False
    scan=obs[0]
    robot_pose=obs[1]
    subgol_pose=obs[2]
    if(robot_pose[0]>10):
      done=True
    return done

if __name__ == '__main__':
    
    rospy.init_node('flatland_gym_env', anonymous=True)
    print("start")

    flatland_env=FlatlandEnv()
    check_env(flatland_env, warn=True)

    # init env
    obs = flatland_env.reset()

    # run model
    n_steps = 200
    for step in range(n_steps):
      action=flatland_env.action_space.sample() #action, _states = model.predict(obs)

      obs, rewards, done, info = flatland_env.step(action)

      time.sleep(2)
      