import numpy as np
import gym
from gym import spaces


class GoLeftEnv(gym.Env):
  """
  Custom Environment that follows gym interface.
  This is a simple env where the agent must learn to go always left. 
  """
  # Because of google colab, we cannot implement the GUI ('human' render mode)
  metadata = {'render.modes': ['console']}
  # Define constants for clearer code
  LEFT = 0
  RIGHT = 1

  def __init__(self, grid_size=10):
    super(GoLeftEnv, self).__init__()

    # Size of the 1D-grid
    self.grid_size = grid_size
    # Initialize the agent at the right of the grid
    self.agent_pos = grid_size - 1

    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    n_actions = 2
    self.action_space = spaces.Discrete(n_actions)
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    self.observation_space = spaces.Box(low=0, high=self.grid_size,
                                        shape=(1,), dtype=np.float32)

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    # Initialize the agent at the right of the grid
    self.agent_pos = self.grid_size - 1
    # here we convert to float32 to make it more general (in case we want to use continuous actions)
    return np.array([self.agent_pos]).astype(np.float32)

  def step(self, action):
    if action == self.LEFT:
      self.agent_pos -= 1
    elif action == self.RIGHT:
      self.agent_pos += 1
    else:
      raise ValueError("Received invalid action={} which is not part of the action space".format(action))

    # Account for the boundaries of the grid
    self.agent_pos = np.clip(self.agent_pos, 0, self.grid_size)

    # Are we at the left of the grid?
    done = bool(self.agent_pos == 0)

    # Null reward everywhere except when reaching the goal (left of the grid)
    reward = 1 if self.agent_pos == 0 else 0

    # Optionally we can pass additional info, we are not using that for now
    info = {}

    return np.array([self.agent_pos]).astype(np.float32), reward, done, info

  def render(self, mode='console'):
    if mode != 'console':
      raise NotImplementedError()
    # agent is represented as a cross, rest as a dot
    print("." * self.agent_pos, end="")
    print("x", end="")
    print("." * (self.grid_size - self.agent_pos))

  def close(self):
    pass


from stable_baselines.common.env_checker import check_env

env = GoLeftEnv()
# If the environment don't follow the interface, an error will be thrown
check_env(env, warn=True)


env = GoLeftEnv(grid_size=10)

obs = env.reset()
env.render()

print(env.observation_space)
print(env.action_space)
print(env.action_space.sample())

GO_LEFT = 0
# Hardcoded best agent: always go left!
n_steps = 20
for step in range(n_steps):
  print("Step {}".format(step + 1))
  obs, reward, done, info = env.step(GO_LEFT)
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render()
  if done:
    print("Goal reached!", "reward=", reward)
    break

#################################################

from stable_baselines import DQN, PPO2, A2C, ACKTR
from stable_baselines.common.cmd_util import make_vec_env

# Instantiate the env
env = GoLeftEnv(grid_size=10)
# wrap it
env = make_vec_env(lambda: env, n_envs=1)

# Train the agent
model = ACKTR('MlpPolicy', env, verbose=1).learn(5000)

# Test the trained agent
obs = env.reset()
n_steps = 20
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=True)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render(mode='console')
  if done:
    # Note that the VecEnv resets automatically
    # when a done signal is encountered
    print("Goal reached!", "reward=", reward)
    break



    #############################################
import gym
from gym import spaces

class CustomEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self, arg1, arg2, ...):
    super(CustomEnv, self).__init__()
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions:
    self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
    # Example for using image as input:
    self.observation_space = spaces.Box(low=0, high=255,
                                        shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

    
    
    
    # action, _ = model.predict(obs, deterministic=True)
  def step(self, action):
    # 1. use the input action to update the states(observations) 
    #       action msg send to simulator server, publish with latch=true
    #       simulator server(diffDrive plugin) receive the action
    
    #       simulator server excute the action, step

    # 2. collect states(observations) from simulator server
    #       DRL start sending step request
    #       simulator server, step(excute the action) 
    #       simulator server, step(excute the action)
    #       simulator server, step...until sensor1 pub a msg out
    #       simulator server, step(excute the action)
    #       simulator server, step(excute the action)
    #       simulator server, step...until sensor2 pub a msg out
    #       simulator server, step...until sensor3 pub a msg out
    #       DRL server: collect all the needed sensor msgs: msg_sensor1,msg_sensor2,msg_sensor3
    #       DRL server: may use a msg_filter here
    #       DRL stop sending step request
    #       DRL server received all the states/observations

    # 3. check if reached the goal(done)
    #       done = check(states/observations)

    # 4. calculate reward
    #       train mode: need reward
    #       test mode: don't need reward actually

    # 5. optionally pass needed info
    #       info = {}

    return observation, reward, done, info
  def reset(self):
    ...
    return observation  # reward, done, info can't be included
  def render(self, mode='human'):
    ...
  def close (self):
    ...


# run env
env = GoLeftEnv(grid_size=10)

obs = env.reset()
env.render()

print(env.observation_space)
print(env.action_space)
print(env.action_space.sample())

GO_LEFT = 0
# Hardcoded best agent: always go left!
n_steps = 20
for step in range(n_steps):
  print("Step {}".format(step + 1))
  obs, reward, done, info = env.step(GO_LEFT)
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render()
  if done:
    print("Goal reached!", "reward=", reward)
    break