from stable_baselines3 import A2C
from rl_agent.envs.flatland_gym_env import FlatlandEnv
from task_generator.tasks import get_predefined_task
import rospy

# init node first
rospy.init_node("test")

# init task
task = get_predefined_task()

# init env
env = FlatlandEnv(task)

# model
model = A2C('MlpPolicy', env, verbose=1)


import time
s = time.time()
model.learn(total_timesteps=1000)
print("steps per second: {}".format(1000/(time.time()-s)))
#obs = env.reset()
#for i in range(1000):
#     action, _state = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render()
#     if done:
#       obs = env.reset()