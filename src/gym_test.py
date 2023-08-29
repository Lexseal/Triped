import gymnasium as gym
import triped_sim
import numpy as np
import time

env = gym.make("triped_sim-v0")
info = env.reset()
print(env.observation_space)
print(env.action_space)

start_time = time.time()
for _ in range(10000):
  # we are gonna do a position controller
  info = env.step([np.sin(time.time()/10)]*9)
  # print(info)
  # time.sleep(0.01)
print(10000/(time.time()-start_time))
