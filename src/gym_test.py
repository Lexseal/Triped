import gymnasium as gym
import triped_sim
import numpy as np
import time

env = gym.make("triped_sim-v0")
info = env.reset()
print(env.observation_space)
print(env.action_space)

while True:
  info = env.step(np.zeros(9))
  print(info)
  time.sleep(1)
