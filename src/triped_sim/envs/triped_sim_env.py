import gymnasium
import numpy as np
import pybullet as p
import pybullet_data
import os
from . import kinematics

class TripedSimSimpleEnv(gymnasium.Env):
  def __init__(self, render_mode="human"):
    self.action_space = self._action_spec()
    self.observation_space = self._observation_spec()

    output = p.GUI if render_mode == "human" else p.DIRECT
    self.physicsClient = p.connect(output)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # this adds urdf paths

    self.revolute_indices = [3, 8, 13, 5, 10, 15, 7, 12, 17]

    self.xyz = None
    self.vel = None
    self.q = None

    self.frames = 0
    self.frame_limit = 10000
    self.truncated_distance = 2  # meters
    self.ctrl_cost_const = 0.2
    self.moving_reward_const = 5

  def step(self, action):
    # check if action is within bounds
    self.frames += 1
    p.setJointMotorControlArray(self.triped, self.revolute_indices, p.POSITION_CONTROL,
                                targetPositions=action, forces=[0.2]*9)
    p.stepSimulation()
    observation = self._get_observation()
    reward = self._get_reward(action)
    self.last_cmd = list(action)
    dist_from_origin = np.linalg.norm(self.xyz)
    truncated = dist_from_origin > self.truncated_distance or self.frames > self.frame_limit
    return observation, reward, self.flipped, truncated, {}

  def _load_models(self):
    self.plane = p.loadURDF("plane.urdf")
    current_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(current_dir, "../../../drawings/Triped_description/urdf/Triped.xacro")
    self.triped = p.loadURDF(urdf_path, [0, 0, 0.1])  # drop the model at z height 0.1m
    p.setGravity(0, 0, -10)

  def reset(self, seed=None, options=None):
    self.frames = 0
    self.cumulative_reward = 0
    p.resetSimulation()
    self._load_models()
    p.stepSimulation()
    self.last_cmd = [0]*9
    self.flipped = False
    return self._get_observation(), {}

  def render(self):
    pass

  def _get_reward(self, action):
    """ must have called _get_observation before calling this function """
    """ mimicing the openai mujoco reward """
    rot_matrix = p.getMatrixFromQuaternion(self.q)
    dot_prod = rot_matrix[-1]  # dotting the frame z-axis with the world z-axis
    if dot_prod < 0:
      self.flipped = True
      return -1
    height_reward = 0.05 < self.xyz[2] < 0.08
    moving_reward = self.vel[0]*self.moving_reward_const
    control_cost = np.sum(np.abs(action-self.last_cmd))*self.ctrl_cost_const
    reward = dot_prod+height_reward+moving_reward-control_cost
    self.cumulative_reward += reward
    return reward
    
    # let's just make it stand first damn it

    # elif abs(self.xyz[1]) > 0.1:  # the expected height is around 0.1m
    #   return -abs(dot_prod*self.xyz[1]*(self.xyz[2]/0.1))
    # else:
    #   return dot_prod*self.xyz[0]*(self.xyz[2]/0.1)  # the further on x-axis the better

  def _get_observation(self):
    """
    we will need the following
    1. the orientation of the baselink w.r.t. world. this we can get from IMU in real world 4
    2. the height of the baselink w.r.t. world. this we can get by forward kinematics 1
    3. the frame of all the joints w.r.t. baselink. this we can also get from forward kinematics 153
    4. last command in radians 9
    """
    self.xyz, self.q = p.getBasePositionAndOrientation(self.triped)
    self.vel, _ = p.getBaseVelocity(self.triped)
    obs = list(self.q)  # start with the orientation
    obs.append(self.xyz[-1])  # add the height
    frame1, frame2, frame3, eef = kinematics.forward_kinematics_1(*(self.last_cmd[0:3]))
    obs += list(frame1.flatten())
    obs += list(frame2.flatten())
    obs += list(frame3.flatten())
    obs += list(eef.flatten())
    frame1, frame2, frame3, eef = kinematics.forward_kinematics_2(*(self.last_cmd[3:6]))
    obs += list(frame1.flatten())
    obs += list(frame2.flatten())
    obs += list(frame3.flatten())
    obs += list(eef.flatten())
    frame1, frame2, frame3, eef = kinematics.forward_kinematics_3(*(self.last_cmd[6:9]))
    obs += list(frame1.flatten())
    obs += list(frame2.flatten())
    obs += list(frame3.flatten())
    obs += list(eef.flatten())
    obs += self.last_cmd  # add the last command
    return np.array(obs)

  def _observation_spec(self):
    """
    we have the quaternion of the baselink w.r.t. word,
    the height of the baselink w.r.t. word, and
    """
    return gymnasium.spaces.Box(-200, 200, shape=(167,))

  def _action_spec(self):
    # we have 9 revolute joints
    lower_bound = np.array([-np.pi/6]*3+[0]*3+[-np.pi/3]*3)
    upper_bound = np.array([np.pi/6]*3+[np.pi/3]*3+[0]*3)
    return gymnasium.spaces.Box(lower_bound, upper_bound)
