import gymnasium
import numpy as np
import pybullet as p
import pybullet_data
import os

class TripedSimSimpleEnv(gymnasium.Env):
  def __init__(self, render_mode="human"):
    self.action_space = self._action_spec()
    self.observation_space = self._observation_spec()

    output = p.GUI if render_mode == "human" else p.DIRECT
    self.physicsClient = p.connect(output)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # this adds urdf paths

    self.revolute_indices = [3, 8, 13, 5, 10, 15, 7, 12, 17]

    self.xyz = None
    self.q = None
    self.omega = None

    self.frames = 0
    self.frame_limit = 10000

  def step(self, action):
    # print("action: ", action)
    self.frames += 1
    p.setJointMotorControlArray(self.triped, self.revolute_indices, p.POSITION_CONTROL, targetPositions=action)
    p.stepSimulation()
    observation = self._get_observation()
    reward = self._get_reward()
    return observation, reward, self.frames>self.frame_limit, False, {}

  def _load_models(self):
    self.plane = p.loadURDF("plane.urdf")
    current_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(current_dir, "../../../drawings/Triped_description/urdf/Triped.xacro")
    self.triped = p.loadURDF(urdf_path, [0, 0, 0.1])  # drop the model at z height 0.1m
    p.setGravity(0, 0, -10)

  def reset(self, seed=None, options=None):
    self.frames = 0
    p.resetSimulation()
    self._load_models()
    p.stepSimulation()
    return self._get_observation(), {}

  def render(self):
    pass

  def _get_reward(self):
    """ must have called _get_observation before calling this function """
    # first check if we are within +-0.1m of the x-axis
    if abs(self.xyz[1]) > 0.1:
      return -abs(self.xyz[1])
    else:
      # get the z-axis of the baselink w.r.t. world
      # to do so, we first the get the rotation matrix of the baselink w.r.t. world
      rot_matrix = p.getMatrixFromQuaternion(self.q)
      discount = rot_matrix[-1]  # discount is dotting the frame z-axis with the world z-axis
      if discount < 0:
        return discount*abs(self.xyz[0])
      return discount*self.xyz[0]  # the further on x-axis the better

  def _get_observation(self):
    (self.xyz, self.q) = p.getBasePositionAndOrientation(self.triped)
    self.omega = p.getBaseVelocity(self.triped)[1]
    obs = list(self.q)  # start with the orientation
    obs += list(self.omega)
    obs.append(self.xyz[-1])  # add the height
    # # get the joint angles for all 9 revolute joints
    # for joint_index in self.revolute_indices:
    #   joint_state = p.getJointState(self.triped, joint_index)
    #   obs.append(joint_state[0])
    # print("obs: ", obs)
    return np.array(obs)

  def _observation_spec(self):
    """
    we have the quaternion of the baselink w.r.t. word,
    the height of the baselink w.r.t. word, and
    """
    return gymnasium.spaces.Box(-1, 1, shape=(8,))

  def _action_spec(self):
    # we have 9 revolute joints
    lower_bound = np.array([-np.pi/6]*3+[0]*3+[-np.pi/3]*3)
    upper_bound = np.array([np.pi/6]*3+[np.pi/3]*3+[0]*3)
    return gymnasium.spaces.Box(lower_bound, upper_bound)
