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
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # this adds urdf paths

    self.revolute_indices = [3, 8, 13, 5, 10, 15, 7, 12, 17]

  def step(self, action):
    for i, joint_index in enumerate(self.revolute_indices):
      p.setJointMotorControl2(self.triped, joint_index, p.POSITION_CONTROL, targetPosition=action[i])
    p.stepSimulation()
    observation = self._get_observation()
    reward = self._get_reward()
    return observation, reward, False, False, {}

  def _load_models(self):
    self.plane = p.loadURDF("plane.urdf")
    current_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(current_dir, "../../../drawings/Triped_description/urdf/Triped.xacro")
    self.triped = p.loadURDF(urdf_path, [0, 0, 0.1])  # drop the model at z height 0.1m

  def reset(self):
    p.resetSimulation()
    self._load_models()

  def render(self):
    pass

  def _get_reward(self):
    return 0

  def _get_observation(self):
    (xyz, q) = p.getBasePositionAndOrientation(self.triped)
    obs = list(p.getMatrixFromQuaternion(q))
    obs.append(xyz[-1])
    return np.array(obs)

  def _observation_spec(self):
    # we have the rotation matrix of the baselink w.r.t. word and its height
    return gymnasium.spaces.Box(-1, 1, shape=(10,))

  def _action_spec(self):
    # we have 9 revolute joints
    lower_bound = np.array([-np.pi/6]*3+[0]*3+[-np.pi/3]*3)
    upper_bound = np.array([np.pi/6]*3+[np.pi/3]*3+[0]*3)
    return gymnasium.spaces.Box(lower_bound, upper_bound)
