import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # this adds urdf paths
p.setGravity(0, 0, -10)

angles_sliders = [p.addUserDebugParameter(f"{i}", -0.5, 0.5, 0) for i in range(9)]
finish_button = p.addUserDebugParameter("Finish", 1, 0, 0)

plane = p.loadURDF('plane.urdf')
triped = p.loadURDF('../drawings/Triped_description/urdf/Triped.xacro', [0, 0, 0.1])

# number_of_joints = p.getNumJoints(triped)
# for joint_number in range(number_of_joints):
#     info = p.getJointInfo(triped, joint_number)
#     print(info)

print(p.getBasePositionAndOrientation(triped))

revolute_indices = [3, 8, 13, 5, 10, 15, 7, 12, 17]

while True:
  for i, joint_index in enumerate(revolute_indices):
    try:
      angle = p.readUserDebugParameter(angles_sliders[i])
      p.setJointMotorControl2(triped, joint_index, p.POSITION_CONTROL, targetPosition=angle)
    except:
      print("Error reading user input")
  try:
    finish = p.readUserDebugParameter(finish_button)
    if finish:
      p.resetSimulation()
  except:
    pass
  p.stepSimulation()

p.disconnect()
