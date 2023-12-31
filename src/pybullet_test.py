import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # this adds urdf paths
p.setGravity(0, 0, -10)

angles_sliders = [p.addUserDebugParameter(f"{i}", -0.5, 0.5, 0) for i in range(9)]
finish_button = p.addUserDebugParameter("Finish", 1, 0, 0)

plane = p.loadURDF('plane.urdf')
triped = p.loadURDF('../drawings/Triped_description/urdf/Triped.xacro', [0, 0, 0.3])

# number_of_joints = p.getNumJoints(triped)
# for joint_number in range(number_of_joints):
#     info = p.getJointInfo(triped, joint_number)
#     print(info)

print(p.getBasePositionAndOrientation(triped))

revolute_indices = [3, 8, 13, 5, 10, 15, 7, 12, 17]

start_time = time.time()
for _ in range(10000):
  angles = []
  try:
    for i, joint_index in enumerate(revolute_indices):
      angle = p.readUserDebugParameter(angles_sliders[i])
      angles.append(angle)
    p.setJointMotorControlArray(triped, revolute_indices, \
                                p.POSITION_CONTROL, targetPositions=angles, forces=[0.25]*9)
    
    finish = p.readUserDebugParameter(finish_button)
    if finish:
      p.resetSimulation()
      plane = p.loadURDF('plane.urdf')
      triped = p.loadURDF('../drawings/Triped_description/urdf/Triped.xacro', [0, 0, 0.1])
  except:
    print("Error reading user input")

  p.stepSimulation()
print(10000/(time.time()-start_time))
p.disconnect()
