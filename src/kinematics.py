from scipy.spatial.transform import Rotation as R
import numpy as np
from matplotlib import pyplot as plt

def to_hom(reg):
  if reg.shape == (3,):
    return np.append(reg, 1)
  elif reg.shape == (3,3):
    hom = np.eye(4)
    hom[:3, :3] = reg
    return hom
  else:
    raise Exception(f"invalid input {reg}")

def from_hom(hom):
  if hom.shape == (4,):
    return hom[:3] / (hom[3] if hom[3] != 0 else 1)
  elif hom.shape == (4,4):
    return hom[:3, :3] / (hom[3,3] if hom[3,3] != 0 else 1)
  else:
    raise Exception(f"invalid input {hom}")
  
def rot_k(k, t):
  """ k is the axis of rotation, t is the angle of rotation """
  k = k/np.linalg.norm(k)
  return to_hom(R.from_rotvec(t*k).as_matrix())

def forward_kinematics(root_basis, offset, t1, t2, t3):
  first_axis = root_basis[0:3, -1]
  second_axis = np.array([1, 0, 0])
  third_axis = second_axis.copy()
  T = rot_k(first_axis, t1)
  T = T.dot(to_hom(root_basis))
  T[0:2, 3] = offset
  frame1 = T.dot(np.eye(4))
  T = T.dot(rot_k(second_axis, t2))
  frame2 = T.dot(np.eye(4))
  trans_third = rot_k(third_axis, t3)
  trans_third[1, 3] = -35.25
  T = T.dot(trans_third)
  frame3 = T.dot(np.eye(4))
  return frame1, frame2, frame3, from_hom(T.dot(np.array([0, -39, 0, 1])))

def forward_kinematics_1(t1, t2, t3):
  root_basis = np.array([[np.sqrt(3)/2, 0, 1/2          ],\
                         [1/2         , 0, -np.sqrt(3)/2],\
                         [0           , 1, 0            ]])
  offset = np.array([48.5, 10])
  return forward_kinematics(root_basis, offset, t1, t2, t3)

def forward_kinematics_2(t1, t2, t3):
  root_basis = np.array([[-np.sqrt(3)/2, 0, 1/2         ],\
                         [1/2         ,  0, np.sqrt(3)/2],\
                         [0           ,  1, 0           ]])
  offset = np.array([-32.8, 37])
  return forward_kinematics(root_basis, offset, t1, t2, t3)

def forward_kinematics_3(t1, t2, t3):
  root_basis = np.array([[0 , 0, -1],\
                         [-1, 0,  0],\
                         [0 , 1,  0]])
  offset = np.array([-15.5, -47])
  return forward_kinematics(root_basis, offset, t1, t2, t3)

def draw_base_link(ax, T=np.eye(4)):
  pts = np.array([[0             , 50 , 0],\
                  [25*np.sqrt(3) , -25, 0],\
                  [-25*np.sqrt(3), -25, 0],\
                  [0             , 50 , 0]])
  pts = [from_hom(T.dot(np.append(pt, 1))) for pt in pts]
  ax.plot(*zip(*pts))

def plot_frames(ax, fs, eef, T=np.eye(4)):
  fs = [T.dot(f) for f in fs]  # apply more transformations
  pts = [from_hom(f.dot(np.array([0, 0, 0, 1]))) for f in fs]
  for f, pt in zip(fs, pts):
    ax.quiver(*pt, *10*from_hom(f.dot(np.array([1, 0, 0, 0]))), color='r')
    ax.quiver(*pt, *10*from_hom(f.dot(np.array([0, 1, 0, 0]))), color='g')
    ax.quiver(*pt, *10*from_hom(f.dot(np.array([0, 0, 1, 0]))), color='b')
  eef = from_hom(T.dot(np.append(eef, 1)))
  ax.scatter(*eef)
  ax.plot(*zip(*pts))

ax = plt.figure().add_subplot(projection='3d')
ax.set_xlim((-100, 100))
ax.set_ylim((-100, 100))
ax.set_zlim((-100, 100))

fks = [forward_kinematics_1, forward_kinematics_2, forward_kinematics_3]
leg_frames = []
eefs = []
for i, fk in enumerate(fks):
  f1, f2, f3, eef = fk(0,0,0)
  if i == 0:
    f1, f2, f3, eef = fk(np.pi/6,0,0)
  leg_frames.append([f1, f2, f3])
  eefs.append(eef)

# calculate the cg of the triangle formed by the end effectors
cg = np.mean(eefs, axis=0)
# find a transformation that first rotates the triangle about the cg to be parallel with the x-y plane
u = eefs[1]-eefs[0]
v = eefs[2]-eefs[0]
n = np.cross(u, v)
# now the transformation is the one that rotates n to be parallel with the z-axis
axis_rot = np.cross(n, np.array([0, 0, 1]))
theta = np.arccos(np.dot(n, np.array([0, 0, 1]))/np.linalg.norm(n))
if theta < 0.02:
  T = np.eye(4)
else:
  T = rot_k(axis_rot, theta)
# finally, add translation and apply the transformation to the end effectors
T[2, 3] = -cg[2]

draw_base_link(ax, T)
for fs, eef in zip(leg_frames, eefs):
  plot_frames(ax, fs, eef, T)
plt.show()
