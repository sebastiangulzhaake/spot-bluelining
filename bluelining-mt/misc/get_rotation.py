import numpy as np
from scipy.spatial.transform import Rotation as R

# # Vector in tracker's coordinate system in x/y/z with placeholder value for z
# a = [50,50, 0]
# # Vector in Spot's coordinate system in x/y/z where x is in the longditudinal direction,
# # y is transverse and z is a placeholder value
# # length of x is the same as the length of the vector in the tracker's coordinate system
# b = [np.sqrt((50**2)+(50**2)), 0, 0]
# # Rotation from a to b, returns a tuple where the first tuple object is a Rotation object giving us a 3x3 rotation matrix
# r = R.align_vectors(np.reshape(b,(1,3)), np.reshape(a,(1,3)))
# # Convert the Rotation object and prints it
# rot_full = r[0].as_matrix()
# print(rot_full)
# rot = np.array([[rot_full[0][0], rot_full[0][1]], [rot_full[1][0], rot_full[1][1]]])
# print(rot)
# c = np.array([[100],[100]])
# print(np.matmul(rot,c))

# x1 = np.arange(9.0).reshape((3, 3))
# print(x1)
# x2 = np.arange(3.0)
# print(x2)
# print(np.matmul(x1,x2))

# tracker_e = [-100, -50]
# tracker_e_flipped = np.flip(tracker_e)
# #print("e_update: ", e_flipped, end='\r')
# tracker_e_flipped_T = np.reshape(tracker_e_flipped, (2, 1))
# print(tracker_e_flipped_T)

# First sample test:  [  -61.561 -3098.473]
# First sample:  [  -61.561 -3098.473]
# Unstow command issued.
# Second sample:  [ -111.494 -2713.884]
# Vector is:  [-49.933 384.589]
# Length is:  387.81697153425347

# Vector in tracker's coordinate system in x/y/z with placeholder value for z
a = [-49.933, 384.589, 0]
a = [-384.589, -49.933, 0]
# Vector in Spot's coordinate system in x/y/z where x is in the longditudinal direction,
# y is transverse and z is a placeholder value
# length of x is the same as the length of the vector in the tracker's coordinate system
b = [387.81697153425347, 0, 0]
# Rotation from a to b, returns a tuple where the first tuple object is a Rotation object giving us a 3x3 rotation matrix
r = R.align_vectors(np.reshape(b,(1,3)), np.reshape(a,(1,3)))
# Convert the Rotation object and prints it
rot_full = r[0].as_matrix()
print(rot_full)
rot = np.array([[rot_full[0][0], rot_full[0][1]], [rot_full[1][0], rot_full[1][1]]])
angle = np.arccos(rot_full[0][0])
print("angle: ", np.rad2deg(angle))
print(rot)
c = np.array([[100],[100]])
print(np.matmul(rot,c))
