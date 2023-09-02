import numpy as np
from scipy.spatial.transform import Rotation as R

p1_t = np.array([-100.0, -100.0, 0.0])
p2_t = np.array([-50.0, -50.0, 0.0])

points_t = np.array([[p1_t], [p2_t]])

v_t = p2_t - p1_t

p1_v = np.array([0.0, 0.0, 0.0])
p2_v = np.array([np.sqrt(v_t[0]**2+v_t[1]**2), 0.0, 0.0])

points_v = np.array([[p1_v], [p2_v]])

#Obtain rotation from a to b

a = np.array([v_t[0], v_t[1], 0.0])
# Vector in Spot's coordinate system in x/y/z where x is in the longditudinal direction,
# y is transverse and z is a placeholder value
# length of x is the same as the length of the vector in the tracker's coordinate system
b = np.array([np.sqrt(v_t[0]**2+v_t[1]**2), 0.0, 0.0])
# Rotation between a and b, returns a tuple where the first tuple object is a Rotation object giving us a 3x3 rotation matrix
r = R.align_vectors(np.reshape(b,[1,3]), np.reshape(a,[1,3]))
# Convert the Rotation object and prints it
rot_full = (r[0].as_matrix())
rot = np.array([[rot_full[0][0], rot_full[0][1]], [rot_full[1][0], rot_full[1][1]]])

#Obtain translation from a to b

centroid_t = (p1_t + p2_t) * 0.5
centroid_v = (p1_v + p2_v) * 0.5

print("centroid_t: ", centroid_t)
print("centroid_v: ", centroid_v)

delta = centroid_v - centroid_t
print("delta: ", delta)

translation_vector = np.matmul(-rot_full, delta)

print("translation_vector: ", translation_vector)

print(rot_full)
print(rot)

full_transform = np.zeros((3,3))
full_transform[:2,:2] = rot
full_transform[0,2] = translation_vector[0]
full_transform[1,2] = translation_vector[1]
full_transform[2,2] = 1

c = np.reshape(p1_t,(3,1))

print("full transform: ", full_transform)
c[2] = 1
print("c: ", c)


print(np.matmul(full_transform, c))