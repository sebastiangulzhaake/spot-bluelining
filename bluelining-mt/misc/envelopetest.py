import numpy as np

reflector_pos = np.array([151,300.0])
goalpoint = np.array([250.0,300.0])
radius = 100

print((reflector_pos[0] - goalpoint[0])**2 + (reflector_pos[1] - goalpoint[1])**2)

print(radius**2)

if ((reflector_pos[0] - goalpoint[0])**2 + (reflector_pos[1] - goalpoint[1])**2 < radius**2):
    print("True")
else:
    print("False")