import numpy as np


# Take smallest and biggest x coordinate and their respective Z to calculate angle in x
reflector_height = 70

x_max = 1600
z_for_x_max = 20
x_min = 1500
z_for_x_min = 10

# Calculate the offset in x
z_diff = z_for_x_max - z_for_x_min
x_diff = x_max - x_min
alpha = np.arctan(z_diff/x_diff)
l = np.tan(alpha) * reflector_height
x_offset = np.cos(alpha) * l
print(x_offset)