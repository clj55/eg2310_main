import numpy as np
front_angle = 2
front_angles = range(-front_angle,front_angle+1,1)
left_front_angles = range(0, front_angle + 1, 1)
right_front_angles = range(-front_angle, 0, 1)
laser_range = np.array([0,1,2,3,4,5,6,7,8,9,10])
left_lri = (laser_range[left_front_angles]<float(stop_distance)).nonzero()
right_lri = (laser_range[right_front_angles]<float(stop_distance)).nonzero()
lri = np.concatenate((left_lri, right_lri), axis = None)