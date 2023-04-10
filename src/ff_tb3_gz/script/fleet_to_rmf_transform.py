#!/usr/bin/env python3

import numpy as np
from rmf_fleet_msgs.msg import Location

def calculate_fleet_to_rmf_transform(fleet_frame_location, rmf_frame_location):
    # Convert the fleet and RMF frame locations to NumPy arrays
    fleet_pos = np.array([fleet_frame_location.x, fleet_frame_location.y])
    rmf_pos = np.array([rmf_frame_location.x, rmf_frame_location.y])

    # Calculate the required translation to transform the fleet frame to the RMF frame
    translation = fleet_pos - rmf_pos

    # Calculate the required rotation to transform the fleet frame to the RMF frame
    theta = rmf_frame_location.yaw - fleet_frame_location.yaw
    rotation = theta

    # Calculate the required scaling factor to transform the fleet frame to the RMF frame

    scale = np.linalg.norm(rmf_pos - fleet_pos) / np.linalg.norm(fleet_pos)
    print(f'''{scale=}''')
    scale = np.linalg.norm(rmf_pos - fleet_pos)
    print(f'''{scale=}''')
    scale = np.linalg.norm(translation)
    print(f'''{scale=}''')

    return translation, rotation, scale


v1 = np.array([-1.895, -0.420])
v2 = np.array([20, -14])

T = np.outer(v2, v1) / np.linalg.norm(v1)**2
print(f'''{T=}''')


result = T.dot(v1)
print(f'{result=} (expected {v2=})')
exit()

# Define the fleet frame location
fleet_frame_location = Location()
fleet_frame_location.x = -1.895
fleet_frame_location.y = -0.420
fleet_frame_location.yaw = 0.0

# Define the corresponding RMF frame location
rmf_frame_location = Location()
rmf_frame_location.x = 20.0
rmf_frame_location.y = -14.0
rmf_frame_location.yaw = 0.0

# Calculate the required transformation parameters
translation, rotation, scale = calculate_fleet_to_rmf_transform(fleet_frame_location, rmf_frame_location)

# Print the transformation parameters
print("Translation: ", translation)
print("Rotation: ", rotation)
print("Scale: ", scale)


