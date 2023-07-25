import numpy as np
import transforms3d

def quaternion_to_axis_angle(quaternion):
    # Normalize the quaternion
    quaternion = np.array(quaternion) / np.linalg.norm(quaternion)
    
    # Extract the scalar and vector parts of the quaternion
    scalar = quaternion[0]
    vector = quaternion[1:]
    
    # Calculate the angle
    angle = 2 * np.arccos(scalar)
    
    # Calculate the axis of rotation
    axis = vector / np.sin(angle / 2)
    
    return axis, angle

# Example usage
quaternion = [0,0.4226, -0.9063, 0]  # Example quaternion [w, x, y, z]
axis, angle = quaternion_to_axis_angle(quaternion)
print("Axis:", axis)
print("Angle:", np.degrees(angle))



import numpy as np


def quaternion_to_rpy(quaternion):
    # Normalize the quaternion
    quaternion /= np.linalg.norm(quaternion)

    # Convert quaternion to RPY angles
    rpy = transforms3d.euler.quat2euler(quaternion, axes='sxyz')

    return rpy

# Example usage
quaternion = np.array([0.924, 0.383, 0.0, 0.0])  # Example quaternion [w, x, y, z]

rpy = quaternion_to_rpy(quaternion)
print('Roll: {:.2f} degrees'.format(np.degrees(rpy[0])))
print('Pitch: {:.2f} degrees'.format(np.degrees(rpy[1])))
print('Yaw: {:.2f} degrees'.format(np.degrees(rpy[2])))
