import tf

# Define the quaternion
quaternion = (0.707, 0.0, 0.0, 0.707)  # (w, x, y, z)

# Convert quaternion to RPY angles
rpy = tf.(quaternion)

# Extract roll, pitch, and yaw angles
roll = rpy[0]
pitch = rpy[1]
yaw = rpy[2]

# Print the RPY angles
print("Roll:", roll)
print("Pitch:", pitch)
print("Yaw:", yaw)
