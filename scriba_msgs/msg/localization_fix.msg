# Absolute localization fix (2D)

Header header
float32 x
float32 y
float32 theta		# Yaw angle

# Row-major representation of the 3x3 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, yaw angle)
float32[9] covariance
