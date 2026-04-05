# Diagnostic.msg
std_msgs/Header header

# Joint space
sensor_msgs/JointState command          # commanded position + velocity
sensor_msgs/JointState state            # measured position + velocity

# Task space
geometry_msgs/PoseStamped setpoint      # desired EEF pose
geometry_msgs/PoseStamped pose          # current EEF pose (from FK)

# Errors
sensor_msgs/JointState state_error      # command - state
geometry_msgs/Vector3 position_error    # p_desired - p_current
geometry_msgs/Vector3 orientation_error # axis * angle (rad)
float64 position_error_norm
float64 orientation_error_norm
