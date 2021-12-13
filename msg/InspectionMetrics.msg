# Metrics captured during an iteration of the inspection's pipeline. Some
# values are always set, some are only set if certain requirements are met.

# Whether or not the robot reached the desired detection pose
bool move_sucessful

# Whether or not the can was successfully detected and inspected
# WARNING: value is only set if move_sucessful==true
bool can_detection_sucessful

# Whether the can was nominal (true) or defective (false)
# WARNING: value is only set if can_detection_sucessful==true
bool can_nominal

# The measured pose of the robot camera frame with respect to the world frame.
# In other words, the measured pose when the robot reached its detection pose.
# WARNING: value is only set if move_sucessful==true
geometry_msgs/Pose transform_WC_measured

# The initial expected/approximate pose of this can, whether its nominal or
# defective
geometry_msgs/Pose transform_WDi_expected

# The measured pose of the defective can with respect to the world frame
# WARNING: value is only set if can_nominal==false
geometry_msgs/Pose transform_WDi_measured
