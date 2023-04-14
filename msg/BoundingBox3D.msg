# 3D bounding box message
# All units in meters
# This kind of message already exists in
# https://github.com/ros-perception/vision_msgs/blob/melodic-devel/msg/BoundingBox3D.msg
# but the corresponding array is not defined for ROS Melodic.

# Header to set a coordinate frame and timestamp
Header header

# Class of the detected object and probability it is that object 
int8 class_id
int8 probability	#to be a value between 1-100

# 3D position and orientation
geometry_msgs/Pose center

# Size/shape of the bounding box
geometry_msgs/Vector3 size
