# Car Pose, Velocities and Acceleration

std_msgs/Header header

# NEx world-fixed frames:
# msg.pos.x = distance from origin north [m]
# msg.pos.y = distance from origin east [m]
# msg.pos.z = unused [x]
geometry_msgs/Vector3 pos   
geometry_msgs/Vector3 vel   # Body NEx Frame
geometry_msgs/Vector3 accel # Body NEx Frame

# rotation about z axis relative to north, clockwise in [rad]
float64 yaw 

# rotational rate about z axis relative to north, clockwise to [rad]
float64 yaw_rate

float64 steering
float64 currentsa