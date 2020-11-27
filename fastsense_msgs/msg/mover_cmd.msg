# cmd types:
# simple_goal (go, alt, rot)
# forward
# tunnel
# relative (go_r, rot_r)

Header header
string type
uint32 sequence_id
geometry_msgs/Pose[] pose
float32 radius
string param