std_msgs/Header header  # header of the message
string type             # type of the node (robot pose or landmark position)
string color            # color of the cone, leave empty for robot pose nodes
float32 x               # x coordinate (map frame)
float32 y               # y coordinate (map frame)
float32 theta           # orientation only for robot poses, leave empty for landmark nodes
int32 id                # id of the node