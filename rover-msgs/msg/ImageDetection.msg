# Header for timestamp and frame id.
std_msgs/Header header

string label

float64 confidence

# the minimum coordinate (xmin, ymin)
geometry_msgs/Point min
# the maximum coordinate (xmax, ymax)
geometry_msgs/Point max
