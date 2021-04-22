# Custom message with traffic light data.
#
# This message is used to give input to the navigation node 
# which can be used to take decisions for speed/navigation.
string type  # string value indicating the type of traffic light: (traffic light).
string color # color of the sign: (red, yellow, green).
float32 distance # approximation of distance that can be used for prioritizatin.