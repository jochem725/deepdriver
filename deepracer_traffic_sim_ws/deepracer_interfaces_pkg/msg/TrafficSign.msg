# Custom message with traffic sign data.
#
# This message is used to give input to the navigation node 
# which can be used to take decisions for speed/navigation.
string type     # string value indicating the type of sign detected
float32 value   # value indicator (-1.0 if not valid)
                # example use case: speed limit signs
float32 distance # approximation of distance that can be used for prioritizatin.