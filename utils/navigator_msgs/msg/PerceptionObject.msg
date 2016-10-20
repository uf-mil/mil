std_msgs/Header header            # Stamped with the time of the last sighting of the buoy

string type

#Constant Standard names to use
string DETECT_DELIVER_PLATFORM=shooter
string IDENTIFY_AND_DOCK=dock
string SCAN_THE_CODE=scan_the_code
string TOTEM=totem
string START_GATE_BUOY=start_gate
string UNKNOWN=unknown

geometry_msgs/Point position      # Estimated position of the buoy stamped (in the frame of the header)

uint32 id                         # If we want to label each buoy and keep track of it
uint8 confidence                  # 0-255 how confident are we that this is a buoy

float64 height                    # Height and width for the detected object
float64 width                     # Can be used for detecting different objects
float64 depth					  # Third dimension in x,y,z...

geometry_msgs/Point32[] points    # Points (in ENU frame) that belong to the buoy
#std_msgs/ColorRGBA color         # Average (?) color of the buoy
