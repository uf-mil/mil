# Represents a single object labeled from LabelMe

string name                   # Name/Type of object entered into labeling program

string attributes             # text entered into attribute box of labeling program

geometry_msgs/Point[] polygon # vertices of polygon drawn in labeling program
                              # X,Y represent pixel cordinates in labeled image
                              # Z is ignored, should be 0
uint8 id                      # unique id (for this frame) of this object
uint8 parent_id               # id of parent object if this is a 'part' of another object, otherwise 0
