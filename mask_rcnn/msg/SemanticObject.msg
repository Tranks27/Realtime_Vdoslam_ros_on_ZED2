Header Header
string label # the semantic label 
int32 label_index #the semantic label index as defined by the COCO dataset

int32 associated_frame
int32 tracking_label #the tracking ID to associated objects between frames. This will be filled out by VDO SLAM and then
# needs to be re associated with one of these objects
int32 semantic_instance #the semantic instance object

#center (x,y) will be starting x, y which is the bottom left corner of the image
#size_x and size_y will be with and height of image
vision_msgs/BoundingBox2D bounding_box