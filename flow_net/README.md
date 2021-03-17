## Flow Net Lite for ROS

FlowNetLite is a model for generating dense optical flow using PyTorch. 

This module requires python3 and cannot be spun up (at the time of writing) using `roslaunch` (see `python_service_starter` module). 

```
$python3 scripts/flow_net_rospy.py
```
Will load the network on the device (either GPU or CPU) and can be interacted with a service `flownet/analyse_image`. The service expects a `FlowNet` service message in the form:

```
sensor_msgs/Image previous_image
sensor_msgs/Image current_image

---
sensor_msgs/Image output_image
sensor_msgs/Image output_viz
bool success
```
- `current_image` should be composed from a RGB uint8 image (numpy or cv::Mat) representing the current image.
- `previous)image` should be composed from a RGB uint8 image (numpy or cv::Mat) representing the image from the previous frame.
- `output_image` will be the predicted dense optical flow of form N x M x 2, encoded with 32FC2. 
- `output_viz` is the output flow map, transformed into RGB space so it can be visualised

### C++ interface

A C++ class has been provided `flow_net::FlowNetInterface` to interact directly with the python node using OpenCV `cv::Mat` for image handling. This simply interacts with the services described above but also can start the script via the `python_service_starter` interface, if you dont want to manualy call python to start the node up every time.

### Original Work:
Wrapper for LiteFlowNet proposed in 

```
@InProceedings{hui18liteflownet,    
 author = {Tak-Wai Hui and Xiaoou Tang and Chen Change Loy},    
 title = {{LiteFlowNet: A Lightweight Convolutional Neural Network for Optical Flow Estimation}},    
 booktitle = {{Proceedings of IEEE Conference on Computer Vision and Pattern Recognition (CVPR)}},    
 year = {2018},  
 pages = {8981--8989},
 url = {http://mmlab.ie.cuhk.edu.hk/projects/LiteFlowNet/} 
}
```

and provided on github at https://github.com/twhui/LiteFlowNet

See `depricated` for the original README.md.

