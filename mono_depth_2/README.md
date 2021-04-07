## MonoDepth 2 for ROS

MonoDepth2 is a self supervised monocular depth prediction framework using PyTorch. 

This module requires python3 and cannot be spun up (at the time of writing) using `roslaunch` (see `python_service_starter` module). 

```
$python3 scripts/mono_depth_rospy.py
```
Will load the network on the device (either GPU or CPU) and can be interacted with a service `monodepth2/analyse_image`. The service expects a `MonoDepth` service message in the form:

```
sensor_msgs/Image current_image

---
sensor_msgs/Image output_image
bool success
```
- `current_image` should be composed from a RGB uint8 image (numpy or cv::Mat)
- `output_image` will be mono uint16 image of the same shape as the input image and will be the resulting depth map inferred by the network

### C++ interface

A C++ class has been provided `mono_depth_2::MonoDepthInterface` to interact directly with the python node using OpenCV `cv::Mat` for image handling. This simply interacts with the services described above but also can start the script via the `python_service_starter` interface, if you dont want to manualy call python to start the node up every time.

### Original Work:
Wrapper for MonoDepth2 proposed in 

```
@article{monodepth2,
  title     = {Digging into Self-Supervised Monocular Depth Prediction},
  author    = {Cl{\'{e}}ment Godard and
               Oisin {Mac Aodha} and
               Michael Firman and
               Gabriel J. Brostow},
  booktitle = {The International Conference on Computer Vision (ICCV)},
  month = {October},
year = {2019}
}
```

See `src/depricated` for the original README.md.

