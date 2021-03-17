## MaskRcnn for ROS

## Building

When changing this module, additional Cuda layers are added underneath the original architecture. See `deprcated/docs/INSTALL.md` To install this run 
```
$sudo python3 install.py install
```
NOTE: the `install.py` file is a standard setup.py style python script but I have changed the name of it so it does not conflict with the ROS setup file

## Program and Running

Mask R-CNN pixel level semantic segmentation model implemeted in PyTorch. This model was trained on the COCO dataset. See `src/mask_rcnn/configs` for other model options. 

This module requires python3 and cannot be spun up (at the time of writing) using `roslaunch` (see `python_service_starter` module). 

```
$python3 scripts/mask_rcnn_rospy.py
```
Will load the network on the device (either GPU or CPU) and can be interacted with a service `maskrcnn/analyse_image`. The service expects a `MaskRcnnVdoSlam` service message in the form:

```
sensor_msgs/Image input_image

---
sensor_msgs/Image output_mask
sensor_msgs/Image output_viz
string[] labels
int32[] label_indexs
bool success
```
- `input_image` should be composed from a RGB uint8 image (numpy or cv::Mat)
- `output_mask` will be mono uint8 image of the same shape, where each pixel represnets a semantic mask corresponding to some label
- `output_viz` will be RGB uint8 image where search mask has been coloured for easier visualisation
- `labels` is a list of labels corresponding to the label of the masks found in the input image
- `label_indexs` is a list of integers corresponding to the semantic labels at the same index as the labels

A set of other services are provided in order to query the entire list of categories trained on (so a separate node with no knowledge of the labels used by this data, can query the labels and index's). See `mask_rcnn::MaskRcnnhInterface` for an example.

### C++ interface

A C++ class has been provided `mask_rcnn::MaskRcnnhInterface` to interact directly with the python node using OpenCV `cv::Mat` for image handling. This simply interacts with the services described above but also can start the script via the `python_service_starter` interface, if you dont want to manualy call python to start the node up every time.

### Original Work:
Wrapper for MaskRcnn proposed in 

```
@misc{massa2018mrcnn,
author = {Massa, Francisco and Girshick, Ross},
title = {{maskrcnn-benchmark: Fast, modular reference implementation of Instance Segmentation and Object Detection algorithms in PyTorch}},
year = {2018},
howpublished = {\url{https://github.com/facebookresearch/maskrcnn-benchmark}},
note = {Accessed: [Insert date here]}
}
```

See `depricated` for the original README.md.

