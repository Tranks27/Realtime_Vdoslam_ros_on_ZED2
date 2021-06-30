# Summary
University of Sydney Undergraduate thesis.
Thesis paper found on: https://drive.google.com/file/d/1gsRZIXxiakbXKQpu4TQrqEdD6hvJWXPG/view?usp=sharing

Modified from https://github.com/jessemorris/multi_robot_perception to do realtime processing on a live camera ZED2.

## Requirements
- Ubuntu 18.04
- ROS Melodic
- Pytorch(1.4.0)
- GPU (at least 4GB of memory, preferred 5GB)
- At least 12 GB RAM
- OpenCV & OpenCV Contrib (at least v3.2, tested with 3.4.11)
- ZED SDK v3.5.0
- apex (for mask-rcnn)
- and others depending on your current setup


## Building
### Mask Rcnn
1. src/mask_rcnn must be build with 
```
python3 install.py install
``` 
This is just a setup.py script but I have changed the name to not conflict with the setup.py script needed by ROS. This builds and installs the CUDA files needed by this network.
2. src/VDO_SLAM/ is not a ROS package and must be build exclusively with cmake:
```
mkdir build
cd build && cmake ..
sudo make install
```
3. install modules from the requirements.txt
```$ pip3 install -r requirements.txt```
[Note: not included in requirements.txt are memory-profiler, so install it by yourself]

4. Several other packages and libraries are also required. 
- nlohmann
- apex
- tf2_sensor_msgs, vision_msgs
- CPARSE
- Eigen
- pytorch/torchvision
- Cparse
- and more

Please refer to src/installation_help.txt for the full list and installation guides that I've followed.

### Flownet2
- To run FlowNet2, pip3 install these (path, imageio). Inside src/flownet/
```$ pip3 install -r requirements.txt```
[Note: cupy takes time to install, don’t stress]

### realtime_vdo_slam
1. Build the g2o library found in  src/VDO_SLAM/vdo_slam_g2o
```
cd vdo_slam_g2o
mkdir build && cd build
sudo make install -j8 OR sudo make install -j($nproc) 
```
2. Build the VDO_SLAM library
```
cd VDO_SLAM
mkdir build && cd build
sudo make install -j8 OR sudo make install -j($nproc) 
```
A dynamic library (vdo_slam.so) will be built to the lib folder in the same directory. All the header files will be installed to /usr/local/include/vdo_slam/. This is so the ROS packages can find the header files globally but currently the dynamic library is linked with relative paths (see the CMakeLists.txt of RealtimeVdoSlam).

3. Building everything else. Go to src/
```
catkin build 
```
should make everything else.

## Current Overview

### Flow Net
ROS wrapper and simplified implentation of FlowNet Lite (https://github.com/sniklaus/pytorch-liteflownet). Runs inference using a current and previous image and responds with a image NxMx2 flow matrix

### Mask Rcnn 
ROS wrapper and simplified implentation of Mask RCNN (https://github.com/facebookresearch/maskrcnn-benchmark). Runs inference on a single image and responds with a image containing masks and labels

### Realtime Vdo Slam
Modified libray from the original VDO SLAM found at: https://github.com/halajun/VDO_SLAM

After installation is done as described in the building section above, the following commands will run the complete project.
```
$ roslaunch zed_wrapper zed2.launch
$ roslaunch realtime_vdo_slam flowNmask.launch
$ roslaunch realtime_vdo_slam ros_vdo_slam.launch
```
To see the groundtruth trajectory of the camera, replace the first command with
```
$ roslaunch vdo_display_rviz display_zed2.launch
```

ROS wrapper for the VDO-SLAM library. This starts the program in the following manner
1. launch the ros node for zed2 camera
2. launch the preprocessing modules flowNet and Mask-RCNN
3. launch the vdo slam

The modified VDO-SLAM library outputs a summary of each analysed frame including
- Estiamted camera pose
- Estimated camera motion
- A list of objects in each frame (pose, velocity, semantic label, tracking label)

### Known error and fixes
- cuda out of memory 
fix: reduce the image input size or use larger GPU memory (>4GB). Here ZED2 camera allows us to downsample the image input, so we did that (70% of HD720 size). 

-undefined reference to: vtable ...
fix: In CmakeList.txt 1) make sure the library  .cpp files are put behind the executables 2) manually add -lvdo_slam library in the project Cmakelist.txt and add -lvdo_slam_g2o in the VDO_SLAM CmakeList.txt.

