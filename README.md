# Summary
University of Sydney Undergraduate thesis (Ongoing) 

Modified from https://github.com/jessemorris/multi_robot_perception to do realtime processing on a live camera ZED2.

## Requirements
- ROS Melodic
- GPU (at least 4GB of memory, preferred 5GB)
- At least 12 GB RAM
- OpenCV & OpenCV Contrib (at least v3.2, tested with 3.4.11)
- apex (for mask-rcnn)
- 

## Building
1. _multi_robot_perception/mask_rcnn_ must be build with 
```
python3 install.py install
``` 
This is just a setup.py script but I have changed the name to not conflict with the setup.py script needed by ROS. This builds and installs the CUDA files needed by this network.
2. _multi_robot_perception/VDO_SLAM/_ is not a ROS package and must be build exclusively with cmake:
```
mkdir build
cd build && cmake ..
sudo make install
```
A dynamic library (vdo_slam.so) will be built to the lib folder in the same directory. All the header files will be installed to /usr/local/include/vdo_slam/. This is so the ROS packages can find the header files globally but currently the dynamic library is linked with relative paths (see the CMakeLists.txt of RealtimeVdoSlam).

3. Catkin build/catkin_make should make everything else.

## Current Overview

### Flow Net
ROS wrapper and simplified implentation of FlowNet Lite (https://github.com/sniklaus/pytorch-liteflownet). Runs inference using a current and previous image and responds with a image NxMx2 flow matrix

### Mask Rcnn 
ROS wrapper and simplified implentation of Mask RCNN (https://github.com/facebookresearch/maskrcnn-benchmark). Runs inference on a single image and responds with a image containing masks and labels

### Realtime Vdo Slam
Follow instructions in the README.ME file of the packge itself to install it correctly. After installation is complete, the following commands will run the complete project.
```
$ roslaunch zed_wrapper zed2.launch
$ roslaunch realtime_vdo_slam flowNmask.launch
$ roslaunch realtime_vdo_slam ros_vdo_slam.launch
```
ROS wrapper and input controller for the VDO-SLAM library. This starts all the networks (using service calls to the PSS) and runs the VDO-SLAM algorithm using a threadsafe queue, using the latest data inferred data. This allows the node to run up around 16 FPS. The modified VDO-SLAM library outputs a summary of each analysed frame including
- Estiamted camera pose
- Estimated camera motion
- A list of objects in each frame (pose, velocity, semantic label, tracking label)