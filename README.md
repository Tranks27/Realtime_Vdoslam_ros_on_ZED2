# Summary
Undergraduate Thesis project at the School of AMME (USyd)

Investigation into real-time/experimental useage of a visual dynamic SLAM pipeline using RGB camera before constructing a higher-level abstract SLAM map to aid in task-planning and semantic based reasoning. 

## Requirements

- ROS 1 (melodic)
- PyTorch (Version: 1.4.0)
- GPU (at least 5MB of memory)
- At least 12 GB RAM
- OpenCV Contrib (this is required for running VDO-SLAM). ROS does not natively come with the Contrib package so you will need to install this separately. I purged OpenCV from my computer with apt and installed OpenCV and OpenCV contrib from source. You MUST checkout version 3.2 for both OpenCV and OpenCV contrib otherwise this will conflict with ROS.
- apex (for mask-rcnn)
- (There are probbably others)

## Building
1. _multi_robot_perception/mask_rcnn_ must be build with ```python3 install.py install```. This is just a setup.py script but I have changed the name to not conflict with the setup.py script needed by ROS. This builds and installs the CUDA files needed by this network.
2. _multi_robot_perception/VDO_SLAM/_ is not a ROS package and must be build exclisively with cmake:
```
mkdir build
cd build && cmake ..
sudo make install
```
A dynamic library (vdo_slam.so) will be built to the lib folder in the same directory. All the header files will be installed to /usr/local/include/vdo_slam/. This is so the ROS packages can find the header files globally but currently the dynamic library is linked with relative paths (see the CMakeLists.txt of RealtimeVdoSlam).

3. Catkin build/catkin_make should make everything else.

## Current Overview

### Flow Net
ROS wrapper and simplified implentation of FlowNet Lite (https://github.com/sniklaus/pytorch-liteflownet). Sets up a ROS service to run inference using a current and previous image and responds with a image NxMx2 flow matrix

### Mask Rcnn 
ROS wrapper and simplified implentation of Mask RCNN (https://github.com/facebookresearch/maskrcnn-benchmark). Sets up a ROS service to run inference on a single image and responds with a image containing masks and labels

### Mono Depth 2
ROS wrapper and simplified implentation of Mono Depth 2 (https://github.com/nianticlabs/monodepth2). Sets up a ROS service to run inference on a single image and responds with a 16 bit depth map. 

### Python Service Starter
```
$ roslaunch python_service_starter python_service_starter.launch
```
As of the time of writing ROS melodic does not support python3 (which is required to run all the NN models). This node will advertise a service for each of the above nodes which when called will fork a process and run the python scripts. This simply allows them to be run and log more effectively in the ROS environment. NOTE: when exiting (CTRL-C), the node will attempt to shut down all the forked processes however sometimes this does not happen and you will need to kill them manually using ```kill <pid>```. (There is also a bug where they cannot be started twice).

### Realtime Vdo Slam
```
$ python realtime_vdo_slam realtime_vdo_slam_launch.launch 
```
ROS wrapper and input controller for the VDO-SLAM library. This starts all the networks (using service calls to the PSS) and runs the VDO-SLAM algorithm using a threadsafe queue, using the latest data inferred data. This allows the node to run up around 16 FPS. The modified VDO-SLAM library outputs a summary of each analysed frame including
- Estiamted camera pose
- Estimated camera motion
- A list of objects in each frame (pose, velocity, semantic label, tracking label)







