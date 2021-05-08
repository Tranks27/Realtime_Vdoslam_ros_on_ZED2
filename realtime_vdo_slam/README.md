### Contains nodes to pre-prrocess image data stream and then run the vdo-algorithm on this output

## Structure
### image_preprocessing_rgb

Takes in an RGB image and the correspondong camera intrinsics and produces the output needed for VDO-SLAM.

#### Running

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam vdo_preprocessing_rgb.launch 
```

_input_ : 
- color_topic:  (defined in vdo_preprocessing_rgb.launch) -> RGB uint8 _sensor_msgs::Image_ datatype. Must contain a comleted time stamp in the header file.
- color_info: (defined in vdo_preprocessing_rgb.launch) -> stream with filled out _sensor_msgs::CameraInfo_ datatypes

### image_preprocessing_rgbd

Takes in an RGB image and a depth image from a stereo camera and does the same pre-processing as without the depth image
#### Running

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam vdo_preprocessing_rgbd.launch 
```

_input_ : 
- color_topic:  (defined in vdo_preprocessing_rgbd.launch) -> RGB uint8 _sensor_msgs::Image_ datatype. Must contain a comleted time stamp in the header file.
- color_info: (defined in vdo_preprocessing_rgbd.launch) -> stream with filled out _sensor_msgs::CameraInfo_ datatypes
- depth_topic: (defined in vdo_preprocessing_rgbd.launch) -> MONO uint8 _sensor_msgs::Image_ datatype. Must contain a comleted time stamp in the header file.
- depth_info: (defined in vdo_preprocessing_rgbd.launch) -> stream with filled out _sensor_msgs::CameraInfo_ datatypes

The output is the same for both nodes. All output topics are prefaced with the namespace _/vdoslam/input/_. 

- _/all/_ -> A _realtime_vdo_slam::VdoInput_ message containing all the Images needed for the VDO-SLAM algorithm
- _/camera/rgb/image_raw_ -> same as the image defined by the input topic (N x M x 3, RGB)
- _/camera/mask/image_raw_ -> image mask with pixels labelled 0...N where 0 notates the background. (N x M x 1, MONO8)
- _/camera/mask/colour_mask_ -> image mask but each mask will be coloured coded for easy visualisation (N x M x 3, RGB)
- _/camera/flow/image_raw_ -> flow map of the previus and current frames. Output form is the same as .flo files use by OpenCV (N x M x 2, Float32)
- _/camera/flow/colour_map_ -> flow map represted in 3D colour space for easy visulisation (N x M x 3, RGB)
- _/camera/depth/image_raw_ -> estimated depth map of input image (N x M x 1, MONO16)


### ros_vdo_slam
Runs the VDO-SLAM algorithm given the correct set of inputs from the pre-processing node (or else!)

_input_ : _vdoslam/input/all/_  A _realtime_vdo_slam::VdoInput_ message containing all the Images needed for the VDO-SLAM algorithm

All output messages will be on the _vdoslam/output/_ namespace
_output_: 

## Running ##

See the  realtime_vdo_slam_launch.launch for all the settings needed to run this program correctly. This includes online/offline behaviour and setting the VDO-SLAM params

### From Raw data
From raw data means the entire pipeline is expected to be run (ie use the pre-processing nodes to generate the correct input for this node). In this case the "online" param should be set to true in the launch file. 

If the data is collected in a bagfile, use "offline=False" in the launch file. The node will expect topics without the "vdoslam/input/" namespace.

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam vdo_preprocessingX.launch 
$ roslaunch realtime_vdo_slam realtime_vdo_slam_launch.launch 
```

Change the input video stream _vdo_preprocessing_ in __realtime_vdo.yaml__.

### From Preprocessed Data
You can use the __vdo_bag_generation__ node to generate a bag file with all the necessary processed data and simply run this and the vdo slam node.

Currently, this node has been structured to generate data using the USYD Campus Dataset (http://its.acfr.usyd.edu.au/datasets/usyd-campus-dataset/) and so will not work with any input data.

See https://drive.google.com/drive/folders/1qOocuHTlipVPB4-kU1hPvRlmEGeXkUja?usp=sharing for an already preprocessed bagfile. 

Note: In order to synch the time stamps properly you must tell ROS to use simulated time from the bagfile rather than the walltime.

Set:
```
$ rosparam set use_sim_time True
```

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam realtime_vdo_slam_launch.launch 
$ rosbag play <path_to_file>.bag --clock
```


