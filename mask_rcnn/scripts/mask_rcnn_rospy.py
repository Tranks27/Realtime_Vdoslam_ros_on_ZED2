import os
import sys

from mask_rcnn.mask_rcnn_ros import MaskRcnnRos

import rospy

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_rospy')
    mask_rcnn_ros = MaskRcnnRos()
    rospy.spin()
