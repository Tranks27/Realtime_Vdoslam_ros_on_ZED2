import os
import sys

from midas_ros.midas_depth_ros import MidasRos
import rospy



if __name__ == '__main__':
    rospy.init_node('midas_depth_rospy')
    midas_ros = MidasRos()
    rospy.spin()