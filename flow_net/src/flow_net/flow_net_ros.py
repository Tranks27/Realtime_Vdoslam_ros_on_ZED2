#!/usr/bin/env python3

import time
import os
import struct
import rospy
import numpy as np
import ros_numpy
import math

from path import Path

import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F
import torchvision.transforms as transforms

from imageio import imread, imwrite



from flow_net.layers import Network

from flow_net.srv import FlowNet, FlowNetResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator

import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import rospkg
# from cv_bridge import CvBridge, CvBridgeError

#TODO relative paths
rospack = rospkg.RosPack()

package_path = rospack.get_path("flow_net")
sys.path.insert(0, package_path)


class FlowNetRos(RosCppCommunicator):

    def __init__(self, model_path= package_path + "/src/flow_net/models/network-default.pytorch"):
        RosCppCommunicator.__init__(self)
        self.model_path = model_path
       
        torch.set_grad_enabled(False)  # make sure to not compute gradients for computational performance
        cudnn.enabled = True # make sure to use cudnn for computational performance

        self.network = Network(self.model_path).cuda().eval()

        #set up service calls
        self.flow_net_service = rospy.Service("flownet/analyse_image",FlowNet, self.flow_net_service_callback)
        self.log_to_ros("Service call ready")


    @torch.no_grad()
    def flow_net_service_callback(self, req):
        response = FlowNetResponse()
        try:
            previous_image = ros_numpy.numpify(req.previous_image)
            current_image = ros_numpy.numpify(req.current_image)

            

            # previous_image = self.bridge.imgmsg_to_cv2(req.previous_image, "bgr8")
            # current_image = self.bridge.imgmsg_to_cv2(req.current_image, "bgr8")
        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response


        output_image = self.analyse_flow(previous_image, current_image)

        # self.log_to_ros(output_image.shape)

        # rgb_flow = self.flow2rgb(output_tensor)
        # output_image = (rgb_flow * 255).astype(np.uint8).transpose(1,2,0)
        rgb_flow = self.flow2rgb(output_image)
        flow_image_msg = ros_numpy.msgify(Image, rgb_flow, encoding='rgb8')

        output_image_msg = ros_numpy.msgify(Image, output_image, encoding='32FC2')
        response.success = True
        response.output_image = output_image_msg
        response.output_viz = flow_image_msg

        return response

    #inputs should be a numpy array
    #output of the nerual network as a tensor 2 x N x M tensor
    #flo files are (R x C x 2) so we need to convert to this form
    @torch.no_grad()
    def analyse_flow(self, previous_image, current_image):

        #convert to tensor array
        tenFirst = torch.FloatTensor(np.ascontiguousarray(np.array(previous_image)[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) * (1.0 / 255.0)))
        tenSecond = torch.FloatTensor(np.ascontiguousarray(np.array(current_image)[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) * (1.0 / 255.0)))

        assert(tenFirst.shape[1] == tenSecond.shape[1])
        assert(tenFirst.shape[2] == tenSecond.shape[2])

        intWidth = tenFirst.shape[2]
        intHeight = tenFirst.shape[1]

        # assert(intWidth == 1024) # remember that there is no guarantee for correctness, comment this line out if you acknowledge this and want to continue
        # assert(intHeight == 436) # remember that there is no guarantee for correctness, comment this line out if you acknowledge this and want to continue

        tenPreprocessedFirst = tenFirst.cuda().view(1, 3, intHeight, intWidth)
        tenPreprocessedSecond = tenSecond.cuda().view(1, 3, intHeight, intWidth)

        intPreprocessedWidth = int(math.floor(math.ceil(intWidth / 32.0) * 32.0))
        intPreprocessedHeight = int(math.floor(math.ceil(intHeight / 32.0) * 32.0))

        tenPreprocessedFirst = torch.nn.functional.interpolate(input=tenPreprocessedFirst, size=(intPreprocessedHeight, intPreprocessedWidth), mode='bilinear', align_corners=False)
        tenPreprocessedSecond = torch.nn.functional.interpolate(input=tenPreprocessedSecond, size=(intPreprocessedHeight, intPreprocessedWidth), mode='bilinear', align_corners=False)

        tenFlow = torch.nn.functional.interpolate(input=self.network(tenPreprocessedFirst, tenPreprocessedSecond), size=(intHeight, intWidth), mode='bilinear', align_corners=False)

        tenFlow[:, 0, :, :] *= float(intWidth) / float(intPreprocessedWidth)
        tenFlow[:, 1, :, :] *= float(intHeight) / float(intPreprocessedHeight)

        flow = tenFlow[0, :, :, :].cpu()
        flow = flow.squeeze(0)

        #this is a 2 x R X C vector -> we need it to be a R X C X 2
        flow_map_np = flow.detach().cpu().numpy()
        flow_map_np = np.moveaxis(flow_map_np, 0, -1)
        flow_map_np = flow_map_np.astype(np.float32)


        del tenFlow
        del tenPreprocessedFirst
        del tenPreprocessedSecond
        del tenFirst
        del tenSecond
        
        return flow_map_np

    def flow2rgb(self, flow_map_np, normalize = True):
        """[Input is flow map generated from NN. Should be in form R X C X 2. Converts
        flow map into visual form in rgb space]

        Args:
            flow_map_np ([type]): [description]

        Returns:
            [type]: [description]
        """        
        # h, w, _ = flow_map_np.shape
        # flow_map_np[:,(flow_map_np[0] == 0) & (flow_map_np[1] == 0)] = float('nan')

        # rgb_map = np.ones((h,w, 3)).astype(np.float32)
    
        # # normalized_flow_map = flow_map_np / (np.abs(flow_map_np).max())
        # # rgb_map[0] += normalized_flow_map[0]
        # # rgb_map[1] -= 0.5*(normalized_flow_map[0] + normalized_flow_map[1])
        # # rgb_map[2] += normalized_flow_map[1]

        # normalized_flow_map = flow_map_np / (np.abs(flow_map_np).max())
        # rgb_map[0] += normalized_flow_map[0]
        # rgb_map[1] -= 0.5*(normalized_flow_map[0] + normalized_flow_map[1])
        # rgb_map[2] += normalized_flow_map[1]
        # return rgb_map.clip(0,1)
        hsv = np.zeros((flow_map_np.shape[0], flow_map_np.shape[1], 3), dtype=np.uint8)
        flow_magnitude, flow_angle = cv2.cartToPolar(flow_map_np[..., 0].astype(np.float32), flow_map_np[..., 1].astype(np.float32))

        # A couple times, we've gotten NaNs out of the above...
        nans = np.isnan(flow_magnitude)
        if np.any(nans):
            nans = np.where(nans)
            flow_magnitude[nans] = 0.

        # Normalize
        hsv[..., 0] = flow_angle * 180 / np.pi / 2
        if normalize is True:
            hsv[..., 1] = cv2.normalize(flow_magnitude, None, 0, 255, cv2.NORM_MINMAX)
        else:
            hsv[..., 1] = flow_magnitude
        hsv[..., 2] = 255
        img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

        return img

class FlowNetTopic():

    def __init__(self, flownet, topic):
        self.flownet = flownet
        self.image = None
        self.is_first = True
        self.previous_image = None
        self.sub = rospy.Subscriber(topic, Image, self.image_callback, queue_size=30)

        #for sync
        self.pub = rospy.Publisher("/flownet_out", String, queue_size=5)

    def image_callback(self, data):
        
        input_image = ros_numpy.numpify(data)
        input_image = input_image[:,:,0:3]

        if self.is_first:
            self.previous_image = input_image
            self.is_first = False
            return 0
            
        start_time = time.time() #time

        composite = self.flownet.analyse_flow(self.previous_image, input_image)
        rgb_flow = self.flownet.flow2rgb(composite)
        
        print("flow Time: {:.2f} s / img".format(time.time() - start_time)) #time
        
        cv2.imshow("RGB Flow", rgb_flow)
        
        self.pub.publish("flownet output") # for sync

        self.previous_image = input_image
        cv2.waitKey(1)

def shutdown_hook():
    cv2.destroyAllWindows()

def main():
    rospy.init_node("flownet_ros_node")
    rospy.on_shutdown(shutdown_hook)

    # import argparse
    # parser = argparse.ArgumentParser()

    # parser.add_argument('--topic', default="0")
    # args = parser.parse_args()
    
    #### topic = rospy.get_param('topic')
    
    # topic = args.topic
    topic = "/zed2/zed_node/left/image_rect_color"


    flownet = FlowNetRos()
    is_first = True

    if topic == "0":
        input_device = "camera"
        cam = cv2.VideoCapture(2)
        previous_image = None
        while True:

            start_time = time.time()
            ret_val, img = cam.read()
            if is_first:
                previous_image = img
                is_first = False
                continue
            composite = flownet.analyse_flow(previous_image, img)
            rgb_flow = flownet.flow2rgb(composite)
            print("Time: {:.2f} s / img".format(time.time() - start_time))
            cv2.imshow("RGB Flow", rgb_flow)
            if cv2.waitKey(1) == 27:
                break  # esc to quit

            previous_image = img
    else:
        input_device = "ros_topic"
        rospy.loginfo("Attempting to subscribe to rostopic {}".format(topic))
        topic_flownet = FlowNetTopic(flownet, topic)
        rospy.spin()


    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

