# Copyright Niantic 2019. Patent Pending. All rights reserved.
#
# This software is licensed under the terms of the Monodepth2 licence
# which allows for non-commercial use only, the full terms of which are made
# available in the LICENSE file.

from __future__ import absolute_import, division, print_function

import os
import sys
import glob
import argparse
import numpy as np
import PIL.Image as pilImage
# from PIL import Image
import matplotlib as mpl
import matplotlib.cm as cm

import torch
from torchvision import transforms, datasets
import torch.backends.cudnn as cudnn
from memory_profiler import profile


from mono_depth_2.networks import DepthDecoder
from mono_depth_2.networks import ResnetEncoder
from mono_depth_2.layers import disp_to_depth
from mono_depth_2.utils import download_model_if_doesnt_exist

from std_msgs.msg import String
from sensor_msgs.msg import Image

from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator
from mono_depth_2.srv import MonoDepth, MonoDepthResponse
import cv2
import time
import rospkg
import rospy
import ros_numpy
import gc

import cv2



rospack = rospkg.RosPack()

package_path = rospack.get_path("mono_depth_2")
sys.path.insert(0, package_path)
#mono_resnet50_640x192
#mono_640x192

class MonoDepth2Ros(RosCppCommunicator):
    def __init__(self, model_path= package_path + "/src/mono_depth_2/models/", model_name = "mono_640x192"):
        RosCppCommunicator.__init__(self)
        self.model_name = model_name

        self.model_path = model_path + self.model_name

        torch.set_grad_enabled(False) 

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        download_model_if_doesnt_exist(self.model_name, self.model_path)
        self.log_to_ros("-> Loading model from {}".format(self.model_path))
        self.encoder_path = os.path.join(self.model_path, "encoder.pth")
        self.depth_decoder_path = os.path.join(self.model_path, "depth.pth")

        # LOADING PRETRAINED MODEL
        self.log_to_ros("Loading pretrained encoder")
        self.encoder = ResnetEncoder(18, False)
        self.loaded_dict_enc = torch.load(self.encoder_path, map_location=self.device)

        # extract the height and width of image that this model was trained with
        self.feed_height = self.loaded_dict_enc['height']
        self.feed_width = self.loaded_dict_enc['width']
        self.filtered_dict_enc = {k: v for k, v in self.loaded_dict_enc.items() if k in self.encoder.state_dict()}
        self.encoder.load_state_dict(self.filtered_dict_enc)
        self.encoder.to(self.device)
        self.encoder.eval()

        self.log_to_ros("Loading pretrained decoder")
        self.depth_decoder = DepthDecoder(
            num_ch_enc=self.encoder.num_ch_enc, scales=range(4))

        self.loaded_dict = torch.load(self.depth_decoder_path, map_location=self.device)
        self.depth_decoder.load_state_dict(self.loaded_dict)

        self.depth_decoder.to(self.device)
        self.depth_decoder.eval()

        #set up service calls
        self.mono_depth_service = rospy.Service("monodepth2/analyse_image",MonoDepth, self.mono_depth_service_callback)
        self.log_to_ros("monodepth2/analyse_image call ready")

    @torch.no_grad()
    def mono_depth_service_callback(self, req):
        response = MonoDepthResponse()
        try:
            current_image = ros_numpy.numpify(req.current_image)
        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response


        depth_image = self.analyse_depth(current_image)

        output_image_msg = ros_numpy.msgify(Image, depth_image, encoding='mono16')

        del depth_image
        response.success = True
        response.output_image = output_image_msg

        return response

    @torch.no_grad()
    # @profile(precision=4)
    def analyse_depth(self, input_image):
        """[Estimates depth of monocular image]

        Args:
            input_image ([numpy array]): [Input image in BGR (OpenCV standard) form]

        Returns:
            [numpy array]: [Depth image of type CV8UC1]
        """ 
        # image = pilImage.fromarray(input_image)
        image = cv2.resize(input_image, (self.feed_width, self.feed_height), interpolation = cv2.INTER_AREA)
        # print(image.shape)
        original_height, original_width, _ = input_image.shape

        tensor_image = torch.FloatTensor(np.ascontiguousarray(np.array(image)[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) * (1.0 / 255.0)))
    
        tensor_image = tensor_image.unsqueeze(0)
        image_predicted = tensor_image.to(self.device)
        features = self.encoder(image_predicted)
        outputs = self.depth_decoder(features)


        disp = outputs[("disp", 0)]
        _, disp = self.disp_to_depth(disp, 0.01, 100)
        disp_resized = torch.nn.functional.interpolate(
            disp, (original_height, original_width), mode="bilinear", align_corners=False)

        disp_resized = disp_resized
        #output is a np.float64. We must cast down to a np.float8 so that ROS encodings can handles this
        #apparently float16 is super slow becuase most intel processors dont support FP16 ops so we're going with np.uint16
        # depth_image_float = disp_resized.squeeze().cpu().numpy()
        depth_image_float = disp_resized.squeeze().cpu().detach().numpy()

        depth_image = cv2.normalize(src=depth_image_float, dst=None, alpha=0, beta=65536, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_16U)

        del tensor_image
        del image 
        del features
        del outputs
        del depth_image_float
        del image_predicted
        del disp

        gc.collect()


        return depth_image

    def disp_to_depth(self, disp, min_depth, max_depth):
            """Convert network's sigmoid output into depth prediction
            The formula for this conversion is given in the 'additional considerations'
            section of the paper.
            """
            min_disp = 1 / max_depth
            max_disp = 1 / min_depth
            scaled_disp = min_disp + (max_disp - min_disp) * disp
            depth = 1 / scaled_disp
            return scaled_disp, depth       

    def depth_image_to_colourmap(self, depth_image):
        """[Converts the depth image to a colour mapping for visualiation]

        Args:
            depth_image ([np array]): [Depth image as output by self.analyse_depth]

        Returns:
            [np array]: [Colour image]
        """        
        vmax = np.percentile(depth_image, 95)
        normalizer = mpl.colors.Normalize(vmin=depth_image.min(), vmax=vmax)
        mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
        return (mapper.to_rgba(depth_image)[:, :, :3] * 255).astype(np.uint8)


class MonoDepthTopic():

    def __init__(self, mono_depth, topic):
        self.mono_depth = mono_depth
        self.image = None
        self.sub = rospy.Subscriber(topic, Image, self.image_callback, queue_size=30)

    def image_callback(self, data):
        input_image = ros_numpy.numpify(data)
        composite = self.mono_depth.analyse_depth(input_image)
        cv2.imshow("Depth", composite)
        cv2.waitKey(1)

        cv2.imshow("Input image", input_image)
        cv2.waitKey(1)


def shutdown_hook():
    cv2.destroyAllWindows()


#TODO: options for type of output
def main():
    rospy.init_node("mono_depth2_ros_node")
    rospy.on_shutdown(shutdown_hook)
    import argparse
    parser = argparse.ArgumentParser()

    parser.add_argument('--topic', default="0")
    parser.add_argument('--image', default="0")
    args = parser.parse_args()
    
    topic = args.topic
    image_path = args.image

    input_device = "camera"
    mono_depth = MonoDepth2Ros()


    if topic == "0" and image_path == "0":
        rospy.loginfo("Using video camera as input")


        cam = cv2.VideoCapture(0)
        while not rospy.is_shutdown():
            start_time = time.time()
            ret_val, img = cam.read()

            composite = mono_depth.analyse_depth(img)

            # test_image = maskrcnn.display_predictions(img)
            cv2.imshow("Depth", composite)
            cv2.waitKey(1)

            cv2.imshow("Input image", img)
            cv2.waitKey(1)




    elif image_path != "0" and topic == "0":
        rospy.loginfo("Loading image from: {}".format(image_path))
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

        image_file_array = image_path.split(".")
        output_image = image_file_array[0] + "_prediction." + image_file_array[1]
        print("Writing output to: {}".format(output_image))
        composite = mono_depth.analyse_depth(image)
        cv2.imwrite(output_image, composite)


    else:
        input_device = "ros_topic"
        rospy.loginfo("Attempting to subscribe to rostopic {}".format(topic))
        topic_mask_rcnn = MonoDepthTopic(mono_depth, topic)
        rospy.spin()

    


if __name__ == "__main__":
    main()
