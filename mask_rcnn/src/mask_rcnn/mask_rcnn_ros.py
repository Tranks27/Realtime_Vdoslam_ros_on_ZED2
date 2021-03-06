#!/usr/bin/env python3

import cv2

import sys
from memory_profiler import profile
import rospkg


from maskrcnn_benchmark.config import cfg
from maskrcnn_benchmark.utils import cv2_util
import ros_numpy
import os
import numpy as np
import math
import matplotlib.path
import torch

from mask_rcnn.predictor import COCODemo

from mask_rcnn.msg import SemanticObject

from mask_rcnn.srv import MaskRcnnVdoSlam, MaskRcnnVdoSlamResponse
from mask_rcnn.srv import MaskRcnnLabelList, MaskRcnnLabelListResponse
from mask_rcnn.srv import MaskRcnnLabel, MaskRcnnLabelResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

import struct
import rospy
import random
from std_msgs.msg import String

import time

####for synchronizer
from synchronizer.msg import Sync
from synchronizer.msg import SemanticObjectArray

rospack = rospkg.RosPack()

package_path = rospack.get_path("mask_rcnn")
sys.path.insert(0, package_path)

#e2e_mask_rcnn_R_101_FPN_1x_caffe2
#e2e_mask_rcnn_X_101_32x8d_FPN_1x_caffe2
#e2e_mask_rcnn_X_101_32x8d_FPN_1x_caffe2
#e2e_mask_rcnn_R_50_FPN_1x_caffe2
class MaskRcnnRos(RosCppCommunicator):

    def __init__(self, config_path = package_path + "/src/mask_rcnn/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml"):
        RosCppCommunicator.__init__(self)
        self.model_config_path = config_path
        cfg.merge_from_file(self.model_config_path)
        cfg.merge_from_list([])
        cfg.freeze()

        self._greyscale_palette = (2 * 25 - 1)
        self._colour_palette = np.array([2 ** 29 - 1, 2 ** 7 - 1, 2 ** 11 - 5])


        # prepare object that handles inference plus adds predictions on top of image
        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.75,
            show_mask_heatmaps=False,
            masks_per_dim=5
        )

        self._greyscale_colours = self._generate_grayscale_values()
        self._colours = self._generate_coloured_values()



        self.mask_rcnn_service = rospy.Service("maskrcnn/analyse_image",MaskRcnnVdoSlam, self.mask_rcnn_service_callback)
        self.mask_rcnn_label_service = rospy.Service("maskrcnn/request_label", MaskRcnnLabel, self.label_request_callback)
        self.mask_rcnn_label_list_service = rospy.Service("maskrcnn/request_label_list", MaskRcnnLabelList, self.label_list_request_callback)
        self.log_to_ros("Service call ready")


    @torch.no_grad()
    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnVdoSlamResponse()
        try: 
            input_image = ros_numpy.numpify(req.input_image)

            response_image, semantic_objects = self.analyse_image(input_image)
            display_image = self.generate_coloured_mask(response_image)
            # test_image = self.display_predictions(input_image)

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='mono8')
            display_image_msg = ros_numpy.msgify(Image, display_image, encoding='rgb8')

            response.success = True
            # response.output_image = output_image_msg
            response.output_mask = output_image_msg
            response.output_viz = display_image_msg
            response.semantic_objects = semantic_objects


            del response_image
            del semantic_objects
            del display_image

            return response



        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response

    def label_request_callback(self, req):
        response = MaskRcnnLabelResponse()
        labels = self.convert_label_index_to_string(req.label_index)
        response.labels = labels
        return response

    def label_list_request_callback(self, req):
        response = MaskRcnnLabelListResponse()
        response.labels = self.coco_demo.CATEGORIES
        return response

    @torch.no_grad()
    def display_predictions(self, image):
        #note: due to changes in predictions.py this will no longer work
        return self.coco_demo.run_on_opencv_image(image)

    # @torch.no_grad()
    def analyse_image(self, image):
        """[Analyses an image using mask rcnn. Creates a semantic instance labelled greyscale image
        and a list of mask_rcnn.SemanticObjects which represent each detected object in the frame]

        Args:
            image ([np.array]): [uint8 rgb image to analyse]

        Returns:
            [np.ndarry, List[SemanticObjects]]: [Semantic instance image, list of detected objects in the scene]
        """
        torch.cuda.empty_cache()
        predictions = self.coco_demo.compute_prediction(image)
        top_predictions = self.coco_demo.select_top_predictions(predictions)

        width = image.shape[0]
        height = image.shape[1]
        blank_mask = np.zeros((width, height),np.uint8)

        if top_predictions is None:
            return blank_mask, []

        #in form [x,y,w,h]
        top_predictions = top_predictions.convert("xywh")
        masks = top_predictions.get_field("mask")
        boxes = top_predictions.bbox
        label_indexs = top_predictions.get_field("labels").numpy()

        # Naing edit
        x = np.where(label_indexs != 1) # get indexes of labels which are not person
        #remove items which are not person class
        masks = np.delete(masks,x, axis=0)
        boxes = np.delete(boxes,x, axis=0)
        label_indexs = np.delete(label_indexs,x)
        
        labels = self.convert_label_index_to_string(label_indexs)



        assert(len(labels) == len(label_indexs) == len(boxes))

        if len(boxes) < 1:
            return blank_mask, []

        
        if masks.ndim < 3:
            masks = np.expand_dims(masks, axis=0)
            masks = np.expand_dims(masks, axis=0)

        #track semantic labels in this map
        # we want unique instance-level semantic labelling per class (so car: [1,2], person: [1,2])
        instance_track = {}

        for label_index in label_indexs:
            # there is at least one of these objects in the list
            instance_track[label_index] = 1

        semantic_objects = []

        index = 1

        for mask, semantic_index, semantic_label, bounding_box in zip(masks, label_indexs, labels, boxes):
            instance_label = instance_track[semantic_index]

            # add semantic instance mask to blank masl
            thresh = mask[0, :, :].astype(np.uint8) * instance_label
            blank_mask += thresh

            # create bounding box for object
            xmin, ymin, w, h = bounding_box.split(1, dim=-1)
            #here we make bounding box msg types
            #we make pose x, y which is bottom left corner of image
            # leave theta as 0
            pose2d = Pose2D(int(xmin.numpy()[0]), int(ymin.numpy()[0]), 0)
            bounding_box_msg = BoundingBox2D()
            #size x and size y will be bounding box width and height respectively
            bounding_box_msg.size_x =  int(w.numpy()[0])
            bounding_box_msg.size_y = int(h.numpy()[0])
            bounding_box_msg.center = pose2d

            # create semantic object 
            semantic_object = SemanticObject()
            semantic_object.bounding_box = bounding_box_msg
            semantic_object.label = semantic_label 
            semantic_object.label_index = semantic_index
            semantic_object.semantic_instance = instance_label
            # semantic_object.tracking_label = -1 #inialise with -1

            semantic_objects.append(semantic_object)

            instance_track[semantic_index] += 1


        composite = blank_mask

        return composite, semantic_objects

    def convert_label_index_to_string(self, labels):
        return [self.coco_demo.CATEGORIES[i] for i in labels]

    def get_single_label_from_index(self, label):
        return self.coco_demo.CATEGORIES[label]

    def get_greyscale_colours(self, label_index):
        return self._greyscale_colours[label_index]
        

    def _generate_grayscale_values(self):
        """[Generates n number of distinct values between 1 and 255 for each label. This should be 
        used for visualisation purposes only as VDOSLAM just needs a distinct value]

        Returns:
            [List]: [List of values]
        """
        numer_of_cats = len(self.coco_demo.CATEGORIES)  
        categories_index = np.linspace(0, numer_of_cats, numer_of_cats + 1)
        colors = np.array(categories_index) * self._greyscale_palette
        colors = (colors % 255).astype("uint8")
        return colors

    def _generate_coloured_values(self):
        numer_of_cats = len(self.coco_demo.CATEGORIES)  
        categories_index = np.linspace(0, numer_of_cats, numer_of_cats + 1)
        colors = [(np.multiply(index,self._colour_palette) % 255).astype('uint8') for index in categories_index]
        # colors = np.array(categories_index) % 255) * self._colour_palette
        # colors = (colors % 255).astype("uint8")
        return colors

    def generated_bounding_boxes(self, rgb_image, semantic_objects):
        """[Draw bounding boxes and semantic labels on images the original image]

        Args:
            rgb_image ([np.ndarray]): [The image you want to draw on. Normally will be the raw rgb image]
            semantic_objects ([list[SemanticObjects]]): [list of semantic objects as generated by analyse image]

        Returns:
            [np.ndarray]: [The image drawn on]
        """
        rgb_image_bb = rgb_image.copy()
        for semantic_object in semantic_objects:
            bounding_box = semantic_object.bounding_box
            label = semantic_object.label 
            cv2.rectangle(rgb_image_bb, (bounding_box.center.x, bounding_box.center.y),
                        (bounding_box.center.x + bounding_box.size_x, bounding_box.center.y + bounding_box.size_y), (0, 255, 255), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(rgb_image_bb, label, (bounding_box.center.x,bounding_box.center.y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return rgb_image_bb

    def generate_coloured_mask(self, mask):
        # mask =  np.expand_dims(mask, 2) 
        # mask = np.repeat(mask, 3, axis=2) # give the mask the same shape as your image
        coloured_img = np.zeros((mask.shape[0], mask.shape[1], 3))
        max_value = np.amax(mask)

        for index in range(max_value+1):
            colour = self._colours[index]
            coloured_img[mask == index] = colour
        coloured_img = coloured_img.astype('uint8')
        return coloured_img

    # def overlay_mask(self, mask, rgb_image):
    #     mask_alpha = cv2.cvtColor(mask_alpha, cv2.COLOR_RGB2RGBA)
    #     mask_alpha[:, :, 3] = alpha_data






class MaskRcnnTopic():

    def __init__(self, mask_rcnn, topic):
        self.mask_rcnn = mask_rcnn
        self.image = None
        self.sub = rospy.Subscriber(topic, Image, self.image_callback, queue_size=30)
        #for synchronizer package
        self.pub = rospy.Publisher("/maskrcnn/maskrcnn_raw", Image, queue_size=5)
        self.pub_sObj = rospy.Publisher("/maskrcnn/maskrcnn_sObj", SemanticObjectArray, queue_size=5)


    def image_callback(self, data):
        input_image = ros_numpy.numpify(data)
        start_time = rospy.Time.from_sec(time.time())
        
        # cv2.imshow("Input image", input_image)
        # cv2.waitKey(1)
        
        response_image, semantic_objects = self.mask_rcnn.analyse_image(input_image[:,:,0:3]) # only selects the first 3 channels excluding alpha channel
        display_image = self.mask_rcnn.generate_coloured_mask(response_image)
        bb_img = self.mask_rcnn.generated_bounding_boxes(input_image, semantic_objects)
       

        current_time = rospy.Time.from_sec(time.time())
        print("mask Time: {:.2f} s / img".format(current_time.to_sec() - start_time.to_sec()))
        
        ## show detections
        # cv2.imshow("maskrcnn detections", bb_img)
        # cv2.waitKey(1)
        # cv2.imshow("maskrcnn masks", display_image)
        # cv2.waitKey(1)

        # publish the maskrcnn output image
        output_mask_msg = ros_numpy.msgify(Image, response_image, encoding='mono8') # data needed for vdo slam
        # output_mask_msg = ros_numpy.msgify(Image, display_image, encoding='rgb8') #for viz purpose
        
        output_mask_msg.header.stamp = start_time #image acquisition time
        output_mask_msg.header.frame_id = "maskrcnn_raw_frame"
        self.pub.publish(output_mask_msg) # for sync

        # publish semantic objects
        sObj_temp = SemanticObjectArray()
        sObj_temp.header.stamp = start_time
        sObj_temp.header.frame_id = "maskrcnn_semanticObj_frame"
        sObj_temp.semantic_objects = semantic_objects
        self.pub_sObj.publish(sObj_temp)

def shutdown_hook():
    cv2.destroyAllWindows()

def main():
    rospy.init_node("mask_rcnn_ros_node")
    rospy.on_shutdown(shutdown_hook)

    ## topic name to get from param server, otherwise use default value
    topic = rospy.get_param('topic',"/camera/left")
    # topic = rospy.get_param('topic',"/zed2/zed_node/left/image_rect_color")

    input_device = "camera"
    maskrcnn = MaskRcnnRos()
    
    ########USE the image or camera directly###########
    if topic == "0":
        rospy.loginfo("Using video camera as input")
        ## Windows for displaying results
        cv2.namedWindow('maskrcnn detections',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('maskrcnn detections', 600, 600)
        cv2.namedWindow('maskrcnn masks',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('maskrcnn masks', 600, 600)

        cam = cv2.VideoCapture(0)
        while not rospy.is_shutdown():
            start_time = time.time()
            ret_val, img = cam.read()

            
            response_image, sematic_objects = maskrcnn.analyse_image(img)
            display_image = maskrcnn.generate_coloured_mask(response_image)
            bb_img = maskrcnn.generated_bounding_boxes(img, sematic_objects)
            
            print("Time: {:.2f} s / img".format(time.time() - start_time))
        
            # cv2.imshow("maskrcnn detections", bb_img)
            # cv2.imshow("maskrcnn masks", display_image)
            
            if cv2.waitKey(1) == 27:
                break  # esc to quit
        

    else:
        input_device = "ros_topic"
        rospy.loginfo("Attempting to subscribe to rostopic {}".format(topic))
        topic_mask_rcnn = MaskRcnnTopic(maskrcnn, topic)
        rospy.spin()

#TODO: options for type of output
if __name__ == "__main__":
    main()

    