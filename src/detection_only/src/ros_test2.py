#!/usr/bin/env python3

import os
# os.environ["OMP_NUM_THREADS"] = "1"
# os.environ["OPENBLAS_NUM_THREADS"] = "1"
# os.environ["MKL_NUM_THREADS"] = "1"
# os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
# os.environ["NUMEXPR_NUM_THREADS"] = "1"
import sys
import pdb
# sys.path.insert(0, '/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/')
# sys.path.insert(0, '/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/yolov5')
# sys.path.insert(0,'/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/deep_sort/deep/reid')

from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors

from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort

import rospy
from std_msgs.msg import String
from detection_only.msg import Bbox_6, Bbox6Array
import json
import time

processing = False
new_msg = False
msg = None

def det_read(det_file):
    with open(det_file, "r") as f:
        # Reading from json file
        total_labels = json.load(f)
    return total_labels
json_labels = det_read("/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/sample.json")
i = 0    

def det_publish():
    pass
    
def det_cb(data):
    # out = rospy.get_param("/output", None)
    # source = rospy.get_param("/source", None)
    global processing, new_msg, msg
    if not processing:
        new_msg = True
        msg = data
    print("det_cb")
    detect(msg)
    # print(msg)

def detect(data):
    print("detect")
    print("data: ",len(data.bboxes))
    global i
    print("json: ", len(json_labels[str(i)][0]))
    i +=1
    print(i)
    

    

if __name__ == '__main__':

    try:
        rospy.init_node('ros_test2', anonymous=False)
        det_sub = rospy.Subscriber('det_result', Bbox6Array, det_cb, queue_size=1)
        
        freq = 1
        timer = rospy.timer.Rate(freq)
        while not rospy.is_shutdown():
            if new_msg:
                #set processing to True
                processing = True
                new_msg = False
                #simulate a process that take 0.2 seconds
                # rospy.loginfo(msg)
                # timer.sleep()
                #set processing to False
                processing = False


    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass


    

"""         while not rospy.is_shutdown():
            hello_str = "det_result %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()
"""