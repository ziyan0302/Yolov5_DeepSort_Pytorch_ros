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

def det_read(det_file):
    with open(det_file, "r") as f:
        # Reading from json file
        total_labels = json.load(f)
    return total_labels
    

def det_publish():
    pass
    
def det_cb(msg):
    # out = rospy.get_param("/output", None)
    # source = rospy.get_param("/source", None)
    print("det_cb")
    detect(msg)
    # print(msg)

def detect(msg):
    print("detect")
    

if __name__ == '__main__':

    try:
        rospy.init_node('ros_test1', anonymous=False)
        
        freq = 0.5
        timer = rospy.timer.Rate(freq)
        # while not rospy.is_shutdown():
            # out = rospy.get_param("~output", None)
            # source = rospy.get_param("~source", None)
            # print(out,source)
            # det_sub = rospy.Subscriber('det_result', Bbox6Array, det_cb, queue_size=1)

        det_results = det_read("/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/sample.json")
        det_pub = rospy.Publisher('det_result', Bbox6Array, queue_size=1)
        n = 0
        for frame_idx in det_results.keys():
            print(frame_idx)
            data = Bbox6Array()
            for i in range(len(det_results[str(frame_idx)][0])):
                bbox = Bbox_6()
                bbox.x1 = det_results[str(frame_idx)][0][i][0]
                bbox.y1 = det_results[str(frame_idx)][0][i][1]
                bbox.x2 = det_results[str(frame_idx)][0][i][2]
                bbox.y2 = det_results[str(frame_idx)][0][i][3]
                bbox.conf = det_results[str(frame_idx)][0][i][4]
                bbox.cls = det_results[str(frame_idx)][0][i][5]
                # pdb.set_trace()
                data.bboxes.append(bbox)
                # pdb.set_trace()
            # det_pub.publish(det_results[str(frame_idx)][0])
            det_pub.publish(data)
            n = n+1
            if n > 0:
                break
        # while not rospy.is_shutdown():
            timer.sleep()
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