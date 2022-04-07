#!/usr/bin/env python3
# publisher for a specific img folder

import rospy
from std_msgs.msg import String
from detection_only.msg import Bbox_6, Bbox6Array
from sensor_msgs.msg import Image
import json
import time
import cv2
from os import listdir
from os.path import isfile, join
from cv_bridge import CvBridge
import pdb

def img_list(img_folder):
    file_list = []
    for f in listdir(img_folder):
        if isfile(join(img_folder, f)):
            file_list.append(join(img_folder, f))
    return file_list
    

if __name__ == '__main__':

    try:
        rospy.init_node('img_test_node', anonymous=False)
        
        freq = 0.5
        timer = rospy.timer.Rate(freq)
        source_folder = "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/test_imgs_less"
        img_list = img_list(source_folder)
        img_pub = rospy.Publisher('raw_image', Image, queue_size=1)
        br = CvBridge()
        n = len(img_list)
        # while not rospy.is_shutdown():
        for i in range(len(img_list)):
            img = cv2.imread(img_list[i])
            data = Image()
            data = br.cv2_to_imgmsg(img)
            img_pub.publish(data)
            pdb.set_trace()
            timer.sleep()
    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass


    
