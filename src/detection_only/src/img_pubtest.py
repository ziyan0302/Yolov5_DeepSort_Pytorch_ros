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
    img_name_list = []
    for image in listdir(img_folder):
        img_name = int(image.split('.')[0])
    #     while len(img_name) != 4:
    #         img_name = '0' + img_name
        img_name_list.append(img_name)
    # pdb.set_trace()
    img_name_list = sorted(img_name_list)
    for n in img_name_list:
        f = str(n) + '.png'
        if isfile(join(img_folder, f)):
            file_list.append(join(img_folder, f))
    # for f in listdir(img_folder):
    #     if isfile(join(img_folder, f)):
    #         file_list.append(join(img_folder, f))
    # pdb.set_trace()
    return file_list
    

if __name__ == '__main__':

    try:
        rospy.init_node('img_test_node', anonymous=False)
        
        freq = 30
        timer = rospy.timer.Rate(freq)
        # source_folder = "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/test_imgs_less"
        source_folder = "/home/ziyan/Downloads/exp1"
        img_list = img_list(source_folder)
        pdb.set_trace()
        img_pub = rospy.Publisher('raw_image', Image, queue_size=1)
        br = CvBridge()
        n = len(img_list)
        # while not rospy.is_shutdown():
        for i in range(len(img_list)):
            img = cv2.imread(img_list[i])
            img = img[:,:,[2,1,0]]
            data = Image()
            data = br.cv2_to_imgmsg(img)
            img_pub.publish(data)
            print("image publish!")
            pdb.set_trace()
            timer.sleep()
    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass


    
