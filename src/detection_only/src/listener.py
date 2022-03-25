#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

processing = False
new_msg = False
msg = None

def callback(data):
    global processing, new_msg, msg
    if not processing:
        new_msg = True
        msg = data

def listener():
    global processing, new_msg, msg
    rospy.init_node('subscriber')
    rospy.Subscriber('topic_name', Int32, callback)
    # r = rospy.Rate(1) #5
    freq = 1
    timer = rospy.timer.Rate(freq)
    while not rospy.is_shutdown():
        if new_msg:
            #set processing to True
            processing = True
            new_msg = False
            #simulate a process that take 0.2 seconds
            rospy.loginfo(msg)
            # r.sleep()
            timer.sleep()
            #set processing to False
            processing = False

if __name__ == '__main__':
    listener()