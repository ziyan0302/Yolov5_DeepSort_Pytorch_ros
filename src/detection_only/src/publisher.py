#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def publisher():
    pub = rospy.Publisher('topic_name', Int32, queue_size=10)
    rospy.init_node('publisher')
    # r = rospy.Rate(1) #30
    freq = 1
    timer = rospy.timer.Rate(freq)
    
    count = 1
    msg = Int32()
    while not rospy.is_shutdown():
        msg.data = count
        count = count+1 if count < 30 else 1
        pub.publish(msg)
        rospy.loginfo(msg)
        # r.sleep()
        timer.sleep()

if __name__ == '__main__':
    publisher()