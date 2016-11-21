#!/usr/bin/env python

import rospy

from std_msgs.msg import Int8

def publisher():
    pub = rospy.Publisher('dummy_topic', Int8, queue_size=10)
    rospy.init_node('dummy_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        strg = 40
        pub.publish(strg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass