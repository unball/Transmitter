#!/usr/bin/env python

import rospy

from communication_node.msg import comm_msg

def publisher():
    pub = rospy.Publisher('dummy_topic', comm_msg, queue_size=10)
    rospy.init_node('dummy_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = comm_msg()
        msg.data = [0, 20, 20]
        msg.ispvariant = True
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass