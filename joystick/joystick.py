#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from communication.msg import joy_msg
from subprocess import call

def callback(data):
    k = 1
    msg = joy_msg()
    msg.x[0] = -data.axes[0]*k
    msg.y[0] =  data.axes[1]*k
    msg.x[1] = -data.axes[2]*k
    msg.y[1] =  data.axes[3]*k

    pub.publish(msg)

def start():
        global pub
        pub = rospy.Publisher('joystick_cvt',joy_msg, queue_size=10)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber('joy', Joy, callback)
        # starts the node
        rospy.init_node('joystick')
        rospy.spin()

if __name__ == '__main__':
    start()