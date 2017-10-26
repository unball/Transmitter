#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from subprocess import call
from communication.msg import robots_speeds_msg

number_of_robots = 3

STEER_AXIS = 0
ACCEL_AXIS = 4
REAR_BUTTON = 2

def callback(data):
    robots_speeds = robots_speeds_msg()

    for robot in range(number_of_robots):
        linear_vel = (data.axes[ACCEL_AXIS] - 1) * (-1)
        if data.buttons[REAR_BUTTON]:
            linear_vel *= -1
        angular_vel = data.axes[STEER_AXIS] * -14
        robots_speeds.linear_vel[robot] = linear_vel
        robots_speeds.angular_vel[robot] = 0 if abs(angular_vel) < 0.1 else angular_vel

    pub.publish(robots_speeds)

def start():
    rospy.init_node('joystick')
    rospy.Subscriber('joy', Joy, callback)
    global pub
    pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    start()
