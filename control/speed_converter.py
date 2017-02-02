import rospy
from communication.msg import robot_speed_msg
from communication.msg import wheels_speeds_msg

r = 1.0
L = 1.0

speeds = wheels_speeds_msg()

def processPosition(robot_speed):
	global speeds
	w1 = (robot_speed.linear_vel - (L/2)*robot_speed.angular_vel)/r
	w2 = (robot_speed.linear_vel + (L/2)*robot_speed.angular_vel)/r
	speeds.right_vel = w1
	speeds.left_vel = w2

def speed_converter_node():
	rospy.init_node('speed_converter', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('wheels_speed', wheels_speeds_msg, queue_size=10)
	rospy.Subscriber('robot_speed', robot_speed_msg, processPosition)

	while not rospy.is_shutdown():
		pub.publish(speeds)
		rate.sleep()
		pass

if __name__ == '__main__':
    try:
        speed_converter_node()
    except rospy.ROSInterruptException:
        pass