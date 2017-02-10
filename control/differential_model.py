import rospy
from communication.msg import robots_speeds_msg
from communication.msg import wheels_speeds_msg

r = 0.035 #wheel radius in cm
L = 0.075 #distance between wheels in cm

number_of_robots = 3

def processPosition(robot_speed):
	for robot in range(number_of_robots):
		w1 = (robot_speed.linear_vel[robot] - (L/2)*robot_speed.angular_vel[robot])/r
		w2 = (robot_speed.linear_vel[robot] + (L/2)*robot_speed.angular_vel[robot])/r
		speeds.right_vel[robot] = w1
		speeds.left_vel[robot] = w2

def differential_model_node():
	global speeds
	speeds = wheels_speeds_msg()

	rospy.init_node('differential_model', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('wheels_speed', wheels_speeds_msg, queue_size=10)
	rospy.Subscriber('robots_speeds', robots_speeds_msg, processPosition)

	while not rospy.is_shutdown():
		pub.publish(speeds)
		rate.sleep()
		pass

if __name__ == '__main__':
    try:
        differential_model_node()
    except rospy.ROSInterruptException:
        pass