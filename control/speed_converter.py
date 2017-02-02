import rospy

r = 1.0
L = 1.0

class RobotSpeed():
	def __init__(self, linear, angular):
		self.linear_vel = linear
		self.angular_vel = angular

def processPosition(robot_speed):
	w1 = (robot_speed.linear_vel - (L/2)*robot_speed.angular_vel)/r
	w2 = (robot_speed.linear_vel + (L/2)*robot_speed.angular_vel)/r
	print(w1)
	print(w2)

def speed_converter_node():
	rospy.init_node('speed_converter_node', anonymous=True)
	rate = rospy.Rate(10)
    
    #pub = rospy.Publisher('wheels_speed', wheels_speed, queue_size=10)
    #rospy.Subscriber('robot_speed', robot_speed, convertSpeed, queue_size = 1)

	while not rospy.is_shutdown():
		#pub.publish(speeds)
		rate.sleep()

if __name__ == '__main__':
    robot_speed = RobotSpeed(1.0,1.0)
    processPosition(robot_speed)

    #try:
    #    speed_converter_node()
    #except rospy.ROSInterruptException:
    #    pass