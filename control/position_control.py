import rospy
from PID import *
from communication.msg import target_positions_msg
from communication.msg import robots_speeds_msg
from math import atan2
from math import pi
from math import fabs
from math import sqrt

#Control constants
Kp_lin = 0

Kp_ang = 3
Ki_ang = 0.1

number_of_robots = 3
linear_controller = []
angular_controller = []

distance_to_saturate = 0.8 #in meters
def saturation(distance):
	if distance > distance_to_saturate:
		return 1
	else:
		return distance

#return: in Degrees
def calculateErrorAngle(y, x):
	if x==0 and y==0:
		return 0

	th = atan2(y, x)
	if th > 0:
		return (pi/2 - th)
	else:
		return -(pi/2 + th)

def calculate_robot_speeds(vector):

	for robot in range(number_of_robots):
		distance = vector.y[robot] #could use the magnitude of the vector. it's a different behaviour, though
		distance = saturation(distance)
		dTh = calculateErrorAngle(vector.y[robot], vector.x[robot])

		speeds.linear_vel[robot] = linear_controller[robot].control(distance)
		speeds.angular_vel[robot] = angular_controller[robot].control(dTh)
	
def robot_speed_control_node():

	for i in range(3):
		linear_controller.append( PID(Kp = Kp_lin) )
		angular_controller.append( PID(Kp = Kp_ang, Ki = Ki_ang) )

	global speeds
	speeds = robots_speeds_msg()

	rospy.init_node('position_control_node', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=10)
	rospy.Subscriber('relative_positions_topic', target_positions_msg, calculate_robot_speeds)

	while not rospy.is_shutdown():
		pub.publish(speeds)
		rate.sleep()
		pass
	
if __name__ == '__main__':
	try:
		robot_speed_control_node()
	except rospy.ROSInterruptException:
		pass
