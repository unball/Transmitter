import rospy
from PID import *
from communication.msg import target_positions_msg
from communication.msg import robots_speeds_msg
from math import isnan
from math import atan2
from math import pi
from math import fabs
from math import sqrt

#Control constants
Kp_lin = [2, 1, 2]
Ki_lin = [0, 0, 0]

Kp_ang = [1, 1, 1]
Ki_ang  = [1, 1, 1]

Kp_ang = [k * -1 for k in Kp_ang] #nao mexe aqui
Ki_ang = [k * -1 for k in Ki_ang]


number_of_robots = 3
linear_controller = []
angular_controller = []

distance_to_saturate = 0.3 #in meters
def saturation(distance):
	if distance > distance_to_saturate:
		return distance_to_saturate
	else:
		return distance

def calculateErrorAngle(y, x):
	if x==0 and y==0:
		return 0

	th = atan2(y, x)
	
	if th > 0:
		return (pi/2 - th)
	else:
		return -(pi/2 + th)

prev_dTh = [0.0, 0.0, 0.0]
def calculate_robot_speeds(vector):

	for robot in range(number_of_robots):
		distance = vector.y[robot] #could use the magnitude of the vector. it's a different behaviour, though
		distance = saturation(distance)
		dTh = calculateErrorAngle(vector.y[robot], vector.x[robot])

		if changed_quadrant(dTh):
			angular_controller[robot].reset_error_i()

		speeds.linear_vel[robot] = linear_controller[robot].control(distance)
		speeds.angular_vel[robot] = angular_controller[robot].control(dTh)

		prev_dTh[robot] = dTh

def changed_quadrant(angle):
	return quadrant(angle) != quadrant(prev_dTh)

def quadrant(angle):
	if angle >= 0 and angle <= pi/2:
		return 1
	elif angle > pi/2 and angle <= pi:
		return 2
	elif angle < 0 and angle >= -90:
		return 3
	elif angle < -pi/2 and angle > -pi:
		return 4
	
def robot_speed_control_node():
	print '[PositionControl]robot_speed_control_node: begin'
	
	for i in range(3):
		linear_controller.append( PID(Kp = Kp_lin[i], Ki = Ki_lin[i]) )
		angular_controller.append( PID(Kp = Kp_ang[i], Ki = Ki_ang[i]) )

	global speeds
	speeds = robots_speeds_msg()

	rospy.init_node('position_control_node', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=10)
	rospy.Subscriber('relative_positions_topic', target_positions_msg, calculate_robot_speeds)

	while not rospy.is_shutdown():
		notAnumber = False
		for i in range(3):
			if isnan(speeds.linear_vel[i]) or isnan(speeds.angular_vel[i]):
				notAnumber = True
		if not notAnumber:
			pub.publish(speeds)
			rate.sleep()
	
if __name__ == '__main__':
	try:
		robot_speed_control_node()
	except rospy.ROSInterruptException:
		pass
