import rospy
from communication.msg import joy_msg
from communication.msg import robot_speed_msg
from math import atan2
from math import pi
from math import fabs

Kp_lin = 120/2
Kp_ang = 1000/2
distance_threshold = 0.1
angular_threshold = 0.1

toDegree = False

speeds = robot_speed_msg()

def Pcontrol(Kp, error, threshold = 0):
	result = 0
	if (fabs(error) > fabs(threshold)):
		result = Kp * error
	return result

def calculateErrorAngle(y, x):
	th = atan2(y, x)
	if th > 0:
		return pi/2 - th
	else:
		return -(pi/2 + th)

def calculate_robot_speeds(vector):
	distance = vector.y

	dTh = calculateErrorAngle(vector.y, vector.x)

	speeds.linear_vel = Pcontrol(Kp_lin, distance, distance_threshold) #in cm/s
	speeds.angular_vel = Pcontrol(Kp_ang, dTh, angular_threshold) * fabs(vector.x)
	
def robot_speed_control_node():
	global speeds

	rospy.init_node('speed_converter_node', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('robot_speed', robot_speed_msg, queue_size=10)
	rospy.Subscriber('joystick_cvt', joy_msg, calculate_robot_speeds)

	while not rospy.is_shutdown():
		pub.publish(speeds)
		rate.sleep()
		pass
	
if __name__ == '__main__':
	try:
		robot_speed_control_node()
	except rospy.ROSInterruptException:
		pass