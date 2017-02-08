import rospy
from communication.msg import joy_msg
from communication.msg import robot_speed_msg
from math import atan2
from math import pi
from math import fabs

Kp_lin = 1
Kp_ang = 1
distance_threshold = 0.1
angular_threshold = 0.1

toDegree = False

speeds = robot_speed_msg()

def Pcontrol(Kp, error, threshold = 0):
	result = 0
	if (fabs(error) > fabs(threshold)):
		result = Kp * error
	return result

def reduceAngle(angle):
	while (angle <= -pi):
		angle += 2*pi
	while (angle > pi):
		angle -= 2*pi
        
	return angle

def callback(vector):
	distance = vector.y[0]
	d_th = reduceAngle(pi/2 - atan2(vector.y[0], vector.x[0]))

	global speeds
	global distance_threshold
	global angular_threshold

	speeds.linear_vel = Pcontrol(Kp_lin, distance, distance_threshold)
	speeds.angular_vel = Pcontrol(Kp_ang, d_th, angular_threshold)

	global toDegree
	if toDegree == True:
		speeds.angular_vel = speeds.angular_vel * 180/pi
	
def robot_speed_control_node():
	global speeds

	rospy.init_node('speed_converter_node', anonymous=True)
	rate = rospy.Rate(10)

	pub = rospy.Publisher('robot_speed', robot_speed_msg, queue_size=10)
	rospy.Subscriber('joystick_cvt', joy_msg, callback)

	while not rospy.is_shutdown():
		pub.publish(speeds)
		rate.sleep()
		pass
	
if __name__ == '__main__':
	try:
		robot_speed_control_node()
	except rospy.ROSInterruptException:
		pass