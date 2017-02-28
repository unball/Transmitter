import rospy
from math import pi
from math import fabs
from communication.msg import robots_speeds_msg
from communication.msg import wheels_speeds_msg

wheel_reduction = 3/ 1 #motor -> wheel
r = 0.035 #wheel radius in m
L = 0.075 #distance between wheels in m

max_tics_per_s = 70000 #equivalent to 700 tics/10ms
encoder_resolution = 512*19
max_motor_speed = (max_tics_per_s) / encoder_resolution 

number_of_robots = 3

def normalize(w1, w2):
	if fabs(w1) >= fabs(w2):
		w2 = max_motor_speed * w2/fabs(w1)
		w1 = max_motor_speed * w1/fabs(w1)
	elif fabs(w2) >= fabs(w1):
		w1 = max_motor_speed * w1/fabs(w2)
		w2 = max_motor_speed * w2/fabs(w2)

	return w1, w2	

#Returns the speed the MOTOR will have to spin in degree/s
def processPosition(robot_speed):
	for robot in range(number_of_robots):
		wheel_radian1 = (robot_speed.linear_vel[robot] - (L/2)*robot_speed.angular_vel[robot]) / r
		wheel_radian2 = (robot_speed.linear_vel[robot] + (L/2)*robot_speed.angular_vel[robot]) / r
		wheel_rotations1 = wheel_radian1 / (2 * pi)
		wheel_rotations2 = wheel_radian2 / (2 * pi)
		motor_rotations1 = wheel_rotations1 * wheel_reduction
		motor_rotations2 = wheel_rotations2 * wheel_reduction
		if fabs(motor_rotations1) > max_motor_speed or fabs(motor_rotations2) > max_motor_speed:
			motor_rotations1, motor_rotations2 = normalize(motor_rotations1, motor_rotations2)
		speeds.right_vel[robot] = motor_rotations1
		speeds.left_vel[robot] = motor_rotations2

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