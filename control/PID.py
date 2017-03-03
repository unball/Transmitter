import time

class PID:
	"""
	Discrete PID control
	"""
	def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
		self.Kp= Kp
		self.Ki= Ki
		self.Kd= Kd

		self.error_i = 0.0

		self.current_time = time.time()
		self.last_time = self.current_time		

	def dt():
		self.current_time = time.time()
		delta_time = self.current_time - self.last_time
		self.last_time = self.current_time

	def control(error):

		self.error_i += error * dt()

		result = self.Kp * error + self.Ki * self.error_i
		return result

	def reset_error_i():
		self.error_i = 0.0