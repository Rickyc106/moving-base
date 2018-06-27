import math
from std_msgs.msg import Float32
import rospy

### Initial variables
rotate_mag = 5
rotate_angle = [45, 135, 225, 315]

def calculate_vectors(theta_yaw, trans_mag, trans_angle, rotate_dir):
	for i in range(4):
		rotate_angle[i] += theta_yaw

		if rotate_dir == 0:
			rotate_angle[i] += 180
			rotate_angle[i] % 360
			
		y_bar = rotate_mag * math.sin(rotate_angle[i]) + trans_mag * math.sin(trans_angle[i])
		x_bar = rotate_mag * math.sin(rotate_angle[i]) + trans_mag * math.sin(trans_angle[i])

		wheel_mag[i] = x_bar**2 + y_bar**2
		wheel_angle[i] = math.atan2(y_bar, x_bar)

def xbox_read():
	### Read the following inputs from controller/keyboard
	# 1) Float - Translation magnitude (pos away from dead center)
	# 2) Float - Translation angle (joystick angle)
	# 3) Binary - CW/CCW rotation
	#
	# Return all three as floats 
	###

def yaw_callback(data):
	global yaw_angle
	yaw_angle = (data.msg + 360) % 360


def publisher():
	t_mag, t_angle, r_dir = xbox_read()
	imu_sub = rospy.Subscriber("yaw_heading", Float32, yaw_callback)
	calculate_vectors(yaw_angle, t_mag, t_angle, r_dir)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		### Output PWM signals to motors
		# 1) Wheel angle to steer motor
		# 2) Wheel magnitude to drive motor
		###
		rate.sleep()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass





