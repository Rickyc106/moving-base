import math
from sensor_msgs.msg import Joy
import rospy

### Initial variables
rotate_mag = 5
rotate_angle = [45, 135, 225, 315]

axes_len = 6
buttons_len = 17

axes_array = [0] * axes_len
buttons_array = [0] * buttons_len

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

def xbox_callback(data):
	#print "Axes [Analog]: ", str(data.axes)
	#print "Buttons [Digital]: ", str(data.buttons)
	#print ""

	global axes_array
	global buttons_array

	for i in range(axes_len):
		axes_array[i] = data.axes[i]

	for i in range(buttons_len):
		buttons_array[i] = data.buttons[i]

	#print(str(axes_array[0]))


def xbox_read():
	### Read the following inputs from controller/keyboard
	# 1) Float - Translation magnitude (pos away from dead center)
	# 2) Float - Translation angle (joystick angle)
	# 3) Binary - CW/CCW rotation
	#
	# Return all three as floats 
	###

	# Axes: [left_x, left_y, left_trigger, right_x, right_y, right_trigger]
	# Buttons: [X, O, ^, [], left_bumper, right_bumper, left_trigger, right_trigger, select,
	#			start, PS, left_joy_press, right_joy_press, up, down, left, right]

	rospy.init_node('testing_motor', anonymous=True)
	rospy.Subscriber('joy',  Joy, xbox_callback)

	while not rospy.is_shutdown():
		print "Axes [Analog]", str(axes_array)
		print "Buttons [Digital]", str(buttons_array)
		print ""


	rospy.spin()


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
		xbox_read()
	except rospy.ROSInterruptException:
		pass