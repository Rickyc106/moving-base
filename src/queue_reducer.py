import rospy
from sensor_msgs.msg import Joy

#queue = None
joy_msg = Joy()

def joy_cb(data):
	joy_msg.axes = data.axes
	joy_msg.buttons = data.buttons
	print str(joy_msg.axes)

def main():
	queued_joy = rospy.Publisher('joy_queued', Joy)

	rospy.init_node('queue_reducer', anonymous=True)

	rospy.Subscriber('joy',  Joy, joy_cb)

	rate = rospy.Rate(30)

	while not rospy.is_shutdown():
		queued_joy.publish(joy_msg)
		rate.sleep()
		#print "Reducing queue size to", queue

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
