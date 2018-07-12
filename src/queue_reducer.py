import rospy
import tf
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

#queue = None
joy_msg = Joy()
pose_array = PoseArray()

resultant = [0,0,0,0]
pose_zero = [[0 for i in range(4)] for i in range(4)]
quat = []

def joy_cb(data):
	joy_msg.axes = data.axes
	joy_msg.buttons = data.buttons
	print str(joy_msg.axes)

def array_cb(msg):
	for i in range(4):
		resultant[i] = msg.data[i]

def main():
	queued_joy = rospy.Publisher('joy_queued', Joy, queue_size = 10)
	marker = rospy.Publisher('resultant_angle', PoseArray, queue_size = 10)

	rospy.init_node('queue_reducer', anonymous=True)

	rospy.Subscriber('joy',  Joy, joy_cb)
	rospy.Subscriber('temp_array_data', Float32MultiArray, array_cb)

	for i in range(4):
		quat.append(tf.transformations.quaternion_from_euler(0,0,resultant[i]))
		pose_array.poses[i].orientation = quat[i]

	pose_array.pose.position = [-5,5,0]
	pose_array.pose[1].position = [-5,-5,0]
	pose_array.pose[2].position = [5,-5,0]
	pose_array.pose[3].position = [5,5,0]

	pose_array.header.frame_id = 'Resultant_angles'

	rate = rospy.Rate(30)

	while not rospy.is_shutdown():
		queued_joy.publish(joy_msg)
		marker.publish(pose_array)
		rate.sleep()
		#print "Reducing queue size to", queue

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
