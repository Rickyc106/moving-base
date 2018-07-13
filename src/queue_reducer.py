import rospy
import tf

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

#queue = None
joy_msg = Joy()

resultant = [0,0,0,0]
pose_zero = [[0 for i in range(4)] for i in range(4)]
quat = []

def joy_cb(data):
	joy_msg.axes = data.axes
	joy_msg.buttons = data.buttons
	#print str(joy_msg.axes)

def array_cb(msg):
	for i in range(4):
		resultant[i] = msg.data[i]

def main():
	queued_joy = rospy.Publisher('joy_queued', Joy, queue_size = 10)
	arrow_pub = rospy.Publisher('resultant_angle', PoseArray, queue_size = 10)

	rospy.init_node('queue_reducer', anonymous=True)

	rospy.Subscriber('joy',  Joy, joy_cb)
	rospy.Subscriber('temp_array_data', Float32MultiArray, array_cb)

	# pose_array.pose.position = [-5,5,0]
	# pose_array.pose[1].position = [-5,-5,0]
	# pose_array.pose[2].position = [5,-5,0]
	# pose_array.pose[3].position = [5,5,0]

	rate = rospy.Rate(45)

	while not rospy.is_shutdown():
		pose_array = PoseArray()
		pose_array.header.frame_id = 'map'
		pose_array.header.stamp = rospy.Time.now()

		for i in range(4):
			resultant[i] = resultant[i] / (180 / 3.141592653)

		quat0 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[0]))
		quat1 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[1]))
		quat2 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[2]))
		quat3 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[3]))

		point0 = Point(-1,1,0)
		point1 = Point(-1,-1,0)
		point2 = Point(1,-1,0)
		point3 = Point(1,1,0)

		pose_array.poses.append(Pose(point0, quat0))
		pose_array.poses.append(Pose(point1, quat1))
		pose_array.poses.append(Pose(point2, quat2))
		pose_array.poses.append(Pose(point3, quat3))

		print str(resultant)

		queued_joy.publish(joy_msg)
		arrow_pub.publish(pose_array)
		rate.sleep()
		#print "Reducing queue size to", queue

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
