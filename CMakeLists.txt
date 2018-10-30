cmake_minimum_required(VERSION 2.8.3)
project(moving_base)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	rosserial_python
	message_generation
)

add_message_files(FILES
    arduino_status.msg
    motordesc.msg
)

  generate_messages(
  	DEPENDENCIES
  	std_msgs
  )

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
