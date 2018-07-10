cmake_minimum_required(VERSION 2.8.3)
project(moving_base)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	rosserial_python
	message_generation
)

  add_message_files(
    FILES
    arduino_status.msg
  )

  generate_messages(
  	DEPENDENCIES
  	std_msgs
  )

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
