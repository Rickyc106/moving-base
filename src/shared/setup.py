#!/usr/bin/env python

import subprocess
import rospy

def main():
	name = "PLAYSTATION(R)3 Controller"
	id_string = "id"

	string = subprocess.check_output(['xinput', 'list'])
	name_index = string.find(name)

	if name_index >= 0:
		new_string = string[name_index:]
		id_index = new_string.find(id_string)

		id_value = new_string[id_index+3:id_index+5]
		print ""
		print "PS3 Controller Found! \t ID = {}".format(id_value)
		print ""

		subprocess.call(["xinput", "disable", id_value])
	else:
		print "PS3 Controller is not connected! Try plugging in a USB cable first."

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
