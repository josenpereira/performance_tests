#!/usr/bin/env python
'''
:author: Jose Pereira
:maintainer: Jose Pereira(josenunopereira@gmail)
:description: Simple python subscriber of SuperAwesome messages via
			  topic super_awesome_topic. To estimate the rate of
			  arriving messages the node contains a global msg_count
			  variable that is reset every 5 seconds.
'''

import rospy
from performance_tests.msg import SuperAwesome

# Global variable storing number of received messages
msg_count = 0

def sub_CB(data):
	'''Subscriber callback, increases msg_count'''
	global msg_count
	msg_count += 1

def timer_CB(event):
	'''Timer callback, calculates the rate of arriving messages in
	   the last 5 seconds, prints the result and resets msg_count.'''
	global msg_count
	rospy.loginfo(msg_count/5.0)
	msg_count = 0

if __name__ == '__main__':

	try:
		rospy.init_node('py_sub')
		
		sub = rospy.Subscriber("super_awesome_topic",SuperAwesome,sub_CB)

		timer = rospy.Timer(rospy.Duration(5),timer_CB)

		rospy.spin()
	except rospy.ROSInterruptException:
		pass