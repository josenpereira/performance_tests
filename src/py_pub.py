#!/usr/bin/env python
'''
:author: Jose Pereira
:maintainer: Jose Pereira(josenunopereira@gmail)
:description: Simple python publisher of SuperAwesome messages in
			  topic super_awesome_topic. Rate of publishing is
			  controlled by pub_timer. To assess limits of topic
			  communication we exponentially increase the rate of
			  publishing by multiplying the publishing rate by 2
			  every 5 seconds.
'''

import rospy
from performance_tests.msg import SuperAwesome
from functools import partial

# global variable defining publishing rate
rate = 1

def pub_timer_CB(event, publisher):
	'''Callback of pub_timer, called with rate rate. Publishes a
	   SuperAwesome message.'''
	msg = SuperAwesome()
	msg.S = 'py_msg'
	pub.publish(msg)

def rate_timer_CB(event, publisher, timer):
	'''Callback of rate_timer. Changes the rate of publishing every
	   5 seconds.'''
	global rate
	rate = rate*2
	timer = rospy.Timer(rospy.Duration(1.0/rate),partial(pub_timer_CB, publisher = publisher))

if __name__ == '__main__':
	try:

		rospy.init_node('py_pub')
		pub = rospy.Publisher('super_awesome_topic', SuperAwesome, queue_size = 100)

		pub_timer = rospy.Timer(rospy.Duration(1),partial(pub_timer_CB, publisher = pub))
		rate_timer = rospy.Timer(rospy.Duration(5),partial(rate_timer_CB, publisher = pub, timer = pub_timer))

		rospy.spin()
	except rospy.ROSInterruptException:
		pass 