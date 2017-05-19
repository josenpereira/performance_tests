/*
	Performance tests
	cpp_sub.cpp
	Purpose: Simple c++ subscriber of SuperAwesome messages via
			  topic super_awesome_topic. To estimate the rate of
			  arriving messages the node contains a global msg_count
			  variable that is reset every 5 seconds.

	@autor Jose Pereira
	@version 1.0 19/05/2017
*/

#include "ros/ros.h"
#include "performance_tests/SuperAwesome.h"
#include "boost/bind.hpp"

// Global variable storing number of received messages
unsigned long long int msg_count = 0;

/*
	Subscriber callback, increases msg_count

	@param msg Message received via super_awesome_topic
*/
void subCB(const performance_tests::SuperAwesome::ConstPtr& msg){
	//ROS_INFO("Got %s",msg->S.c_str());
	msg_count++;
}

/*
	Timer callback, calculates the rate of arriving messages in
	the last 5 seconds, prints the result and resets msg_count.

	@param const ros::TimerEvent& Timer trigger
*/
void timerCB(const ros::TimerEvent&){
	ROS_INFO("Rate = %.2f",static_cast<float>(msg_count)/5.0);
	msg_count = 0;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "cpp_sub");
	ros::NodeHandle n;

	msg_count = 0;
	ros::Subscriber sub = n.subscribe("super_awesome_topic", 100, subCB);

	ros::Timer timer1 = n.createTimer(ros::Duration(5.0),timerCB);

	ros::spin();	

	return 0;
}