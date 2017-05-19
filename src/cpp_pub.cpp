/*
  Performance tests
  cpp_sub.cpp
  Purpose: Simple c++ publisher of SuperAwesome messages in
        topic super_awesome_topic. Rate of publishing is
        controlled by pub_timer. To assess limits of topic
        communication we exponentially increase the rate of
        publishing by multiplying the publishing rate by 2
        every 5 seconds.

  @autor Jose Pereira
  @version 1.0 19/05/2017
*/

#include "ros/ros.h"
#include <string>
#include "boost/bind.hpp"
#include "performance_tests/SuperAwesome.h"

/*
  Callback of pub_timer, called with rate rate. Publishes a
  SuperAwesome message.

  @param const ros::TimerEvent& Timer trigger
  @param pub Publisher that publishes the message
*/
void pub_timer_CB(const ros::TimerEvent&, ros::Publisher& pub){
  performance_tests::SuperAwesome msg;
  msg.S = std::string("cpp_msg");
  pub.publish(msg);
}

/*
  Callback of rate_timer. Changes the rate of publishing every
  5 seconds.

  @param const ros::TimerEvent& Timer trigger
  @param timer Timer controlling publishing rate
  @param rate Current rate to be updated
*/
void rate_timer_CB(const ros::TimerEvent&, ros::Timer& timer, unsigned int& rate){
  rate *= 2;
  std::cout << "rate = " << rate << "\n";
  timer.setPeriod(ros::Duration(1.0/static_cast<float>(rate)));
}

int main(int argc, char** argv){

  ros::init(argc,argv,"cpp_pub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<performance_tests::SuperAwesome>("super_awesome_topic", 100);

  // variable defining publishing rate
  unsigned int rate = 1;

  ros::Timer pub_timer = n.createTimer(ros::Duration(1.0/static_cast<float>(rate)),boost::bind(pub_timer_CB,_1,pub));
  ros::Timer rate_timer = n.createTimer(ros::Duration(5.0),boost::bind(rate_timer_CB,_1,pub_timer,rate));

  ros::spin();

  return 0;
}