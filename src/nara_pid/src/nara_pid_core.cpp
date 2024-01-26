#include "nara_pid/nara_pid_core.h"

NaraPID::NaraPID()
{
}

NaraPID::~NaraPID()
{
}

void NaraPID::publishMessage(ros::Publisher *pub_message)
{
  nara_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void NaraPID::messageCallback(const nara_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void NaraPID::configCallback(nara_pid::naraPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
