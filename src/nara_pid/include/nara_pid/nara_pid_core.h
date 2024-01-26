#ifndef SR_NARA_PID_CORE_H
#define SR_NARA_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include <nara_msgs/PID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <nara_pid/naraPIDConfig.h>

class NaraPID
{
public:
  NaraPID();
  ~NaraPID();
  void configCallback(nara_pid::naraPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const nara_msgs::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif
