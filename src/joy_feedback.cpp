// Copyright Lucas Walter November 2013
//

#include <errno.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"

class JoyFeedback {
public:

  JoyFeedback();
  
private:
  void freqCallback(const std_msgs::Float32 & msg);
  bool init();
  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  std::string device_;
  int fd_;
  unsigned long features_[4];
  std::vector<struct ff_effect> effects_; 
  bool initted_;
};

void JoyFeedback::freqCallback(const std_msgs::Float32 & msg)
{
  //ros
  if (!initted_) return;
}

JoyFeedback::JoyFeedback()
{
  sub_ = nh_.subscribe("frequency", 1, freqCallback, this);
  initted_ = init();
}

bool JoyFeedback::init()
{
  ros::param::param<std::string>("feedback_device", device_, "/dev/input/event13");

  fd_ = open(device_.c_str(), O_RDWR);
  if (fd_ == -1) {
    ROS_WARN_STREAM("Open device file " << strerror(errno));
    return false;
  }

  if (ioctl(fd_, EVIOCGBIT(EV_FF, sizeof(unsigned long) * 4), features_) < 0) {
    ROS_WARN_STREAM("ioctl feature query " << strerror(errno));
    return false;
  }

  int max_effects;
  if (ioctl(fd_, EVIOCGEFFECTS, &max_effects) < 0) {
    ROS_WARN_STREAM("ioctl max num effects query " << strerror(errno));
  }
  ROS_INFO_STREAM("number of ff effects " << max_effects);
  effects_.resize(max_effects);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_feedback");

  JoyFeedback joy_feedback;

  ros::spin();

  return 0;
}
