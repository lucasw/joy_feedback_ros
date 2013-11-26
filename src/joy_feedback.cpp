// Copyright Lucas Walter November 2013
//

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

#include "joy_feedback_ros/Envelope.h"
#include "joy_feedback_ros/Rumble.h"
#include "joy_feedback_ros/Periodic.h"

class JoyFeedback {
public:

  JoyFeedback();
  
private:
  void rumbleCallback(const joy_feedback_ros::Rumble::ConstPtr& msg);
  void playCallback(const std_msgs::UInt16::ConstPtr& msg);
  bool init();
  ros::NodeHandle nh_;

  ros::Subscriber rumble_sub_;
  ros::Subscriber play_sub_;

  std::string device_;
  int fd_;
  unsigned long features_[4];
  std::vector<struct ff_effect> effects_; 
  bool initted_;
};

void JoyFeedback::playCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  const int ind = msg->data;
  if (ind >= effects_.size()) {
    ROS_WARN_STREAM("play index is too big " << ind << " " << effects_.size()
      << " " << strerror(errno)); 
  }

  struct input_event play;
  play.type = EV_FF;
  play.code = effects_[ind].id;
  // has this effect been uploaded yet?
  if (play.code < 0) return;
  // set this to zero to stop the effect
  play.value = 1;

  if (write(fd_, (const void*) &play, sizeof(play)) == -1) {
    ROS_WARN_STREAM("play failed " << ind << " " << strerror(errno)); 
  }
}



void JoyFeedback::rumbleCallback(const joy_feedback_ros::Rumble::ConstPtr& msg)
{
  //ros
  if (!initted_) return;

  const int ind = 0;
  effects_[ind].type = FF_RUMBLE;
  effects_[ind].id = -1;
  effects_[ind].u.rumble.strong_magnitude = msg->strong_magnitude;
  effects_[ind].u.rumble.weak_magnitude   = msg->weak_magnitude;
  effects_[ind].replay.length = 5000;
  effects_[ind].replay.delay = 1000;

  if (ioctl(fd_, EVIOCSFF, &effects_[ind]) < 0) {
    ROS_WARN_STREAM("ioctl upload effect " << ind << " " << strerror(errno));
  }
}

JoyFeedback::JoyFeedback() : 
  fd_(0)
{
  rumble_sub_ = nh_.subscribe<joy_feedback_ros::Rumble>("rumble", 1, &JoyFeedback::rumbleCallback, this);
  play_sub_ = nh_.subscribe("play", 1, &JoyFeedback::playCallback, this);
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
  
  if (max_effects < 1) return false;

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
