#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <time.h>

namespace turtlebot_controller {

/**
 * Class containing the Turtlebot Controller
 */
class TurtlebotController {
 public:
  /** Constructor */
  TurtlebotController(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~TurtlebotController();

 private:
  bool readParameters();
  void bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void publishVelocity(float angle, float distance);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber bboxSubscriber_;
  ros::Publisher velPublisher_;
  std::string bboxTopic_, cmdTopic_;
  int subscriberQueueSize_;
  float angVel_, xVel_,kpAng_, kiAng_, kdAng_, prevAngle_, sumErrorAngle_;
  geometry_msgs::Twist velMsg_;
};

}