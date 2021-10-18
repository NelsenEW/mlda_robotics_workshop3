#include <cmath>
#include <algorithm>
#include <turtlebot3_controller/turtlebot3Controller.hpp>

namespace turtlebot_controller {

TurtlebotController::TurtlebotController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), 
		bboxTopic_("/bbox_array"), cmdTopic_("/bbox_array"), xVel_(0.5), kpAng_(20) , kiAng_(20), kdAng_(20){
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	bboxSubscriber_ = nodeHandle_.subscribe(bboxTopic_, subscriberQueueSize_,
			&TurtlebotController::bboxCallback, this);

    velPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdTopic_, 0);
}

TurtlebotController::~TurtlebotController() {
}

bool TurtlebotController::readParameters() {
	bool success = true;
	success &= nodeHandle_.getParam(
			"subscriber_topic_name", bboxTopic_);
	success &= nodeHandle_.getParam(
			"publisher_topic_name", cmdTopic_);
	success &= nodeHandle_.getParam(
			"subscriber_queue_size",
			subscriberQueueSize_);
	success &= nodeHandle_.getParam(		
			"angular_scan_vel", angVel_);
    success &= nodeHandle_.getParam(
			"x_vel", xVel_);
	success &= nodeHandle_.getParam(
			"kp", kpAng_);
	success &= nodeHandle_.getParam(
			"ki", kiAng_);
	success &= nodeHandle_.getParam(
			"kd", kdAng_);
	return success;
}


void TurtlebotController::bboxCallback(
		const std_msgs::Float32MultiArray::ConstPtr &msg) {
	static float xMid = 0, yMid = 0;
	// Step 1: Check if the target robot is detected
	if ((msg->data.size())){
		// Step 2: If it is detected, get the pixel location of the middle of the bounding box
		xMid = (msg->data[0] + msg->data[2])/2;
		yMid = 1- (msg->data[1] + msg->data[3])/2;
		// Step 3: Get the angle from the center of the robot to the target in degree
		float targetAngle = atan2((0.5 - xMid), yMid) * 180.0/ M_PI;
		float area = (msg->data[2] - msg->data[0]) * (msg->data[3] - msg->data[1]);
		ROS_INFO_STREAM_THROTTLE(1.0, "yMid:" << yMid);
		ROS_INFO_STREAM_THROTTLE(1.0, "area:" << area);
		ROS_INFO_STREAM_THROTTLE(1.0, "target Angle:" << targetAngle);
		publishVelocity(targetAngle, xVel_ * (1 + (1 - area)));
	}
	else{
		// Step 4: If it is not detected, rotate based on the last position of the robot
		if (0.5 > xMid)
			publishVelocity(angVel_, 0);
		else
			publishVelocity(-angVel_, 0);
	}
}

void TurtlebotController::publishVelocity(float angle, float speed){
	velMsg_.linear.x = speed;
	sumErrorAngle_ += angle;
	velMsg_.angular.z = kpAng_*angle + kiAng_*sumErrorAngle_ + kdAng_ * (angle - prevAngle_);
	prevAngle_ = angle;
	velPublisher_.publish(velMsg_);
	ROS_INFO_STREAM_THROTTLE(1.0, "angular z: " << velMsg_.angular.z);
}

}