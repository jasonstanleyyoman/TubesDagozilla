#ifndef BALL_POSITION_H_
#define BALL_POSITION_H_

#include <ros/ros.h>
#include "robot_control/ModelState.h"
#include "gazebo_msgs/ModelState.h"



class BallPosition
{
public:
	
	BallPosition(ros::NodeHandle* nh);
	
private:

	ros::Subscriber ball_position_subscriber_;
	ros::Publisher ball_position_publisher_;

	void ballPositionCallback(const gazebo_msgs::ModelState::ConstPtr& message_holder);

};
#endif
