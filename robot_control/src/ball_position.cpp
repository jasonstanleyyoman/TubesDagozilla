#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"
#include "robot_control/ModelState.h"
#include <iostream>
ros::Publisher ball_position_publisher;
ros::Subscriber ball_position_subscriber;
void ballPositionCallback(const gazebo_msgs::ModelStates::ConstPtr& message_holder){
	robot_control::ModelState sent_message;
	int size = 6;
	for(int i = 0; i < size;i++){
		if(message_holder->name[i] == "ball"){
			sent_message.point = message_holder->pose[i].position;
			sent_message.orientation = message_holder->pose[i].orientation;
			sent_message.twist = message_holder->twist[i];
			ball_position_publisher.publish(sent_message);
		}
	}
}
int main(int argc, char *argv[]){
	ros::init(argc,argv,"ball_position");
	ros::NodeHandle nh;
	
	ball_position_publisher = nh.advertise<robot_control::ModelState>("/ball_position",1,true);
	ball_position_subscriber = nh.subscribe("/gazebo/model_states",1,ballPositionCallback);

	ros::Rate rate(30);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}

}
