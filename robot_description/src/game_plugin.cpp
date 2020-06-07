#include <functional>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "robot_control/ModelState.h"
#include "std_msgs/String.h"


#include <chrono>
#include <ctime>


#define im ignition::math
namespace gazebo{
	class GamePlugin : public WorldPlugin{
		public : void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/){
			this->world = _parent;
			
			

			if (ros::ok()){
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "game",ros::init_options::NoSigintHandler);

			  this->rosNode.reset(new ros::NodeHandle("game_master"));

			  
			  this->ball_position_publisher = this->rosNode->advertise<robot_control::ModelState>("/ball_position",1,true);
			  this->ballPosessionMaster = this->rosNode->advertise<std_msgs::String>("/ball_possesion_master",1,true);



			  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::String>("/ball_possesion",1,boost::bind(&GamePlugin::BallPosCallback, this, _1),ros::VoidPtr(), &this->rosQueue);

			  this->ballPossSub = this->rosNode->subscribe(so);

			  this->rosQueueThread = std::thread(std::bind(&GamePlugin::QueueThread, this));

			  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GamePlugin::OnUpdate, this));
			}
		}

		public : void OnUpdate(){
			
			this->ballModel = this->world->ModelByName("ball");
			
			publishBallMessage();
			if(checkGoal()){
				this->world->Reset();
			}

			checkPos();
			
		}

		public : void publishBallMessage(){
			robot_control::ModelState sent_message;
			sent_message.point.x = this->ballModel->WorldPose().Pos().X();
			sent_message.point.y = this->ballModel->WorldPose().Pos().Y();
			sent_message.point.z = this->ballModel->WorldPose().Pos().Z();
			sent_message.twist.linear.x = this->ballModel->RelativeLinearVel().X();
			sent_message.twist.linear.y = this->ballModel->RelativeLinearVel().Y();
			sent_message.twist.linear.z = this->ballModel->RelativeLinearVel().Z();
			this->ball_position_publisher.publish(sent_message);
		}
		public : bool checkGoal(){
			double ballX = this->ballModel->WorldPose().Pos().X();
			double ballY = this->ballModel->WorldPose().Pos().Y();

			return (ballY < 0.9 and ballY > -0.9 and (ballX < -4.5 or ballX > 4.5));
		}
		public :void QueueThread(){
			static const double timeout = 0.01;
			  	while (this->rosNode->ok()){
			   	this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		public : void BallPosCallback(const std_msgs::String::ConstPtr &msg){
			this->timeFlag = time(NULL) + 1;

			std_msgs::String newMsg;
			newMsg.data = msg->data;
			this->ballPosessionMaster.publish(newMsg);

		}

		public : void checkPos(){
			if(time(NULL) > this->timeFlag){
				std_msgs::String newMsg;
				newMsg.data = "None";
				this->ballPosessionMaster.publish(newMsg);
				
			}
		}
		private: event::ConnectionPtr updateConnection;

		private: std::unique_ptr<ros::NodeHandle> rosNode;

		private: ros::CallbackQueue rosQueue;

	    private: std::thread rosQueueThread;

		private:
			physics::WorldPtr world;
			physics::ModelPtr ballModel;

			ros::Publisher ball_position_publisher;
			ros::Publisher ballPosessionMaster;
			ros::Subscriber ballPossSub;
			
			std::chrono::high_resolution_clock::time_point timeExp;

			int timeFlag;
			
		
	};
	GZ_REGISTER_WORLD_PLUGIN(GamePlugin)
}