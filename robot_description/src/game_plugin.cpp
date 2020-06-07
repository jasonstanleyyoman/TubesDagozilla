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

#define im ignition::math
namespace gazebo{
	class GamePlugin : public WorldPlugin{
		public : void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/){
			this->world = _parent;
			
			

			if (ros::ok()){
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "game",ros::init_options::NoSigintHandler);

			  
			  this->ball_position_publisher = this->nh.advertise<robot_control::ModelState>("/ball_position",1,true);
			  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	          std::bind(&GamePlugin::OnUpdate, this));
			}
		}

		public : void OnUpdate(){
			
			this->ballModel = this->world->ModelByName("ball");
			
			publishBallMessage();
			if(checkGoal()){
				this->world->Reset();
			}
			
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

		private: event::ConnectionPtr updateConnection;
		private:
			physics::WorldPtr world;
			physics::ModelPtr ballModel;

			ros::NodeHandle nh;
			ros::Publisher ball_position_publisher;
			

			
		
	};
	GZ_REGISTER_WORLD_PLUGIN(GamePlugin)
}