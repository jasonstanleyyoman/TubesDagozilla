#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include <ctime>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/ros.h>
#include "bits/stdc++.h"
#include "robot_control/ModelState.h"
#include "std_msgs/String.h"
#include <ignition/math/Pose3.hh>
#include <cmath>

#include <ignition/math/Angle.hh>


#define im ignition::math
#define pi 3.141593

namespace gazebo
{
	class RobotPlugin : public ModelPlugin{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	    {
	      
			this->model = _parent;
			this->world = this->model->GetWorld();
			this->ballModel = this->world->ModelByName("ball");

			this->robotName = this->model->GetName();
			this->team = this->robotName[0];
			
			this->model->SetWorldPose(im::Pose3d(this->model->WorldPose().Pos().X(),this->model->WorldPose().Pos().Y(),0,0,0,0.1));

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  std::bind(&RobotPlugin::OnUpdate, this));


			
			if (!ros::isInitialized()){
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, this->robotName ,ros::init_options::NoSigintHandler);
			}
			this->rosNode.reset(new ros::NodeHandle(this->robotName));


			ros::SubscribeOptions so = ros::SubscribeOptions::create<robot_control::ModelState>("/ball_position",1,boost::bind(&RobotPlugin::BallCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			so = ros::SubscribeOptions::create<std_msgs::String>("/ball_possesion_master",1,boost::bind(&RobotPlugin::ballPosCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
			this->ballPosSub = this->rosNode->subscribe(so);


			this->rosQueueThread = std::thread(std::bind(&RobotPlugin::QueueThread, this));


			this->rosPub = this->rosNode->advertise<std_msgs::String>("/ball_possesion",1,true);

			
		}
	    public: void OnUpdate()
	    {
	      
	      this->position.Set(this->model->WorldPose().Pos().X(),this->model->WorldPose().Pos().Y());
	      this->velocity.Set(this->model->RelativeLinearVel().X(),this->model->RelativeLinearVel().Y());

	      this->rotation.Radian(this->model->WorldPose().Rot().Yaw());
	      this->rotation.Normalize();
	      if(this->rotation.Radian() < 0){
	      	this->rotation += 2 * pi;
	      }

	      checkBallPossesion();
	      if(this->has_ball){
	      	std_msgs::String msg;
	      	msg.data = this->robotName;
	      	this->rosPub.publish(msg);
	      }

	      if(this->has_ball and !this->readyToShoot){
	      	moveBall();
	      }


	      if(this->readyToShoot){
	      	
	      	if(this->timeEndShoot == 0){
	      		this->timeEndShoot = time(NULL) + 1;	
	      		shoot();
	      	}else{
	      		if(time(NULL) > this->timeEndShoot){
	      			this->readyToShoot = false;
	      			this->timeEndShoot = 0;
	      		}
	      	}
	      	
	      }
	      
	    }
	    public : void BallCallback(const robot_control::ModelState::ConstPtr &_msg){
	    	this->ballPosition.Set(_msg->point.x,_msg->point.y);
	    	std::cout << this->curBallPosTeam << std::endl;
	    	if(!this->has_ball and this->curBallPosTeam ==  "None"){
	    		moveTo(_msg->point.x,_msg->point.y,0,true);	
	    	}else if(this->has_ball){
	    		if(this->curBallPosTeam == "B"){
	    			moveTo(3,0,0,false);	
	    		}else{
	    			moveTo(-3,0,pi,false);	
	    		}
	    		
	    	}else{
	    		if(this->curBallPosTeam == "A"){
	    			moveTo(3,0,0,false);	
	    		}else{
	    			moveTo(-3,0,pi,false);	
	    		}
	    	}
	    	
	    }
		
	    public : void ballPosCallback(const std_msgs::String::ConstPtr &msg){
	    	if(msg->data == "None"){
	    		this->curBallPos = "None";
	    		this->curBallPosTeam = "None";
	    	}else{
	    		this->curBallPos = msg->data;
	    		this->curBallPosTeam = msg->data[0];	
	    	}
	    	
	    }
		
		private:
			void moveTo(double x, double y,double orientation,bool isBall){
				im::Vector2d target(x,y);
				im::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;

				im::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X() * 0.7;
				double ySpeed = this->velocity.Y() + steer.Y() * 0.8;

				im::Vector2d newSpeed(xSpeed,ySpeed);

				if(newSpeed.Distance(im::Vector2d(0,0)) >= this->maxSpeed){
					newSpeed.Normalize();
					newSpeed*=this->maxSpeed;
				}
				if(isBall){
					im::Angle desiredOrientation(atan2(desired.Y(),desired.X()));
					
					if(desiredOrientation.Radian() < 0){
						desiredOrientation += 2 * pi;
					}
					if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.01){
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,0));	
					}else{
						if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
							this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,2));	
						}else{
							this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,-2));	
						}
					}
						
				}else{
					if(abs(orientation - this->rotation.Radian()) < 0.01){
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,0));	
					}else{
						if(orientation - this->rotation.Radian() > 0){
							this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,2));	
						}else{
							this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,-2));
						}
						
					}

					if(this->position.Distance(target) <= 0.01 and abs(orientation - this->rotation.Radian()) < 0.01){
						this->readyToShoot = true;
					}
				}
				
			}	
		
			
			public : void moveToBall(){

			}

			public : void moveAndShoot(){

			}
			
		
			
			void checkBallPossesion(){
				im::Vector2d edge(this->position.X() + this->radius * cos(this->rotation.Radian()),this->position.Y() + this->radius * sin(this->rotation.Radian()));
				if(edge.Distance(this->ballPosition) - this->ballRadius <= 0.01){
					this->has_ball = true;
				}else{
					this->has_ball = false;
				}
			}

			void moveBall(){
				double x = this->position.X() + (this->radius + this->ballRadius) * cos(this->rotation.Radian());
				double y = this->position.Y() + (this->radius + this->ballRadius) * sin(this->rotation.Radian());
				this->ballModel->SetWorldPose(im::Pose3d(x,y,0,0,0,0));
			}

			void shoot(){
				if(this->has_ball and this->curBallPosTeam == "A"){
					this->has_ball = false;
					this->ballModel->SetWorldTwist(im::Vector3d(-2,0,3),im::Vector3d(0,0,0));	
				}else if(this->has_ball){
					this->has_ball = false;
					this->ballModel->SetWorldTwist(im::Vector3d(2,0,3),im::Vector3d(0,0,0));
				}
				
			}



			void QueueThread(){
			  static const double timeout = 0.01;
			  while (this->rosNode->ok()){
			    this->rosQueue.callAvailable(ros::WallDuration(timeout));
			  }
			}
	    // Pointer to the model
	    private: physics::ModelPtr model;

	    private: physics::ModelPtr ballModel;

		private: physics::WorldPtr world;

	    // Pointer to the update event connection
	    private: event::ConnectionPtr updateConnection;

	    private: std::unique_ptr<ros::NodeHandle> rosNode;

	    private: ros::Subscriber rosSub;

	    private: ros::Subscriber ballPosSub;

		private: ros::Publisher rosPub;

	    private: ros::CallbackQueue rosQueue;

	    private: std::thread rosQueueThread;

	    private : 

	    	im::Vector2d position;

	    	im::Vector2d velocity;

	    	im::Vector2d acceleration;

	    	im::Vector2d ballPosition;

			im::Angle rotation;

	    	std::string robotName;
	    	std::string team;

	    	std::string curBallPos;
	    	std::string curBallPosTeam;

	    	double maxSpeed = 1.5;
	    	double maxForce;
	    	double maxAngularSpeed = 0.3;
	    	double radius = 0.09;
	    	double ballRadius = 0.043;

	    	bool has_ball = false;
	    	bool readyToShoot = false;

	    	
	    	int timeEndShoot = 0;

	  
	};
	GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}

