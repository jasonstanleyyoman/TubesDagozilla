#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include <chrono>
#include <ctime>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/ros.h>
#include "bits/stdc++.h"
#include "robot_control/ModelState.h"
#include <ignition/math/Pose3.hh>
#include <cmath>

#define im ignition::math
#define pi 3.141593

namespace gazebo
{
	class RobotPlugin : public ModelPlugin{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	    {
	      // Store the pointer to the model
			this->model = _parent;
			this->world = this->model->GetWorld();
			this->ballModel = this->world->ModelByName("ball");
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->model->SetWorldPose(ignition::math::Pose3d(this->model->WorldPose().Pos().X(),this->model->WorldPose().Pos().Y(),0,0,0,0));
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  std::bind(&RobotPlugin::OnUpdate, this));
			this->robotName = this->model->GetName();
			this->team = this->robotName[0];
			if (!ros::isInitialized()){
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "gazebo_client",
			      ros::init_options::NoSigintHandler);
			}
			this->rosNode.reset(new ros::NodeHandle("robot_handler"));
			ros::SubscribeOptions so = ros::SubscribeOptions::create<robot_control::ModelState>(
		      "/ball_position",
		      1,
		      boost::bind(&RobotPlugin::BallCallback, this, _1),
		      ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);
			this->rosQueueThread = std::thread(std::bind(&RobotPlugin::QueueThread, this));

	      
	    }
	    public: void OnUpdate()
	    {
	      
	      this->position.Set(this->model->WorldPose().Pos().X(),this->model->WorldPose().Pos().Y());
	      this->velocity.Set(this->model->RelativeLinearVel().X(),this->model->RelativeLinearVel().Y());
	      this->rotation = this->model->WorldPose().Rot().Yaw();
	      checkBallPossesion();
	      if(this->has_ball and !this->readyToShoot){
	      	moveBall();
	      }
	      if(this->readyToShoot){
	      	
	      	shoot();
	      	if(this->timeEndShoot == 0){
	      		this->timeEndShoot = time(NULL) + 1;	
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
	    	if(!this->has_ball){
	    		moveTo(_msg->point.x,_msg->point.y,0,true);	
	    	}else{
	    		moveTo(-3,0,pi,false);
	    	}
	    	
	    }
		

		
		private:
			void moveTo(double x, double y,double orientation,bool isBall){
				ignition::math::Vector2d target(x,y);
				ignition::math::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;

				ignition::math::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X() * 0.3;
				double ySpeed = this->velocity.Y() + steer.Y() * 0.3;

				ignition::math::Vector2d newSpeed(xSpeed,ySpeed);

				if(newSpeed.Distance(ignition::math::Vector2d(0,0)) >= this->maxSpeed){
					newSpeed.Normalize();
					newSpeed*=this->maxSpeed;
				}
				if(isBall){
					double desiredOrientation = atan(desired.Y() / desired.X());
					if(desired.X() < 0){
						if(desiredOrientation < 0){
							desiredOrientation += pi;
						}else{
							desiredOrientation -= pi;
						}
					}
					
					if(abs(desiredOrientation - this->rotation) < 0.001){
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,0));	
					}else{
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,2));
					}	
				}else{
					if(abs(orientation - this->rotation) < 0.001){
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,0));	
					}else{
						this->model->SetWorldTwist(im::Vector3d(newSpeed.X(),newSpeed.Y(),0),im::Vector3d(0,0,2));
					}

					if(this->position.Distance(target) <= 0.01 and abs(orientation - this->rotation) < 0.01){
						this->readyToShoot = true;
					}
				}
				
				

			}

			
		
			
			void checkBallPossesion(){
				im::Vector2d edge(this->position.X() + this->radius * cos(this->rotation),this->position.Y() + this->radius * sin(this->rotation));
				if(edge.Distance(this->ballPosition) - this->ballRadius <= 0.01){
					this->has_ball = true;
				}else{
					this->has_ball = false;
				}
			}

			void moveBall(){
				double x = this->position.X() + (this->radius + this->ballRadius) * cos(this->rotation);
				double y = this->position.Y() + (this->radius + this->ballRadius) * sin(this->rotation);
				this->ballModel->SetWorldPose(im::Pose3d(x,y,0,0,0,0));
			}

			void shoot(){
				this->has_ball = false;
				this->ballModel->SetWorldTwist(im::Vector3d(-2,0,0),im::Vector3d(0,0,0));
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

	    private: ros::CallbackQueue rosQueue;

	    private: std::thread rosQueueThread;

	    private : 
	    	double initialX;
	    	double initialY;



	    	ignition::math::Vector2d position;

	    	ignition::math::Vector2d velocity;

	    	ignition::math::Vector2d acceleration;

	    	im::Vector2d ballPosition;


	    	
	    	double rotation;

	    	std::string robotName;
	    	std::string team;

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
