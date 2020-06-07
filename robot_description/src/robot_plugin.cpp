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
#include "robot_control/BallPos.h"
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

			

			if(this->team == 'A'){
				this->goalX = -4.5;
			}else{
				this->goalX = 4.5;
			}
			this->goalY = 0;
			
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

			so = ros::SubscribeOptions::create<robot_control::BallPos>("/ball_possesion_master",1,boost::bind(&RobotPlugin::ballPosCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
			this->ballPosSub = this->rosNode->subscribe(so);


			this->rosQueueThread = std::thread(std::bind(&RobotPlugin::QueueThread, this));


			this->rosPub = this->rosNode->advertise<std_msgs::String>("/ball_possesion",1,true);

			
		}
	    public: void OnUpdate()
	    {
	      if(!this->checkTeam){
	      	for(int i = 0; i < this->world->ModelCount();i++){
				std::string name = this->world->ModelByIndex(i)->GetName();
				if(name[0] == this->team and name.substr(1,5) == "Robot"){
					this->teamList.push_back(this->world->ModelByIndex(i)->GetName());
				}
			}
			this->checkTeam = true;
	      }
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
	    	decideMove();
	    }
		
	    public : void ballPosCallback(const robot_control::BallPos::ConstPtr &msg){
	    	if(msg->robotName == "None"){
	    		this->curBallPos = "None";
	    		this->curBallPosTeam = "None";
	    	}else{
	    		this->curBallPos = msg->robotName;
	    		this->curBallPosTeam = msg->robotName[0];	
	    		this->ballOwner.Set(msg->x,msg->y);
	    	}
	    	
	    }
		
		private:
			void moveTo(double x, double y,double orientation){
				im::Vector2d target(x,y);
				im::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;

				im::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X() * 0.7;
				double ySpeed = this->velocity.Y() + steer.Y() * 0.7;

				im::Vector2d newSpeed(xSpeed,ySpeed);

				if(newSpeed.Distance(im::Vector2d(0,0)) >= this->maxSpeed){
					newSpeed.Normalize();
					newSpeed*=this->maxSpeed;
				}
				if(abs(orientation - this->rotation.Radian()) < 0.01){
					applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),0);
				}else{
					if(orientation - this->rotation.Radian() > 0){
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),3);
					}else{
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-3);
					}
					
				}

				if(this->position.Distance(target) <= 0.01 and abs(orientation - this->rotation.Radian()) < 0.01){
					this->readyToShoot = true;
				}
			}
				
		
		
			
			void moveToBall(){
				im::Vector2d target(this->ballPosition.X(),this->ballPosition.Y());
				im::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;

				im::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X() * 0.7;
				double ySpeed = this->velocity.Y() + steer.Y() * 0.7;

				im::Vector2d newSpeed(xSpeed,ySpeed);

				im::Angle desiredOrientation(atan2(desired.Y(),desired.X()));
					
				if(desiredOrientation.Radian() < 0){
					desiredOrientation += 2 * pi;
				}
				if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
					applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),0);
				}else{
					if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),4);
					}else{
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-4);
					}
				}
			}

			void moveAndShoot(double x, double y){
				im::Vector2d target(x,y);
				im::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;

				im::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X() * 0.7;
				double ySpeed = this->velocity.Y() + steer.Y() * 0.7;

				im::Vector2d newSpeed(xSpeed,ySpeed);

				im::Angle desiredOrientation(atan2(this->goalY - y,this->goalX - x));
				if(desiredOrientation.Radian() < 0){
					desiredOrientation += 2 * pi;
				}

				if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
					applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),0);
				}else{
					if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),3);
					}else{
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-3);
					}
				}

				if(this->position.Distance(target) <= 0.01 and abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
						this->readyToShoot = true;
				}


			}

			void decideMove(){
				if(!this->has_ball and this->curBallPosTeam ==  "None"){
		    		moveToBall();
		    	}else if(this->has_ball){
		    		if(this->curBallPosTeam == "B"){
		    			moveAndShoot(3,1);	
		    		}else{
		    			moveAndShoot(-3,1);	
		    		}
		    		
		    	}else{
		    		if(this->curBallPosTeam != std::string(1,this->team)){
		    			defend();
		    		}else{
		    			attack();
		    		}
		    	}
			}

			void defend(){
				physics::ModelPtr hasBallModel;
				std::cout << "Ball Owner From " << this->robotName << ": x : " << this->ballOwner.X() << " y : " << this->ballOwner.Y() << std::endl;
				// double hasBallX = hasBallModel->WorldPose().Pos().X();
				// double hasBallY = hasBallModel->WorldPose().Pos().Y();

				// std::cout << this->robotName << " x : " << hasBallX << " y : " << hasBallY << std::endl;
				
				// im::Vector2d hasBallPos(hasBallX,hasBallY);
				// std::string closestName;
				// double minimum = INT_MAX;

				// for(int i = 0; i < this->teamList.size();i++){
				// 	physics::ModelPtr teamModel = this->world->ModelByName(teamList[i]);
				// 	im::Vector2d teamPos(teamModel->WorldPose().Pos().X(),teamModel->WorldPose().Pos().Y());
				// 	if(hasBallPos.Distance(teamPos) < minimum){
				// 		minimum = hasBallPos.Distance(teamPos);
				// 		closestName = teamList[i];
				// 	}
				// }
				// if(closestName == this->robotName){}
				if(this->team == 'A'){
					moveTo(this->ballOwner.X() + 0.25 * (4.5 - this->ballOwner.X()), 0.75 * this->ballOwner.Y(),0);
				}else{
					moveTo(this->ballOwner.X() + 0.25 * (-4.5 - this->ballOwner.X()), 0.75 * this->ballOwner.Y(),0);
				}
				

			}
			void attack(){

			}


			void applyLinearAngularSpeed(double x, double y, double angular){
				this->model->SetWorldTwist(im::Vector3d(x,y,0),im::Vector3d(0,0,angular));
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
				this->ballModel->SetWorldTwist(im::Vector3d(0,0,0), im::Vector3d(0,0,0));
			}

			void shoot(){
				if(this->has_ball){
					this->has_ball = false;
					im::Vector2d speed(cos(this->rotation.Radian()),sin(this->rotation.Radian()));
					double goalDistance = this->position.Distance(im::Vector2d(this->goalX,this->goalY));
					speed *= goalDistance;

					this->ballModel->SetWorldTwist(im::Vector3d(speed.X() * 0.8,speed.Y() * 0.8,goalDistance * 0.7),im::Vector3d(0,0,0));	
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

			im::Vector2d ballOwner;

	    	std::string robotName;
	    	char team;

	    	std::vector<std::string> teamList;

	    	std::string curBallPos;
	    	std::string curBallPosTeam;

	    	double maxSpeed = 1.5;
	    	double maxForce;
	    	double maxAngularSpeed = 0.3;
	    	double radius = 0.09;
	    	double ballRadius = 0.043;

	    	bool has_ball = false;
	    	bool readyToShoot = false;

	    	bool checkTeam = false;
	    	
	    	int timeEndShoot = 0;

	    	double goalX;
	    	double goalY;

	  
	};
	GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}

