#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/ros.h>
#include "bits/stdc++.h"
#include "std_msgs/String.h"
#include "robot_control/ModelState.h"
#include "robot_control/BallPos.h"
#include "robot_control/RobotsPosition.h"

#define im ignition::math
#define pi 3.141593

namespace gazebo
{
	class KeeperPlugin : public ModelPlugin{
		private: physics::ModelPtr model;

		private: physics::ModelPtr ballModel;

		private: physics::WorldPtr world;

		private: event::ConnectionPtr updateConnection;

	    private: std::unique_ptr<ros::NodeHandle> rosNode;

	    private: ros::Subscriber ballSub;

	    private: ros::Subscriber ballPosSub;

	    private: ros::Subscriber robotsPosSub;

	    private: ros::Publisher rosPub;

	    private: ros::CallbackQueue rosQueue;

	    private: std::thread rosQueueThread;

	    private : 
	    	im::Vector2d position;

	    	im::Vector2d velocity;

	    	im::Vector2d ballPosition;

	    	double ballHeight;

	    	im::Angle rotation;
	    	im::Angle defaultOrientation;

	    	std::string robotName;

	    	char team;

	    	std::string curBallPos;
	    	char curBallPosTeam;
	    	im::Vector2d ballOwner;

	    	std::map<std::string,im::Vector2d> robotsPosition;

	    	double maxSpeed = 1;
	    	double maxAngularSpeed = pi;

	    	double radius = 0.09;
	    	double ballRadius = 0.043;


	    	bool has_ball = false;
	    	bool readyToShoot = false;

	    	int timeEndShoot = 0;

	    	double goalTargetX;
	    	double goalTargetY;

	    	double goalSelfX;
	    	double goalSelfY;
	    public: 
	    	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
		    	this->model = _parent;

		    	this->world = this->model->GetWorld();

				this->ballModel = this->world->ModelByName("ball");

				this->robotName = this->model->GetName();
				this->team = this->robotName[0];

				if(this->team == 'A'){
					this->goalTargetX = -4.5;
					this->goalSelfX = 4.5;
				}else{
					this->goalTargetX = 4.5;
					this->goalSelfX = -4.5;
				}
				this->goalTargetY = 0;
				this->goalSelfY = 0;

		    	if (!ros::isInitialized()){
				  int argc = 0;
				  char **argv = NULL;
				  ros::init(argc, argv, this->robotName ,ros::init_options::NoSigintHandler);
				}

				this->rosNode.reset(new ros::NodeHandle(this->robotName));

				ros::SubscribeOptions so = ros::SubscribeOptions::create<robot_control::ModelState>("/ball_position",1,boost::bind(&KeeperPlugin::BallCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
				this->ballSub = this->rosNode->subscribe(so);

				so = ros::SubscribeOptions::create<robot_control::BallPos>("/ball_possesion_master",1,boost::bind(&KeeperPlugin::BallPosCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
				this->ballPosSub = this->rosNode->subscribe(so);

				so = ros::SubscribeOptions::create<robot_control::RobotsPosition>("/robots_position",1,boost::bind(&KeeperPlugin::RobotsPositionCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
				this->robotsPosSub = this->rosNode->subscribe(so);

				this->rosPub = this->rosNode->advertise<std_msgs::String>("/ball_possesion",1,true);

				this->rosQueueThread = std::thread(std::bind(&KeeperPlugin::QueueThread, this));

		    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				  std::bind(&KeeperPlugin::OnUpdate, this));
	    	}

	    	void OnUpdate(){
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
			    if(this->readyToShoot and time(NULL) > this->timeEndShoot){
			    	this->readyToShoot = false;
			    	this->timeEndShoot = 0;
			    }
	    	}

		private:
			void BallCallback(const robot_control::ModelState::ConstPtr &msg){
				this->ballPosition.Set(msg->point.x,msg->point.y);
				this->ballHeight = msg->point.z;
				this->defaultOrientation.Radian(atan2(this->ballPosition.Y() - this->position.Y(), this->ballPosition.X() - this->position.X()));
				if(this->defaultOrientation.Radian() < 0){
					this->defaultOrientation += 2 * pi;
				}
				decideMove();
			}
			void BallPosCallback(const robot_control::BallPos::ConstPtr &msg){
		    	if(msg->robotName == "None"){
		    		this->has_ball = false;
		    		this->curBallPos = "None";
		    		this->curBallPosTeam = 'N';
		    	}else{
		    		this->curBallPos = msg->robotName;
		    		this->curBallPosTeam = msg->robotName[0];	
		    		this->ballOwner.Set(msg->x,msg->y);
		    		if(msg->robotName == this->robotName){
		    			this->has_ball = true;
		    		}
		    	}
		    	
	    	
	    	}

	    	void RobotsPositionCallback(const robot_control::RobotsPosition::ConstPtr &msg){
		    	for(int i = 0; i < msg->robotsPos.size();i++){
		    		this->robotsPosition[msg->robotsPos[i].robotName] = im::Vector2d(msg->robotsPos[i].x,msg->robotsPos[i].y);
		    	}
		    }
		    void applyLinearAngularSpeed(double x, double y, double angular){
				this->model->SetWorldTwist(im::Vector3d(x,y,0),im::Vector3d(0,0,angular));
			}

			void checkBallPossesion(){
				im::Vector2d edge(this->position.X() + this->radius * cos(this->rotation.Radian()),this->position.Y() + this->radius * sin(this->rotation.Radian()));
				if(edge.Distance(this->ballPosition) - this->ballRadius <= 0.01 and this->ballHeight < 0.15){
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
			void moveToBall(){
				im::Vector2d target(this->ballPosition.X(),this->ballPosition.Y());
				im::Vector2d desired = target - this->position;
				desired.Normalize();
				desired *= this->maxSpeed;
				if(target.Distance(this->position) < 1){
					desired *= 1.2;
				}
				im::Vector2d steer = desired - this->velocity;
				double xSpeed = this->velocity.X() + steer.X();
				double ySpeed = this->velocity.Y() + steer.Y();

				im::Vector2d newSpeed(xSpeed,ySpeed);
				if(newSpeed.Distance(im::Vector2d(0,0)) >= this->maxSpeed){
					newSpeed.Normalize();
					newSpeed*=this->maxSpeed;
				}
				im::Angle desiredOrientation(atan2(desired.Y(),desired.X()));
					
				if(desiredOrientation.Radian() < 0){
					desiredOrientation += 2 * pi;
				}
				if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
					applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),0);
				}else{
					if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
						if(desiredOrientation.Radian() - this->rotation.Radian() - pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);	
						}
					}else{
						if(desiredOrientation.Radian() - this->rotation.Radian() + pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);	
						}
					}
				}
			}
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
						if(orientation - this->rotation.Radian() - pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);	
						}
					}else{
						if(orientation - this->rotation.Radian() + pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);	
						}
					}
					
				}
			}

			void QueueThread(){
			  static const double timeout = 0.01;
			  while (this->rosNode->ok()){
			    this->rosQueue.callAvailable(ros::WallDuration(timeout));
			  }
			}

			void decideMove(){
				if(this->ballPosition.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) < 2 and this->curBallPosTeam == 'N' and !this->has_ball){
					moveToBall();
				}else if(this->ballPosition.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) < 2 and this->curBallPosTeam != this->team and !this->has_ball){
					moveTo(this->ballPosition.X() + 0.25 * (this->goalSelfX - this->ballPosition.X()), 0.75 * this->ballPosition.Y(),0);
				}else if(this->ballPosition.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) >= 2){
					im::Vector2d target(this->goalSelfX + 0.25 * (this->ballPosition.X() - this->goalSelfX), 0.25 * this->ballPosition.Y());
					if(target.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) >= 2){
						if(this->team == 'A'){
							target.X(this->goalSelfX - target.X());
							target.Normalize();
							target *= 2;
							target.X(target.X() + this->goalSelfX);	
						}else{
							target.X(target.X() - this->goalSelfX);
							target.Normalize();
							target *= 2;
							target.X(target.X() + this->goalSelfX);	
						}
						
					}
					moveTo(target.X(),target.Y(),this->defaultOrientation.Radian());
				}else if(this->has_ball and !this->readyToShoot){
					im::Vector2d longest(0,0);
					double longestDistance = -1;
					for(auto const&robot : this->robotsPosition){
						if(robot.first[0] == this->team){
							if(this->position.Distance(robot.second) > longestDistance){
								longest.Set(robot.second.X(),robot.second.Y());
								longestDistance = this->position.Distance(robot.second);
							}
						}
					}
					passBall(longest.X(),longest.Y());
				}

			}
			void passBall(double x, double y){
				im::Angle desiredOrientation(atan2(y - this->position.Y(),x - this->position.X()));
				if(desiredOrientation.Radian() < 0){
					desiredOrientation += 2 * pi;
				}
				if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.01){
					this->readyToShoot = true;
					if(this->has_ball and this->readyToShoot){
						this->has_ball = true;
						this->curBallPos = "None";
						this->curBallPosTeam = 'N';
						im::Vector2d speed(cos(this->rotation.Radian()),sin(this->rotation.Radian()));
						double goalDistance = this->position.Distance(im::Vector2d(x,y));
						speed *= goalDistance;
						this->ballModel->SetWorldTwist(im::Vector3d(speed.X() * 0.8,speed.Y() * 0.8,goalDistance * 0.7),im::Vector3d(0,0,0));
						this->timeEndShoot = time(NULL) + 1;
					}
				}else{
					if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
						if(desiredOrientation.Radian() - this->rotation.Radian() - pi > 0){
							applyLinearAngularSpeed(0,0,-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(0,0,this->maxAngularSpeed);	
						}
					}else{
						if(desiredOrientation.Radian() - this->rotation.Radian() + pi > 0){
							applyLinearAngularSpeed(0,0,-1 * this->maxAngularSpeed);	
						}else{
							applyLinearAngularSpeed(0,0,this->maxAngularSpeed);	
						}
					}
				}
				

			}
	};
	GZ_REGISTER_MODEL_PLUGIN(KeeperPlugin)
}