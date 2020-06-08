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

	    	im::Angle rotation;

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
	    	}

		private:
			void BallCallback(const robot_control::ModelState::ConstPtr &msg){
				this->ballPosition.Set(msg->point.x,msg->point.y);
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
		    	decideMove();
	    	
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
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);
					}else{
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);
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
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed);
					}else{
						applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed);
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
				if(this->ballPosition.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) < 1 and this->curBallPosTeam == 'N' and !this->has_ball){
					moveToBall();
				}else if(this->ballPosition.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) < 1 and this->curBallPosTeam != this->team){
					moveTo(this->ballOwner.X() + 0.25 * (this->goalSelfX - this->ballOwner.X()), 0.75 * this->ballOwner.Y(),0);
				}else{
					im::Vector2d target(this->goalSelfX + 0.25 * (this->ballPosition.X() - this->goalSelfX), 0.25 * this->ballPosition.Y());
					if(target.Distance(im::Vector2d(this->goalSelfX,this->goalSelfY)) >= 1){
						target.Normalize();
						target *= 1;
					}
					moveTo(target.X(),target.Y(),0);
				}

			}
	};
	GZ_REGISTER_MODEL_PLUGIN(KeeperPlugin)
}