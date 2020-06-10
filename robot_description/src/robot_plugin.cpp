#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <cmath>
#include <ctime>
#include <thread>
#include <bits/stdc++.h>
#include <robot_control/ModelState.h>
#include <std_msgs/String.h>
#include <robot_control/BallPos.h>
#include <robot_control/RobotsPosition.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Pose3.hh>
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
				this->goalTargetX = -4.5;
				this->goalSelfX = 4.5;
			}else{
				this->goalTargetX = 4.5;
				this->goalSelfX = -4.5;
			}
			this->goalTargetY = 0;
			this->goalSelfY = 0;
			

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

			so = ros::SubscribeOptions::create<robot_control::RobotsPosition>("/robots_position",1,boost::bind(&RobotPlugin::RobotsPositionCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
			this->robotsPosSub = this->rosNode->subscribe(so);


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
	    	this->ballHeight = _msg->point.z;
	    	this->ballVelocity.Set(_msg->twist.linear.x,_msg->twist.linear.y,_msg->twist.linear.z);
	    	this->defaultOrientation.Radian(atan2(this->ballPosition.Y() - this->position.Y(), this->ballPosition.X() - this->position.X()));
	    	if(this->defaultOrientation.Radian() < 0){
	    		this->defaultOrientation += 2 * pi;
	    	}
	    	decideMove();
	    }
		
	    public : void ballPosCallback(const robot_control::BallPos::ConstPtr &msg){
	    	if(msg->robotName == "None"){
	    		this->has_ball = false;
	    		this->ballOwner = "None";
	    		this->ballOwnerTeam = 'N';
	    	}else{
	    		this->ballOwner = msg->robotName;
	    		this->ballOwnerTeam = msg->robotName[0];	
	    		this->ballOwnerPos.Set(msg->x,msg->y);
	    		if(msg->robotName == this->robotName){
	    			this->has_ball = true;
	    		}
	    	}
	    	decideMove();
	    	
	    }




	    public : void RobotsPositionCallback(const robot_control::RobotsPosition::ConstPtr &msg){
	    	for(int i = 0; i < msg->robotsPos.size();i++){
	    		this->robotsPosition[msg->robotsPos[i].robotName] = im::Vector2d(msg->robotsPos[i].x,msg->robotsPos[i].y);
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

				im::Angle desiredOrientation(atan2(desired.Y(),desired.X()));
					
				if(desiredOrientation.Radian() < 0){
					desiredOrientation += 2 * pi;
				}
				if(abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
					applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),0);
				}else{
					if(desiredOrientation.Radian() - this->rotation.Radian() > 0){
						if(desiredOrientation.Radian() - this->rotation.Radian() - pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed - 1);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed + 1);	
						}
					}else{
						if(desiredOrientation.Radian() - this->rotation.Radian() + pi > 0){
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),-1 * this->maxAngularSpeed - 1);	
						}else{
							applyLinearAngularSpeed(newSpeed.X(),newSpeed.Y(),this->maxAngularSpeed + 1);	
						}
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

				im::Angle desiredOrientation(atan2(this->goalTargetY - y,this->goalTargetX - x));
				if(desiredOrientation.Radian() < 0) desiredOrientation += 2 * pi;

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
				if(this->position.Distance(target) <= 0.01 and abs(desiredOrientation.Radian() - this->rotation.Radian()) < 0.1){
						this->readyToShoot = true;
				}
			}

			void decideMove(){
				if(!this->has_ball and this->ballOwnerTeam ==  'N'){
					if(checkClosest(this->ballPosition.X(),this->ballPosition.Y())){
						moveToBall();	
					}else{
						if(abs(this->goalTargetX - this->ballPosition.X()) < abs(this->goalSelfX - this->ballPosition.X())){
							attack();
						}else{

							defend();
						}
					}
		    		
		    	}else if(this->has_ball){
		    		if(this->ballOwnerTeam == 'B'){
		    			moveAndShoot(2,2);	
		    		}else{
		    			moveAndShoot(-2,-2);	
		    		}
		    		
		    	}else{
		    		if(this->ballOwnerTeam != this->team){
		    			defend();
		    		}else{
		    			attack();
		    		}
		    	}
			}

			void defend(){
				if(checkClosest(this->ballOwnerPos.X(),this->ballOwnerPos.Y())){
					moveTo(this->ballOwnerPos.X() + 0.25 * (this->goalSelfX - this->ballOwnerPos.X()), 0.75 * this->ballOwnerPos.Y(),this->defaultOrientation.Radian());
					
				}else if(this->ballOwnerTeam != this->team and this->ballOwnerTeam != 'N'){
					for(auto const&robot : this->robotsPosition){
						if(robot.first[0] != this->team and robot.first != this->ballOwner and robot.first[6] != 'K'){
							moveTo(this->ballOwnerPos.X() + 0.75 * (robot.second.X() - this->ballOwnerPos.X()), this->ballOwnerPos.Y() + 0.75 * (robot.second.Y() - this->ballOwnerPos.Y()),this->defaultOrientation.Radian());
						}
					}
				}
			}
			void attack(){
				if(this->team == 'A'){
					moveTo(-1,-1,this->defaultOrientation.Radian());
				}else{
					moveTo(1,-1,this->defaultOrientation.Radian());
				}
			}


			void applyLinearAngularSpeed(double x, double y, double angular){
				this->model->SetWorldTwist(im::Vector3d(x,y,0),im::Vector3d(0,0,angular));
			}

			bool checkClosest(double x, double y){
				std::string closestName;
				double minimum = INT_MAX;
				for(auto const&robot : this->robotsPosition){
					if(robot.first[0] == this->team and robot.first[6] != 'K'){
						double distance = robot.second.Distance(im::Vector2d(x,y));
						if(distance < minimum){
							minimum = distance;
							closestName = robot.first;
						}
					}
				}
				if(closestName == this->robotName){
					return true;
				}
				return false;
			}
			
		
			
			void checkBallPossesion(){
				im::Vector2d edge(this->position.X() + this->radius * cos(this->rotation.Radian()),this->position.Y() + this->radius * sin(this->rotation.Radian()));
				if(edge.Distance(this->ballPosition) - this->ballRadius <= 0.01 and this->ballHeight <= 0.15){
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
					this->ballOwner = "None";
					this->ballOwnerTeam = 'N';
					im::Vector2d speed(cos(this->rotation.Radian()),sin(this->rotation.Radian()));
					double goalDistance = this->ballPosition.Distance(im::Vector2d(this->goalTargetX,this->goalTargetY));
					speed *= goalDistance;

					this->ballModel->SetWorldTwist(im::Vector3d(speed.X(),speed.Y(),goalDistance),im::Vector3d(0,0,0));	
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

	    private: event::ConnectionPtr updateConnection;

	    private: std::unique_ptr<ros::NodeHandle> rosNode;

	    private: ros::Subscriber rosSub;

	    private: ros::Subscriber ballPosSub;

	    private: ros::Subscriber robotsPosSub;

		private: ros::Publisher rosPub;

	    private: ros::CallbackQueue rosQueue;

	    private: std::thread rosQueueThread;

	    private : 

	    	im::Vector2d position;

	    	im::Vector2d velocity;

	    	im::Vector2d ballPosition;

	    	im::Vector3d ballVelocity;

	    	double ballHeight;

			im::Angle rotation;
			im::Angle defaultOrientation;

			im::Vector2d ballOwnerPos;

	    	std::string robotName;

	    	char team;

	    	std::string ballOwner;
	    	char ballOwnerTeam;

	    	std::map<std::string,im::Vector2d> robotsPosition;

	    	double maxSpeed = 2;
	    	double maxAngularSpeed = 4;
	    	double radius = 0.09;
	    	double ballRadius = 0.043;

	    	bool has_ball = false;
	    	bool readyToShoot = false;
	    	bool decided = false;
	    	
	    	int timeEndShoot = 0;

	    	double goalTargetX;
	    	double goalTargetY;

	    	double goalSelfX;
	    	double goalSelfY;
	};
	GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}

