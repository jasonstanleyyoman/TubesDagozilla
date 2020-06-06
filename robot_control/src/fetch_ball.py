#!/usr/bin/env python

import rospy
import sys
import math
from robot_control.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


robotMaxSpeed = 2.0
robotMaxAngularSpeed = 1.5
robotMaxLinearAcceleration = 0.5


maxSteeringForce = 0.75
robotMaxStoppingForce = 1
dt = float(1.0/30.0)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class PVector():
	def __init__(self,x,y):
		self.x = float(x)
		self.y = float(y)

	def mag(self):
		return math.hypot(self.x,self.y)
	def normalize(self):
		self.x = self.x / self.mag()
		self.y = self.y / self.mag()
	def add(self,x,y):
		self.x = self.x + x
		self.y = self.y + y
	def sub(self,x,y):
		self.x = self.x - x
		self.y = self.y - y
	def mult(self,factor):
		self.x = self.x * factor
		self.y = self.y * factor
	def div(self,factor):
		self.x = self.x / factor
		self.y = self.y / factor
	def limit(self,value):
		if(self.mag() > value):
			self.normalize()
			self.mult(value)
	def __str__(self):
		return "This vector has components : x : {}, y : {}".format(self.x,self.y)
	@staticmethod
	def addVector(vectorA,vectorB):
		return PVector(vectorA.x + vectorB.x, vectorA.y + vectorB.y)
	@staticmethod
	def subVector(vectorA,vectorB):
		return PVector(vectorA.x - vectorB.x, vectorA.y - vectorB.y)
	@staticmethod
	def crossProduct(vectorA,vectorB):
		return vectorA.x * vectorB.x + vectorA.y * vectorB.y
	@staticmethod
	def angleBetween(vectorA,vectorB):
		return math.acos((PVector.crossProduct(vectorA,vectorB))/(vectorA.mag() * vectorB.mag()))
	@staticmethod
	def vectorDistance(vectorA,vectorB):
		return math.sqrt(math.pow((vectorA.x - vectorB.x),2) + math.pow((vectorA.y - vectorB.y),2))

class Callback():
	global robotMaxSpeed
	global robotMaxAngularSpeed
	global maxSteeringForce
	global dt
	global robotMaxLinearAcceleration
	global robotMaxSentripetalAcceleration
	global robotMaxStoppingForce
	

	def __init__(self,robotName):
		self.ballPosition = None
		self.robotPosition = None
		self.pub =  rospy.Publisher(robotName, Twist, queue_size=10)
	def robotCallback(self,data):
		rospy.loginfo("robot received")
		self.robotPosition = ModelState()
		self.robotPosition.point = data.pose.pose.position
		self.robotPosition.orientation = data.pose.pose.orientation
		self.robotPosition.twist.linear = data.twist.twist.linear
		self.robotPosition.twist.angular = data.twist.twist.angular
		self.compute()
	def ballCallback(self,data):
		rospy.loginfo("ball received")
		self.ballPosition = ModelState()
		self.ballPosition = data
		self.compute()
	def compute(self):
		rospy.loginfo("computed called")
		if self.ballPosition != None and self.robotPosition != None:
			
			
			ballVector = PVector(self.ballPosition.point.x,self.ballPosition.point.y)
			robotVector = PVector(self.robotPosition.point.x,self.robotPosition.point.y)
			desired = PVector.subVector(ballVector,robotVector)

			(roll,pitch,yaw) = euler_from_quaternion([self.robotPosition.orientation.x,self.robotPosition.orientation.y,self.robotPosition.orientation.z,self.robotPosition.orientation.w])
			velocityVector = PVector(self.robotPosition.twist.linear.x * math.cos(yaw),self.robotPosition.twist.linear.x * math.sin(yaw))

			robotOrientationVector = PVector(math.cos(yaw),math.sin(yaw))
			
			desired.normalize()
			
			
			distance = PVector.vectorDistance(ballVector,robotVector)
			
			if distance < 1.5:
				multiplier = translate(distance,0,0.2,0,robotMaxSpeed)
				desired.mult(multiplier)

				steer = PVector.subVector(desired,velocityVector)

				steer.limit(robotMaxStoppingForce)
				
			else:
				desired.mult(robotMaxSpeed)
				steer = PVector.subVector(desired,velocityVector)
				steer.limit(maxSteeringForce)

			
			
			
			
			angleBetween = PVector.angleBetween(steer,robotOrientationVector)
			
			


			linearAcceleration = math.cos(angleBetween) * steer.mag()
			
			if(linearAcceleration > robotMaxLinearAcceleration):
				linearAcceleration = robotMaxLinearAcceleration
			linearAcceleration = linearAcceleration * dt

			
			angularForce = math.sin(angleBetween) * steer.mag()
			


			if(robotOrientationVector.x == 0):
				angle = math.radians(90)
			else:
				angle = math.atan(robotOrientationVector.y/robotOrientationVector.x)
			if robotOrientationVector.x < 0 and robotOrientationVector.y < 0:
				angle -= math.pi
			elif robotOrientationVector.x < 0 and robotOrientationVector.y > 0:
				angle += math.pi
			
			y = -steer.x * math.sin(angle) + steer.y * math.cos(angle)
			
			if y < 0:
				angularVelocity = -1 * translate(angularForce,0,maxSteeringForce,0,robotMaxAngularSpeed)
			else:
				angularVelocity =  translate(angularForce,0,maxSteeringForce,0,robotMaxAngularSpeed)
			
			


			twist = Twist()


			if abs(self.robotPosition.twist.linear.x + linearAcceleration) > robotMaxSpeed:
				if self.robotPosition.twist.linear.x + linearAcceleration < 0:
					twist.linear.x = -1 * robotMaxSpeed
				else:
					twist.linear.x =  robotMaxSpeed
			else:
				twist.linear.x = self.robotPosition.twist.linear.x + linearAcceleration


				
			twist.linear.y = 0
			twist.linear.z = 0

			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = angularVelocity
			
			self.pub.publish(twist)


			






if __name__ == "__main__":
	global dt
	dt = float(1.0/30.0)
	rospy.init_node("fetch_ball")
	
	if len(sys.argv) != 2:
		rospy.logwarn("This Node Need One Robot Name Parameter")
	else:
		callback = Callback("/"+sys.argv[1].lower()+"/cmd_vel")
		if sys.argv[1] == "Robot1" or sys.argv[1] == "Robot2" or sys.argv[1] == "Robot3":
			rospy.Subscriber("/ball_position",ModelState,callback.ballCallback)
			rospy.Subscriber("/"+sys.argv[1].lower()+"/odom",Odometry,callback.robotCallback)
		else:
			rospy.logwarn("No Robot With That Name")
	rate = rospy.Rate(int(1.0/dt))
	rospy.spin()
	while not rospy.is_shutdown():
		rate.sleep()