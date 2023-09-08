from asyncore import write
import imp
from multiprocessing.connection import wait
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile
import math
from localization import Localization
import numpy as np
import utm
import csv
import time

class Motion(Node):
	robot = None
	
	def __init__(self):
		super().__init__('motion')
		self._publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
		self.robot=Localization()
		
		while not (self.robot.odom):			
			print("waiting")
			rclpy.spin_once(self.robot)
   
		#self.robot.odom_reset()
		rclpy.spin_once(self.robot)

		print("okker")
	
	def create_twist(self, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
		twist = Twist()
		twist.linear.x = lin_x
		twist.linear.y = lin_y
		twist.linear.z = lin_z
		twist.angular.x = ang_x
		twist.angular.y = ang_y
		twist.angular.z = ang_z
		return twist
	
	def stop_twist(self):
		twist = self.create_twist()
		self._publisher.publish(twist)
	
	def turn(self, speed, goal_angle, corr_speed = 0.05, corr=True):
		if goal_angle >= 179:
			goal_angle = 179.5
		elif goal_angle <= -179:
			goal_angle = 179.5
		rclpy.spin_once(self.robot)
		#curr_angle = math.degrees(self.robot.theta)
		curr_angle = self.robot.theta_deg
		diff_angle = goal_angle - curr_angle
		print("diff",diff_angle)
		if diff_angle > 0 and diff_angle <= 180:
			while self.robot.theta_deg < goal_angle:
				self.turn_one(speed=-speed)
		elif diff_angle < 0 and diff_angle >= -180:
			while self.robot.theta_deg > goal_angle:
				self.turn_one(speed=speed)
		elif diff_angle > 180:
			while self.robot.theta_deg < 0 or self.robot.theta_deg > goal_angle:
				self.turn_one(speed=speed)
		elif diff_angle < -180:
			while self.robot.theta_deg > 0 or self.robot.theta_deg < goal_angle:
				self.turn_one(speed=-speed)
		self.stop_twist()

		#if corr:
		#	self.turn(speed=corr_speed, goal_angle=goal_angle, corr=False)
		print(self.robot.theta_deg)
		

	def turn_one(self, speed):
			twist = self.create_twist(ang_z=speed)
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
						
	
	def turn_angle(self, deg, speed):
		rclpy.spin_once(self.robot)
		curr_angle = self.robot.theta_deg
		goal_angle = ((curr_angle + deg + 180) % 360) -180
		self.turn(speed, goal_angle)
		return goal_angle

	def dumped_turn(self, speed, goal_angle):
		pass
	
	def straight(self, speed, dist):
		rclpy.spin_once(self.robot)
		orig_x = self.robot.odom_pose.position.x
		orig_y = self.robot.odom_pose.position.y
		while self.distance(orig_x,orig_y) < abs(dist):
			twist = self.create_twist(lin_x=abs(speed))
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
			print("dist:", self.distance(orig_x,orig_y))
		self.stop_twist()
		#print(self.pos_x, self.pos_y)

	def distance(self, x, y):
		rclpy.spin_once(self.robot)
		dist = math.sqrt((self.robot.odom_pose.position.x-x)**2+(self.robot.odom_pose.position.y-y)**2)
		return dist

def main():
	rclpy.init(args=None)
	rosbot=Motion()
	rosbot.straight(speed=0.5, dist=1.0)
	print(rosbot.robot.odom_pose, rosbot.robot.theta_deg)
	
	
	rosbot.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
