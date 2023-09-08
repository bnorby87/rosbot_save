import asyncio
from operator import ipow
from os import PRIO_PGRP
from time import sleep, time
from mavsdk import System
from mavsdk import telemetry
import transformations
import utm
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Int32

class PX4_Publisher(Node):
	yaw_deg = 0.0
	pose = Pose()
	gnss_sat = 0
	gnss_type = ""

	def __init__(self):
		super().__init__('PX4')
		self.publisher_imu = self.create_publisher(Float32, '/PX4_yaw_deg', 1)
		self.publisher_gnss = self.create_publisher(Pose, '/PX4_pose', 1)
		self.publisher_gnss_sat = self.create_publisher(Int32, '/PX4_GNSS_SAT', 1)
		self.publisher_gnss_type = self.create_publisher(String, '/PX4_GNSS_RTK_TYPE', 1)
		timer_period = 0.1
		self.timer = self.create_timer(timer_period, self.yawdeg_callback)
		self.timer = self.create_timer(timer_period, self.gnss_callback)
		self.timer = self.create_timer(timer_period, self.gnss_sat_callback)
		self.timer = self.create_timer(timer_period, self.gnss_type_callback)
		
	def yawdeg_callback(self):
		msg = Float32()
		msg.data = self.yaw_deg
		self.publisher_imu.publish(msg)
		#self.get_logger().info('Publishing: "%s"' % msg.data)
	
	def gnss_callback(self):
		msg = Pose()
		msg = self.pose
		self.publisher_gnss.publish(msg)
		#self.get_logger().info('Publishing: "%s"' % msg)
	
	def gnss_sat_callback(self):
		msg = Int32()
		msg.data = self.gnss_sat
		self.publisher_gnss_sat.publish(msg)
		#self.get_logger().info('Publishing: "%s"' % msg)
	
	def gnss_type_callback(self):
		msg = String()
		msg.data = self.gnss_type
		self.publisher_gnss_type.publish(msg)
		#self.get_logger().info('Publishing: "%s"' % msg)

	async def PX4_sensor(self):
		robot = System()
		#await robot.connect(system_address="serial:///dev/ttyPX4:57600")
		await robot.connect(system_address="udp://:14550")
		print("conn_ok")
		PX4_tasks=[]
		PX4_tasks.append(asyncio.create_task(self.IMU(robot)))
		PX4_tasks.append(asyncio.create_task(self.GNSS(robot)))
		PX4_tasks.append(asyncio.create_task(self.GNSS_info(robot)))		
		for task in PX4_tasks:
			await task
	
	async def IMU(self, robot):
		#async for info in robot.telemetry.attitude_quaternion():
		#	print(math.degrees(transformations.euler_from_quaternion([info.x, info.y, info.z, info.w])[0]))
		async for info in robot.telemetry.attitude_euler():
			self.yaw_deg = info.yaw_deg
			#print(info.yaw_deg)
			rclpy.spin_once(self)
	
	async def GNSS(self, robot):
		async for info in robot.telemetry.raw_gps():
			self.pose.position.x, self.pose.position.y =  utm.from_latlon(info.latitude_deg,info.longitude_deg)[0:2]
			#print(info.latitude_deg,info.longitude_deg)
			rclpy.spin_once(self)
	
	async def GNSS_info(self, robot):
		async for info in robot.telemetry.gps_info():
			self.gnss_sat = info.num_satellites
			if info.fix_type==telemetry.FixType.NO_GPS:
				self.gnss_type = "NO_GPS"
			elif info.fix_type==telemetry.FixType.NO_FIX:
				self.gnss_type = "NO_FIX"
			elif info.fix_type==telemetry.FixType.FIX_2D:
				self.gnss_type = "FIX_2D"
			elif info.fix_type==telemetry.FixType.FIX_3D:
				self.gnss_type = "FIX_3D"
			elif info.fix_type==telemetry.FixType.FIX_DGPS:
				self.gnss_type = "FIX_DGPS"
			elif info.fix_type==telemetry.FixType.RTK_FLOAT:
				self.gnss_type = "RTK_FLOAT"
			elif info.fix_type==telemetry.FixType.RTK_FIXED:
				self.gnss_type = "RTK_FIXED"
			else:
				self.gnss_type = "NO_GPS"
			#print(info.num_satellites, info.fix_type)
			rclpy.spin_once(self)

def main(args=None):
	rclpy.init(args=args)
	PX4 = PX4_Publisher()
	asyncio.run(PX4.PX4_sensor())
	#PX4.destroy_node()
	#rclpy.shutdown()


if __name__ == '__main__':
    main()
