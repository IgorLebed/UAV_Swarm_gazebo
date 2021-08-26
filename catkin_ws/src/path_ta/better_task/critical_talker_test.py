import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Float64, Int64MultiArray
from mavros_msgs.msg import State
from threading import Thread


'''
	Where to push:

".../mavros/local_position/pose"	PoseStamped
".../mavros/state"					State
"scout0/takeoff_point"				Point
"scout0/grouping_point"				Point	
"scout0/drop_point"					Point
"scout0/landing_point"				Point
".../cargo"							Bool
".../fuel_resource"					Float64
".../fuel_consume"					Float64

	We have:
scout0
bomber1
bomber2
bomber3
'''
takeoff_point_publisher = rospy.Publisher("/scout0/takeoff_point", Point, queue_size=10)
grouping_point_publisher = rospy.Publisher("/scout0/grouping_point", Point, queue_size=10)
drop_point_publisher = rospy.Publisher("/scout0/drop_point", Point, queue_size=10)
landing_point_publisher = rospy.Publisher("/scout0/landing_point", Point, queue_size=10)

active_pose_scout0_publisher = rospy.Publisher("/scout0/mavros/local_position/pose", PoseStamped, queue_size=10)
armed_scout0_publisher = rospy.Publisher("/scout0/mavros/state", State, queue_size=10)
cargo_scout0_publisher = rospy.Publisher("/scout0/cargo", Bool, queue_size=10)
fuel_resource_scout0_publisher = rospy.Publisher("/scout0/fuel_resource", Float64, queue_size=10)
fuel_consume_scout0_publisher = rospy.Publisher("/scout0/fuel_consume", Float64, queue_size=10)

active_pose_bomber1_publisher = rospy.Publisher("/bomber1/mavros/local_position/pose", PoseStamped, queue_size=10)
armed_bomber1_publisher = rospy.Publisher("/bomber1/mavros/state", State, queue_size=10)
cargo_bomber1_publisher = rospy.Publisher("/bomber1/cargo", Bool, queue_size=10)
fuel_resource_bomber1_publisher = rospy.Publisher("/bomber1/fuel_resource", Float64, queue_size=10)
fuel_consume_bomber1_publisher = rospy.Publisher("/bomber1/fuel_consume", Float64, queue_size=10)

active_pose_bomber2_publisher = rospy.Publisher("/bomber2/mavros/local_position/pose", PoseStamped, queue_size=10)
armed_bomber2_publisher = rospy.Publisher("/bomber2/mavros/state", State, queue_size=10)
cargo_bomber2_publisher = rospy.Publisher("/bomber2/cargo", Bool, queue_size=10)
fuel_resource_bomber2_publisher = rospy.Publisher("/bomber2/fuel_resource", Float64, queue_size=10)
fuel_consume_bomber2_publisher = rospy.Publisher("/bomber2/fuel_consume", Float64, queue_size=10)

active_pose_bomber3_publisher = rospy.Publisher("/bomber3/mavros/local_position/pose", PoseStamped, queue_size=10)
armed_bomber3_publisher = rospy.Publisher("/bomber3/mavros/state", State, queue_size=10)
cargo_bomber3_publisher = rospy.Publisher("/bomber3/cargo", Bool, queue_size=10)
fuel_resource_bomber3_publisher = rospy.Publisher("/bomber3/fuel_resource", Float64, queue_size=10)
fuel_consume_bomber3_publisher = rospy.Publisher("/bomber3/fuel_consume", Float64, queue_size=10)


takeoff_point = Point()
grouping_point = Point()
drop_point = Point()
landing_point = Point()

takeoff_point.x, takeoff_point.y = 0, 0
grouping_point.x, grouping_point.y = 10, 10
drop_point.x, drop_point.y = 100, 100
landing_point.x, landing_point.y = 200, 50






active_pose_scout0 = PoseStamped()
armed_scout0 = State()
cargo_scout0 = Bool()
fuel_resource_scout0 = Float64()
fuel_consume_scout0 = Float64()

active_pose_bomber1 = PoseStamped()
armed_bomber1 = State()
cargo_bomber1 = Bool()
fuel_resource_bomber1 = Float64()
fuel_consume_bomber1 = Float64()

active_pose_bomber2 = PoseStamped()
armed_bomber2 = State()
cargo_bomber2 = Bool()
fuel_resource_bomber2 = Float64()
fuel_consume_bomber2 = Float64()

active_pose_bomber3 = PoseStamped()
armed_bomber3 = State()
cargo_bomber3 = Bool()
fuel_resource_bomber3 = Float64()
fuel_consume_bomber3 = Float64()







active_pose_scout0.pose.position.x, active_pose_scout0.pose.position.y = 30, 30
armed_scout0.armed = True
cargo_scout0.data = True
fuel_resource_scout0.data = 0.075
fuel_consume_scout0.data = 0.8

active_pose_bomber1.pose.position.x, active_pose_bomber1.pose.position.y = 25, 25
armed_bomber1.armed = True
cargo_bomber1.data = True
fuel_resource_bomber1.data = 0.6
fuel_consume_bomber1.data = 0.8

active_pose_bomber2.pose.position.x, active_pose_bomber2.pose.position.y = 30, 25
armed_bomber2.armed = True
cargo_bomber2.data = True
fuel_resource_bomber2.data = 0.7
fuel_consume_bomber2.data = 0.8

active_pose_bomber3.pose.position.x, active_pose_bomber3.pose.position.y = 100, 25
armed_bomber3.armed = True
cargo_bomber3.data = True
fuel_resource_bomber3.data = 0.5
fuel_consume_bomber3.data = 0.8

rospy.init_node("test_crit")

while True:
	takeoff_point_publisher.publish(takeoff_point)
	grouping_point_publisher.publish(grouping_point)
	drop_point_publisher.publish(drop_point)
	landing_point_publisher.publish(landing_point)

	active_pose_scout0_publisher.publish(active_pose_scout0)
	armed_scout0_publisher.publish(armed_scout0)
	cargo_scout0_publisher.publish(cargo_scout0)
	fuel_resource_scout0_publisher.publish(fuel_resource_scout0)
	fuel_consume_scout0_publisher.publish(fuel_consume_scout0)

	active_pose_bomber1_publisher.publish(active_pose_bomber1)
	armed_bomber1_publisher.publish(armed_bomber1)
	cargo_bomber1_publisher.publish(cargo_bomber1)
	fuel_resource_bomber1_publisher.publish(fuel_resource_bomber1)
	fuel_consume_bomber1_publisher.publish(fuel_consume_bomber1)

	active_pose_bomber2_publisher.publish(active_pose_bomber2)
	armed_bomber2_publisher.publish(armed_bomber2)
	cargo_bomber2_publisher.publish(cargo_bomber2)
	fuel_resource_bomber2_publisher.publish(fuel_resource_bomber2)
	fuel_consume_bomber2_publisher.publish(fuel_consume_bomber2)

	active_pose_bomber3_publisher.publish(active_pose_bomber3)
	armed_bomber3_publisher.publish(armed_bomber3)
	cargo_bomber3_publisher.publish(cargo_bomber3)
	fuel_resource_bomber3_publisher.publish(fuel_resource_bomber3)
	fuel_consume_bomber3_publisher.publish(fuel_consume_bomber3)

	if rospy.is_shutdown():
		break




