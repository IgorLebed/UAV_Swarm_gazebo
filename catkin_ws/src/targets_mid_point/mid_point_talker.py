#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Int64
from uavis.msg import TargetCoordinates
import utils


class MidPointTalker():

	def __init__(self):
		self.targets_mid_point = None
		self.scout0_mode = None
		self.bomber1_mode = None
		self.bomber2_mode = None
		self.bomber3_mode = None

	def scout0_mode_listener(self, data):
		self.scout0_mode = data
		self.decision_maker()

	def bomber1_mode_listener(self, data):
		self.bomber1_mode = data

	def bomber2_mode_listener(self, data):
		self.bomber2_mode = data

	def bomber3_mode_listener(self, data):
		self.bomber3_mode = data

	def targets_point_listener(self, data):
		targets_positions = []
		coordinates = data.coordinates
		for i in range(len(coordinates)):
			pos = (coordinates[i].x, coordinates[i].y)
			targets_positions.append(pos)
		self.targets_mid_point = utils.calc_targets_mid_point(targets_positions)

	def decision_maker(self):
		if self.scout0_mode == 1 and self.bomber1_mode == 1:
			scout0_set_mode_publisher.publish(1)
			bomber1_set_mode_publisher.publish(1)
			middle_point_publisher.publish(self.targets_mid_point)


if __name__ == "__main__":
	rospy.init_node("targets_mid_point_talker")

	middle_point_publisher = rospy.Publisher("targets_mid_point", Point, queue_size=10)

	scout0_set_mode_publisher = rospy.Publisher("scout0/set_mode", Int64, queue_size=10)
	bomber1_set_mode_publisher = rospy.Publisher("bomber1/set_mode", Int64, queue_size=10)

	MPT = MidPointTalker()
	rospy.Subscriber("scout0/change_mode_is_available", Int64, MPT.scout0_mode_listener)
	rospy.Subscriber("bomber1/change_mode_is_available", Int64, MPT.bomber1_mode_listener)

	rospy.Subscriber("/scout0/target_coordinates", TargetCoordinates, MPT.targets_point_listener)

	rospy.spin()

