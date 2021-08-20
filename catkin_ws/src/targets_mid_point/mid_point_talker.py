#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from uavis.msg import TargetCoordinates
import utils

def gazebo_co_callback(data):
	targets_positions = []
	coordinates = data.coordinates
	for i in range(len(coordinates)):
		pos = (coordinates[i].x, coordinates[i].y)
		targets_positions.append(pos)
	targets_mid_point = utils.calc_targets_mid_point(targets_positions)
	middle_point_publisher.publish(targets_mid_point)


if __name__ == "__main__":
	rospy.init_node("targets_mid_point_talker")
	middle_point_publisher = rospy.Publisher("targets_mid_point", Point, queue_size=10)
	rospy.Subscriber("/scout0/target_coordinates", TargetCoordinates, gazebo_co_callback)
	rospy.spin()

