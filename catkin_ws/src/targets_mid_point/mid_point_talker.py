#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
import utils

def gazebo_co_callback(data):
	targets_positions = []
	names = data.name
	positions = data.pose
	twists = data.twist
	new_names = {k: v for v, k in enumerate(names)}
	heightmap_index = new_names["heightmap"]
	for i in range(len(names)): 
		if i != heightmap_index and names[i][:4] == "p3at" and len(names[i]) <= 10:
			pos = (positions[i].position.x, positions[i].position.y)
			targets_positions.append(pos)
	targets_mid_point = utils.calc_targets_mid_point(targets_positions)
	middle_point_publisher.publish(targets_mid_point)

if __name__ == "__main__":
	rospy.init_node("targets_mid_point_talker")
	middle_point_publisher = rospy.Publisher("targets_mid_point", Point, queue_size=10)
	rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_co_callback)
	rospy.spin()

