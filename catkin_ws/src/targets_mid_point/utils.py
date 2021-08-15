import rospy
from geometry_msgs.msg import Point

def calc_targets_mid_point(targets_positions):
	"""
	Calculating average position of the group of targets
	
	Input:
		- targets_positions: list of tuples; [(point.x, point.y, point.z)]
	Output:
		- average_point: Point; [x_mid, y_mid, 0]
	"""

	n = len(targets_positions)
	x_mid, y_mid = 0, 0
	for point in targets_positions:
		x, y = point
		x_mid += x
		y_mid += y
	x_mid /= n
	y_mid /= n
	average_point = Point()
	average_point.x = x_mid
	average_point.y = y_mid
	average_point.z = 0
	return average_point

