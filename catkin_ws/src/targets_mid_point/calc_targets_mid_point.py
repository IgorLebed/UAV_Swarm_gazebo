def calc_targets_mid_point(targets_positions):
	"""
	Calculating average position of the group of targets.
	
	Input:
		- targets_positions: list of tuples; [(x, y, z)]
	Output:
		- average_point: tuple; (x_mid, y_mid)
	"""

	n = len(targets_positions)
	x_mid, y_mid = 0, 0
	for point in targets_positions:
		x, y, z = point
		x_mid += x
		y_mid += y
	x_mid /= n
	y_mid /= n
	average_point = (x_mid, y_mid)
	return average_point
