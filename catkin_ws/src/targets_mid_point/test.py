from calc_targets_mid_point import calc_targets_mid_point

def test():
	target_1 = (1, 5, 0)
	target_2 = (2.1, 4.45, 1)
	target_3 = (1.99, 0, 0.34)
	targets_positions = [target_1, target_2, target_3]
	average_point = calc_targets_mid_point(targets_positions)
	x1, y1, z1 = target_1
	x2, y2, z2 = target_2
	x3, y3, z3 = target_3
	n = len(targets_positions)
	assert average_point == ((x1+x2+x3)/n, (y1+y2+y3)/n)

if __name__ == "__main__":
	test()
