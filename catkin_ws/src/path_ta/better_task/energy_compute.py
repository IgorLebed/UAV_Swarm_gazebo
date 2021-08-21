import numpy as np
import time

# !!!
#TODO: write method, that simulates fuel reduction simulation
# !!!
'''
def start(self):
	start_time = time.time()
	if time.time() - start_time > self.time_bound:
		self.fuel_resource -= (self.time_bound*3600)*self.fuel_consume
		resource_data = self.landing_duel_resource()
		#public resource_data

'''

class ORLAN():
	"""
	ORLAN - class for calculating mission time and fuel resource of Orlan-10 UAV.

	Initialization attributes:
		- mission params (dictionary):
			~ "trajectory":				trajectory in the scouting region (list of tuples: [(x, y, z)])
			~ "uav_position":			active UAV popsition (tuple: (x, y, z))
			~ "takeoff_point":			UAV's take-off point (tuple: (x, y, z))
			~ "grouping_point":			UAV's grouping point (tuple: (x, y, z)) 
											### Caution! Simulation starts from grouping point! ###
			~ "drop_point":				point for cargo drop (tuple: (x, y, z))
			~ "landing_point":			point for UAV landing (tuple: (x, y, z))
		- uav_params (dictionary):
			~ "fuel_resource":			available fuel resource (in hours)
			~ "fuel_consume":			fuel consuption (per hour)
			~ "cargo":					presence of cargo on the UAV (True/False)
		- uav_type (string: "scout" or "bomber")
	"""
	def __init__(self, mission_params, uav_params, uav_type):
		self.trajectory = mission_params["trajectory"]
		self.uav_position = mission_params["uav_position"]
		self.takeoff_point = mission_params["takeoff_point"]
		self.grouping_point = mission_params["grouping_point"]
		self.drop_point = mission_params["drop_point"]
		self.landing_point = mission_params["landing_point"]

		self.fuel_resource = uav_params["fuel_resource"]
		self.fuel_consume = uav_params["fuel_consume"]
		self.cargo = uav_params["cargo"]

		self.uav_type = uav_type

		self.max_fuel_resource = 16.0 # hours
		self.max_velocity = 150.0 # km/h
		self.min_velocity = 90.0 # km/h
	
		self.time_bound = 10.0

	def calc_takeoff_time(self):
		gain_alt_speed = 28.8 # km/h (ORLAN's speed of gaining altitude)
		scout_height = 1.2 # km
		bomber_height = 0.4 # km
		if self.uav_type == "scout":
			takeoff_time = scout_height/gain_alt_speed # hours
		elif self.uav_type == "bomber":
			takeoff_time = bomber_height/gain_alt_speed # hours
		return takeoff_time

	def calc_grouping_time(self):
		distance = calc_distance(self.takeoff_point, self.grouping_point)/1000 # kilometers
		grouping_time = distance/self.max_velocity # hours
		return grouping_time

	def calc_movetotask_time(self):
		distance = calc_distance(self.grouping_point, self.trajectory[0])/1000 #kilometers
		movetotask_time = distance/self.max_velocity # hours
		return movetotask_time

	def calc_scouting_time(self):
		distance = path_length_meters(self.trajectory)/1000 # kilometers
		scouting_time = distance/self.max_velocity # hours
		return scouting_time

	def calc_cargodrop_time(self):
		distance = calc_distance(self.trajectory[-1], self.drop_point)/1000 # kilometers
		cargodrop_time = distance/self.max_velocity # hours
		return cargodrop_time

	def calc_landing_time(self):
		distance = calc_distance(self.drop_point, self.landing_point)/1000 # kilometers
		landing_time = (distance)/self.max_velocity # hours
		return landing_time

	def calc_mission_time(self):
		takeoff_time = self.calc_takeoff_time()
		grouping_time = self.calc_grouping_time()
		movetotask_time = self.calc_movetotask_time()
		scouting_time = self.calc_scouting_time()
		cargodrop_time = self.calc_cargodrop_time()
		landing_time = self.calc_landing_time()
		mission_time = takeoff_time + grouping_time + movetotask_time + scouting_time + cargodrop_time + landing_time
		return mission_time

	def calc_path_distance(self):
		'''
		Calculating path distance from uav_position to landing point in meters. 
		Takes into account the presence of cargo on the UAV and location of the drop point.

		Output:
			- distance (in meters)
		'''
		if self.cargo == True:
			dist_1 = calc_distance(self.uav_position, self.drop_point)
			dist_2 = calc_distance(self.drop_point, self.landing_point)
			distance = dist_1 + dist_2
		else:
			distance = calc_distance(self.uav_position, self.landing_point)
		return distance

	def landing_fuel_resource(self):
		'''
		Calculating amount of fuel resource that will be
		available to the drone at the moment it returns to base now.

		Output:
			- fuel_data (dictionary):
				"hours": fuel_left_hours
				"percent": fuel_left_percent (0 <= fuel_left_percent <= 1)
		'''
		distance = self.calc_path_distance()/1000 # kilometers
		movement_in_hours = (distance/self.max_velocity)

		fuel_left_hours = self.fuel_resource - movement_in_hours*self.fuel_consume
		fuel_left_percent = (fuel_left_hours/self.max_fuel_resource)*100

		fuel_data = {"hours": fuel_left_hours, "percent": fuel_left_percent}
		return fuel_data

	def start(self):
		start_time = time.time()
		while True:
			if time.time() - start_time > self.time_bound:
				self.fuel_resource -= (self.time_bound/3600.0)*self.fuel_consume
				resource_data = self.landing_fuel_resource()
				print(resource_data)
				start_time = time.time()
				#public resource_data

class RALX6():
	"""
	RALX6T - class for calculating mission time and fuel resource of RAL-X6 UAV.

	Initialization attributes:
		- mission params (dictionary):
			~ "trajectory":				trajectory in the scouting region (list of tuples: [(x, y, z)])
			~ "uav_position":			active UAV popsition (tuple: (x, y, z))
			~ "takeoff_point":			UAV's take-off point (tuple: (x, y, z))
			~ "grouping_point":			UAV's grouping point (tuple: (x, y, z)) 
											### Caution! Simulation starts from grouping point! ###
			~ "drop_point":				point for cargo drop (tuple: (x, y, z))
			~ "landing_point":			point for UAV landing (tuple: (x, y, z))
		- uav_params (dictionary):
			~ "fuel_resource":			available fuel resource (in hours)
			~ "fuel_consume":			fuel consuption (per hour)
			~ "cargo":					presence of cargo on the UAV (True/False)
		- uav_type (string: "scout" or "bomber")
	"""
	def __init__(self, mission_params, uav_params, uav_type):
		self.trajectory = mission_params["trajectory"]
		self.uav_position = mission_params["uav_position"]
		self.takeoff_point = mission_params["takeoff_point"]
		self.grouping_point = mission_params["grouping_point"]
		self.drop_point = mission_params["drop_point"]
		self.landing_point = mission_params["landing_point"]

		self.fuel_resource = uav_params["fuel_resource"]
		self.fuel_consume = uav_params["fuel_consume"]
		self.cargo = uav_params["cargo"]

		self.uav_type = uav_type

		self.max_fuel_resource = 0.8333 # hours
		self.max_velocity = 60.0 # km/h

		self.time_bound = 10.0

	def calc_takeoff_landing_time(self):
		gain_alt_speed = 40.0 # km/h (RALX6's speed of gaining altitude)
		scout_height = 1.2 # km
		bomber_height = 0.4 # km
		if self.uav_type == "scout":
			takeoff_time = scout_height/gain_alt_speed # hours
		elif self.uav_type == "bomber":
			takeoff_time = bomber_height/gain_alt_speed # hours
		return takeoff_time

	def calc_grouping_time(self):
		distance = calc_distance(self.takeoff_point, self.grouping_point)/1000 # kilometers
		grouping_time = distance/self.max_velocity # hours
		return grouping_time

	def calc_movetotask_time(self):
		distance = calc_distance(self.grouping_point, self.trajectory[0])/1000 #kilometers
		movetotask_time = distance/self.max_velocity # hours
		return movetotask_time

	def calc_scouting_time(self):
		distance = path_length_meters(self.trajectory)/1000 # kilometers
		scouting_time = distance/self.max_velocity # hours
		return scouting_time

	def calc_cargodrop_time(self):
		distance = calc_distance(self.trajectory[-1], self.drop_point)/1000 # kilometers
		cargodrop_time = distance/self.max_velocity # hours
		return cargodrop_time

	def calc_landing_time(self):
		distance = calc_distance(self.drop_point, self.landing_point)/1000 # kilometers
		landing_time = (distance + self.calc_takeoff_landing_time())/self.max_velocity # hours
		return landing_time

	def calc_mission_time(self):
		takeoff_time = self.calc_takeoff_landing_time()
		grouping_time = self.calc_grouping_time()
		movetotask_time = self.calc_movetotask_time()
		scouting_time = self.calc_scouting_time()
		cargodrop_time = self.calc_cargodrop_time()
		landing_time = self.calc_landing_time()
		mission_time = takeoff_time + grouping_time + movetotask_time + scouting_time + cargodrop_time + landing_time
		return mission_time

	def calc_path_distance(self):
		'''
		Calculating path distance from uav_position to landing_point in meters.  
		Takes into account the presence of cargo on the UAV and location of the drop point.

		Output:
			- distance (in meters)
		'''
		if self.cargo == True:
			dist_1 = calc_distance(self.uav_position, self.drop_point)
			dist_2 = calc_distance(self.drop_point, self.landing_point)
			distance = dist_1 + dist_2
		else:
			distance = calc_distance(self.uav_position, self.landing_point)
		return distance

	def landing_fuel_resource(self):
		'''
		Calculating amount of fuel resource that will be
		available to the drone at the moment it returns to base now.

		Output:
			- fuel_data (dictionary):
				"hours": fuel_left_hours
				"percent": fuel_left_percent (0 <= fuel_left_percent <= 1)
		'''
		distance = self.calc_path_distance()/1000 # kilometers
		movement_in_hours = (distance/self.max_velocity) + self.calc_takeoff_landing_time()

		fuel_left_hours = self.fuel_resource - movement_in_hours*self.fuel_consume
		fuel_left_percent = (fuel_left_hours/self.max_fuel_resource)*100

		fuel_data = {"hours": fuel_left_hours, "percent": fuel_left_percent}
		return fuel_data

	def start(self):
		start_time = time.time()
		while True:
			if time.time() - start_time > self.time_bound:
				self.fuel_resource -= (self.time_bound/3600.0)*self.fuel_consume
				resource_data = self.landing_fuel_resource()
				print(resource_data)
				start_time = time.time()
				#public resource_data


def calc_distance(point_1, point_2):
	x_1, y_1, _ = point_1
	x_2, y_2, _ = point_2
	distance = np.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
	return distance

def path_length_meters(path):
	length = 0
	for i in range(1, len(path)):
		p1 = path[i-1]
		p2 = path[i]
		x1, y1, _ = p1
		x2, y2, _ = p2
		l = np.sqrt((x1-x2)**2 + (y1-y2)**2)
		length += l
	return length


#Example:
mission_params = {"trajectory": [(-10, -10, 0), (-5, 4,0), (10, 10, 0)],"uav_position": (0, 0, 0),"takeoff_point": (0, -10, 0),"grouping_point": (-5, -10, 0),"drop_point": (10, 5, 0), "landing_point": (10, 0, 0)}
uav_type = 'bomber'
uav_params = {"fuel_resource": 16,"fuel_consume": 1,"cargo": True}

#orlan_scout = ORLAN(mission_params, uav_params, uav_type)
#print(orlan_scout.calc_mission_time())
#print(orlan_scout.landing_fuel_resource())
#orlan_scout.start()