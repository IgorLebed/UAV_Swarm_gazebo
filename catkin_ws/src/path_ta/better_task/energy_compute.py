import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Float64, Int64MultiArray, Float64MultiArray
from mavros_msgs.msg import State
from threading import Thread


class UAV(Thread):
	"""
	UAV - class for calculating mission time and fuel resource.

	Initialization attributes:
		- uav_type:				(string: "ralx6" or "orlan")
		- uav_name: 			(string: "scoutX" or "bomberX", X - uav_num)
		- uav_num:				(int: number of UAV)
		- uav_position:			active UAV popsition
		- takeoff_point:		UAV's take-off point
		- grouping_point:		UAV's grouping point
		- drop_point:			point for cargo drop
		- landing_point:		point for UAV landing
		- armed:				UAV's state (True - alive, False - lost)
		- fuel_resource:		available fuel resource (in hours)
		- fuel_consume:			fuel consuption (per hour)
		- cargo:				presence of cargo on the UAV (True/False)
		
	"""
	def __init__(self, uav_type, uav_name, uav_num):
		Thread.__init__(self)
		self.uav_type = uav_type
		self.uav_name = uav_name
		self.uav_num = uav_num

		self.uav_position = Point()
		self.takeoff_point = Point()
		self.grouping_point = Point()
		self.drop_point = Point()
		self.landing_point = Point()

		self.armed = bool
		self.fuel_resource = None
		self.fuel_consume = None
		self.cargo = None

		self.max_fuel_resource = 0.8333 if uav_type=="ralx6" else 16 # hours
		self.max_velocity = 60.0 if uav_type=="ralx6" else 150 # km/h

		self.time_bound = 10.0

		self.fuel_resource_subscribe = Float64()
		self.fuel_consume_subscribe = Float64()

		self.topic_uav_position = self.uav_name + str(self.uav_num) + "/mavros/local_position/pose"
		self.topic_armed = self.uav_name + str(self.uav_num) + "/mavros/state"
		#self.topic_takeoff_point = uav_name + str(uav_num) + "/takeoff_point"
		self.topic_takeoff_point = "scout0/takeoff_point"
		#self.topic_grouping_point = uav_name + str(uav_num) + "/grouping_point"
		self.topic_grouping_point = "scout0/grouping_point"
		#self.topic_drop_point = uav_name + str(uav_num) + "/drop_point"
		self.topic_drop_point = "scout0/drop_point"
		#self.topic_landing_point = uav_name + str(uav_num) + "/landing_point"
		self.topic_landing_point = "scout0/landing_point"
		self.topic_cargo = self.uav_name + str(self.uav_num) + "/cargo"
		self.topic_fuel_resource = self.uav_name + str(self.uav_num) + "/fuel_resource"
		self.topic_fuel_consume = self.uav_name + str(self.uav_num) + "/fuel_consume"
		rospy.Subscriber(self.topic_uav_position, PoseStamped, self.uav_position_callback)
		rospy.Subscriber(self.topic_armed, State, self.armed_callback)
		rospy.Subscriber(self.topic_takeoff_point, Point, self.takeoff_point_callback)
		rospy.Subscriber(self.topic_grouping_point, Point, self.grouping_point_callback)
		rospy.Subscriber(self.topic_drop_point, Point, self.drop_point_callback)
		rospy.Subscriber(self.topic_landing_point, Point, self.landing_point_callback)
		rospy.Subscriber(self.topic_cargo, Bool, self.cargo_callback)
		rospy.loginfo("Wait for fuel_resource of " + self.uav_name)
		self.fuel_resource_subscribe = rospy.wait_for_message(self.topic_fuel_resource, Float64)
		rospy.loginfo("Done waiting for fuel_resource of " + self.uav_name)
		#rospy.Subscriber(self.topic_fuel_resource, Float64, self.fuel_resource_callback)
		rospy.loginfo("Wait for fuel_consume of " + self.uav_name)
		self.fuel_consume_subscribe = rospy.wait_for_message(self.topic_fuel_consume, Float64)
		rospy.loginfo("Done waiting for fuel_consume of " + self.uav_name)
		#rospy.Subscriber(self.topic_fuel_consume, Float64, self.fuel_consume_callback)
		

	def uav_position_callback(self, data):
		self.uav_position.x = data.pose.position.x
		self.uav_position.y = data.pose.position.y
		self.uav_position.z = data.pose.position.z

	def armed_callback(self, data):
		self.armed = data.armed

	def takeoff_point_callback(self, data):
		self.takeoff_point = data

	def grouping_point_callback(self, data):
		self.grouping_point = data

	def drop_point_callback(self, data):
		self.drop_point = data

	def landing_point_callback(self, data):
		self.landing_point = data

	def cargo_callback(self, data):
		self.cargo = data.data

	def fuel_resource_callback(self, data):
		self.fuel_resource_subscribe.data = data.data # in hours

	def fuel_consume_callback(self, data):
		self.fuel_consume_subscribe.data = data.data # in hours

	def calc_takeoff_landing_time(self):
		gain_alt_speed = 40.0 if self.uav_type == "ralx6" else 28.8 # km/h
		scout_height = 1.2 # km
		bomber_height = 0.4 # km
		if self.uav_name == "scout":
			takeoff_time = scout_height/gain_alt_speed # hours
		elif self.uav_name == "bomber":
			takeoff_time = bomber_height/gain_alt_speed # hours
		return takeoff_time

	def calc_path_distance(self):
		'''
		Calculating path distance from uav_position to landing_point in meters.  
		Takes into account the presence of cargo on the UAV and location of the drop point.

		Output:
			- distance (in meters)
		'''
		if self.cargo:
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
				"percent": fuel_left_percent
		'''
		distance = self.calc_path_distance()/1000 # kilometers
		movement_in_hours = (distance/self.max_velocity) + self.calc_takeoff_landing_time() if self.uav_type=="ralx6" else (distance/self.max_velocity)

		fuel_left_hours = self.fuel_resource - movement_in_hours*self.max_fuel_resource
		fuel_left_percent = (fuel_left_hours/self.max_fuel_resource)*100
		#print(self.fuel_resource)

		fuel_data = {"hours": fuel_left_hours, "percent": fuel_left_percent}
		return fuel_data


def calc_distance(point_1, point_2):
	x_1, y_1, _ = point_1.x, point_1.y, point_1.z
	x_2, y_2, _ = point_2.x, point_2.y, point_2.z
	distance = np.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
	return distance

def check_armed_uavs(uavs):
	armed_uavs = []
	for uav in uavs:
		if uav.armed == True:
			armed_uavs.append(uav)
	return armed_uavs
	

def fuel_check(n, scout_fuel, bombers_fuel, p):
	"""
	:param scout_fuel: scout fuel level
	:param bombers_fuel: fuel levels of bombers
	:param n: maximum amount of UAVs with critical fuel level
	:param p: critical fuel level
	:return: who is returns
	"""
	if scout_fuel <= p:
		print('return all UAVs')
		return [0] # Return all UAVs
	else:
		tmp = list()
		check = 0
		for i, b in bombers_fuel.items():
			if b <= p:
				check += 1
				tmp.append(i)
				if check >= n:
					print('return all UAVs')
					return [0] # Return all UAVs
		if check != 0:
			#Make message in list
			output = [i for i in tmp]
			output.insert(0, 1)
			return output # Return not all (output = [0, ...])
		else:
			print('no one returns')
			return [-1] # No one returns



def start(uavs):
	start_time = time.time()
	while True:

		armed_uavs = check_armed_uavs(uavs)

		if time.time() - start_time > TIME_BOUND:
			print("2 sec.")
			start_time = time.time()
			if len(armed_uavs) > 0:
				scout_fuel = 0.0 # Why 0.0? # TODO maybe here is error
				bombers_fuel = {}
				for uav in armed_uavs:
					uav.fuel_resource -= (TIME_BOUND/3600.0)*uav.fuel_consume
					resource_data = uav.landing_fuel_resource()
					if uav.uav_name == "bomber":
						bombers_fuel[uav.uav_num] = resource_data["percent"]
					elif uav.uav_name == "scout":
						scout_fuel = resource_data["percent"]
					print(resource_data["percent"], uav.uav_name)
					
				# Crit signal
				crit_signal = fuel_check(N, scout_fuel, bombers_fuel, P)
				battery_scout_info = [scout_fuel]
				battery_bomber = [bombers_fuel]
				# Make message
				message_crit_signal = Int64MultiArray(data=crit_signal) # TODO maybe here is error
				message_battery_scout = Float64MultiArray(data=battery_scout_info)
				message_battery_bomber = Float64MultiArray(data=battery_bomber)
				print(message_crit_signal.data)
				# Publish messageW
				crit_sit_info.publish(message_crit_signal)
				battery_stat_info.publish(message_battery_scout)
				battery_bomber_info.publish(message_battery_bomber)
		if rospy.is_shutdown():
			rospy.loginfo('Shutdown.')
			break


if __name__ == "__main__":
	TIME_BOUND = 2.0 # sec
	N = 2 # Bound num of uav's
	P = 5 # Percents

	rospy.init_node("energy_evaluation")
	crit_sit_info = rospy.Publisher("/critical_status_info", Int64MultiArray, queue_size=10)
	battery_stat_info = rospy.Publisher("/battery_status_info", Float64MultiArray, queue_size=10)
	battery_bomber_info = rospy.Publisher("bomber/battery_status_info", Float64MultiArray, queue_size=10)
	# Init UAV's
	scout0 = UAV("ralx6", "scout", 0)
	bomber1 = UAV("ralx6", "bomber", 1)
	bomber2 = UAV("ralx6", "bomber", 2)
	bomber3 = UAV("ralx6", "bomber", 3)

	uavs = [scout0, bomber1, bomber2, bomber3]
	#uavs = [scout0]

	scout0.fuel_resource = scout0.fuel_resource_subscribe.data
	scout0.fuel_consume = scout0.fuel_consume_subscribe.data
	
	bomber1.fuel_resource = bomber1.fuel_resource_subscribe.data
	bomber1.fuel_consume = bomber1.fuel_consume_subscribe.data
	
	bomber2.fuel_resource = bomber2.fuel_resource_subscribe.data
	bomber2.fuel_consume = bomber2.fuel_consume_subscribe.data
	
	bomber3.fuel_resource = bomber3.fuel_resource_subscribe.data
	bomber3.fuel_consume = bomber3.fuel_consume_subscribe.data

	start(uavs)	
