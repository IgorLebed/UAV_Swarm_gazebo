#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division
from pickle import NONE
from sys import path
import calculate_3d_point_coords as cp


PKG = 'px4'

import rospy
import math
import time
import numpy as np
from pymavlink import mavutil
from six.moves import xrange
from threading import Thread
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Quaternion , Point
from std_msgs.msg import Header, Int64 ,Float64, Bool

from mavros_scout import MavrosTestCommon


class swarm_parametr(object):
    def __init__(self):
        super(swarm_parametr, self).__init__()

        #path_ta.pos_listener()

        #x = MavrosTestCommon().goal_pose_x
        #y = MavrosTestCommon().goal_pose_y
        with open("/home/igor/UAV_Swarm_gazebo/catkin_ws/src/path_planning/scripts/path_global_wind.txt", "r") as ins:
            path_cpp = []
            for line in ins:
                path_cpp.append([float(line) for line in line.split()])# here send a list path_cpp
        self.positions =( #path_cpp
             (-845, -830, 100), (-895, -880, 100))
        self.altutude_height = 20 
        self.radius = 15
        #self.positions = ((int(x), int(y), 10),(int(x), int(y), 10))

class MavrosOffboardPosctlTest_0(MavrosTestCommon):

    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def setUp(self):

        self.uav_id = rospy.get_param('id')
        rospy.logwarn("id scout:%s", self.uav_id)
        
        super(MavrosOffboardPosctlTest_0, self).setUp(self.uav_id) 
        self.takeoff_height = swarm_parametr().altutude_height
        
        self.pos = PoseStamped()
        self.change_bomber_mode= PoseStamped()
        self.check_battery_scout = Float64()
        self.follower_mode = Bool()
        self.radius = swarm_parametr().radius

        for i in range(self.uav_id, self.uav_id + 20):
            self.__dict__['personal_land%d' % i] = PoseStamped()

        self.cargo_scout0 = Bool()
        self.fuel_resource_scout0 = Float64()
        self.fuel_consume_scout0 = Float64()

        self.takeoff_point = Point()
        self.grouping_point = Point()
        self.drop_point = Point()
        self.landing_point = Point()
        
        self.pos_setpoint_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.check_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/mavros/check_mission', PoseStamped, queue_size=1)
        self.follower_mode_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/follower_mode', Bool, queue_size=1 )
        self.change_mode_is_available_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/change_mode_is_available', Int64, queue_size=1) 

        for i in range(self.uav_id, self.uav_id + 20):
            self.__dict__['crit_sit_one_pub%d' % i] = rospy.Publisher('/bomber' + str(i) + '/mavros/personal_land/check_mission', PoseStamped, queue_size=1)
    

        self.check_battery_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/mavros/battery_status', Float64, queue_size=1)

        #-------------------------------Crisis Situation-------------------------------------
        self.takeoff_point_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/takeoff_point', Point, queue_size=10)
        self.grouping_point_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/grouping_point', Point, queue_size=10)
        self.drop_point_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/drop_point', Point, queue_size=10)
        self.landing_point_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/landing_point', Point, queue_size=10)
        
        self.cargo_scout0_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/cargo', Bool, queue_size=10)
        self.fuel_resource_scout0_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/fuel_resource', Float64, queue_size=10)
        self.fuel_consume_scout0_publisher = rospy.Publisher('/scout' + str(self.uav_id) + '/fuel_consume', Float64, queue_size=10)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # send change_bomber_mode in seperate thread to better prevent failsafe
        self.change_bomber_mode_thread = Thread(target=self.change_bomber_mode_send, args=())
        self.change_bomber_mode_thread.daemon = True
        self.change_bomber_mode_thread.start()
        
        # send check scout in seperate thread to better prevent failsafe
        self.check_battery_thread = Thread(target=self.send_check_battery_scout, args=())
        self.check_battery_thread.daemon = True
        self.check_battery_thread.start()

        # send cargo scout in seperate thread to better prevent failsafe
        self.cargo_thread = Thread(target=self.send_cargo, args=())
        self.cargo_thread.daemon = True
        self.cargo_thread.start()

        # send fuel_consume in seperate thread to better prevent failsafe
        self.fuel_consume_thread = Thread(target=self.send_fuel_consume, args=())
        self.fuel_consume_thread.daemon = True
        self.fuel_consume_thread.start()

        # send fuel_resource in seperate thread to better prevent failsafe
        self.fuel_resource_thread = Thread(target=self.send_fuel_resource, args=())
        self.fuel_resource_thread.daemon = True
        self.fuel_resource_thread.start()

        # send personal in seperate thread to better prevent failsafe
        self.personal_thread = Thread(target=self.send_personal, args=())
        self.personal_thread.daemon = True
        self.personal_thread.start()

        # send follow in seperate thread to better prevent failsafe
        self.follower_mode_thread = Thread(target=self.send_follower_mode, args=())
        self.follower_mode_thread.daemon = True
        self.follower_mode_thread.start()

        global takeoff_height

    def tearDown(self):
        super(MavrosOffboardPosctlTest_0, self).tearDown()
    #
    # ---------------------Publisher's methods-------------------------
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "Active_pose"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def change_bomber_mode_send(self):
        #TODO Rename!!!!!!!!!!!!!!!!!!!!!!!!
        rate = rospy.Rate(10)  # Hz
        self.change_bomber_mode.header = Header()
        self.change_bomber_mode.header.frame_id = "map"

        while not rospy.is_shutdown():
            self.change_bomber_mode.header.stamp = rospy.Time.now()
            self.check_pub.publish(self.change_bomber_mode)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_check_battery_scout(self):
        rate = rospy.Rate(10)  # Hz
        #self.check_battery_scout.header = Header()
        #self.check_battery_scout.header.frame_id = "map"

        while not rospy.is_shutdown():
            #self.check_battery_scout.header.stamp = rospy.Time.now()
            self.check_battery_pub.publish(self.check_battery_scout)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass        
     #
    #
    # -----------------------Helper methods------------------------
    #
    def orientation_yaw(self, x_1, y_1, x_2, y_2):
        yaw = (y_2 - y_1)/(x_2 - x_1)
        if (x_1 > x_2):
            ori_yaw_ = math.atan(yaw)
            ori_yaw = ori_yaw_ + math.pi 
        else: 
            ori_yaw = math.atan(yaw)
        return ori_yaw

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):

        print("======================= PRINT")
        rospy.loginfo("======================= loginfo")
        self.low_battery_mode()
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = self.orientation_yaw(self.local_position.pose.position.x, self.local_position.pose.position.y, x, y) 
        yaw = yaw_degrees #math.radians(yaw_degrees) 
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            self.low_battery_mode()
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    # Crisis situation-------------------------
    #
    def send_cargo(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.cargo_scout0_publisher.publish(self.cargo_scout0)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_personal(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            for i in range(self.uav_id, self.uav_id + 20):
                self.__dict__['crit_sit_one_pub%d' % i].publish(self.__dict__['personal_land%d' % i])
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_fuel_resource(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.fuel_resource_scout0_publisher.publish(self.fuel_resource_scout0)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_fuel_consume(self):
            rate = rospy.Rate(10)  # Hz
            while not rospy.is_shutdown():
                self.fuel_consume_scout0_publisher.publish(self.fuel_consume_scout0)
                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
    
    def send_takeoff_point(self):
            rate = rospy.Rate(10)  # Hz
            while not rospy.is_shutdown():
                self.takeoff_point_publisher.publish(self.takeoff_point)
                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass

    def send_grouping_point(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.grouping_point_publisher.publish(self.grouping_point)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass    

    def send_drop_point(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.drop_point_publisher.publish(self.drop_point)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass    

    def send_landing_point(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.landing_point_publisher.publish(self.landing_point)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass    

    def info_mode(self):
        rospy.loginfo("====================================")
        rospy.loginfo("Enter 1 go to")
        rospy.loginfo("Enter 2 global_path")
        rospy.loginfo("Enter 3 landing")
        rospy.loginfo("Enter 4 go to launch")
        rospy.loginfo("Enter 5 takeoff")
        rospy.loginfo("Enter 9 Exit this program")
        rospy.loginfo("====================================")

    def input(self):
        #rospy.loginfo("You have 5 seconds to type in your stuff...") 
        foo = int(raw_input())
        return foo
    
    # ---------------------FOR MISSION MODE--------------------------
    #
    def send_follower_mode(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.follower_mode_pub.publish(self.follower_mode)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
    
    def target_find_mode(self):
        if (self.target_find.data == True):
            rospy.logwarn("Target Find")
            self.go_to_target_point_mode()
   
    def mode_listener(self, data):
        self.mode = data   
    #
    # -----------------------Flight mode's----------------------------
    #
    def global_path_flight_mode(self):
        #takeoff_height = swarm_parametr().altutude_height
        positions = swarm_parametr().positions

        self.set_mode("OFFBOARD", 5)

        work0 = True
        while (work0 == True):
            first_point = 0
            for i in xrange(first_point, len(positions)):
                self.damage_calculate()
                self.low_battery_mode()
                if (self.total_damage >= self.UPPER_DAMAGE_LIMIT):
                    rospy.loginfo("Total damage: %s, Upper damage: %s", self.total_damage, self.UPPER_DAMAGE_LIMIT)
                    self.set_mode("AUTO.LAND", 5)
                    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
                    self.set_model_state()
                    self.unregister_subs()
                    #self.set_arm(False, 5) 
                    work0 = False
                    break
                elif(self.total_battery <= self.LOWER_BATTARY_LIMIT):
                    rospy.loginfo("Total damage: %s, Upper damage: %s", self.total_battery, self.LOWER_BATTARY_LIMIT)
                    rospy.logwarn("Low Battary...")
                    self.set_mode("AUTO.LAND", 5)
                    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
                else:
                    self.target_find_mode()
                        #x = 0
                        #if (x == 0):#TODO Make a condition
                            #rospy.loginfo("End mission")
                    rospy.loginfo("Total damage: %s, Upper damage: %s", self.total_damage, self.UPPER_DAMAGE_LIMIT)
                    self.reach_position(positions[i][0], positions[i][1], self.takeoff_height, 99999) # X, Y, Z
                    time.sleep(1)
                    rospy.loginfo("%s" %(i))
                    if (i+1 == len(positions)):
                        work0 = False
                        break

    def go_to_target_point_mode(self):
        #rospy.loginfo("Hold mode\nExit from program")
        self.set_mode("OFFBOARD", 5)
        #work1 = False
        check = True
        while (check == True):#(self.goal_pose_x == None and self.goal_pose_y == None):
            try:
                if (self.scout0_exit_mode.data == True):
                        self.set_mode("AUTO.LOITER", 5) 
                        while (self.scout0_exit_mode.data == True):
                            rospy.loginfo("Pub 1")
                            self.change_bomber_mode.pose.position.x = 1
                            time.sleep(2)
                        self.change_bomber_mode.pose.position.x = 0
                        check = False 
                elif (self.goal_pose_x == None and self.goal_pose_y == None):
                    target_position_x = str(self.goal_pose_x)
                    target_position_y = str(self.goal_pose_y)
                    rospy.loginfo("Taget Pos: %s and %s",target_position_x, target_position_y)
                    if (self.scout0_exit_mode.data == True):
                        #TODO pub 0 scout0_exit_mode
                        check = False
                    time.sleep(2)
                else:
                    self.set_mode("OFFBOARD", 5)
                    rospy.loginfo("This is number!")
                    rospy.loginfo("Goal pose: x=%s and y=%s", str(self.goal_pose_x), str(self.goal_pose_y))  
                    takeoff_height = swarm_parametr().altutude_height
                    calculate_point_coords = cp.main(self.local_position.pose.position.x, self.local_position.pose.position.y, self.goal_pose_x, self.goal_pose_y)
                    local_pose_x = self.local_position.pose.position.x
                    local_pose_y = self.local_position.pose.position.y
                    x = 0 #TODO signal from operator
                    while x < 4:
                        #TODO Need refactor!!!!!!
                        rospy.logerr("goal x %s, goal y %s", self.goal_pose_x, self.goal_pose_y)
                        time.sleep(5)
                        self.reach_position(int(self.goal_pose_x), int(self.goal_pose_y), takeoff_height, 99999)
                        self.reach_position(int(calculate_point_coords[0]), int(calculate_point_coords[1]), takeoff_height, 99999)
                        self.reach_position(int(local_pose_x), int(local_pose_y), takeoff_height, 99999)
                        rospy.logerr("local x %s, local y %s", local_pose_x, local_pose_y)
                        rospy.logwarn("Calculate_3d_point_coords!!!!!!!!_______%s time there are 4", x) 
                        x += 1
                        time.sleep(2)   
                    self.reach_position(int(self.goal_pose_x), int(self.goal_pose_y), takeoff_height, 99999)
                    
                    self.goal_pose_x = None
                    self.goal_pose_y = None
                    time.sleep(2)
            except rospy.ROSInterruptException:
                self.set_mode("AUTO.LOITER", 5)
                check = False

    def landing_mode(self):
        rospy.loginfo("Exit from program")
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)
        self.follower_mode.data = False
        self.change_bomber_mode.pose.position.x = 0
        work1 = False

    def rtl_mode(self):
        rospy.loginfo("Return to launch")
        check = True
        while (check == True):
            try:
                self.set_mode("AUTO.RTL", 5)
                for i in range(3):
                    rospy.loginfo("Pub 1")
                    self.change_bomber_mode.pose.position.x = 1 
                    self.exit_p_num += 1
                    time.sleep(1)
                check = False
            except rospy.ROSInterruptException:
                self.set_mode("AUTO.LAND", 5)
                check = False 

    def takeoff_mode(self):
        rospy.loginfo("=============")
        rospy.loginfo("TAKEOFF MODE!!!")
        rospy.loginfo("=============")
        for i in range(3):
            self.follower_mode.data = True
            rospy.loginfo("Pub to follow mode, %s sec", i)
            i += 1
            time.sleep(1)
        self.set_arm(True, 5)
        #self.set_mode("AUTO.TAKEOFF", 5)
        #time.sleep(5)
        self.set_mode("OFFBOARD", 5)
        self.reach_position(int(self.local_position.pose.position.x), 
                            int(self.local_position.pose.position.x), 
                                self.takeoff_height, 99999)
        self.low_battery_mode()
        time.sleep(5)
        #work1 = False

    def low_battery_mode(self):

        rospy.loginfo("This critical situation: %s", self.crit_sit.data[0])
        if (self.crit_sit.data[0] == NONE):
            rospy.loginfo("ERROR NONE VALUE")
        elif (self.crit_sit.data[0] == 0): # crisis situation for all drone
            rospy.logerr("=============")
            rospy.logerr("LOW BATTERY!!!")
            rospy.logerr("=============")
            for i in range(1, 3):
                rospy.loginfo("Pub signal x, y = 1. sec %s", i)
                self.change_bomber_mode.pose.position.x = 1
                self.change_bomber_mode.pose.position.y = 1
                time.sleep(1)
            self.set_mode("AUTO.RTL", 5)
            work0 = False
            return work0
        elif (self.crit_sit.data[0] == 1): # crisis situation for the drone
            check_status = True
            while check_status:
                try: 
                    #self.crit_sit.data[0]
                    self.id_drone = self.crit_sit.data[1]
                    self.__dict__['personal_land%d' % self.id_drone].pose.position.x = 1
                    rospy.logerr("=============")
                    rospy.logerr("LOW BATTERY!!!")
                    rospy.logerr("=============")
                    rospy.loginfo("YES VALUE!")
                    
                    time.sleep(2)
                except:
                    self.crit_sit.data[0] = 0
                    rospy.loginfo("NO VALUE!")
                    self.set_arm(True, 5)
                    time.sleep(2)
                else: 
                    check_status = False
    
    #
    # -----------------------Flight method----------------------------
    #
    def test_posctl(self):
        self.id_drone = 0
        """Test respawn in start mission point"""
        #self.set_model_state() #For test 
        #time.sleep(100)        #For test

        #self.damage_calculate()
        """Send messages for crisis situation"""
        self.cargo_scout0.data = True
        self.fuel_resource_scout0.data = 0.8
        self.fuel_consume_scout0.data = 0.8

        self.takeoff_point.x, self.takeoff_point.y = 0, 0
        self.grouping_point.x, self.grouping_point.y = 10, 10
        self.drop_point.x, self.drop_point.y = 100, 100
        self.landing_point.x, self.landing_point.y = 200, 50

        """Test offboard position control"""

        self.log_topic_vars()
        #self.set_mode("OFFBOARD", 5)
        #self.set_arm(True, 5)
        rospy.loginfo("This is scout")

        check_status = True
        while check_status:
            try: 
                self.situation = self.crit_sit.data[0]
                self.change_bomber_mode.pose.position.y = 0
                rospy.loginfo("YES VALUE!")
                time.sleep(2)
            except:
                self.situation = None
                rospy.loginfo("NO VALUE!")
                self.change_bomber_mode.pose.position.y = 1 
                self.set_arm(True, 4)
                time.sleep(5)
            else: 
                check_status = False
        rospy.logwarn("This critical situation: %s", self.situation)
        self.low_battery_mode()
        rospy.loginfo("run mission")  
        self.damage_calculate()
        #self.global_path_flight_mode()

        #self.start_mission.data = False     # Rewrite value, if start mission have old value
        work1 = True                        # While true menu working and have you may change other flight mode, with the exeption 9 mode "exit"
        while (work1 == True):
            try:
                rospy.Subscriber('scout'+ str(self.uav_id) + '/set_mode', Int64, self.mode_listener)
                if (self.start_mission.data == False):
                    self.info_mode()
                    rospy.loginfo("Input: ")
                    self.exit_p_num = self.input()#self.mode.data #
                    rospy.loginfo("This is number: %s", self.exit_p_num)
                    self.low_battery_mode()
                else:
                    self.exit__p_num = None
                    rospy.loginfo("LOADING...")

            except ValueError:
                rospy.loginfo("This is not num!")
                work1 = True
                self.low_battery_mode()
            else:
                if self.start_mission.data == False:
                    if (self.exit_p_num == 1):
                        self.go_to_target_point_mode()
                    elif (self.exit_p_num == 2):
                        rospy.loginfo("This is global path")
                        self.global_path_flight_mode()
                    elif (self.exit_p_num == 3):
                        self.landing_mode()
                    elif (self.exit_p_num == 4):
                        self.rtl_mode()
                    elif (self.exit_p_num == 5):
                        self.takeoff_mode()    
                    elif (self.exit_p_num == 9):
                        work1 = False
                        break
                    #----
                    elif (self.exit_p_num != 1 or
                        self.exit_p_num != 2 or
                        self.exit_p_num != 3 or 
                        self.exit_p_num != 4 or
                        self.exit_p_num != 5 or
                        self.exit_p_num != 9 
                        ):
                        rospy.loginfo("Try again!")
                        work1 = True
                else:
                    i = 1
                    while (i == 1 and self.start_mission.data == True): 
                        if(self.crit_sit.data[0] == -1):  
                            rospy.loginfo("===========")
                            rospy.loginfo ("RUN MISSION!")
                            rospy.loginfo("===========")
                            self.takeoff_mode()
                            time.sleep(5)
                            i = 0
                        elif (self.crit_sit.data[0] == 0 or self.crit_sit.data[0] == 1):
                            rospy.logerr("ERROR: Land")
                            self.follower_mode.data = False
                            self.low_battery_mode()
                            self.landing_mode()
                            time.sleep(5)
                            i = 1
                    rospy.logwarn("Start mission")
                    for i in range(3):
                        self.follower_mode.data = True
                        rospy.loginfo("Pub to follow mode, %s sec", i)
                        i += 1
                        time.sleep(1)
                    rospy.logwarn("Wait 10 sec")
                    time.sleep(10)
                    #self.landing_mode()
                    #TODO Check target point
                    self.global_path_flight_mode()

                    self.set_mode("AUTO.LOITER", 5)
                    for i in range(1, 3):
                        rospy.loginfo("Exit follower mode! sec %s", i)
                        self.follower_mode.data = False
                        #rospy.loginfo("Pub signal x, y = 1. sec %s", i)
                        #self.change_bomber_mode.pose.position.x = 1
                        #self.change_bomber_mode.pose.position.y = 1
                        time.sleep(1)
                    time.sleep(5)

                    for i in range(3):
                        rospy.loginfo("Enter follower mode! sec %s", i)
                        self.follower_mode.data = True
                    bomber_massage = True # TODO Sub on bomber, while something like with....
                    if (bomber_massage == True):
                        for i in range(45):
                            rospy.loginfo("Wait... Bombers while doesn't ending mission")
                            time.sleep(2)     
                            if (i > 45):                  #TEST!!!!!!!!!!
                                bomber_massage = False    #TEST!!!!!!!!!!
                    else:
                        rospy.logwarn("End mission")
                        for i in range(3):
                            self.follower_mode.data = True
                            rospy.loginfo("Pub to follow mode, %s sec", i)
                            i += 1
                            time.sleep(1)
                        rospy.logwarn("Wait 10 sec")
                        time.sleep(10)
                    rospy.logwarn("RTL MODE")    
                    self.set_mode("AUTO.RTL", 5)
                    for i in range(3):
                        rospy.logwarn("Mission End %s", i)
                        self.start_mission.data = False



if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node', anonymous=True)
    
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_0)