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
#from operator import pos
from pickle import NONE
#from re import template
from sys import path

from numpy.core.numeric import rollaxis
from numpy.lib.function_base import select
from rospy.exceptions import ROSException, ROSInternalException


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
from std_msgs.msg import Header, Float64, Bool

from better_mavros_scout0 import MavrosTestCommon


class swarm_parametr(object):
    def __init__(self):
        super(swarm_parametr, self).__init__()

        #path_ta.pos_listener()

        #x = MavrosTestCommon().goal_pose_x
        #y = MavrosTestCommon().goal_pose_y
        with open("UAV_Swarm_gazebo/catkin_ws/src/path_planning/scripts/path_global_wind.txt", "r") as ins:
            path_cpp = []
            for line in ins:
                path_cpp.append([float(line) for line in line.split()])# here send a list path_cpp
        self.positions = path_cpp
            #  (10,10,10),
            #  (11,13,10),
            #  (12,15,10),
            #  (13,18,10),
            #  (16,21,10),
            #  (19,23,10),
            #  (23,25,10)
            #  )
        self.altutude_height = 20 
        #self.positions = ((int(x), int(y), 10),(int(x), int(y), 10))

class MavrosOffboardPosctlTest_0(MavrosTestCommon):

    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def setUp(self):
        super(MavrosOffboardPosctlTest_0, self).setUp()

        self.pos = PoseStamped()
        self.change_bomber_mode= PoseStamped()
        self.check_battery_scout = Float64()
        self.radius = 30
        #armed_scout0 = State()
        for i in range(1, 20):
            self.__dict__['personal_land%d' % i] = PoseStamped()

        self.cargo_scout0 = Bool()
        self.fuel_resource_scout0 = Float64()
        self.fuel_consume_scout0 = Float64()

        self.takeoff_point = Point()
        self.grouping_point = Point()
        self.drop_point = Point()
        self.landing_point = Point()
        
        self.pos_setpoint_pub = rospy.Publisher('/scout0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.check_pub = rospy.Publisher('/scout0/mavros/check_mission', PoseStamped, queue_size=1)
        
        #TODO PROBLEM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        for uav_id in range(1, 20):
            self.__dict__['crit_sit_one_pub%d' % uav_id] = rospy.Publisher('/bomber' + str(uav_id) + '/mavros/personal_land/check_mission', PoseStamped, queue_size=1)
    

        self.check_battery_pub = rospy.Publisher('/scout0/mavros/battery_status', Float64, queue_size=1)

        #-------------------------------Crisis Situation-------------------------------------
        self.takeoff_point_publisher = rospy.Publisher("/scout0/takeoff_point", Point, queue_size=10)
        self.grouping_point_publisher = rospy.Publisher("/scout0/grouping_point", Point, queue_size=10)
        self.drop_point_publisher = rospy.Publisher("/scout0/drop_point", Point, queue_size=10)
        self.landing_point_publisher = rospy.Publisher("/scout0/landing_point", Point, queue_size=10)
        
        self.cargo_scout0_publisher = rospy.Publisher("/scout0/cargo", Bool, queue_size=10)
        self.fuel_resource_scout0_publisher = rospy.Publisher("/scout0/fuel_resource", Float64, queue_size=10)
        self.fuel_consume_scout0_publisher = rospy.Publisher("/scout0/fuel_consume", Float64, queue_size=10)

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
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
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
            for i in range(1, 20):
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
    #
    # -----------------------Flight mode's----------------------------
    #
    def global_path_flight_mode(self):
        takeoff_height = swarm_parametr().altutude_height
        positions = swarm_parametr().positions

        self.set_mode("OFFBOARD", 5)

        work0 = True
        while (work0 == True):
            for i in xrange(len(positions)):
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
                    rospy.WARN("Low Battary...")
                    self.set_mode("AUTO.LAND", 5)
                    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
                else:
                    rospy.loginfo("Total damage: %s, Upper damage: %s", self.total_damage, self.UPPER_DAMAGE_LIMIT)
                    self.reach_position(positions[i][0], positions[i][1], takeoff_height, 99999) # X, Y, Z
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
                if (self.scout0_exit_mode.pose.position.y == 1):
                        self.set_mode("AUTO.LOITER", 5) 
                        while (self.scout0_exit_mode.pose.position.y == 1):
                            rospy.loginfo("Pub 1")
                            self.change_bomber_mode.pose.position.x = 1
                            time.sleep(2)
                        self.change_bomber_mode.pose.position.x = 0
                        check = False 
                        
                elif (self.goal_pose_x == None and self.goal_pose_y == None):
                    target_position_x = str(self.goal_pose_x)
                    target_position_y = str(self.goal_pose_y)
                    rospy.loginfo("Taget Pos: %s and %s",target_position_x, target_position_y)
                    if (self.scout0_exit_mode.pose.position.y == 1):
                        #TODO pub 0 scout0_exit_mode
                        check = False
                    time.sleep(2)
                else:
                    self.set_mode("OFFBOARD", 5)
                    rospy.loginfo("This is number!")
                    rospy.loginfo("Goal pose: x=%s and y=%s", str(self.goal_pose_x), str(self.goal_pose_y))  
                    takeoff_height = 15
                    self.reach_position(int(self.goal_pose_x), int(self.goal_pose_y), takeoff_height, 30)
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
        self.change_bomber_mode.pose.position.x = 0
        work1 = False

    def rtl_mode(self):
        rospy.loginfo("Return to launch")
        check = True
        while (check == True):
            try:
                if (self.exit_p_num == 4):
                    self.set_mode("AUTO.RTL", 5)
                    while (self.exit_p_num < 8):
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
        self.set_arm(True, 5)
        self.set_mode("AUTO.TAKEOFF", 5)
        #self.reach_position(int(self.local_position.pose.position.x), int(self.local_position.pose.position.x), 5, 30)
        time.sleep(5)
        work1 = False

    def low_battery_mode(self):

        rospy.loginfo("This critical situation: %s", self.crit_sit.data[0])
        if (self.crit_sit.data[0] == NONE):
            rospy.loginfo("ERROR NONE VALUE")
        elif (self.crit_sit.data[0] == 0): # crisis situation for all drone
            rospy.loginfo("=============")
            rospy.loginfo("LOW BATTERY!!!")
            rospy.loginfo("=============")
            for i in range(1, 3):
                rospy.loginfo("Pub critical signal x, y = 1. sec %s", i)
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
                    rospy.loginfo("=============")
                    rospy.loginfo("LOW BATTERY!!!")
                    rospy.loginfo("=============")
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
        self.fuel_resource_scout0.data = 0.18
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
                rospy.loginfo("YES VALUE!")
                time.sleep(2)
            except:
                self.situation = NONE
                rospy.loginfo("NO VALUE!")
                self.set_arm(True, 5)
                time.sleep(2)
            else: 
                check_status = False
        rospy.loginfo("This critical situation: %s", self.situation)
        rospy.loginfo("run mission")  
        self.damage_calculate()
        #self.global_path_flight_mode()

        work1 = True
        while (work1 == True):
            self.info_mode()
            try:
                rospy.loginfo("Input: ")
                self.exit_p_num = self.input()
                rospy.loginfo("This is number: %s", self.exit_p_num)
                self.low_battery_mode()

            except ValueError:
                rospy.loginfo("This is not num!")
                work1 = True
                self.low_battery_mode()
            else:
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
                elif (self.exit_p_num != 1 or
                      self.exit_p_num != 2 or
                      self.exit_p_num != 3 or 
                      self.exit_p_num != 4 or
                      self.exit_p_num != 5 or
                      self.exit_p_num != 9 
                      ):
                    rospy.loginfo("Try again!")
                    work1 = True

if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node_0', anonymous=True)
    
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_0)
