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


PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from better_mavros_bomber3 import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String, Header, Float64, Bool
import time
import better_scout_0 as uv
#import setpoint_listener as path_ta


class MavrosOffboardPosctlTest_3(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def setUp(self):
        super(MavrosOffboardPosctlTest_3, self).setUp()

        self.pos = PoseStamped()
        self.cargo_bomber3 = Bool()
        self.fuel_resource_bomber3 = Float64()
        self.fuel_consume_bomber3 = Float64()

        self.radius = uv.swarm_parametr().radius

        self.pos_setpoint_pub = rospy.Publisher('/bomber3/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        #-------------------------------Crisis Situation-------------------------------------
        self.cargo_bomber3_publisher = rospy.Publisher("/bomber3/cargo", Bool, queue_size=10)
        self.fuel_resource_bomber3_publisher = rospy.Publisher("/bomber3/fuel_resource", Float64, queue_size=10)
        self.fuel_consume_bomber3_publisher = rospy.Publisher("/bomber3/fuel_consume", Float64, queue_size=10)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # send cargo scout in seperate thread to better prevent failsafe
        self.cargo_thread = Thread(target=self.send_cargo, args=())
        self.cargo_thread.daemon = True
        self.cargo_thread.start()

        # send fuel_resource in seperate thread to better prevent failsafe
        self.fuel_resource_thread = Thread(target=self.send_fuel_resource, args=())
        self.fuel_resource_thread.daemon = True
        self.fuel_resource_thread.start()
        
        # send fuel_consume in seperate thread to better prevent failsafe
        self.fuel_consume_thread = Thread(target=self.send_fuel_consume, args=())
        self.fuel_consume_thread.daemon = True
        self.fuel_consume_thread.start()

        global takeoff_height
        takeoff_height = 2
        
    def tearDown(self):
        super(MavrosOffboardPosctlTest_3, self).tearDown()
    #
    # ---------------------------Publisher's methods---------------------------
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # ---------------------------Helper methods---------------------------
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
    #
    # Crisis situation-------------------------
    #
    def send_cargo(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.cargo_bomber3_publisher.publish(self.cargo_bomber3)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_fuel_resource(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.fuel_resource_bomber3_publisher.publish(self.fuel_resource_bomber3)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_fuel_consume(self):
            rate = rospy.Rate(10)  # Hz
            while not rospy.is_shutdown():
                self.fuel_consume_bomber3_publisher.publish(self.fuel_consume_bomber3)
                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
    
    def info_mode(self):
        rospy.loginfo("Enter 1 FOLLOWER")
        rospy.loginfo("Enter 2 Target Point")
        rospy.loginfo("Enter 3 AUTO.LAND")
        rospy.loginfo("Enter 4 AUTO.RTL")
        rospy.loginfo("Enter 5 AUTO.TAKEOFF")
        rospy.loginfo("Enter 9 Exit")   
    
    def crisis_mode(self):
        rospy.loginfo("=============")
        rospy.loginfo("LOW BATTERY!!!")
        rospy.loginfo("=============")
        self.rtl_mode()
    #
    #----------------------------Flight mode's-----------------------------
    #
    def glogal_path_flight(self):
        takeoff_height = uv.swarm_parametr().altutude_height
        positions = uv.swarm_parametr().positions

        self.set_mode("OFFBOARD", 5)

        rospy.loginfo("run mission")                  
        self.situation = 3
        work0 = True
        while (work0 == True):
            for i in xrange(len(positions)):
                self.damage_calculate()
                rospy.loginfo("This critical situation: %s", self.situation)
                if (self.scout0_check.pose.position.y == 1):
                    rospy.loginfo("=============")
                    rospy.loginfo("LOW BATTERY!!!")
                    rospy.loginfo("=============")
                    self.set_mode("AUTO.RTL", 5)
                    work0 = False
                elif (self.total_damage >= self.UPPER_DAMAGE_LIMIT):
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

    def follower_mode(self): 
        rospy.loginfo("Follower")
        self.set_mode("OFFBOARD", 5)
        #work = False
        check =True
        while (check == True):
            if (self.scout0_check.pose.position.x != 1):
                while (self.scout0_check.pose.position.x != 1 or self.scout0_check.pose.position.y !=1):
                    self.set_mode("OFFBOARD", 5)
                    scout_pose_x =  self.local_scout0_position.pose.position.x
                    scout_pose_y =  self.local_scout0_position.pose.position.y
                    scout_pose_z =  self.local_scout0_position.pose.position.z # test
                    self.reach_position(int(scout_pose_x) + 2, int(scout_pose_y), int(scout_pose_z) - 2, 50) # X, Y, Z
                    rospy.loginfo("local pos x: %s and pos y: %s ", scout_pose_x, scout_pose_y)
                    if (self.personal_land.pose.position.x == 1):
                        self.scout0_check.pose.position.x = 1
                        self.scout0_check.pose.position.y = 1
                        self.crisis_mode()
                        check = False
                        break
                    if (self.scout0_follower_mode.data == False):
                        self.scout0_check.pose.position.x = 1
                        self.scout0_check.pose.position.y = 1
                        rospy.logwarn("False follower mode!!!")
                        self.rtl_mode()
                        check = False
                        break
                    #time.sleep(0.3)
                if self.scout0_check.pose.position.x == 1: #exit form this mode
                    check = False
                if self.scout0_check.pose.position.y == 1: #critical situation
                    self.crisis_mode()
                    check = False 

    def target_point_mode(self):
        rospy.loginfo("This is target fly mode")
        self.set_mode("OFFBOARD", 5)
        check =True
        while (check == True):
            try:
                if (self.goal_pose_x == None and self.goal_pose_y == None):
                    target_position_x = str(self.goal_pose_x)
                    target_position_y = str(self.goal_pose_y)
                    rospy.loginfo("Taget Pos: %s and %s",target_position_x, target_position_y)
                    # if (self.scout0_check.pose.position.y == 1):
                    #     #TODO pub 0 scout0_check
                    #     check = False
                    time.sleep(2)
                else:
                    while (self.scout0_check.pose.position.x == 1):
                        self.set_mode("OFFBOARD", 5)
                        pose_x =  self.goal_pose_x
                        pose_y =  self.goal_pose_y
                        pose_z =  13 #test
                        self.reach_position(int(pose_x) + 2, int(pose_y), int(pose_z) - 2, 50) # X, Y, Z 
                        time.sleep(0.3)      
                    check = False   
                    #work = False
            except rospy.ROSInterruptException:
                self.set_mode("AUTO.LOITER", 5)
                check = False
    
    def landing_mode(self):
        rospy.loginfo("Exit from program")
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)
        work = False
    
    def rtl_mode(self):
        rospy.loginfo("Return to launch")
        self.set_mode("AUTO.RTL", 5)
        work = False
    
    def takeoff_mode(self):
        rospy.loginfo("=============")
        rospy.loginfo("TAKEOFF MODE!!!")
        rospy.loginfo("=============")
        self.set_arm(True, 5)
        self.set_mode("OFFBOARD", 5)
        self.reach_position(int(self.local_position.pose.position.x), int(self.local_position.pose.position.x), 10, 30)
        time.sleep(5)
        work1 = False
    #
    # -----------------------Flight method----------------------------
    #
    def test_posctl(self):
        """Send messages for crisis situation"""
        self.cargo_bomber3.data = True
        self.fuel_resource_bomber3.data = 0.2
        self.fuel_consume_bomber3.data = 0.8

        """Test offboard position control"""
        self.log_topic_vars()
        #self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("This is boomber")

        work = True
        end_mission_mode = 0
        while (work == True):   
            try:
                if self.start_mission.data == False:
                    self.info_mode()
                    rospy.loginfo("Input: ")
                    exit_p_num = int(raw_input())
                    rospy.loginfo("This is number: %s", exit_p_num)
                    if (self.personal_land.pose.position.x == 1): 
                        rospy.logwarn("Low battery!")
                        self.crisis_mode()
                else:
                    rospy.loginfo("LOADING...") 
            except ValueError:
                rospy.loginfo("This is not num!")
                work = True
            else:
                if self.start_mission.data == False:
                    try:
                        if (exit_p_num == 1):
                            self.follower_mode()
                        elif (exit_p_num == 2):
                            self.target_point_mode()
                        elif (exit_p_num == 3):
                            self.landing_mode()
                        elif (exit_p_num == 4):
                            self.rtl_mode()
                        elif (exit_p_num == 5):
                            self.takeoff_mode()    
                        elif (exit_p_num == 9):
                            work = False
                            break
                        elif (exit_p_num != 1 or
                            exit_p_num != 2 or 
                            exit_p_num != 3 or
                            exit_p_num != 4 or
                            exit_p_num != 5 or 
                            exit_p_num != 9
                            ):
                            rospy.loginfo("Try again!")
                            work = True               
                    except:
                        exit_p_num = f
                while (self.start_mission.data == True and end_mission_mode == 0):
                    rospy.logwarn("Mission mode")
                    self.set_arm(True, 5)
                    if (self.scout0_follower_mode.data == False or self.scout0_follower_mode.data == True):
                        f = 0
                        self.takeoff_mode()
                        while (self.scout0_follower_mode.data == False):
                            if (f>30):
                                rospy.logerr("Connection denied")
                                self.set_mode("AUTO.LAND", 5)
                                break
                            rospy.logwarn("Wait conection with scout")
                            f += 1
                            time.sleep(1)
                        exit_follower_mode = 1
                        if (self.scout0_follower_mode.data == True and exit_follower_mode == 1):
                            self.follower_mode()
                            if (self.scout0_check.pose.position.x == 1 or self.scout0_check.pose.position.y == 1):
                                exit_follower_mode = 0

                    rospy.logerr("END Mission mow")        
                    rospy.loginfo("=============")
                    end_mission_mode = 1
                    #time.sleep(10)
                    



if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node_3', anonymous=True)
    
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_3)
