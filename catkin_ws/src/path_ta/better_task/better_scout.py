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
from better_mavros_scout import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header, Int64
from threading import Thread
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import time
#import setpoint_listener as path_ta



class swarm_parametr(object):
    def __init__(self):
        super(swarm_parametr, self).__init__()

        #path_ta.pos_listener()
        
        #x = MavrosTestCommon().goal_pose_x
        #y = MavrosTestCommon().goal_pose_y
        self.positions = (
            (60,-59,10), # (97,-39,10)
            (60, -76, 10)
            #(66,-70,25)
            )
        self.altutude_height = 150 #200

        #self.positions = ((int(x), int(y), 10),(int(x), int(y), 10))

class MavrosOffboardPosctlTest_0(MavrosTestCommon):

    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """


    def setUp(self):
        #super(MavrosOffboardPosctlTest_0, self, uav_id=0).__init__()
        self.uav_id = rospy.get_param('id')
        print("HAAAAAAAAAAHAHAHAHAHAHAHAHAHHAHAH")
        print(self.uav_id)
        super(MavrosOffboardPosctlTest_0, self).setUp(self.uav_id)        
        # MavrosTestCommon.__init__(self, id=rospy.get_param('~ID'))
        self.mode = None
        self.pos = PoseStamped()
        self.check_scout= PoseStamped()
        self.radius = 1


        self.pos_setpoint_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.check_pub = rospy.Publisher('/scout' + str(self.uav_id) + '/mavros/check_mission', Int64, queue_size=1)
        self.change_mode_is_available_pub = rospy.Publisher(
            '/scout' + str(self.uav_id) + '/change_mode_is_available',
            Int64,
            queue_size=1
        )
        # self.change_mode_is_available_pub_bomber = rospy.Publisher(
        #     '/bomber1/change_mode_is_available',
        #     Int64,
        #     queue_size=1
        # )

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # send check scout in seperate thread to better prevent failsafe
        self.check_thread = Thread(target=self.send_check_scout, args=())
        self.check_thread.daemon = True
        self.check_thread.start()
        
        global takeoff_height
        # takeoff_height = 2
        
    def tearDown(self):
        super(MavrosOffboardPosctlTest_0, self).tearDown()

    #
    # Helper methods
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

    def send_check_scout(self):
        rate = rospy.Rate(10)  # Hz
        self.check_scout.header = Header()
        self.check_scout.header.frame_id = "map"

        while not rospy.is_shutdown():
            # self.check_scout.header.stamp = rospy.Time.now()
            # self.check_pub.publish(self.check_scout)
            self.check_pub.publish(1)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

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

    # Test method
    def mode_listener(self, data):

        self.mode = data
    def test_posctl(self):
                   
        """Test offboard position control"""
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        
        rospy.loginfo("This is bomber")

        takeoff_height = swarm_parametr().altutude_height
        positions = swarm_parametr().positions

        rospy.loginfo("run mission")  

        for i in xrange(len(positions)):
            
            #rospy.loginfo("Position talker x: %s", path_ta.goal_pose_x)
            #rospy.loginfo("Position talker y: %s", path_ta.goal_pose_y)
            self.reach_position(positions[i][0], positions[i][1], takeoff_height, 1000) # X, Y, Z
            #self.reach_position(positions[i][0], positions[i][1], positions[i][2], 30)
            rospy.loginfo("%s" %(i))
        self.change_mode_is_available_pub.publish(1)
        # self.change_mode_is_available_pub_bomber.publish(1)
        work = True
        while work:
            rospy.loginfo("Enter 1 hold")
            rospy.loginfo("Enter 2 coverage")
            rospy.loginfo("Enter 3 landing")
            rospy.loginfo("Enter 4 go to launch")
            try:
                # rospy.loginfo("Input: ")
                print("FFFFF")
                rospy.Subscriber('scout'+ str(self.uav_id) +'/set_mode', Int64, self.mode_listener)
                if self.mode:
                    exit_p_num = self.mode.data
                    print(exit_p_num)
                    print("YES YES YES")
                    rospy.loginfo("This is number: %s", exit_p_num)
                else:
                    exit_p_num = None
                
                #rospy.loginfo("Position talker after input x: %s", path_ta.goal_pose_x)
                #rospy.loginfo("Position talker after input y: %s", path_ta.goal_pose_y)

            except ValueError:
                rospy.loginfo("This is not num!")
                work = True
            else:
                if exit_p_num == 1:
                    rospy.loginfo("Hold mode\nExit from program")
                    self.set_mode("OFFBOARD", 5)
                    #work = False
                    check = True
                    while (check == True):#(self.goal_pose_x == None and self.goal_pose_y == None):
                        try:
                            if (self.scout0_check == 1):
                                    self.set_mode("AUTO.LOITER", 5) 
                                    while self.scout0_check == 1:
                                       rospy.loginfo("Pub 1")
                                       self.check_scout.pose.position.x = 1
                                       time.sleep(2)
                                    self.check_scout.pose.position.x = 0
                                    check = False 
                                    
                            elif self.goal_pose_x == None and self.goal_pose_y == None:
                                target_position_x = str(self.goal_pose_x)
                                target_position_y = str(self.goal_pose_y)
                                rospy.loginfo("Taget Pos: %s and %s",target_position_x, target_position_y)
                                if (self.scout0_check == 1):
                                    # self.change_mode_is_available_pub.publish(2)
                                    #TODO pub 0 scout0_check
                                    check = False
                                time.sleep(2)
                            else:
                                self.set_mode("OFFBOARD", 5)
                                rospy.loginfo("This is number!")
                                rospy.loginfo("Goal pose: x=%s and y=%s", str(self.goal_pose_x), str(self.goal_pose_y))  
                                # takeoff_height = 15
                                self.reach_position(int(self.goal_pose_x), int(self.goal_pose_y), takeoff_height, 1000)
                                self.goal_pose_x = None 
                                self.goal_pose_y = None   
                                time.sleep(2)

                        except rospy.ROSInterruptException:
                            self.set_mode("AUTO.LOITER", 5)
                            check = False
                elif (exit_p_num == 2):

                    rospy.loginfo("This is coverage input: ")
                    #TODO 
                    """
                    Write start coverage node!
                    for i in xrange(len(positions)):
                        self.reach_position(positions[i][0], positions[i][1], takeoff_height, 30) # X, Y, Z
                        #self.reach_position(positions[i][0], positions[i][1], positions[i][2], 30)
                        rospy.loginfo("%s" %(i))
                        self.talker()
                    Then possition not a rrt goal node. The possition this is a coverage goal node
                    """
                    #work = False
                elif (exit_p_num == 3):
                    rospy.loginfo("Exit from program")
                    self.set_mode("AUTO.LAND", 5)
                    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
                    self.set_arm(False, 5)
                    work = False
                elif (exit_p_num == 4):
                    rospy.loginfo("Return to launch")
                    self.set_mode("AUTO.RTL", 5)
                    work = False
                elif (exit_p_num != 1 or exit_p_num != 2 or exit_p_num != 3 or exit_p_num !=4):
                    rospy.loginfo("Try again!")
                    work = True

if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node_0', anonymous=True)
    
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_0)
