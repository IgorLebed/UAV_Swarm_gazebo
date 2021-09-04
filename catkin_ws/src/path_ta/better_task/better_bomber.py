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
from better_mavros_bomber import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header, Int64
from threading import Thread
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import time
import better_scout as uv
#import setpoint_listener as path_ta


class MavrosOffboardPosctlTest_1(MavrosTestCommon):

    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    

    def setUp(self):
        self.uav_id = rospy.get_param('id')
        self.scout_id = rospy.get_param('scout_id')
        #super(MavrosOffboardPosctlTest_1, self).__init__(id=rospy.get_param('~ID'), scout_id=rospy.get_param('~scout_id'))
        super(MavrosOffboardPosctlTest_1, self).setUp(self.uav_id, self.scout_id)

        self.pos = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher('/bomber' + str(self.uav_id) + '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
        global takeoff_height
        takeoff_height = 2
        self.change_mode_is_available_pub = rospy.Publisher(
            '/bomber' + str(self.uav_id) + '/change_mode_is_available',
            Int64,
            queue_size=1
        )
        self.mode = None
        
    def tearDown(self):
        super(MavrosOffboardPosctlTest_1, self).tearDown()

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

    def mode_listener(self, data):
        self.mode = data
    # Test method
    def test_posctl(self):

        """Test offboard position control"""
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        
        rospy.loginfo("This is boomber")
        takeoff_height = uv.swarm_parametr().altutude_height
        positions = uv.swarm_parametr().positions

        rospy.loginfo("run mission")                  
            
        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1], takeoff_height, 1000) # X, Y, Z
            rospy.loginfo("%s" %(i))
        self.change_mode_is_available_pub.publish(1)
        work = True
        while (work == True):
            rospy.loginfo("Enter 1 FOLLOWER")
            rospy.loginfo("Enter 2 coverage")
            rospy.loginfo("Enter 3 to go landing")
            rospy.loginfo("Enter 4 AUTO.RTL")
            try:
                rospy.loginfo("Input: ")
                rospy.Subscriber('bomber' + str(self.uav_id) + '/set_mode', Int64, self.mode_listener)
                if self.mode:
                    exit_p_num = self.mode.data
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
                    rospy.loginfo("Follower\nExit from program")
                    self.set_mode("OFFBOARD", 5)
                    #work = False
                    # self.change_mode_is_available_pub.publish(2)
                    check =True
                    while check:
                        if self.scout0_check != 1:
                            print("EBAT")
                            while self.scout0_check != 1:
                                print("ZDAROVA")
                                self.set_mode("OFFBOARD", 5)
                                scout_pose_x = self.local_scout0_position.pose.position.x
                                scout_pose_y = self.local_scout0_position.pose.position.y
                                scout_pose_z = self.local_scout0_position.pose.position.z # test
                                self.reach_position(int(scout_pose_x) + 2, int(scout_pose_y), int(scout_pose_z) - 2, 1000) # X, Y, Z
                                rospy.loginfo("local pos x: %s and pos y: %s ", scout_pose_x, scout_pose_y)
                                time.sleep(0.3)
                            if self.scout0_check != 1:
                                check = False
                elif (exit_p_num == 2):
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
                                    self.reach_position(int(pose_x) + 2, int(pose_y), int(pose_z) - 2, 1000) # X, Y, Z
                                    time.sleep(0.3)      
                                check = False   
                                #work = False
                        except rospy.ROSInterruptException:
                            self.set_mode("AUTO.LOITER", 5)
                            check = False
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
                elif (exit_p_num != 1 or exit_p_num != 2 or exit_p_num != 3 or exit_p_num != 4):
                    rospy.loginfo("Try again!")
                    work = True


if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node_1', anonymous=True)
    
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_1)
