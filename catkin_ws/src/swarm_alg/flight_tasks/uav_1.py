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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic



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
from mavros_1 import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import uav_0 as uv


class MavrosOffboardPosctlTest_1(MavrosTestCommon):

    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    
    def setUp(self):
        super(MavrosOffboardPosctlTest_1, self).setUp()

        self.pos = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher('/bomber1/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        global takeoff_height
        global positions
        
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
        
    
    def listener(self):
        def callback(data):
            rospy.loginfo(rospy.get_caller_id()+ "I heard %s", data.data)
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        #rospy.init_node('listener', anonymous=True)
        a = True
        while a == True:
            rospy.Subscriber("chatter", String, callback)
            a = False
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()


    # Test method
    def test_posctl(self):
        """Test offboard position control"""
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        
        rospy.loginfo("This is a slaver")

        takeoff_height = uv.swarm_parametr().altutude_height
        positions = uv.swarm_parametr().positions

        rospy.loginfo(takeoff_height)

        rospy.loginfo("run mission")  

        for i in xrange(len(positions)):
            
            self.reach_position(positions[i][0], positions[i][1], takeoff_height, 30) # X, Y, Z
            #self.reach_position(positions[i][0], positions[i][1], positions[i][2], 30)
            rospy.loginfo("%s" %(i))
            self.listener()

        work = True
        while (work == True):
            rospy.loginfo("Enter 1 hold and exit program")
            rospy.loginfo("Enter 2 auto landing")
            rospy.loginfo("Enter 3 return to launch")
            rospy.loginfo("Enter 4 coverage planning")
            try:
                rospy.loginfo("Input: ")
                exit_p_num = int(raw_input())
                rospy.loginfo("This is number: %s", exit_p_num)

            except ValueError:
                rospy.loginfo("This is not num!")
                work = True
            else:
                if (exit_p_num == 1):
                    rospy.loginfo("Exit from program")
                    self.set_mode("AUTO.LOITER", 5)
                    #execfile('/media/igor/LaCie/UAV_Swarm_gazebo/catkin_ws/src/path_ta/flight_tasks/uav_0.py')
                    work = False
                elif (exit_p_num == 2):
                    rospy.loginfo("Exit from program")
                    self.set_mode("AUTO.LAND", 5)
                    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
                    self.set_arm(False, 5)
                    work = False
                elif (exit_p_num == 3):
                    rospy.loginfo("Return to launch")
                    self.set_mode("AUTO.RTL", 5)
                    work = False
                #elif (exit_p_num == 4):
                #    rospy.loginfo("This is coverage input: ")
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
                elif (exit_p_num != 1 or exit_p_num != 2 or exit_p_num != 3):
                    rospy.loginfo("Try again!")
                    work = True


if __name__ == '__main__':
    import rostest
    rospy.init_node('multiply_node_1', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest_1)