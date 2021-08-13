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
from re import T, X
import rospy
from mavros_msgs.msg import MountControl
from mavros_msgs.msg import ActuatorControl

def gimbal_tallker():
    rospy.init_node('gimbal_tallker')
    gimbal_pub = rospy.Publisher('/mavros/mount_control/command', MountControl, queue_size=1)
    actuator_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)

    gimbal_= MountControl()
    actuator_ = ActuatorControl()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown(): 

        for i in range(1):
            
            x = input("Input pitch: ")
            y = input("Input Roll: ")
            z = input("Input Yaw: ")


            actuator_.controls[1] = x / 3.14 # Pitch
            actuator_.controls[2] = y / 3.14 # Yaw

            gimbal_.header.stamp = rospy.get_rostime()
            gimbal_.header.frame_id = "map"
            gimbal_.mode = 2
            gimbal_.pitch = x / 3.14 * 180 # Pitch
            gimbal_.roll = y / 3.14        # Roll
            gimbal_.yaw = z / 3.14 * 180   # Yaw

            gimbal_pub.publish(gimbal_)
            actuator_pub.publish(actuator_)
            rate.sleep()


if __name__ == '__main__':
    try:
        gimbal_tallker() 
    except rospy.ROSInterruptException:
        pass
