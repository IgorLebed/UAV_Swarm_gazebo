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

from re import T
import rospy

from geometry_msgs.msg import Point


def target_point():
    
    rospy.init_node('target_point')
    
    target_scout_publisher = rospy.Publisher('/scout0/target_point/position', Point, queue_size=10)
    # target_bomber1_publisher = rospy.Publisher('/bomber1/target_point/position', Point, queue_size=10)
    # target_bomber2_publisher = rospy.Publisher('/bomber2/target_point/position', Point, queue_size=10)
    # target_bomber3_publisher = rospy.Publisher('/bomber3/target_point/position', Point, queue_size=10)
    target_scout_point = Point()
    # target_bomber1_point = Point()
    # target_bomber2_point = Point()
    # target_bomber3_point = Point()
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown(): #check_input == True:

        target_scout_point.x = input("Input scout x: ")
        target_scout_point.y = input("Input scout y: ")
        target_scout_publisher.publish(target_scout_point)

        # target_bomber1_point.x = input("Input bomber1 x: ")
        # target_bomber1_point.y = input("Input bomber1 y: ")
        # target_bomber1_publisher.publish(target_bomber1_point)

        # target_bomber2_point.x = input("Input bomber2 x: ")
        # target_bomber2_point.y = input("Input bomber2 y: ")
        # target_bomber2_publisher.publish(target_bomber2_point)

        # target_bomber3_point.x = input("Input bomber3 x: ")
        # target_bomber3_point.y = input("Input bomber3 y: ")
        # target_bomber3_publisher.publish(target_bomber3_point)

        rate.sleep()


if __name__ == '__main__':
    try:
        target_point()
    except rospy.ROSInterruptException:
        pass
