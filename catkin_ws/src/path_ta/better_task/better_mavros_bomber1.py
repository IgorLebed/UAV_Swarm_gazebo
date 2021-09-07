# Embedded file name: /media/igor/LaCie/UAV_Swarm_gazebo/catkin_ws/src/path_ta/flight_tasks/mavros_0.py
from __future__ import division
import unittest
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelProperties, SpawnModel, GetWorldProperties, GetLinkState, DeleteModel
from gazebo_msgs.msg import ModelState

from pymavlink import mavutil
from rospy.exceptions import ROSInternalException
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange
from geometry_msgs.msg import Point as P
from std_msgs.msg import Float32, Int64MultiArray, Bool

class MavrosTestCommon(unittest.TestCase):
    goal_pose_x = None
    goal_pose_y = None

    def __init__(self, *args):
        super(MavrosTestCommon, self).__init__(*args)

        self.UPPER_DAMAGE_LIMIT = 0.9
        self.total_damage = 0.0001
        self.LOWER_BATTARY_LIMIT = 5 
        self.total_battery = 100

    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        #test
        self.local_scout0_position = PoseStamped()
        self.scout0_check = PoseStamped()
        self.damage = Float32()
        self.scout0_state = State()
        self.personal_land = PoseStamped()
        #-----
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None
        self.target_sub = P()
        self.start_mission = Bool()
        self.scout0_follower_mode = Bool()
        self.sub_topics_ready = {key:False for key in ['alt',
         'ext_state',
         'global_pos',
         'home_pos',
         'local_pos',
         'mission_wp',
         'state',
         'imu',
         'target_pose',
         'local_scout0_pos',
         'scout0_state',
         'scout0_check',
         'damage',
         'personal_land',
         'scout0_start_mission',
         'scout0_follower_mode']}
        service_timeout = 30
        rospy.loginfo('waiting for ROS services')
        try:
            rospy.wait_for_service('/bomber1/mavros/param/get', service_timeout)
            rospy.wait_for_service('/bomber1/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('/bomber1/mavros/mission/push', service_timeout)
            rospy.wait_for_service('/bomber1/mavros/mission/clear', service_timeout)
            rospy.wait_for_service('/bomber1/mavros/set_mode', service_timeout)
            rospy.loginfo('ROS services are up')
        except rospy.ROSException:
            self.fail('failed to connect to services')

        self.get_param_srv = rospy.ServiceProxy('/bomber1/mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('/bomber1/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/bomber1/mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('/bomber1/mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('/bomber1/mavros/mission/push', WaypointPush)
        self.alt_sub = rospy.Subscriber('/bomber1/mavros/altitude', Altitude, self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('/bomber1/mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('/bomber1/mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('/bomber1/mavros/imu/data', Imu, self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('/bomber1/mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('/bomber1/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber('/bomber1/mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('/bomber1/mavros/state', State, self.state_callback)
        self.target_sub = rospy.Subscriber('/scout0/target_point/position', P, self.goal_pos_callback)

        #test swarm
        self.state_scout0_sub = rospy.Subscriber('/scout0/mavros/state', State, self.state_scout0_callback)
        self.local_scout0_pos_sub = rospy.Subscriber('/scout0/mavros/local_position/pose', PoseStamped, self.local_scout0_position_callback)
        self.scout0_check_sub = rospy.Subscriber('/scout0/mavros/check_mission', PoseStamped, self.scout0_check_callback)
        self.def_prob_sub = rospy.Subscriber('/bomber1/damage', Float32, self.damage_callback)
        self.personal_land_sub = rospy.Subscriber('/bomber1/mavros/personal_land/check_mission', PoseStamped, self.personal_land_callback)
        self.start_mission_sub = rospy.Subscriber('/scout0/start_mission', Bool, self.start_mission_callback)
        self.follower_mode_sub = rospy.Subscriber('/scout0/follower_mode', Bool, self.follower_callback)
        #---
        return

    def tearDown(self):
        self.log_topic_vars()
    #
    # ----------------My_Callback-------------------
    #
    def damage_callback(self, data):
        self.damage = data
        if not self.sub_topics_ready['damage']:
            self.sub_topics_ready['damage'] = True

    def goal_pos_callback(self, data):
        self.goal_pose_x = data.x
        self.goal_pose_y = data.y
   
    def personal_land_callback(self, data):
        self.personal_land = data
        if not self.sub_topics_ready['personal_land']:
            self.sub_topics_ready['personal_land'] = True
    
    def start_mission_callback(self, data):
        self.start_mission = data
        if not self.sub_topics_ready['scout0_start_mission']:
            self.sub_topics_ready['scout0_start_mission'] = True
    
    def follower_callback(self, data):
        self.scout0_follower_mode = data
        if not self.sub_topics_ready['scout0_follower_mode']:
            self.sub_topics_ready['scout0_follower_mode'] = True
    
    #
    # --------------Standart_Callback------------
    #
    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo('VTOL state changed from {0} to {1}'.format(mavutil.mavlink.enums['MAV_VTOL_STATE'][self.extended_state.vtol_state].name, mavutil.mavlink.enums['MAV_VTOL_STATE'][data.vtol_state].name))
        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo('landed state changed from {0} to {1}'.format(mavutil.mavlink.enums['MAV_LANDED_STATE'][self.extended_state.landed_state].name, mavutil.mavlink.enums['MAV_LANDED_STATE'][data.landed_state].name))
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data
        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo('current mission waypoint sequence updated: {0}'.format(data.current_seq))
        self.mission_wp = data
        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo('armed state changed from {0} to {1}'.format(self.state.armed, data.armed))
        if self.state.connected != data.connected:
            rospy.loginfo('connected changed from {0} to {1}'.format(self.state.connected, data.connected))
        if self.state.mode != data.mode:
            rospy.loginfo('mode changed from {0} to {1}'.format(self.state.mode, data.mode))
        if self.state.system_status != data.system_status:
            rospy.loginfo('system_status changed from {0} to {1}'.format(mavutil.mavlink.enums['MAV_STATE'][self.state.system_status].name, mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    #----scout0 callback------
    def state_scout0_callback(self, data):
        #if self.scout0_state.armed != data.armed:
            #rospy.loginfo('armed scout0_state changed from {0} to {1}'.format(self.scout0_state.armed, data.armed))
        #if self.scout0_state.connected != data.connected:
            #rospy.loginfo('connected changed from {0} to {1}'.format(self.scout0_state.connected, data.connected))
        #if self.scout0_state.mode != data.mode:
            #rospy.loginfo('mode changed from {0} to {1}'.format(self.scout0_state.mode, data.mode))
        #if self.scout0_state.system_status != data.system_status:
            #rospy.loginfo('system_status changed from {0} to {1}'.format(mavutil.mavlink.enums['MAV_STATE'][self.scout0_state.system_status].name, mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))
        self.scout0_states = data
        if not self.sub_topics_ready['scout0_state'] and data.connected:
            self.sub_topics_ready['scout0_state'] = True
    
    def local_scout0_position_callback(self, data):
        self.local_scout0_position = data
        if not self.sub_topics_ready['local_scout0_pos']:
            self.sub_topics_ready['local_scout0_pos'] = True

    def scout0_check_callback(self, data):
        self.scout0_check = data
        if not self.sub_topics_ready['scout0_check']:
            self.sub_topics_ready['scout0_check'] = True

    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo('setting FCU arm: {0}'.format(arm))
        old_arm = self.state.armed
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo('set arm success | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr('failed to send arm command')
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(arm_set, 'failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}'.format(arm, old_arm, timeout))


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo('setting FCU mode: {0}'.format(mode))
        old_mode = self.state.mode
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo('set mode success | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr('failed to send mode command')
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(mode_set, 'failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}'.format(mode, old_mode, timeout))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo('waiting for subscribed topics to be ready')
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all((value for value in self.sub_topics_ready.values())):
                simulation_ready = True
                rospy.loginfo('simulation topics ready | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(simulation_ready, 'failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}'.format(self.sub_topics_ready, timeout))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo('waiting for landed state | state: {0}, index: {1}'.format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name, index))
        loop_freq = 10
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo('landed state confirmed | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(landed_state_confirmed, 'landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}'.format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name, mavutil.mavlink.enums['MAV_LANDED_STATE'][self.extended_state.landed_state].name, index, timeout))

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo('waiting for VTOL transition | transition: {0}, index: {1}'.format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name, index))
        loop_freq = 10
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo('transitioned | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                transitioned = True
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(transitioned, 'transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3}'.format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name, mavutil.mavlink.enums['MAV_VTOL_STATE'][self.extended_state.vtol_state].name, index, timeout))

    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo('clear waypoints success | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr('failed to send waypoint clear command')
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(wps_cleared, 'failed to clear waypoints | timeout(seconds): {0}'.format(timeout))

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo('sending mission waypoints')
        if self.mission_wp.waypoints:
            rospy.loginfo('FCU already has mission waypoints')
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo('waypoints successfully transferred')
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            elif len(waypoints) == len(self.mission_wp.waypoints):
                rospy.loginfo('number of waypoints transferred: {0}'.format(len(waypoints)))
                wps_verified = True
            if wps_sent and wps_verified:
                rospy.loginfo('send waypoints success | seconds: {0} of {1}'.format(i / loop_freq, timeout))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(wps_sent and wps_verified, 'mission could not be transferred and verified | timeout(seconds): {0}'.format(timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo('waiting for MAV_TYPE')
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo('MAV_TYPE received | type: {0} | seconds: {1} of {2}'.format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type].name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(res.success, 'MAV_TYPE param get failed | timeout(seconds): {0}'.format(timeout))
    #---------Del_Sub_After_Critical_Damage-------
    """
    FIXME: Add all sub
    """
    def damage_calculate(self): 
        try: 
            self.total_damage += self.damage.data
            if self.total_damage >= self.UPPER_DAMAGE_LIMIT:
                rospy.loginfo("ral_x60" + ' was exploded!')
                #self.set_model_state()
                #self.unregister_subs()
                #del self
            else:
                rospy.loginfo("ral_x60" + ' current def prob value: ' + str(self.total_damage))
        except ROSInternalException:
            rospy.loginfo(("Error damage"))

    def unregister_subs(self):
        self.def_prob_sub.unregister()
        self.scout0_exit_mode_sub.unregister()
        self.state_sub.unregister()
        self.target_sub.unregister()
        self.mission_wp_sub.unregister()
        self.home_pos_sub.unregister()
        self.imu_data_sub.unregister()
        self.global_pos_sub.unregister()
        self.ext_state_sub.unregister()
        self.alt_sub.unregister()

    def set_model_state(self):
        self.state_msg = self.prepare_model_state_msg()
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(self.state_msg)
        except:
            print("Service call failed: ")

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo('========================')
        rospy.loginfo('===== topic values =====')
        rospy.loginfo('========================')
        rospy.loginfo('altitude:\n{}'.format(self.altitude))
        rospy.loginfo('========================')
        rospy.loginfo('extended_state:\n{}'.format(self.extended_state))
        rospy.loginfo('========================')
        rospy.loginfo('global_position:\n{}'.format(self.global_position))
        rospy.loginfo('========================')
        rospy.loginfo('home_position:\n{}'.format(self.home_position))
        rospy.loginfo('========================')
        rospy.loginfo('local_position:\n{}'.format(self.local_position))
        rospy.loginfo('========================')
        rospy.loginfo('mission_wp:\n{}'.format(self.mission_wp))
        rospy.loginfo('========================')
        rospy.loginfo('state:\n{}'.format(self.state))
        rospy.loginfo('========================')
        rospy.loginfo('target_position:\n{}'.format(self.target_sub))
        rospy.loginfo('========================')

        rospy.loginfo('damage\n{}'.format(self.damage))
        rospy.loginfo('========================')