import rospy
import numpy as np
from mavros_msgs.msg import ActuatorControl
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class Gimbal_test():

    def __init__(self,
                 ):

        self.message_pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=10)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)

        self.actuator_control_message = ActuatorControl()
        self.seq = 0
        self.local_position = PoseStamped()
        self.heading = 0.0

    def local_position_callback(self, data):
        self.local_position = data
        self.heading = self.quat2yaw(data.pose.orientation)

    def quat2yaw(self, this_quat):
        '''
        converts from a quaternion (developed using message type: posestamped.pose.orientation) to a yaw
        we assume that the UAV is level enough for this to work
        :param this_quat: 
        :return: 
        '''
        (_, _, yaw) = euler_from_quaternion([this_quat.x, this_quat.y, this_quat.z, this_quat.w])
        return yaw

    def run(self):

        r =rospy.Rate(1)
                        #   r    p    y?
        inputs = np.array((0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        while not rospy.is_shutdown():

            self.actuator_control_message.header.stamp = rospy.Time.now()
            self.actuator_control_message.header.seq = self.seq
            self.actuator_control_message.group_mix = 2 # ActuatorControl.PX4_MIX_PAYLOAD
            rospy.loginfo_throttle(3, inputs)
            self.actuator_control_message.controls = inputs
            self.message_pub.publish(self.actuator_control_message)

            self.seq = self.seq + 1
            r.sleep()


if __name__ == '__main__':

    rospy.init_node('gimbal_test', anonymous=True, log_level= rospy.INFO)
    gt = Gimbal_test()
    gt.run()
