#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf

class IMUParser:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        sub = rospy.Subscriber("imu/data", Imu, self.callback)

    def first_topic_callback(self, data):
        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

    def second_msg_publish(self):

if __name__ == "__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass