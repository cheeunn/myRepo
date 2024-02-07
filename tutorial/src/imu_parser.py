#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    quar_x = data.orientation.x
    quar_y = data.orientation.y
    quar_z = data.orientation.z
    quar_w = data.orientation.w

    print(quar_x, quar_y, quar_z, quar_w)

def listener():
    rospy.Subscriber("imu/data", Imu, callback)
    rospy.init_node("imu_parser_node",anonymous=True)

    rospy.spin()

if __name__ == "__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass