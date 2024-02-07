#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Turtle_sim :
    def __init__(self) :
        rospy.init_node("turtle_goal", anonymous=False)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)


        rate = rospy.Rate(10)
        while not rospy.is_shutdown() :
            self.pubb()
            rate.sleep()

    def callback(self, data) :
        rospy.loginfo(data)

    def pubb(self) :
        cmd_vel = Twist()
        cmd_vel.linear.x = 1
        cmd_vel.angular.z = 2
        self.pub.publish(cmd_vel)

if __name__ == "__main__" :
    Turtle_sim()