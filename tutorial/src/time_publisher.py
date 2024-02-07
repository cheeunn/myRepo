#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def talker():
	pub = rospy.Publisher('my_time', Float64, queue_size=10)
	rospy.init_node('talker_node', anonymous=True)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		pub_time = rospy.get_time()
		rospy.loginfo("Published time : %f", pub_time)
		pub.publish(pub_time)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass