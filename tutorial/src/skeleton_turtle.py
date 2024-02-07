#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

KV = 1.5
KA = 6
TOLERANCE = 0.1 # 목표와 0.1이내로 거리가 들어오면 goal에 도달한 것으로 처리합니다.

class TurtleBot:

    def __init__(self):
        """
        1. initnode, Publisher, Subscriber 모두 선언해줍니다. 
        2. 거북이의 pose를 담고있을 변수인 self.pose를 생성합니다.
        3. 10Hz로 topic을 Publish할 self.rate를 선언합니다.다
        """
        rospy.init_node("ctrl_turtle", anonymous=False)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)
        self.pose =  Pose() # 거북이 포즈 담고 있을 거니까
        self.rate = rospy.Rate(10)


    def callback(self, data): 
        """
        해당 callback 함수에서 거북이의 pose값을 업데이트합니다.
        pose값에 subscribe 받은 x,y 값을 round 함수를 이용하여 소수점 4자리까지 저장합니다.
        """
        self.pose = data # 일단 self.pose에 값들을 모두 받아옵니다. 
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)




    def euclidean_distance(self, goal_pose): # 목표점과 거북이와의 유클리드 거리 
        return sqrt(pow((goal_pose[0] - self.pose.x), 2) +
                    pow((goal_pose[1] - self.pose.y), 2))

    def linear_vel(self, goal_pose): # 속도값을 P제어로 결정
        return KV * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose): # 목표 theta는 목표점과 거북이의 일직선상 경로의 각도로 설정
        return atan2(goal_pose[1] - self.pose.y, goal_pose[0] - self.pose.x)

    def angular_vel(self, goal_pose): # 목표 theta와 현재 거북이 theta의 오차에 P제어로 각속도 결정
        return KA * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):

        """
        거북이가 이동할 path를 list의 형태로 input 받습니다. 
        ex) [1,2] [3,4] [10,8] 
        -> [1,2]좌표로 이동 후 [3,4]로 이동 후 [10,8]으로 이동완료하면 종료
        그리고 해당 좌표들을 paths라는 list에 저장합니다.
        위 예시의 경우는 "paths = [[1,2],[3,4],[10,8]]"이 되겠네요. 
        """
        ################# path input을 받아오는 부분. 수정 금지 ###########################
        inputs = input("Set your path for [x,y] : ").split()
        paths = [list(map(int, element.strip("[]").split(','))) for element in inputs]
        ##############################################################################

        """
        
        제어값을 publish 하는 부분을 작성하면 됩니다.아래 순서대로 차근차근 작성해봅시다.

        1. paths안에 있는 각각의 목표 점들에 대한 반복문을 설정합니다.
        2. 2중 반복문으로, "euclidean_distance" 함수의 결과값이 TOLERANCE보다 작거나 같을때까지 while문을 설정합니다.
        3. while문 안에서는 "linear_vel", "angular_vel" 함수를 이용하여 선속도, 각속도를 결정합니다.
        4. 함수들을 통해 결정된 제어값들을 publish 해주고, sleep을 진행해줍니다.
        4. 한 goal에 도달할때마다 rospy.loginfo를 이용하여 " Goal ! " 을 출력해줍니다.
        5. 주어진 모든 path들을 모두 도달했다면 마찬가지로 " Finish ! " 를 출력하고 종료합니다.

        """
        cmd_vel = Twist()
        for goal in paths:
            while self.euclidean_distance(goal) > TOLERANCE and not rospy.is_shutdown():
                cmd_vel.linear.x = self.linear_vel(goal)
                cmd_vel.linear.y = self.linear_vel(goal)
                self.pub.publish(cmd_vel)
                self.rate.sleep()
                rospy.loginfo("Goal !")
            rospy.loginfo("Finish !")


"""
class를 불러와서 코드를 실행시키는 main문을 최종적으로 작성해주세요 !
"""
if __name__ == "__main__" :
    turtle1 = TurtleBot
    turtle1.move2goal;()