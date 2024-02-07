#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy		# ROS 라이브러리
import random		# 랜덤 수를 추출하기 위한 라이브러리
from tutorial.msg import first_msg	# first_msg.msg 파일 불러오기

def main():
    # 퍼블리시 노드 초기화
    rospy.init_node('start_node', anonymous=False)
    
    # 퍼블리셔 변수
    pub = rospy.Publisher('first_topic', first_msg, queue_size=10)
    
    # 1초마다 반복(변수=rate)
    rate = rospy.Rate(1) #1Hz

    msg = first_msg()	# 메시지 변수 선언
    count = 1

    # 중단되거나 사용자가 강제종료(ctrl+C) 전까지 계속 실행
    while not rospy.is_shutdown():
        # 메시지 내용 담기
        ## start_time: 메시지를 생성해 publish 하는 시각
        ## msg_seq: 메시지 순서(번호)
        ## original_num: 계산의 대상이 될 수로, 1부터 100까지의 수 중 무작위로 하나를 뽑음
        msg.start_time = rospy.Time.now()
        msg.msg_seq = count
        msg.original_num = random.randrange(1,101)

        # 터미널에 출력
        rospy.loginfo("------")
        rospy.loginfo("Start Time(sec): %d", msg.start_time.secs)
        rospy.loginfo("Message Sequence: %d", msg.msg_seq)
        rospy.loginfo("Original Number: %d", msg.original_num)
                
        # 메시지를 퍼블리시
        pub.publish(msg)
        
        # 정해둔 주기(hz)만큼 일시중단
        rate.sleep()

        count += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass