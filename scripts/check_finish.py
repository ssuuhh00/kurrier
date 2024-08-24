#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CollisionData
from std_msgs.msg import Bool

class FinishNode:
    def __init__(self):
        # check_finish 토픽 발행자 생성
        self.finish_pub = rospy.Publisher("/check_finish", Bool, queue_size=1)

        # 상태 플래그 초기화
        self.is_finished = False

        rospy.loginfo("FinishNode initialized. Ready to publish to /check_finish.")

    def publish_finish(self):
        rate = rospy.Rate(10)  # 10Hz로 발행
        while not rospy.is_shutdown():
            # check_finish 토픽에 is_finished 상태를 발행
            self.finish_pub.publish(self.is_finished)
            rate.sleep()

    def set_finished(self):
        # 외부에서 호출하여 상태를 True로 변경
        self.is_finished = True
        rospy.loginfo("Set /check_finish to True")

class CollisionHandler:
    def __init__(self):
        # collision_handler 노드 초기화
        rospy.init_node('collision_handler', anonymous=True)
        
        # 상태 플래그 초기화
        self.collision_detected = False

        # FinishNode 인스턴스 생성
        self.finish_node = FinishNode()

        # CollisionData 토픽 구독
        self.collision_sub = rospy.Subscriber("/CollisionData", CollisionData, self.collision_callback)
        
        # FinishNode의 publish_finish를 별도의 스레드에서 실행
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.spin()

    def timer_callback(self, event):
        # FinishNode의 publish_finish를 주기적으로 호출
        self.finish_node.publish_finish()

    def collision_callback(self, data):
        # CollisionData 메시지에서 collision_object를 확인
        if not self.collision_detected:
            for obj in data.collision_object:
                if obj.name == 'hole-cover':
                    rospy.loginfo(f"Collision with object '{obj.name}' detected. Setting finish state.")

                    # 상태 플래그 설정
                    self.collision_detected = True

                    # FinishNode 상태를 True로 설정
                    self.finish_node.set_finished()
                    break

if __name__ == '__main__':
    try:
        # CollisionHandler 인스턴스 생성 및 실행
        CollisionHandler()
    except rospy.ROSInterruptException:
        pass
