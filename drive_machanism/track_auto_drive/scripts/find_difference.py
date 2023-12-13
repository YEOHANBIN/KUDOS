#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8

roi_track_T = (int(500),int(340))
roi_track_B = (int(600),int(360))

class TrackFollower:
    def __init__(self):
        rospy.init_node('track_follower', anonymous=True)
        self.bridge = CvBridge()

        # 이미지 및 트위스트 메시지 발행
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/image_info', Int8, queue_size=10)

        self.blue_lower = np.array([70, 50, 145])
        self.blue_upper = np.array([95, 220, 168])

        self.left_top = (550, 400)
        self.left_bottom = (400, 720)
        self.right_bottom = (880, 720)
        self.right_top = (730, 400)
        self.src_points = np.float32([self.left_top,self.left_bottom,self.right_bottom,self.right_top])
        self.dst_points = np.float32([(200,0),(200,720),(1080,720),(1080,0)])    

    def image_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            height, width = cv_image.shape[:2]
            bird_view = self.bird_eye_view(cv_image, height, width)

            cv2.imshow('bird_view',bird_view)
        except CvBridgeError as e:
            print(e)
            return
        
        roi_track = cv_image[roi_track_T[1]:roi_track_B[1], roi_track_T[0]:roi_track_B[0]]


        # 트랙 찾기
        deviation = self.find_track(bird_view, self.blue_lower, self.blue_upper)

        # 트랙이 감지된 경우
        if deviation is not None:
            rospy.loginfo("Deviation: %f", deviation)

            # 일정 이상의 트랙 중앙에서의 차이가 발생하면 이를 보정하여 조향 등을 조절할 수 있음
            if abs(deviation) > 500:
                image_info = Int8()
                image_info = 0 if deviation < 0 else 1
                self.pub.publish(image_info)
        
        cv2.imshow("Track Detect", cv_image)
        cv2.waitKey(1)

    def bird_eye_view(self,frame,height,width):
        M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)           #perspective matrix
        warp_frame = cv2.warpPerspective(frame,M,(width,height))
        
        return warp_frame

    def find_track(self, image, target_color_lower, target_color_upper):
        # BGR 이미지를 HSV로 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 지정한 색상 범위에 해당하는 부분을 추출
        mask = cv2.inRange(hsv, target_color_lower, target_color_upper)

        # 추출된 부분의 중심 좌표 계산
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # 가장 큰 외곽선(트랙) 선택
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)

            if M['m00'] != 0:
                # 중심 좌표 계산
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 이미지 중앙 좌표
                image_center_x = image.shape[1] // 2

                # 중심 좌표와 이미지 중앙 좌표의 차이 계산
                deviation = cx - image_center_x

                return deviation
        return None

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down...")

if __name__ == '__main__':
    try:
        follower = TrackFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
