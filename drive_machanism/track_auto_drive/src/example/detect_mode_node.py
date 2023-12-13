#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Int8

bridge = CvBridge()

mode_stack = [0,0,0,0,0,0]
change_mode = 100

left_top = (500, 100)
left_bottom = (0, 720)
right_bottom = (1280, 720)
right_top = (780, 100)
src_points = np.float32([left_top,left_bottom,right_bottom,right_top])
dst_points = np.float32([(300,0),(300,720),(980,720),(980,0)])

def imuCB(data):
    if data.linear_acceleration.x < -3.0:
        #print("Go_straight")
        mode_stack[3] += 1
    else:
        if data.linear_acceleration.y > 1.5:
            #print("Slope_up_to_left")
            mode_stack[1] += 1
        elif data.linear_acceleration.y < -1.5:
            #print("Slope_up_to_right")
            mode_stack[2] += 1
        else:
            #print("Go_straight")
            mode_stack[0] += 1

def bird_eye_view(frame,height,width):
        M = cv2.getPerspectiveTransform(src_points,dst_points)           #perspective matrix
        warp_frame = cv2.warpPerspective(frame,M,(width,height))
        
        return warp_frame

def find_lanes(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        height, width = edges.shape[:2]
        roi_vertices = [(0,380),(0,260),(width,260),(width,380)]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], np.int32), 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)

        data = []
        steer_status = ""
        line_image = np.zeros_like(image)

        if lines is not None:
            #print("line found")
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)

                if x2 - x1 != 0:
                    slope = (y2-y1) / (x2-x1)
                    slope_degree = np.arctan(slope)*180/np.pi

                    data.append(x1)

                if abs(max(data)-min(data))<30:
                    if min(data) > 320:
                        mode_stack[4] += 1
                        #steer_status = "Turn_Left"

                    else:
                        #self.status_msg = 1
                        #steer_status = "Go_straight"
                        pass
                else:
                    if abs(max(data)-320) > abs(320-min(data)):
                        mode_stack[5] += 1
                        #steer_status = "Turn_Right"
						# print("check")
                    #elif abs(max(data)-320) < abs(320-min(data)):
                    #    self.status_msg = 3
                    #    steer_status = "Calculating"
						# print("double check")
        else:
            pass
            #print("line not found")
        #print("steer_status",self.steer_msg.linear.x)
        #print("steer_status",steer_status)
        #print("degree",slope_degree)
        #self.steer_pub.publish(self.steer_msg)
        #self.status_pub.publish(self.status_msg)
        #self.rate.sleep()
		#원본 이미지와 직선 이미지 합치기
        result = cv2.addWeighted(image,0.8,line_image,1,0)
        cv2.imshow("Hough_lane",result)

def imageCB(data):
        try:
            frame = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            height, width = frame.shape[:2]
            bird_view = bird_eye_view(frame, height, width)
            find_lanes(bird_view)

            cv2.imshow("OG_frame",frame)
            cv2.imshow('bird_view',bird_view)
            #self.status_pub.publish(self.status_msg.data)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
        except Exception as e:
            print("Error:", e)

def detection():
    rospy.init_node("drive_mode_node", anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imuCB, queue_size=1)
    rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    pub = rospy.Publisher('drive_mode', Int8, queue_size=1)
    rate = rospy.Rate(100)
    rate_2 = rospy.Rate(5)

    global mode_stack

    while not rospy.is_shutdown():
        #print(f'[1]: {mode_stack[0]}    [2]: {mode_stack[1]}    [3]: {mode_stack[2]}    [4]: {mode_stack[3]}    [5]: {mode_stack[4]}')
        if mode_stack[0] > change_mode:
            mode_stack[4], mode_stack[5] = 0, 0
            rate_2.sleep()
            if mode_stack[4] > change_mode:
                print("Turn_left")
                pub.publish(4)
                mode_stack[4], mode_stack[5] = 0, 0
                rate.sleep()
            elif mode_stack[5] > change_mode:
                print("Turn_right")
                pub.publish(5)
                mode_stack[4], mode_stack[5] = 0, 0
                rate.sleep()
            else:
                print("Go_straight")
                pub.publish(0)
                mode_stack = [0,0,0,0,0,0]
                rate.sleep()
        if mode_stack[1] > change_mode:
            print("Slope_up_to_left")
            pub.publish(1)
            mode_stack = [0,0,0,0,0,0]
            rate.sleep()
        if mode_stack[2] > change_mode:
            print("Slope_up_to_right")
            pub.publish(2)
            mode_stack = [0,0,0,0,0,0]
            rate.sleep()
        if mode_stack[2] > change_mode:
            print("Just_Go")
            pub.publish(3)
            mode_stack = [0,0,0,0,0,0]
            rate.sleep()
        
        #cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        detection()
    
    except rospy.ROSInterruptException: pass    