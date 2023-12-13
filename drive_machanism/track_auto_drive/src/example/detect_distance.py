# 171.0(mm) 아래로는 측정 안됨(0.0)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

average_color = [0,0,0]
pixel_count = 0
roi_top_left = (500,380)
roi_bottom_right = (780, 260)
roi_mode_T = (0,0)
roi_mode_B = (0,0)
roi_track_T = (0,0)
roi_track_B = (0,0)
running = True

def depth_callback(data):
    bridge = CvBridge()
    try:
        # 이미지 메시지를 OpenCV 이미지로 변환
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except Exception as e:
        print(e)
        return

    # 깊이 데이터 처리
    depth_array = np.array(depth_image, dtype=np.float32)
    # 여기서 depth_array를 사용하여 거리 정보를 처리하면 됨

    # 중앙 픽셀 좌표 계산
    height, width = depth_array.shape
    center_pixel_coordinates = depth_array[int(height / 2), int(width / 2)]

    print("Center Pixel Depth:", center_pixel_coordinates)

    ## 빨간색 사각형의 크기 정의
    #rectangle_size = 30

    ## 이미지에 중앙 픽셀 주변으로 빨간색 사각형 표시
    #cv2.rectangle(depth_image,
    #              (center_pixel_coordinates[0] - rectangle_size, center_pixel_coordinates[1] - rectangle_size),
    #              (center_pixel_coordinates[0] + rectangle_size, center_pixel_coordinates[1] + rectangle_size),
    #              (0, 0, 255), 2)  # 빨간색으로 표시, 두께는 2

    ## 이미지 표시
    #cv2.imshow("Depth Image with Center Pixel", depth_image)
    #cv2.waitKey(1)

def color_callback(data):
    global average_color, pixel_count, running

    bridge = CvBridge()
    try:
        # 이미지 메시지를 OpenCV 이미지로 변환
        color_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except Exception as e:
        print(e)
        return

    # 중앙 좌표 계산
    height, width, _ = color_image.shape
    center_pixel_coordinates = (int(width / 2), int(height / 2))

    # ROI 좌표 설정 (예: 왼쪽 상단에서 너비의 20%와 높이의 30% 지점부터 시작하여 너비와 높이의 60% 범위)
    roi_start = (int(0.4 * width), int(0.4 * height))
    roi_end = (int(0.6 * width), int(0.6 * height))

    # BGR을 HSV로 변환
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # 특정 영역을 추출
    roi_1 = hsv_image[roi_start[1]:roi_end[1], roi_start[0]:roi_end[0]]

    # ROI 영역 추출
    roi_2 = color_image[roi_start[1]:roi_end[1], roi_start[0]:roi_end[0]]

    # 픽셀 수 계산
    pixel_count_1 = roi_1.shape[0] * roi_1.shape[1]
    pixel_count_2 = roi_2.shape[0] * roi_2.shape[1]

    # 평균 색상값 계산
    average_color_hsv = np.mean(roi_1, axis=(0, 1))
    average_color_bgr = np.mean(roi_2, axis=(0, 1))

    # 출력
    print("Pixel Count_1:", pixel_count_1)
    print("Average Color:", average_color_hsv)
    print('\n')
    print("Pixel Count_1:", pixel_count_2)
    print("Average Color:", average_color_bgr)
    print('====================================================')

    # 빨간색 사각형의 크기 정의
    rectangle_size = 10

    # 이미지에 중앙 픽셀 주변으로 빨간색 사각형 표시
    cv2.rectangle(color_image,
                  (center_pixel_coordinates[0] - rectangle_size, center_pixel_coordinates[1] - rectangle_size),
                  (center_pixel_coordinates[0] + rectangle_size, center_pixel_coordinates[1] + rectangle_size),
                  (0, 0, 255), 2)  # 빨간색으로 표시, 두께는 2
    
    # 이미지에 중앙 픽셀 주변으로 빨간색 사각형 표시
    cv2.rectangle(color_image,
                  (int(0.4 * width), int(0.4 * height)),
                  (int(0.6 * width), int(0.6 * height)),
                  (0, 255, 0), 2)

    # 이미지 표시
    cv2.imshow("Color Image with Center Pixel", color_image)
    cv2.waitKey(1)

def listener():
    rospy.init_node('color_image_listener', anonymous=True)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

