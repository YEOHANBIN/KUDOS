import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Twist

class lane_detect:
    def __init__(self):
        rospy.init_node('lane_detection')
        #self.steer_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('status', Int64, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_CB)
        
        self.bridge = CvBridge()
        self.rate = rospy.Rate(100)
        
        #self.steer_msg = Twist()
        self.status_msg = Int64()
        
        self.left_top = (285, 270)
        self.left_bottom = (0, 480)
        self.right_bottom = (640, 480)
        self.right_top = (378, 270)
        self.src_points = np.float32([self.left_top,self.left_bottom,self.right_bottom,self.right_top])
        self.dst_points = np.float32([(160,0),(160,480),(480,480),(480,0)])
        
    def image_CB(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            height, width = frame.shape[:2]
            bird_view = self.bird_eye_view(frame, height, width)
            self.find_lanes(bird_view)

            cv2.imshow("OG_frame",frame)
            cv2.imshow('bird_view',bird_view)
            #self.status_pub.publish(self.status_msg.data)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
        except Exception as e:
            print("Error:", e)

    def histogram(self,frame):
        histogram_left = np.sum(frame[frame.shape[0]//2:,:frame.shape[1]//2], axis=0)
        histogram_right = np.sum(frame[frame.shape[0]//2:,frame.shape[1]//2:], axis=0)
        
        left_x_base = np.argmax(histogram_left)
        right_x_base = np.argmax(histogram_right)+frame.shape[1]//2
        
        return left_x_base,right_x_base
    
    def bird_eye_view(self,frame,height,width):
        M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        warp_frame = cv2.warpPerspective(frame,M,(width,height))
        
        return warp_frame
    
    def find_lanes(self,image):
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
            print("line found")
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)

                if x2 - x1 != 0:
                    slope = (y2-y1) / (x2-x1)
                    slope_degree = np.arctan(slope)*180/np.pi

                    data.append(x1)
                    #if abs(slope)>0.5:
                    #    self.status_msg.data = 0
                    

                #if abs(max(data)-min(data))<30:
                #    if min(data) > 320:
                #        self.steer_msg.data = self.aver_steer - 0.2
                #    else:
                #        self.steer_msg.data = self.aver_steer + 0.35
				#		# print("enter")
                #else:
                #    if abs(max(data)-320) > abs(320-min(data)):
                #        self.steer_msg.data = self.aver_steer + 0.2
				#		# print("check")
                #    elif abs(max(data)-320) < abs(320-min(data)):
                #        self.steer_msg.data = self.aver_steer - 0.2
						# print("double check")

                if abs(max(data)-min(data))<30:
                    if min(data) > 320:
                        self.status_msg = 0
                        steer_status = "Turn_Left"

                    else:
                        self.status_msg = 1
                        steer_status = "Go_straight"
                else:
                    if abs(max(data)-320) > abs(320-min(data)):
                        self.status_msg = 2
                        steer_status = "Turn_Right"
						# print("check")
                    elif abs(max(data)-320) < abs(320-min(data)):
                        self.status_msg = 3
                        steer_status = "Calculating"
						# print("double check")
        else:
            print("line not found")
        #print("steer_status",self.steer_msg.linear.x)
        print("steer_status",steer_status)
        #print("degree",slope_degree)
        #self.steer_pub.publish(self.steer_msg)
        self.status_pub.publish(self.status_msg)
        self.rate.sleep()
		#원본 이미지와 직선 이미지 합치기
        result = cv2.addWeighted(image,0.8,line_image,1,0)
        cv2.imshow("Hough_lane",result)
		
def main():
	start = lane_detect()
	rospy.spin()
	cv2.destroyAllWindows() 

if __name__=="__main__":
	main()