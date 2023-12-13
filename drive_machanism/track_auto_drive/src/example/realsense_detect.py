import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Twist

left_right = [0,0,0]
change_mode = 20

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
        
        self.left_top = (500, 100)
        self.left_bottom = (0, 720)
        self.right_bottom = (1280, 720)
        self.right_top = (780, 100)
        self.src_points = np.float32([self.left_top,self.left_bottom,self.right_bottom,self.right_top])
        self.dst_points = np.float32([(300,0),(300,720),(980,720),(980,0)])                                 #self.dst_points = np.float32([(160,0),(160,480),(480,480),(480,0)])
        
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
        M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)           #perspective matrix
        warp_frame = cv2.warpPerspective(frame,M,(width,height))
        
        return warp_frame
    
    def find_lanes(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        height, width = edges.shape[:2]
        roi_vertices = [(500,200),(500,520),(780,520),(780,200)]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], np.int32), 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)

        global left_right
        data = []
        steer_status = ""
        line_image = np.zeros_like(image)

        if lines is not None:
            print("line found")
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)

                slope_degree = 0

                if x2 - x1 != 0:
                    slope = (y2-y1) / (x2-x1)
                    slope_degree = np.arctan(slope)*180/np.pi
                    print(f"Degree: {slope_degree}")

                    #data.append(x1)
                    #if abs(slope)>0.5:
                    #    self.status_msg.data = 0

                if abs(slope_degree) > 40.0:
                    if slope_degree < 0: left_right[0] += 1     #print("Turn_left")
                    else: left_right[1] += 1        #print("Turn_right")
                #else: print("GO_straight")
                
        else:
            print("line not found")

        if left_right[0] > change_mode:
            self.status_pub.publish(0)
            left_right = [0,0,0]
        if left_right[1] > change_mode:
            self.status_pub.publish(1)
            left_right = [0,0,0]
    
        #print("steer_status",steer_status)
        #self.status_pub.publish(self.status_msg)
        print(f"left: {left_right[0]}   right: {left_right[1]}\n===========================================================")
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