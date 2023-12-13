import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

class judgement:
    def __init__(self):
        rospy.init_node('motor_speed')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rospy.Subscriber("status", Int64, self.status_CB)

        self.rate = rospy.Rate(50)

        self.cmd_msg = Twist()

    def status_CB(self,data):
        print(data)
        if data == 0:
            self.cmd_msg.linear.x = 0.3
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = 0.0
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = 0.0
        elif data == 1:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = 0.0
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = 0.15
        elif data == 2:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = 0.0
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = -0.15
        else:
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = 0.0
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = 0.0

        self.cmd_pub.publish(self.cmd_msg)
        self.rate.sleep()

def main():
    start = judgement()
    rospy.spin()

if __name__=="__main__":
    main()