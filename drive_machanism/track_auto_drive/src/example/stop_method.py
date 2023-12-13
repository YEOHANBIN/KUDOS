#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class KeyboardController:
    def __init__(self):
        rospy.init_node('keyboard_controller_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.running = True

        # 키 입력을 받을 토픽 구독자
        rospy.Subscriber('/keyboard_input', String, self.keyboard_callback)

    def keyboard_callback(self, data):
        user_input = data.data.lower()

        if user_input == 'p':
            self.running = not self.running
            if self.running:
                rospy.loginfo("Resuming...")
            else:
                rospy.loginfo("Pausing...")

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                rospy.loginfo("Node is running...")
                # 여기에 반복하고자 하는 작업을 추가할 수 있습니다.

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
