#!/usr/bin/env python3

import serial
import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

sendPacketInteger = 15001500
mode_stack = [0,0,0,0,0]           # 0: 직진    1: 경사로_우커브    2: 경사로_좌커브    3: 언덕
change_mode = 100

def imuCB(data):
    #(f"lin_acc_x: {data.linear_acceleration.x}     lin_acc_x: {data.linear_acceleration.y}")
    if data.linear_acceleration.z >5.0:
        mode_stack[4] += 1
    elif abs(data.linear_acceleration.x) > 1.5:
        if data.linear_acceleration.x < 0:
            mode_stack[1] += 1
        else:
            mode_stack[2] += 1
    elif data.linear_acceleration.z > 2.0:
        #print("Go_straight")
        mode_stack[3] += 1
    else:
        mode_stack[0] += 1

def talker():
    rospy.init_node('imu_node', anonymous = True)
    rospy.Subscriber("/camera/imu", Imu, imuCB, queue_size=1)
    pub = rospy.Publisher('IMU_CB', Int8, queue_size =1)
    rate = rospy.Rate(50)

    global sendPacketInteger
    global mode_stack

    while not rospy.is_shutdown():
        #print(f'[1]: {mode_stack[0]}    [2]: {mode_stack[1]}    [3]: {mode_stack[2]}')
        if mode_stack[0] > change_mode:
            print("Go_straight")
            pub.publish(0)
            mode_stack = [0,0,0,0,0]
            rate.sleep()
        if mode_stack[1] > change_mode:
            print("Slope_up_to_left")
            pub.publish(1)
            mode_stack = [0,0,0,0,0]
            rate.sleep()
        if mode_stack[2] > change_mode:
            print("Slope_up_to_right")
            pub.publish(2)
            mode_stack = [0,0,0,0,0]
            rate.sleep()
        if mode_stack[3] > change_mode:
            print("Just_GO")
            pub.publish(3)
            mode_stack = [0,0,0,0,0]
            rate.sleep()
        if mode_stack[4] > change_mode:
            print("STOP")
            pub.publish(6)
            mode_stack = [0,0,0,0,0]
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    
    except rospy.ROSInterruptException: pass