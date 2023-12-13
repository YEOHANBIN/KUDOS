import serial
import rospy
import time
from std_msgs.msg import Int8

sendPacketInteger=15001500
message = 15001500
slope_flag = 0
image_stack = [0,0]
mode_stack = [0,0]
mode_list = [0,0,0,0,0,0]
serial_directory = {
    "Go":15001530,
    "Head_up":15001550,
    "Left":16001515,
    "Right":14001515,
    "Slope_L":17001550,
    "Slope_R":13001550,
    "Stop":15001500,
    "Turn_L":15201515,
    "Turn_R":14801515,
    "BACK":15001470
}
run_time = 2

def STOP():
    global sendPacketInteger
    sendPacketInteger = int(serial_directory.get("Stop"))

def BACK():
    global sendPacketInteger, run_time
    sendPacketInteger = int(serial_directory.get("BACK"))

def SLOPE_R():
    global sendPacketInteger, message, slope_flag

    slope_flag += 1
    if slope_flag < 3:
        message = serial_directory["Go"]
    elif slope_flag < 5:
        message = serial_directory["Turn_R"]
    else:
        slope_flag = 0

def SLOPE_L():
    global sendPacketInteger, message, slope_flag

    slope_flag += 1
    if slope_flag < 3:
        message = serial_directory["Go"]
    elif slope_flag < 5:
        message = serial_directory["Turn_L"]
    else:
        slope_flag = 0

def modeCB(data):
    global mode_stack, message
    mode = int(data.data)
    
    mode_stack[1] = mode

    if image_stack[0] > 10 or image_stack[1] > 10:
        if image_stack[0] > 10:
            message = serial_directory["Turn_R"]
            image_stack[0] = 0
        if image_stack[1] > 10:
            message = serial_directory["Turn_L"]
            image_stack[1] = 0
    else:
        if mode == 0: message = 15001530
        if mode == 1: SLOPE_R()
        if mode == 2: SLOPE_L()
        if mode == 3: message = 15001550
        if mode == 4: message = 15001500

        if mode_stack[0] != mode_stack[1]: message = 15001500 + ((message-15001500)/2)

    global sendPacketInteger
    sendPacketInteger = int(message)
    mode_stack[0] = mode_stack[1]

def imageCB(data):
    global image_stack
    image_mode = int(data.data)

    if image_mode == 0: image_stack[0] += 1
    else: image_stack[1] += 1


def talker():
    #serialObject = serial.Serial('/dev/ttyUSB0','9600',parity=serial.PARITY_ODD, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('IMU_CB', Int8, modeCB, queue_size=1)
    rospy.Subscriber('image_info', Int8, imageCB, queue_size=1)
    pub = rospy.Publisher('check_running', Int8, queue_size=10)
    rate = rospy.Rate(50)

    global sendPacketInteger

    while not rospy.is_shutdown():


        sendPacket=str(sendPacketInteger)+'\n'
        #serialObject.write(sendPacket.encode())
        print('packet:', sendPacket, type(sendPacket))
        pub.publish(10)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()


    except rospy.ROSInterruptException:
        pass