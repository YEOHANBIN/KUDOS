import serial
import rospy
from std_msgs.msg import Int8

sendPacketInteger=15001500
mode_stack = [0,0]

def modeCB(data):
    global mode_stack
    mode = int(data.data)
    
    mode_stack[1] = mode

    if mode == 0: message = 15001530
    if mode == 1: message = 18001550
    if mode == 2: message = 12001550
    if mode == 3: message = 15001550
    if mode == 4: message = 17001515
    if mode == 5: message = 13001515
    if mode == 6: message = 15001500

    if mode_stack[0] != mode_stack[1]: message = 15001500 + ((message-15001500)/2)

    global sendPacketInteger
    sendPacketInteger = int(message)
    mode_stack[0] = mode_stack[1]
    

def talker():
    serialObject = serial.Serial('/dev/ttyUSB0','9600',parity=serial.PARITY_ODD, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('drive_mode', Int8, modeCB, queue_size=1)
    pub = rospy.Publisher('check_running', Int8, queue_size=10)
    rate = rospy.Rate(50)

    global sendPacketInteger

    while not rospy.is_shutdown():
        sendPacket=str(sendPacketInteger)+'\n'
        serialObject.write(sendPacket.encode())
        print('packet:', sendPacket, type(sendPacket))
        pub.publish(10)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()


    except rospy.ROSInterruptException:
        pass