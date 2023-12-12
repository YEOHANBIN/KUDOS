#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
                                // Uses DYNAMIXEL SDK library
//원하는 다이나믹셀의 위치값 설정
#define ADDR_PRO_TORQUE_ENABLE          24                // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_PRESENT_POSITION       2044

//열결한 다이나믹셀에 맞는 정보 입력
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define DXL_ID                          11                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0       // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4048           // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main(int argc, char **argv)
{
  //ROS 기본 생성 코드(노드명,nodeHandle 생성)
  ros::init(argc, argv, "dynamixel12_node");
  ros::NodeHandle nh;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  //PortHandler 초기화를 통한 포트 경로(DEVICENAME=ttyUSB0) 설정
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  //PacketHandler 초기화를 통한 프로토콜 버전(PROTOCOL_VERSION=2.0) 설정
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  //openPort()의 값이 1(true)인지 판단(포트 연결 여부 확인)
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  //setBaudRate(BAUDRATE)의 값이 1(true)인지 판단(BAUDRATE 설정 확인)
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  //write1ByteTxRx에 입력된 기기에 대한 정보로 dxl_comm_result 초기화,COMM_SUCCESS, dxl_error 여부 확인
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  while(1)
  {
    //esc를 누를 경우, while문 종료
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    //write4ByteTxRx에 입력된 기기에 대한 정보(목표값)로 dxl_comm_result 초기화,COMM_SUCCESS, dxl_error 여부 확인
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION,dxl_goal_position[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    usleep(10000);

    do
    {
       // Read present position
        //read4ByteTxRx에 입력된 기기에 대한 정보로 dxl_comm_result 초기화,COMM_SUCCESS, dxl_error 여부 확인 한번 무조건 실행 후 while문 실행
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID,dxl_goal_position[index], dxl_present_position);

    }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)); //goal position과 dxl present position의 차가 DXL MOVING STATUS THRESHOLD보다 큰 경우 반복

    // Change goal position
    // goal position의 maximum값과 minimum 값 번갈아가며 반복
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  // Disable Dynamixel Torque
  //write1ByteTxRx에 입력된 기기에 대한 정보로 dxl_comm_result 초기화,COMM_SUCCESS, dxl_error 여부 확인
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  //closePort() 실행으로 port 닫음
  portHandler->closePort();

  return 0;
}
