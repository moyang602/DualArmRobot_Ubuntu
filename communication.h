#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <sys/socket.h>
#include <netinet/in.h>


// 与上位机UDP通讯变量声明
#define CTRLHOST_PORT 8000
#define CTRLHOST_IP "192.168.1.3"
#define DESTPC_PORT 8001
#define DESTPC_IP "192.168.1.4"
#define UDPCYCLE 30			// UDPCYCLE*程序运行周期 = UDP通讯周期

/***********************************************************
                      定义机器人回传数据
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct RobotDataUDP_Struct
{
	unsigned int TrustFlag;
	float LeftArmAngle[7];
	float RightArmAngle[7];
	float LeftHandAngle[4];
	float RightHandAngle[4];
	float HeadAngle[2];
	float WaistAngle[2];
	float LeftArmCurrent[7];
	float RightArmCurrent[7];
	float LeftHandCurrent[4];
	float RightHandCurrent[4];
	float HeadCurrent[2];
	float WaistCurrent[2];
	float LeftArmAngleEX[7];
	float RightArmAngleEX[7];
	float LeftHandAngleEX[4];
	float RightHandAngleEX[4];
	float HeadAngleEX[2];
	float WaistAngleEX[2];
	unsigned int CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义单关节运动指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct SingleJointCMD_Struct
{
	unsigned char Mode;
	unsigned char CANCH;
	unsigned char CANID;
	float Data;
	float time;
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义末端运动指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct EndCMD_Struct
{
	unsigned char Mode;
	unsigned char ArmSelect;
	float Data[12];
	float time;
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义遥操作运动指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct RemoteCMD_Struct
{
	unsigned char Mode;
	float Data[14];
	unsigned char CheckSum;
};
#pragma pack(pop)

struct SingleJointCMD_Struct SingleJointData;
struct EndCMD_Struct EndData;
struct RemoteCMD_Struct RemoteData;

int UDP_Sock;
struct sockaddr_in DestPCAddr;


//函数声明
int RobotUDPComm_init(void);
int RobotFBSend(struct RobotDataUDP_Struct RobotFBData);
int UDPRecv(void);
int GetSingleJointData(long* can_channel_main,long* can_id_main,float* JointMoveData,double* JointMoveTime);
int GetEndData(int* ArmSelect,float EndData[12], double* EndMoveTime);
int GetRemoteData(float RemoteMotionData[14]);

#endif
