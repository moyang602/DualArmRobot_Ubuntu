#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <sys/socket.h>
#include <netinet/in.h>


// 与上位机UDP通讯变量声明
#define CTRLHOST_PORT 8000
#define CTRLHOST_IP "192.168.1.3"
#define DESTPC_PORT 8001
#define DESTPC_IP "192.168.1.4"
#define UDPCYCLE 1			// UDPCYCLE*程序运行周期 = UDP通讯周期


/***********************************************************
                      定义实际机器人数据结构
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct RealRobot_Struct
{
	float LeftArm[7];
	float RightArm[7];
	float LeftHand[4];
	float RightHand[4];
	float Head[2];
	float Waist[2];
};
#pragma pack(pop)


/***********************************************************
                      定义机器人回传数据
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct RobotDataUDP_Struct
{
	unsigned int TrustFlag;
	struct RealRobot_Struct RobotAngle;
	struct RealRobot_Struct RobotCurrent;
	struct RealRobot_Struct RobotAngleEX;
	float ForceL[6];
	float ForceR[6];
	float Posture[3];
	float GPS[2];
	float Voltage;
	float Current;
	uint8_t PowerStatus;
	uint8_t InitStatus;
	uint8_t ServoStatus;
	uint8_t MotionStatus;
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
                      定义单臂整体关节运动指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct SingleArmCMD_Struct
{
	unsigned char Mode;
	unsigned char ArmSelect;
	float Data[7];
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
	unsigned char MotionSpace;
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
struct RemoteDATA_Struct
{
	unsigned char Mode;
	short rocker[2];
	float Data[14];
	unsigned char CheckSum;
};
#pragma pack(pop)
/***********************************************************
                      定义遥操作控制指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct RemoteCMD_Struct
{
	unsigned char Mode;
	unsigned char Command;
	unsigned char CheckSum;
};
#pragma pack(pop)


/***********************************************************
                      定义控制指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct ControlCMD_Struct
{
	unsigned char Mode;
	unsigned char Command;
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义手部控制指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct HandCMD_Struct
{
	unsigned char Mode;
	unsigned char Command;
	unsigned char HandSelect;
	float DataL;
	float DataR;
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义单关节找零指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct FindHomeCMD_Struct
{
	unsigned char Mode;
	unsigned char CANCH;
	unsigned char CANID;
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义力控制指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct ForceCMD_Struct
{
	unsigned char Mode;
	unsigned char Command;
	unsigned char Param;
	float Data[6];
	unsigned char CheckSum;
};
#pragma pack(pop)

/***********************************************************
                      定义开门任务控制指令
************************************************************/
#pragma pack(push)
#pragma pack(1)
struct DutyCMD_Struct
{
	unsigned char Mode;
	unsigned char Command;
	unsigned char CheckSum;
};
#pragma pack(pop)

//函数声明
int RobotUDPComm_init(void);
int RobotFBSend(struct RobotDataUDP_Struct RobotFBData);
int UDPRecv(void);
int GetSingleJointData(long* can_channel_main,long* can_id_main,float* JointMoveData,double* JointMoveTime);
int GetSingleArmData(int* ArmSelect,float ArmMoveData[7], double* ArmMoveTime);
int GetEndData(int* ArmSelect, int* MotionSpace, float EndData[12], double* EndMoveTime);
int GetRemoteData(short *rockerL, short *rockerR, float RemoteMotionData[14]);
int GetRemoteCMD(void);
int GetControlCMD(void);
int GetHandCMD(int* HandSelect, float* HandAngleL, float* HandAngleR);
int GetForceCMD(int *ParamType, float ForceParam[7]);
int GetDutyCMD(void);
int GetFetchCMD(void);
int GetPlaceCMD(void);

#endif
