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

// 与力传感器TCP通讯变量声明
#define ForceServer_Port 4008
#define ForceServer_IP "192.168.0.108"
#define ForceHOST_PORT 5008
#define ForceHOST_IP	"192.168.0.2"
#define RX_BUFFER_SIZE	16384
#define dataBufferLen 8192
#define M812X_CHN_NUMBER 6


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


//函数声明
int RobotUDPComm_init(void);
int RobotFBSend(struct RobotDataUDP_Struct RobotFBData);
int UDPRecv(void);
int GetSingleJointData(long* can_channel_main,long* can_id_main,float* JointMoveData,double* JointMoveTime);
int GetSingleArmData(int* ArmSelect,float ArmMoveData[7], double* ArmMoveTime);
int GetEndData(int* ArmSelect,float EndData[12], double* EndMoveTime);
int GetRemoteData(short *rockerL, short *rockerR, float RemoteMotionData[14]);
int GetRemoteCMD(void);
int GetControlCMD(void);
int GetHandCMD(int* HandSelect, float* HandAngleL, float* HandAngleR);

// 力传感器信号读取
int ForceSensorTCP_init(void);
int ForceSensorTCP_end(void);
int TCPSend(void *buffer, int length);
int TCPRecv(void *buffer, int length);
int ConfigSystem(int *nStatus, int *bIsSendFlag, int *bReceived);
int GetChParameter(char *pInstr,double *pdBuffer);
int GetData(int* bReceived, double* m_dDecouplingValue);
int GetADCounts(void);
int ShowAlgorithmData(double* m_dDecouplingValue);
int ForceSensorRecv(void);
#endif
