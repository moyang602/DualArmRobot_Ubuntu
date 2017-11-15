#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <sys/socket.h>
#include <netinet/in.h>


/**********************  UDP通讯 Start ***********************/
#pragma pack(push)
#pragma pack(1)
struct DataUDP_Struct
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

#pragma pack(push)
#pragma pack(1)
struct CommandUDP_Struct
{
	unsigned char Mode;
	unsigned char CANCH;
	unsigned char CANID;
	float Data;
	float time;
	unsigned char CheckSum;
};
#pragma pack(pop)

struct DataUDP_Struct UploadData;
struct CommandUDP_Struct DownloadData;



// UDP通讯变量声明
#define HOST_PORT 8000
#define HOST_IP "192.168.0.3"
#define DEST_PORT 8001
#define DEST_IP "192.168.0.4"
#define UDPCYCLE 30			// UDPCYCLE*程序运行周期 = UDP通讯周期

int UDP_Sock;
struct sockaddr_in DestAddr;
socklen_t nAddrLen;
int UDPComm_init(void);

unsigned char sendbuff[400];
unsigned char recvbuff[400];



/**********************  UDP通讯 End ***********************/

//函数声明
int UDPComm_init(void);
int UDPSend(void);
int UDPRecv(int motion_mode_control,long* can_channel_main,long* can_id_main,float* JointMoveData);

#endif
