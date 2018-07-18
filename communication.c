#include "communication.h"
#include "global_def.h"
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

struct SingleJointCMD_Struct SingleJointData;
struct SingleArmCMD_Struct SignleArmData;
struct EndCMD_Struct EndData;
struct RemoteDATA_Struct RemoteData;
struct RemoteCMD_Struct RemoteCMD;
struct ControlCMD_Struct ControlCMD;
struct HandCMD_Struct HandCMD;
struct FindHomeCMD_Struct FindHomeData;
struct ForceCMD_Struct ForceCMD;
struct DutyCMD_Struct DutyCMD;
struct DutyCMD_Struct FetchCMD;
struct DutyCMD_Struct PlaceCMD;

int UDP_Sock;
struct sockaddr_in DestPCAddr;


int RobotUDPComm_init(void)
{

    if ( (UDP_Sock=socket(AF_INET, SOCK_DGRAM, 0)) <0)
	{
		perror("PC socket created failed!\n");
	}
	struct sockaddr_in HostAddr;
	bzero(&HostAddr,sizeof(HostAddr));
	HostAddr.sin_family = AF_INET;
	HostAddr.sin_port = htons(CTRLHOST_PORT);
	HostAddr.sin_addr.s_addr = inet_addr(CTRLHOST_IP);
	if (bind(UDP_Sock, (struct sockaddr*)&HostAddr, sizeof(HostAddr)) < 0)
	{
		perror("PC socket binded failed!\n");
	}

	DestPCAddr.sin_family = AF_INET;
	DestPCAddr.sin_port = htons(DESTPC_PORT);
	DestPCAddr.sin_addr.s_addr = inet_addr(DESTPC_IP);
}

int RobotFBSend(struct RobotDataUDP_Struct RobotFBData)
{
	unsigned char sendbuff[400];
	int n;
	memset(sendbuff,0,sizeof(sendbuff));
	memcpy(sendbuff,&RobotFBData,sizeof(RobotFBData));

	socklen_t nAddrLen = sizeof(DestPCAddr);
	n = sendto(UDP_Sock, sendbuff, sizeof(RobotFBData), 0, (struct sockaddr *)&DestPCAddr, nAddrLen);
}

int UDPRecv()
{
	int n;
	int i = 0;
	unsigned char recvbuff[400];
	memset(recvbuff,0,sizeof(recvbuff));

	socklen_t nAddrLen = sizeof(DestPCAddr);
	n = recvfrom(UDP_Sock, recvbuff, sizeof(recvbuff), MSG_DONTWAIT, (struct sockaddr *)&DestPCAddr, &nAddrLen);

	if(n>=0)
	{
		switch(recvbuff[0])
		{
			case SINGLE_JOINT_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(SingleJointData) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(SingleJointData) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}
				memcpy(&SingleJointData,recvbuff,sizeof(SingleJointData));
				return SINGLE_JOINT_MOTION;
			}
			break;

			case ONE_ARM_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(SignleArmData) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(SignleArmData) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}
				memcpy(&SignleArmData,recvbuff,sizeof(SignleArmData));

				return ONE_ARM_MOTION;
			}

			break;

			case END_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(EndData) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(EndData) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}
				memcpy(&EndData,recvbuff,sizeof(EndData));

				return END_MOTION;
			}

			break;

			case ROBOT_CONTROL:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(ControlCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(ControlCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&ControlCMD,recvbuff,sizeof(ControlCMD));

				return ROBOT_CONTROL;
			}

			break;

			case REMOTE_CONTROL:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(RemoteCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(RemoteCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&RemoteCMD,recvbuff,sizeof(RemoteCMD));

				return REMOTE_CONTROL;
			}

			break;

			case REMOTE_DATA:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(RemoteData) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(RemoteData) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&RemoteData,recvbuff,sizeof(RemoteData));

				return REMOTE_DATA;
			}
			break;

			case HAND_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(HandCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(HandCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&HandCMD,recvbuff,sizeof(HandCMD));

				return HAND_MOTION;
			}
			break;

			case FIND_HOME_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(FindHomeData) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(FindHomeData) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}
				memcpy(&FindHomeData,recvbuff,sizeof(FindHomeData));
				return FIND_HOME_MOTION;
			}
			break;

			case FORCE_CONTROL:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(ForceCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(ForceCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&ForceCMD,recvbuff,sizeof(ForceCMD));

				return FORCE_CONTROL;
			}

			break;

			case DUTY_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(DutyCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(DutyCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&DutyCMD,recvbuff,sizeof(DutyCMD));

				return DUTY_MOTION;
			}
			break;

			case FETCH_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(FetchCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(FetchCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&FetchCMD,recvbuff,sizeof(FetchCMD));

				return FETCH_MOTION;
			}
			break;

			case PLACE_MOTION:
			{
				unsigned char sum = 0;
				for (i = 0; i < (sizeof(PlaceCMD) -1); ++i)
				{
					sum += recvbuff[i];
				}
				if (sum != recvbuff[sizeof(PlaceCMD) -1])	//校验和不正确直接退出
				{
					printf("recv %d bytes wrong data!\n",n);
					return 0;
				}

				memcpy(&PlaceCMD,recvbuff,sizeof(PlaceCMD));

				return PLACE_MOTION;
			}
			break;

			default:
			break;
		}

	}
	else if(n<0)
	{
	//	printf("UDPRecv wrong!!!\n");
		return 0;
	}
}

int GetSingleJointData(long* can_channel_main,long* can_id_main,float* JointMoveData,double* JointMoveTime)
{
	*can_channel_main = SingleJointData.CANCH;
	*can_id_main = SingleJointData.CANID;
	*JointMoveData = SingleJointData.Data * Degree2Rad;
	if(SingleJointData.time<=5.0)
		*JointMoveTime = 5.0;
	else
		*JointMoveTime = SingleJointData.time;
	printf("motion_mode = SINGLE_JOINT_MOTION  CH %ld CANID %ld, MOVE %6.3f DEG, TIME %5.2f s\n", *can_channel_main+1,*can_id_main+1, SingleJointData.Data, *JointMoveTime);
}

int GetSingleArmData(int* ArmSelect,float ArmMoveData[7], double* ArmMoveTime)
{
	int i =0;
	*ArmSelect = SignleArmData.ArmSelect;
	for (i = 0; i < 7; i++)
	{
		ArmMoveData[i] = SignleArmData.Data[i];
	}
	if(SignleArmData.time<=5.0)
		*ArmMoveTime = 5.0;
	else
		*ArmMoveTime = SignleArmData.time;
	printf("motion_mode = ONE_ARM_MOTION  ARM %d TIME  %8.3f s\n", *ArmSelect, *ArmMoveTime);
}

int GetEndData(int* ArmSelect, int* MotionSpace, float EndMoveData[12], double* EndMoveTime)
{
	int i = 0;
	*ArmSelect = EndData.ArmSelect;
	*MotionSpace = EndData.MotionSpace;
	for(i=0;i<12;i++)
	{
		EndMoveData[i] = EndData.Data[i];
	}
	if(EndData.time<=5.0)
		*EndMoveTime = 5.0;
	else
		*EndMoveTime = EndData.time;
	printf("motion_mode = END_MOTION  ARM %d TIME  %8.3f s\n", *ArmSelect, *EndMoveTime);

}
int GetRemoteData(short *rockerL, short *rockerR, float RemoteMotionData[14])
{
	int i = 0;
	for(i=0;i<14;i++)
	{
		RemoteMotionData[i] = RemoteData.Data[i];
	}
	*rockerL = RemoteData.rocker[0];
	*rockerR = RemoteData.rocker[1];

//	printf("Recv RemoteData\n");
}
int GetRemoteCMD(void)
{
	printf("RemoteControlMode = %02x\n",RemoteCMD.Command);
	return RemoteCMD.Command;
}
int GetControlCMD(void)
{
	printf("ControlMode = %02x\n",ControlCMD.Command);
	return ControlCMD.Command;
}

int GetHandCMD(int* HandSelect, float* HandAngleL, float* HandAngleR)
{
	*HandSelect = HandCMD.HandSelect;
	*HandAngleL = HandCMD.DataL;
	*HandAngleR = HandCMD.DataR;
	printf("HandCMD = %02x\n",HandCMD.Command);
	return HandCMD.Command;
}

int GetFindHomeData(long* can_channel_main,long* can_id_main)
{
	*can_channel_main = FindHomeData.CANCH;
	*can_id_main = FindHomeData.CANID;
	printf("motion_mode = FIND_HOME_MOTION  CH %ld CANID %ld\n", *can_channel_main+1,*can_id_main+1);
}

int GetForceCMD(int *ParamType, float ForceParam[7])
{
	*ParamType = ForceCMD.Param;
	int i = 0;
	for(i=0;i<6;i++)
	{
		ForceParam[i] = ForceCMD.Data[i];
	}
	printf("ForceControlMode = %02x\n",ForceCMD.Command);
	return ForceCMD.Command;
}

int GetDutyCMD(void)
{
	printf("DutyMode = %02x\n",DutyCMD.Command);
	return DutyCMD.Command;
}

int GetFetchCMD(void)
{
	printf("FetchMode = %02x\n",FetchCMD.Command);
	return FetchCMD.Command;
}

int GetPlaceCMD(void)
{
	printf("PlaceMode = %02x\n",PlaceCMD.Command);
	return PlaceCMD.Command;
}