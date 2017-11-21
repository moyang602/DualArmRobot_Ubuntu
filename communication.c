#include "communication.h"
#include "global_def.h"
#include <errno.h>

struct SingleJointCMD_Struct SingleJointData;
struct SingleArmCMD_Struct SignleArmData;
struct EndCMD_Struct EndData;
struct RemoteDATA_Struct RemoteData;
struct RemoteCMD_Struct RemoteCMD;
struct ControlCMD_Struct ControlCMD;
struct HandCMD_Struct HandCMD;

int UDP_Sock;
struct sockaddr_in DestPCAddr;

int ForceClientSock = -1;
struct sockaddr_in ForceServerAddr;

int ForceSensorTCP_init(void)
{
	ForceClientSock = socket(AF_INET, SOCK_STREAM, 0);
	if (ForceClientSock<0)
	{
		perror("ForceClient created failed!\n");
		return -1;
	}
	printf("ForceClient create successfully\n");

	memset(&ForceServerAddr,0,sizeof(ForceServerAddr));
	ForceServerAddr.sin_family=AF_INET;
    ForceServerAddr.sin_port=htons(ForceServer_Port);
    ForceServerAddr.sin_addr.s_addr=inet_addr(ForceServer_IP);

    if (connect(ForceClientSock,&ForceServerAddr,sizeof(struct sockaddr_in))<0)
    {
    	perror("Connect failed!\n");
    	return -1;
    }
    printf("ForceServer connect successfully\n");
    return 0;
}

int TCPSend(void *buffer, int length)
{
	int bytes_left;
	int sended_bytes;
	char *ptr;

	ptr = buffer;
	bytes_left = length;
	while(bytes_left>0)
	{
		sended_bytes = send(ForceClientSock,ptr,bytes_left,MSG_DONTWAIT);
		if (sended_bytes <=0)
		{
			if(errno == EINTR)
				sended_bytes = 0;
			else
				return -1;
		}
		bytes_left -=sended_bytes;
		ptr += sended_bytes;
	}
	return 0;

/*	struct my_struct my_struct_client;
	write(fd,(void *)&my_struct_client,sizeof(struct my_struct);


	char buffer[sizeof(struct my_struct)];
	struct *my_struct_server;
	read(fd,(void *)buffer,sizeof(struct my_struct));
	my_struct_server=(struct my_struct *)buffer;    */
}

int TCPRecv(void *buffer, int length)
{
	int bytes_left;
	int Received_bytes;
	char *ptr;

	bytes_left = length;
	while(bytes_left>0)
	{
		Received_bytes = recv(ForceClientSock,ptr,bytes_left,MSG_DONTWAIT);
		if (Received_bytes<0)
		{
			if (errno == EINTR)
				Received_bytes = 0;
			else
				return -1;
		}
		else if(Received_bytes == 0)
			break;
		bytes_left -= Received_bytes;
		ptr += Received_bytes;

	}
	return (length - bytes_left);
}

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
	if (bind(UDP_Sock, &HostAddr, sizeof(HostAddr)) < 0)
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
			//	if (sum != recvbuff[sizeof(SingleJointData) -1])	//校验和不正确直接退出
			//	{
			//		printf("recv %d bytes wrong data!\n",n);
			//		return 0;
			//	}
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

int GetEndData(int* ArmSelect,float EndMoveData[12], double* EndMoveTime)
{
	int i = 0;
	*ArmSelect = EndData.ArmSelect;
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
int GetRemoteData(float RemoteMotionData[14])
{
	int i = 0;
	for(i=0;i<14;i++)
	{
		RemoteMotionData[i] = RemoteData.Data[i];
	}
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
/*
int UDPRecv(int motion_mode_control,long* can_channel_main,long* can_id_main,float* JointMoveData)
{
	int n;
	int motion_mode = 0;
	memset(recvbuff,0,sizeof(recvbuff));
	n = recvfrom(UDP_Sock, recvbuff, sizeof(recvbuff), MSG_DONTWAIT, (struct sockaddr *)&DestPCAddr, &nAddrLen);

	if((n == sizeof(DownloadData)) && (motion_mode_control == 0))
	{
		memcpy(&DownloadData,recvbuff,sizeof(DownloadData));

		switch(DownloadData.Mode)
		{

/*
			case 02:
				motion_mode = ONE_ARM_MOTION;
				can_channel_main = DownloadData.CANCH;
				One_arm_main[0] = DownloadData.Data[0] * degree2rad;
				One_arm_main[1] = DownloadData.Data[1] * degree2rad;
				One_arm_main[2] = DownloadData.Data[2] * degree2rad;
				One_arm_main[3] = DownloadData.Data[3] * degree2rad;

				printf("motion_mode = ONE_ARM_MOTION  Arm %ld  %f  %f  %f  %f\n", can_channel_main+1, DownloadData.Data[0],DownloadData.Data[1],DownloadData.Data[2],DownloadData.Data[3]);
			break;

			case 03:
				motion_mode = TWO_ARMS_MOTION;
				End_Numb = DownloadData.CANCH;
				for(i=0;i<3;i++)
				{
					for(j=0;j<4;j++)
					{
						T_END_main[i][j] = DownloadData.Data[i*4+j];
					}
				}
				T_END_main[3][0] = 0.0;
				T_END_main[3][1] = 0.0;
				T_END_main[3][2] = 0.0;
				T_END_main[3][3] = 1.0;
				two_arms_time = DownloadData.Data[12];

				printf("motion_mode = TWO_ARMS_MOTION  end_numb %d\n, time length is %f", End_Numb,two_arms_time);

			break;

			case 04:
				motion_mode = VISION_MOTION;
				End_Numb = DownloadData.CANCH;

				if((DownloadData.Data[0] ==0.0)&&(DownloadData.Data[1] ==0.0)&&(DownloadData.Data[2] ==0.0))
				{
					motion_mode = 0;
				}
				else
				{
					for(i=0;i<3;i++)
					{
						for(j=0;j<4;j++)
						{
							DeltaMatrix[i][j] = DownloadData.Data[i*4+j];
						}
					}
					DeltaMatrix[3][0] = 0.0;
					DeltaMatrix[3][1] = 0.0;
					DeltaMatrix[3][2] = 0.0;
					DeltaMatrix[3][3] = 1.0;
					vision_motion_time = DownloadData.Data[12];
					printf("motion_mode = VISION_MOTION  end_numb %d\n, time length is %f\n", End_Numb, vision_motion_time);
				}
			break;

			case 06:
				motion_mode = FIND_HOME_MOTION;
				can_channel_main = DownloadData.CANCH;
				can_id_main = DownloadData.CANID;
			break;

			default:
			break;

		}

	}
	else if(n>0)
	{
		printf("Motion has not completed, Recv %d Byte data\n",n);
	}

}
*/