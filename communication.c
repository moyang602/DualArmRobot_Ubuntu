#include "communication.h"
#include "global_def.h"


int UDPComm_init(void)
{

    	if ( (UDP_Sock=socket(AF_INET, SOCK_DGRAM, 0)) <0)
	{
		perror("socket created failed!\n");
	}
	struct sockaddr_in HostAddr;
	bzero(&HostAddr,sizeof(HostAddr));
	HostAddr.sin_family = AF_INET;
	HostAddr.sin_port = htons(HOST_PORT);
	HostAddr.sin_addr.s_addr = inet_addr(HOST_IP);
	if (bind(UDP_Sock, &HostAddr, sizeof(HostAddr)) < 0)
	{
		perror("socket binded failed!\n");
	}

	DestAddr.sin_family = AF_INET;
	DestAddr.sin_port = htons(DEST_PORT);
	DestAddr.sin_addr.s_addr = inet_addr(DEST_IP);
}

int UDPSend(void)
{
	memset(sendbuff,0,sizeof(sendbuff));
	memcpy(sendbuff,&UploadData,sizeof(UploadData));

	int n;

	nAddrLen = sizeof(DestAddr);
	n = sendto(UDP_Sock, sendbuff, sizeof(UploadData), 0, (struct sockaddr *)&DestAddr, nAddrLen);

}

int UDPRecv(int motion_mode_control,long* can_channel_main,long* can_id_main,float* JointMoveData)
{
	int n;
	int motion_mode = 0;
	memset(recvbuff,0,sizeof(recvbuff));
	n = recvfrom(UDP_Sock, recvbuff, sizeof(recvbuff), MSG_DONTWAIT, (struct sockaddr *)&DestAddr, &nAddrLen);

	if((n == sizeof(DownloadData)) && (motion_mode_control == 0))
	{
		memcpy(&DownloadData,recvbuff,sizeof(DownloadData));

		switch(DownloadData.Mode)
		{
			case 01:
				motion_mode = SINGLE_JOINT_MOTION;
				*can_channel_main = DownloadData.CANCH;
				*can_id_main = DownloadData.CANID;
				*JointMoveData = DownloadData.Data * 1.0;

				printf("motion_mode = SINGLE_JOINT_MOTION  CH %ld  CANID %ld,  MOVE %f DEGREE\n", *can_channel_main+1,*can_id_main+1, DownloadData.Data);
			break;
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
*/
			default:
			break;

		}

	}
	else if(n>0)
	{
		printf("Motion has not completed, Recv %d Byte data\n",n);
	}

}
