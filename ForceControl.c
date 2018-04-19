#include "ForceControl.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

// 与力传感器TCP通讯变量声明
#define ForceServer_PortL 4008
#define ForceServer_IPL "192.168.0.108"
#define ForceHOST_PORTL 5008
#define ForceHOST_IPL	"192.168.0.3"

#define ForceServer_PortR 4008
#define ForceServer_IPR "192.168.1.108"
#define ForceHOST_PORTR 5008
#define ForceHOST_IPR	"192.168.1.3"

int ForceControl_flagL =0;
int FirstForceControlL = 1;
int ForceControl_flagR =0;
int FirstForceControlR = 1;

int ForceClientSockL = -1;
int ForceTCPFlagL = 0;
int force_stepL = 1;		// 力传感器读数整体步骤值
int nStatusL = 0;		// 力传感器设置步骤值
int bIsSendFlagL = 1;	// 发送标志
int bReceivedL	= 0;	// 接收标志
int First_ForceL = 1;

int ForceClientSockR = -1;
int ForceTCPFlagR = 0;
int force_stepR = 1;		// 力传感器读数整体步骤值
int nStatusR = 0;		// 力传感器设置步骤值
int bIsSendFlagR = 1;	// 发送标志
int bReceivedR	= 0;	// 接收标志
int First_ForceR = 1;

double ForceInitL[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double ForceInitR[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double m_dDecouplingCoefficientL[M812X_CHN_NUMBER][M812X_CHN_NUMBER] =
{
	{-0.356073,		0.22652,		1.144266,		71.802291,	0.5515,			-72.609276},
	{0.268013,		-81.448258,	1.41573,		41.814438,	1.362226,		41.580507},
	{226.093891,	-1.465781,	227.962006,	-2.353303,	229.051028,	-4.600184},
	{-0.162402,		-0.094572,	-8.296902,	0.119678,		7.982737,		-0.160643},
	{9.39356,			-0.009313,	-4.710392,	-0.0501,		-4.652383,	0.169629},
	{-0.011642,		3.224638,		0.057849,		3.263569,		0.004462,		3.165842},
};


double m_dDecouplingCoefficientR[M812X_CHN_NUMBER][M812X_CHN_NUMBER] =
{
	{0.583129,	0.374904,	0.375319,	71.26446,	0.060775,	-71.392659},
	{0.588984,	-82.627101,	0.310336,	41.684648,	1.035492,	41.117846},
	{227.934224,	0.324415,	229.601791,	-3.250975,	226.307973,	-3.168233},
	{-0.112713,	-0.045798,	-8.465209,	0.07628,	7.933784,	-0.084778},
	{9.427496,	-0.044594,	-4.613457,	0.003019,	-4.571049,	0.074759},
	{-0.024469,	3.247267,	0.018507,	3.23357,	0.011676,	3.137815},
};		// SN5164

double m_dAmpZeroL[M812X_CHN_NUMBER];
double m_dChnGainL[M812X_CHN_NUMBER];
double m_dChnExL[M812X_CHN_NUMBER];
int    m_nADCountsL[M812X_CHN_NUMBER];
unsigned char mRxBufferL[dataBufferLen];

double m_dAmpZeroR[M812X_CHN_NUMBER];
double m_dChnGainR[M812X_CHN_NUMBER];
double m_dChnExR[M812X_CHN_NUMBER];
int    m_nADCountsR[M812X_CHN_NUMBER];
unsigned char mRxBufferR[dataBufferLen];

int ForceSensorTCP_init(int select)
{
	switch(select)
	{
		case 1:
		{
			ForceClientSockL = socket(AF_INET, SOCK_STREAM, 0);
			if (ForceClientSockL<0)
			{
				perror("ForceClient created failed!\n");
				return -1;
			}
			printf("ForceClient create successfully\n");

			struct sockaddr_in HostAddr;
			memset(&HostAddr,0,sizeof(HostAddr));
			HostAddr.sin_family = AF_INET;
			HostAddr.sin_port = htons(ForceHOST_PORTL);
			HostAddr.sin_addr.s_addr = inet_addr(ForceHOST_IPL);
			if (bind(ForceClientSockL, (struct sockaddr*)&HostAddr, sizeof(HostAddr)) < 0)
			{
				perror("PC socket binded failed!\n");
			}
			printf("ForceClient binded successfully\n");
			struct sockaddr_in ForceServerAddr;
			memset(&ForceServerAddr,0,sizeof(ForceServerAddr));
			ForceServerAddr.sin_family=AF_INET;
			ForceServerAddr.sin_port=htons(ForceServer_PortL);
			ForceServerAddr.sin_addr.s_addr=inet_addr(ForceServer_IPL);

			if (connect(ForceClientSockL,(struct sockaddr*)&ForceServerAddr,sizeof(struct sockaddr_in))<0)
			{
				perror("Connect failed!\n");
				return -1;
			}
			printf("ForceServer connect successfully\n");
			return 1;
		}
		break;

		case 2:
		{
			ForceClientSockR = socket(AF_INET, SOCK_STREAM, 0);
			if (ForceClientSockR<0)
			{
				perror("ForceClient created failed!\n");
				return -1;
			}
			printf("ForceClient create successfully\n");

			struct sockaddr_in HostAddr;
			memset(&HostAddr,0,sizeof(HostAddr));
			HostAddr.sin_family = AF_INET;
			HostAddr.sin_port = htons(ForceHOST_PORTR);
			HostAddr.sin_addr.s_addr = inet_addr(ForceHOST_IPR);
			if (bind(ForceClientSockR, (struct sockaddr*)&HostAddr, sizeof(HostAddr)) < 0)
			{
				perror("PC socket binded failed!\n");
			}
			printf("ForceClient binded successfully\n");
			struct sockaddr_in ForceServerAddr;
			memset(&ForceServerAddr,0,sizeof(ForceServerAddr));
			ForceServerAddr.sin_family=AF_INET;
			ForceServerAddr.sin_port=htons(ForceServer_PortR);
			ForceServerAddr.sin_addr.s_addr=inet_addr(ForceServer_IPR);

			if (connect(ForceClientSockR,(struct sockaddr*)&ForceServerAddr,sizeof(struct sockaddr_in))<0)
			{
				perror("Connect failed!\n");
				return -1;
			}
			printf("ForceServer connect successfully\n");
			return 1;
		}
		break;

		default:
		break;
	}

}

int ForceSensorTCP_end(void)
{
	if(ForceClientSockL>0)
	{
		close(ForceClientSockL);
		ForceClientSockL = 0;
	}
	else
	{
		ForceClientSockL = 0;
	}
	if(ForceClientSockR>0)
	{
		close(ForceClientSockR);
		ForceClientSockR = 0;
	}
	else
	{
		ForceClientSockR = 0;
	}
}

int TCPSend(int select, void *buffer, int length)
{
	int bytes_left;
	int sended_bytes;
	char *ptr;

	switch(select)
	{
		case 1:
		{
			ptr = (char*)buffer;
			bytes_left = length;
			while(bytes_left>0)
			{
				sended_bytes = send(ForceClientSockL,ptr,bytes_left,MSG_DONTWAIT);
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
		}
		break;

		case 2:
		{
			ptr = (char*)buffer;
			bytes_left = length;
			while(bytes_left>0)
			{
				sended_bytes = send(ForceClientSockR,ptr,bytes_left,MSG_DONTWAIT);
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
		}

		break;
		default:
		break;
	}


}

int ForceTCPRecv(int select)
{
	unsigned char recvbuff[1000];
	memset(recvbuff,0,sizeof(recvbuff));
	int RTN;

	switch(select)
	{
		case 1:
		{
			RTN = recv(ForceClientSockL,recvbuff,1000,MSG_DONTWAIT);
			if (RTN<0)
			{
				if (errno == EINTR)
					RTN = 0;
				else
			//		printf("errno.%02d is: %s\n", errno, strerror(errno));
					return -1;
			}
			else
			{
			//	printf("Recv:%d data, %s\n",RTN,recvbuff);
				bReceivedL = 1;
				int i=0;

				for(i=0;i<RTN;i++)
				{
					mRxBufferL[i] = recvbuff[i];
				}
			}
			return 0;
		}
		break;
		case 2:
		{
			RTN = recv(ForceClientSockR,recvbuff,1000,MSG_DONTWAIT);
			if (RTN<0)
			{
				if (errno == EINTR)
					RTN = 0;
				else
			//		printf("errno.%02d is: %s\n", errno, strerror(errno));
					return -1;
			}
			else
			{
			//	printf("Recv:%d data, %s\n",RTN,recvbuff);
				bReceivedR = 1;
				int i=0;

				for(i=0;i<RTN;i++)
				{
					mRxBufferR[i] = recvbuff[i];
				}
			}
			return 0;
		}
		break;
		default:
		break;
	}


}


int ConfigSystemL(int *nStatus, int *bIsSendFlag, int *bReceived)
{
	switch(*nStatus)
	{
		case 0:
		{
			if (*bIsSendFlag == 1)
			{
				char* Command = "AT+AMPZ=?\r\n";
				int CommandLens = 11;
				TCPSend(1,(void *)Command,CommandLens);
				*bIsSendFlag = 0;

			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(1,"AMPZ",m_dAmpZeroL) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 1;
					printf("Get AMPZ successful\n");
				}
			}
		}
		break;

		case 1:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+CHNAPG=?\r\n";
				int CommandLens = 13;
				TCPSend(1,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(1,"CHNAPG",m_dChnGainL) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 2;
					printf("Get CHNAPG successful\n");
				}

			}
		}
		break;
		case 2:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+EXMV=?\r\n";
				int CommandLens = 11;
				TCPSend(1,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(1,"EXMV",m_dChnExL) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 3;
					printf("Get EXMV successful\n");
				}

			}
		}
		break;
		case 3:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+SMPR=2000\r\n";
				int CommandLens = 14;
				TCPSend(1,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					*nStatus = 4;
					printf("Set SMPR successful\n");
				}
			}
		}
		break;
		case 4:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1,1,1,1,1)\r\n";
				int CommandLens = 55;
				TCPSend(1,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					*nStatus = 5;
					printf("Set SGDM successful\n");
					printf("Config system finish\n");
					return 2;
				}
			}
		}
		break;
		default:
		{
			*nStatus = 0;
			return 0;
		}
		break;
	}
	return 1;
}
int ConfigSystemR(int *nStatus, int *bIsSendFlag, int *bReceived)
{
	switch(*nStatus)
	{
		case 0:
		{
			if (*bIsSendFlag == 1)
			{
				char* Command = "AT+AMPZ=?\r\n";
				int CommandLens = 11;
				TCPSend(2,(void *)Command,CommandLens);
				*bIsSendFlag = 0;

			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(2,"AMPZ",m_dAmpZeroR) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 1;
					printf("Get AMPZ successful\n");
				}
			}
		}
		break;

		case 1:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+CHNAPG=?\r\n";
				int CommandLens = 13;
				TCPSend(2,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(2,"CHNAPG",m_dChnGainR) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 2;
					printf("Get CHNAPG successful\n");
				}

			}
		}
		break;
		case 2:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+EXMV=?\r\n";
				int CommandLens = 11;
				TCPSend(2,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter(2,"EXMV",m_dChnExR) != 1)
					{
						*nStatus = 0;
						return 0;
					}
					*nStatus = 3;
					printf("Get EXMV successful\n");
				}

			}
		}
		break;
		case 3:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+SMPR=2000\r\n";
				int CommandLens = 14;
				TCPSend(2,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					*nStatus = 4;
					printf("Set SMPR successful\n");
				}
			}
		}
		break;
		case 4:
		{
			if(*bIsSendFlag == 1)
			{
				char* Command = "AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1,1,1,1,1)\r\n";
				int CommandLens = 55;
				TCPSend(2,(void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					*nStatus = 5;
					printf("Set SGDM successful\n");
					printf("Config system finish\n");
					return 2;
				}
			}
		}
		break;
		default:
		{
			*nStatus = 0;
			return 0;
		}
		break;
	}
	return 1;
}

int GetChParameter(int select, char *pInstr,double *pdBuffer)
{
	char *pIndexBuffer = NULL;
	char CharTemp[16] = {0x00};
	double pdTemp[6] = {0x00};

	switch(select)
	{
		case 1:
		{
			pIndexBuffer = strstr((char*)mRxBufferL,pInstr);
			if(pIndexBuffer == NULL) return 0;

			int k = 0x00;
			double dTemp = 0x00;
			int len = strlen(pInstr)+1;
			pIndexBuffer = pIndexBuffer+len;
			int i = 0;
			for(i = 0x00; i < 6; i++)
			{
				memset(CharTemp,0x00,16);
				k = 0x00;
				while(*pIndexBuffer != ';' && *pIndexBuffer != '$')
				{
					CharTemp[k++] = *pIndexBuffer++;
					if(k >= 16) return 0;
				}
				if(sscanf(CharTemp,"%lf",&dTemp) != 1) return 0;
				pdTemp[i] = dTemp;
				printf("%lf\n",dTemp);
				pIndexBuffer++;
			}

			for(i = 0x00; i < 6; i++)
				pdBuffer[i] = pdTemp[i];
			printf("GetChParameter successful\n");
			return 1;
		}
		break;
		case 2:
		{
			pIndexBuffer = strstr((char*)mRxBufferR,pInstr);
			if(pIndexBuffer == NULL) return 0;

			int k = 0x00;
			double dTemp = 0x00;
			int len = strlen(pInstr)+1;
			pIndexBuffer = pIndexBuffer+len;
			int i = 0;
			for(i = 0x00; i < 6; i++)
			{
				memset(CharTemp,0x00,16);
				k = 0x00;
				while(*pIndexBuffer != ';' && *pIndexBuffer != '$')
				{
					CharTemp[k++] = *pIndexBuffer++;
					if(k >= 16) return 0;
				}
				if(sscanf(CharTemp,"%lf",&dTemp) != 1) return 0;
				pdTemp[i] = dTemp;
				printf("%lf\n",dTemp);
				pIndexBuffer++;
			}

			for(i = 0x00; i < 6; i++)
				pdBuffer[i] = pdTemp[i];
			printf("GetChParameter successful\n");
			return 1;
		}
		break;
		default:
		break;
	}

}

int GetData(int select, double* m_dDecouplingValue)
{
	char* Command = "AT+GOD\r\n";
	int CommandLens = 8;

	switch(select)
	{
		case 1:
		{
			memset(mRxBufferL,0,sizeof(mRxBufferL));
			TCPSend(1,(void *)Command,CommandLens);

			//	struct timespec sleeptime;
			//	sleeptime.tv_nsec = 500000;
			//	sleeptime.tv_sec = 0;
			//	nanosleep(&sleeptime,NULL);

			ForceTCPRecv(1);

			if(bReceivedL)
			{
				if(GetADCounts(mRxBufferL, m_nADCountsL))
					ShowAlgorithmData(1,m_dDecouplingValue);
				bReceivedL = 0;
				return 1;
			}
			else
			{
				printf("can't get force data\n");
			}

			return 0;
		}
		break;
		case 2:
		{
			memset(mRxBufferR,0,sizeof(mRxBufferR));
			TCPSend(2,(void *)Command,CommandLens);

			//	struct timespec sleeptime;
			//	sleeptime.tv_nsec = 500000;
			//	sleeptime.tv_sec = 0;
			//	nanosleep(&sleeptime,NULL);

			ForceTCPRecv(2);

			if(bReceivedR)
			{
				if(GetADCounts(mRxBufferR, m_nADCountsR))
					ShowAlgorithmData(2,m_dDecouplingValue);
				bReceivedR = 0;
				return 1;
			}
			else
			{
				printf("can't get force data\n");
			}

			return 0;
		}
		break;
		default:
		break;
	}


}

int GetADCounts(unsigned char mRxBuffer[dataBufferLen], int m_nADCounts[M812X_CHN_NUMBER])
{
//  structure
//	FrameHeader	PackageLength	DataNo	     Data	     ChkSum
//  0xAA,0x55	   HB,LB          2B	(ChNum*N*PNpCH)B   1B

	unsigned char* pRxBuffer = (unsigned char*)mRxBuffer;

	int i = 0x00;
	while(1)
	{
		if(pRxBuffer[i] == 0xAA && pRxBuffer[i+1] == 0x55)
		{
			break;
		}
		i++;
		if(i >= RX_BUFFER_SIZE)	return 0;
	}

	int Length = pRxBuffer[i+2]*256 + pRxBuffer[i+3];


	int Index = i + 6;
	int j = 0;
	unsigned char* CheckSum = 0x00;
	for(j = 0x00; j < M812X_CHN_NUMBER*2; j++)
	{
		CheckSum += pRxBuffer[Index++];
	}

	Index = i + 6;
	int k = 0;
	for(k = 0x00; k < M812X_CHN_NUMBER; k++)
	{
		m_nADCounts[k] = pRxBuffer[Index]*256 + pRxBuffer[Index+1];
		Index = Index+2;
	}

	return 1;
}

double F_recL[5][6] = {0.0};
double F_recR[5][6] = {0.0};
double F_thresholdL[6] = {0.5, 0.5, 1.0, 0.05, 0.05, 0.05};
double F_thresholdR[6] = {0.5, 0.5, 1.0, 0.05, 0.05, 0.05};

int ShowAlgorithmData(int select, double* m_dDecouplingValue)
{
	double m_dResultChValue[M812X_CHN_NUMBER];
	int i = 0;

	switch(select)
	{
		case 1:
		{
			for(i = 0x00; i < M812X_CHN_NUMBER; i++)
			{
				//Matrix Decoupling Loadcell Type A.
				m_dResultChValue[i] = 1000*( (m_nADCountsL[i] - m_dAmpZeroL[i]) / (double)65535*(double)5 ) / m_dChnGainL[i] / m_dChnExL[i];
				//Matrix Decoupling Loadcell Type B.
				//m_dResultChValue[i] = 1000*( (m_nADCounts[i] - m_dAmpZero[i]) / (double)65535*(double)5 ) / m_dChnGain[i];
			}

			//Decouple signals
			for(i = 0x00; i < M812X_CHN_NUMBER; i++)
			{
				F_recL[0][i] =   m_dResultChValue[0]*m_dDecouplingCoefficientL[i][0] +
								m_dResultChValue[1]*m_dDecouplingCoefficientL[i][1] +
								m_dResultChValue[2]*m_dDecouplingCoefficientL[i][2] +
								m_dResultChValue[3]*m_dDecouplingCoefficientL[i][3] +
								m_dResultChValue[4]*m_dDecouplingCoefficientL[i][4] +
								m_dResultChValue[5]*m_dDecouplingCoefficientL[i][5];
			}
			double ForceMean[6] = {0.0};
			for(i=0;i<6;i++)
			{
				ForceMean[i] = F_recL[0][i] + F_recL[1][i] + F_recL[2][i] + F_recL[3][i] + F_recL[4][i];
			}

			if(First_ForceL == 1)
			{
				for(i=0;i<6;i++)
				{
					ForceInitL[i] = ForceMean[i];
				}
				First_ForceL = 0;
			}

			for(i=0;i<6;i++)
			{
				m_dDecouplingValue[i] = (ForceMean[i] - ForceInitL[i])/5.0;				// moyang602
			}

			for(i=0;i<6;i++)
			{
				if(fabs(m_dDecouplingValue[i])<F_thresholdL[i])
					m_dDecouplingValue[i] = 0.0;
			}

			for(i=0;i<6;i++)
			{
				F_recL[1][i] = F_recL[0][i];
				F_recL[2][i] = F_recL[1][i];
				F_recL[3][i] = F_recL[2][i];
				F_recL[4][i] = F_recL[3][i];
			}
		}
		break;

		case 2:
		{
			for(i = 0x00; i < M812X_CHN_NUMBER; i++)
			{
				//Matrix Decoupling Loadcell Type A.
				m_dResultChValue[i] = 1000*( (m_nADCountsR[i] - m_dAmpZeroR[i]) / (double)65535*(double)5 ) / m_dChnGainR[i] / m_dChnExR[i];
				//Matrix Decoupling Loadcell Type B.
				//m_dResultChValue[i] = 1000*( (m_nADCounts[i] - m_dAmpZero[i]) / (double)65535*(double)5 ) / m_dChnGain[i];
			}

			//Decouple signals
			for(i = 0x00; i < M812X_CHN_NUMBER; i++)
			{
				F_recR[0][i] =   m_dResultChValue[0]*m_dDecouplingCoefficientR[i][0] +
								m_dResultChValue[1]*m_dDecouplingCoefficientR[i][1] +
								m_dResultChValue[2]*m_dDecouplingCoefficientR[i][2] +
								m_dResultChValue[3]*m_dDecouplingCoefficientR[i][3] +
								m_dResultChValue[4]*m_dDecouplingCoefficientR[i][4] +
								m_dResultChValue[5]*m_dDecouplingCoefficientR[i][5];
			}
			double ForceMean[6] = {0.0};
			for(i=0;i<6;i++)
			{
				ForceMean[i] = F_recR[0][i] + F_recR[1][i] + F_recR[2][i] + F_recR[3][i] + F_recR[4][i];
			}

			if(First_ForceR == 1)
			{
				for(i=0;i<6;i++)
				{
					ForceInitR[i] = ForceMean[i];
				}
				First_ForceR = 0;
			}

			for(i=0;i<6;i++)
			{
				m_dDecouplingValue[i] = (ForceMean[i] - ForceInitR[i])/5.0;				// moyang602
			}

			for(i=0;i<6;i++)
			{
				if(fabs(m_dDecouplingValue[i])<F_thresholdR[i])
					m_dDecouplingValue[i] = 0.0;
			}

			for(i=0;i<6;i++)
			{
				F_recR[1][i] = F_recR[0][i];
				F_recR[2][i] = F_recR[1][i];
				F_recR[3][i] = F_recR[2][i];
				F_recR[4][i] = F_recR[3][i];
			}
		}
		break;

		default:
		break;
	}

}
