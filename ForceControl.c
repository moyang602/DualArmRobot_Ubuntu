#include "ForceControl.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stddef.h>

int ForceControl_flag =0;
int FirstForceControl = 1;

// 与力传感器TCP通讯变量声明
#define ForceServer_Port 4008
#define ForceServer_IP "192.168.1.108"
#define ForceHOST_PORT 5008
#define ForceHOST_IP	"192.168.1.3"
#define RX_BUFFER_SIZE	16384
#define dataBufferLen 8192
#define M812X_CHN_NUMBER 6
int ForceClientSock = -1;
struct sockaddr_in ForceServerAddr;
int ForceTCPFlag = 0;
int force_step = 1;		// 力传感器读数整体步骤值
int nStatus = 0;		// 力传感器设置步骤值
int bIsSendFlag = 1;	// 发送标志
int bReceived	= 0;	// 接收标志
int First_Force = 1;


double ForceInit[6] = {0.0};

double m_dDecouplingCoefficient[M812X_CHN_NUMBER][M812X_CHN_NUMBER] =
{
	{-0.356073,		0.22652,		1.144266,		71.802291,	0.5515,			-72.609276},
	{0.268013,		-81.448258,	1.41573,		41.814438,	1.362226,		41.580507},
	{226.093891,	-1.465781,	227.962006,	-2.353303,	229.051028,	-4.600184},
	{-0.162402,		-0.094572,	-8.296902,	0.119678,		7.982737,		-0.160643},
	{9.39356,			-0.009313,	-4.710392,	-0.0501,		-4.652383,	0.169629},
	{-0.011642,		3.224638,		0.057849,		3.263569,		0.004462,		3.165842},
};

double m_dAmpZero[M812X_CHN_NUMBER];
double m_dChnGain[M812X_CHN_NUMBER];
double m_dChnEx[M812X_CHN_NUMBER];
int    m_nADCounts[M812X_CHN_NUMBER];
unsigned char mRxBuffer[dataBufferLen];


int ForceSensorTCP_init(int select)
{
	switch(select)
	{
		case 1:
		{
			ForceClientSock = socket(AF_INET, SOCK_STREAM, 0);
			if (ForceClientSock<0)
			{
				perror("ForceClient created failed!\n");
				return -1;
			}
			printf("ForceClient create successfully\n");

			struct sockaddr_in HostAddr;
			memset(&HostAddr,0,sizeof(HostAddr));
			HostAddr.sin_family = AF_INET;
			HostAddr.sin_port = htons(ForceHOST_PORT);
			HostAddr.sin_addr.s_addr = inet_addr(ForceHOST_IP);
			if (bind(ForceClientSock, &HostAddr, sizeof(HostAddr)) < 0)
			{
				perror("PC socket binded failed!\n");
			}
			printf("ForceClient binded successfully\n");
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
			return 1;
		}
		break;

		case 2:
		{
			return 1;

		}
		break;

		default:
		break;
	}

}

int ForceSensorTCP_end(void)
{
	if(ForceClientSock>0)
	{
		close(ForceClientSock);
		ForceClientSock = 0;
	}
	else
		ForceClientSock = 0;
}

int TCPSend(void *buffer, int length)
{
	int bytes_left;
	int sended_bytes;
	char *ptr;

	ptr = (char*)buffer;
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

}

int ForceTCPRecv(void)
{
	unsigned char recvbuff[1000];
	memset(recvbuff,0,sizeof(recvbuff));
	int RTN;
	RTN = recv(ForceClientSock,recvbuff,1000,MSG_DONTWAIT);
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
		bReceived = 1;
		int i=0;

		for(i=0;i<RTN;i++)
		{
			mRxBuffer[i] = recvbuff[i];
		}
	}
	return 0;

}


int ConfigSystem(int *nStatus, int *bIsSendFlag, int *bReceived)
{
	switch(*nStatus)
	{
		case 0:
		{
			if (*bIsSendFlag == 1)
			{
				char* Command = "AT+AMPZ=?\r\n";
				int CommandLens = 11;
				TCPSend((void *)Command,CommandLens);
				*bIsSendFlag = 0;

			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter("AMPZ",m_dAmpZero) != 1)
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
				TCPSend((void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter("CHNAPG",m_dChnGain) != 1)
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
				TCPSend((void *)Command,CommandLens);
				*bIsSendFlag = 0;
			}
			else
			{
				if (*bReceived == 1)
				{
					*bReceived = 0;
					*bIsSendFlag = 1;
					if (GetChParameter("EXMV",m_dChnEx) != 1)
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
				TCPSend((void *)Command,CommandLens);
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
				TCPSend((void *)Command,CommandLens);
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

int GetChParameter(char *pInstr,double *pdBuffer)
{
	char *pIndexBuffer = NULL;
	char CharTemp[16] = {0x00};
	double pdTemp[6] = {0x00};

	pIndexBuffer = strstr((char*)mRxBuffer,pInstr);
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

int GetData(double* m_dDecouplingValue)
{
/*	static int Getdata_Flag = 1;
	if (Getdata_Flag)
	{
		memset(mRxBuffer,0,sizeof(mRxBuffer));
		char* Command = "AT+GOD\r\n";
		int CommandLens = 8;
		TCPSend((void *)Command,CommandLens);
		Getdata_Flag = 0;
	}
	else
	{
		if(*bReceived)
		{
			if(GetADCounts())
				ShowAlgorithmData(m_dDecouplingValue);
			*bReceived = 0;
			Getdata_Flag = 1;
			return 1;
		}
	}
*/

	memset(mRxBuffer,0,sizeof(mRxBuffer));
	char* Command = "AT+GOD\r\n";
	int CommandLens = 8;
	TCPSend((void *)Command,CommandLens);

//	struct timespec sleeptime;
//	sleeptime.tv_nsec = 500000;
//	sleeptime.tv_sec = 0;
//	nanosleep(&sleeptime,NULL);

	ForceTCPRecv();

	if(bReceived)
	{
		if(GetADCounts())
			ShowAlgorithmData(m_dDecouplingValue);
		bReceived = 0;
		return 1;
	}
	else
	{
		printf("can't get force data\n");
	}

	return 0;
}

int GetADCounts(void)
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
	//if(CheckSum !=  pRxBuffer[Index])	return 0;

	Index = i + 6;
	int k = 0;
	for(k = 0x00; k < M812X_CHN_NUMBER; k++)
	{
		m_nADCounts[k] = pRxBuffer[Index]*256 + pRxBuffer[Index+1];
		Index = Index+2;
	}
//	printf("%d, %d, %d, %d, %d, %d\n",m_nADCounts[0],m_nADCounts[1],m_nADCounts[2],m_nADCounts[3],m_nADCounts[4],m_nADCounts[5]);
	return 1;
}

double F_rec[5][6] = {0.0};

double F_threshold[6] = {0.5, 0.5, 1.0, 0.05, 0.05, 0.05};

int ShowAlgorithmData(double* m_dDecouplingValue)
{
	double m_dResultChValue[M812X_CHN_NUMBER];
	int i = 0;

	for(i = 0x00; i < M812X_CHN_NUMBER; i++)
	{

		//Matrix Decoupling Loadcell Type A.
		m_dResultChValue[i] = 1000*( (m_nADCounts[i] - m_dAmpZero[i]) / (double)65535*(double)5 ) / m_dChnGain[i] / m_dChnEx[i];
    //Matrix Decoupling Loadcell Type B.
		//m_dResultChValue[i] = 1000*( (m_nADCounts[i] - m_dAmpZero[i]) / (double)65535*(double)5 ) / m_dChnGain[i];

	}

	//Decouple signals
	for(i = 0x00; i < M812X_CHN_NUMBER; i++)
	{

		F_rec[0][i] =   m_dResultChValue[0]*m_dDecouplingCoefficient[i][0] +
						m_dResultChValue[1]*m_dDecouplingCoefficient[i][1] +
						m_dResultChValue[2]*m_dDecouplingCoefficient[i][2] +
						m_dResultChValue[3]*m_dDecouplingCoefficient[i][3] +
						m_dResultChValue[4]*m_dDecouplingCoefficient[i][4] +
						m_dResultChValue[5]*m_dDecouplingCoefficient[i][5];

	}
	double ForceMean[6] = {0.0};
	for(i=0;i<6;i++)
	{
		ForceMean[i] = F_rec[0][i] + F_rec[1][i] + F_rec[2][i] + F_rec[3][i] + F_rec[4][i];
	}


	if(First_Force == 1)
	{
		for(i=0;i<6;i++)
		{
			ForceInit[i] = ForceMean[i];
		}
		First_Force = 0;
	}

	for(i=0;i<6;i++)
	{
		m_dDecouplingValue[i] = (ForceMean[i] - ForceInit[i])/5.0;				// moyang602
	}

	for(i=0;i<6;i++)
	{
		if(fabs(m_dDecouplingValue[i])<F_threshold[i])
			m_dDecouplingValue[i] = 0.0;
	}

	for(i=0;i<6;i++)
	{
		F_rec[1][i] = F_rec[0][i];
		F_rec[2][i] = F_rec[1][i];
		F_rec[3][i] = F_rec[2][i];
		F_rec[4][i] = F_rec[3][i];
	}
}
