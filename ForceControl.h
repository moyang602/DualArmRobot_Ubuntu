#ifndef ForceControl_H
#define ForceControl_H

#define RX_BUFFER_SIZE	16384
#define dataBufferLen 8192
#define M812X_CHN_NUMBER 6

int ForceSensorTCP_init(int select);
int ForceSensorTCP_end(void);
int TCPSend(int select, void *buffer, int length);
int ForceTCPRecv(int select);

int ConfigSystemL(int *nStatus, int *bIsSendFlag, int *bReceived);
int ConfigSystemR(int *nStatus, int *bIsSendFlag, int *bReceived);
int GetChParameter(int select, char *pInstr,double *pdBuffer);
int GetData(int select, double* m_dDecouplingValue);
int GetADCounts(unsigned char mRxBuffer[dataBufferLen], int m_nADCountsL[M812X_CHN_NUMBER]);
int ShowAlgorithmData(int select, double* m_dDecouplingValue);

extern int ForceTCPFlagL;
extern int force_stepL;		// 力传感器读数整体步骤值
extern int nStatusL;		// 力传感器设置步骤值
extern int bIsSendFlagL;	// 发送标志
extern int bReceivedL;	// 接收标志
extern int ForceControl_flagL;
extern int FirstForceControlL;
extern int First_ForceL;

extern int ForceTCPFlagR;
extern int force_stepR;		// 力传感器读数整体步骤值
extern int nStatusR;		// 力传感器设置步骤值
extern int bIsSendFlagR;	// 发送标志
extern int bReceivedR;	// 接收标志
extern int ForceControl_flagR;
extern int FirstForceControlR;
extern int First_ForceR;

#endif