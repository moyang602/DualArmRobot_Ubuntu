#ifndef ForceControl_H
#define ForceControl_H

int ForceSensorTCP_init(int select);
int ForceSensorTCP_end(void);
int TCPSend(void *buffer, int length);
int ForceTCPRecv(void);

int ConfigSystem(int *nStatus, int *bIsSendFlag, int *bReceived);
int GetChParameter(char *pInstr,double *pdBuffer);
int GetData(double* m_dDecouplingValue);
int GetADCounts(void);
int ShowAlgorithmData(double* m_dDecouplingValue);

extern int ForceTCPFlag;
extern int force_step;		// 力传感器读数整体步骤值
extern int nStatus;		// 力传感器设置步骤值
extern int bIsSendFlag;	// 发送标志
extern int bReceived;	// 接收标志
extern int ForceControl_flag;
extern int FirstForceControl;
extern int First_Force;
#endif