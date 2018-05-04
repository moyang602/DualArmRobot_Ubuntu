/* Program to Control Robot
 *
 * Copyright (C) 2018 Beijing Institute of Technology
 *
 * Copyright (C) 2018 Institute of Intelligent Robot
 *                         				 <moyang602@163.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

//#define _GNU_SOURCE
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sched.h>
#include <semaphore.h>
#include <math.h>
#include <termios.h>
//#include <curses.h>


#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/keysymdef.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rtdm/rtcan.h>
#include "trajectory.h"
#include "global_def.h"
#include "communication.h"
#include "HandControl.h"
#include "SerialComm.h"
#include "ForceControl.h"

#define hand2base 300.0

/*********************************函数声明**********************************/
void send_event();
void timespec_add_us(struct timespec *t, long us);
int timespec_cmp(struct timespec *a, struct timespec *b);
int can_send(int channel, long can_id, int can_mode, long can_content[8], int can_lenth);
void rt_can_recv(void *arg);
void driver_init_status(void);
int elmo_init(void);
void servo_off(void);
void servo_on(void);
void power_off(void);
void power_on(void);
void find_home(int channel_num, int id_num);
void rad_send(int i, int j, double angle_in);
double JointDetect(int Can_CH, int Can_ID, double angle_in);

void SYNC_Receive(void);
int AllJointMove(struct RealRobot_Struct RealPos, double time, int joint, int hand, int head, int waist);

// CAN定义到实际机器人数据结构转换
void CanDef2RealRobot(double CanDef[4][7], struct RealRobot_Struct* RealRobot);
// 实际机器人数据结构到CAN定义转换
void RealRobot2CanDef(struct RealRobot_Struct RealRobot, double CanDef[4][7]);
int  control_handL(short u16_sig, double I_f1, double I_f2, double I_f3,double * output);
int  control_handR(short u16_sig, double I_f1, double I_f2, double I_f3,double * output);
/*********************************变量声明**********************************/
/////////////// 系统变量
RT_TASK demo_task_rvcan;
RT_TASK rt_task_desc;
RT_TASK rt_task_view;
RTIME now, previous, period, now2, previous2, period2;

GC MyGC;

Display *MyDisplay;
Window MyWindow;
Atom MyAtom;
Atom wm_delete_window;
Atom wm_protocols;
sem_t low_go;

int canrv_running = 1;

int view_time = 1;

static int txsock[4], rxsock[4];
static int s=-1, dlc=0, rtr=0, extended=0, verbose=1, loops=1;
static nanosecs_rel_t timeout_rv = 100;
static nanosecs_rel_t timeout_send = 1000;
static struct can_frame frame;
static struct sockaddr_can to_addr;

struct  ifreq ifr[4];
struct sockaddr_can recv_addr;

double time_interval = 0.006;
double runtime = 0.0;

////////////// 显示变量
char buf[200] = {0};
char buf1[200] = {0};
char bufs1[100] = {0};
char bufs2[100] = {0};
char buf2[200] = {0};
char buf3[200]={0};
char buf4[200]={0};
char buf5[200]={0};
char buf6[200]={0};
char buf7[200]={0};
char buf8[200]={0};
char buf9[200]={0};
char buf10[200]={0};
char buf11[200]={0};
char buf12[200]={0};
char buf13[200]={0};
char buf14[200]={0};
char buf15[200]={0};
char buf16[200]={0};
char buf17[200]={0};
char buf18[200]={0};
char buf19[200]={0};
char buf20[200]={0};
char buf21[200]={0};
char buf22[200]={0};
char buf23[200]={0};
char buf24[200]={0};
char buf25[200]={0};
char buf26[200]={0};
char buf27[200]={0};

int motion_mode = 0;
int control_mode = 0;
long can_channel_main = 0;
long can_id_main = 0;
int End_Numb = 0;
int can_node_number[4] = {7,7,6,6};
int can_channel_number = 4;

// CAN发送开关（远程帧无效）
//int can_switch[4][7] = {{1, 1, 1, 1, 1, 1, 1},
//						{1, 1, 1, 1, 1, 1, 1},
//						{1, 1, 1, 1, 1, 1, 1},
//						{1, 1, 1, 1, 1, 1, 1}};
/*int can_switch[4][7] = {{0, 0, 0, 0, 1, 1, 1},
						{0, 0, 0, 0, 1, 1, 1},
						{1, 1, 1, 1, 1, 1, 0},
						{1, 1, 1, 1, 1, 1, 1}};*/
int can_switch[4][7] = {{0, 0, 0, 0, 1, 1, 1},
						{1, 1, 1, 1, 0, 0, 0},
						{1, 1, 0, 0, 0, 0, 0},
						{1, 1, 0, 0, 0, 0, 1}};

// 各节点速度方向
double joint_direction[4][7] = {{1, 1, 1, 1, 1, -1, 1},
								{1, 1, 1, 1, 1, -1, 1},
								{1, -1, 1, -1, -1, -1},
								{-1, 1, -1, 1, -1, -1}};

// 各节点初始零位值
double home_offset[4][7] = {{0.0, 0.0, 0.0, 0.0, -18.6*Degree2Rad, -1.7*Degree2Rad, 0.0},
							{0.0, 0.0, 0.0, 0.0, 6.5*Degree2Rad, -2.7*Degree2Rad, 0.0},
							{2.4*Degree2Rad, -2.7*Degree2Rad, 2.7*Degree2Rad, -1.7*Degree2Rad, 0.0, 0.0, 0.0},
							{-1.4*Degree2Rad, -19.8*Degree2Rad, 0.0, -19.8*Degree2Rad, 0.0, 0.0, 0.0}};
// remote limit
// 各节点最大位置限位
/*double AngleMax_deg[4][7] = {{85, 60, 60, 60, 90, 60, 90},
							 {85, 60, 60, 60, 90, 60, 90},
							 {60, 15, 90, 80, 80, 80},
							 {15, 90, 15, 90, 30, 30}};

// 各节点最小位置限位
double AngleMin_deg[4][7] = {{0.0, 0.0, 0.0, 0.0, -90, -60, -90},
							 {0.0, 0.0, 0.0, 0.0, -90, -60, -90},
							 {-90, -80, -60, -15, -80, -80},
							 {-90, -90, -90, -90, -30, -30}};
*/
// 各节点最大位置限位
double AngleMax_deg[4][7] = {{180, 120, 120, 120, 160, 90, 160},
							 {180, 120, 120, 120, 160, 90, 160},
							 {90, 90, 90, 90, 80, 80},
							 {90, 90, 90, 90, 30, 30}};

// 各节点最小位置限位
double AngleMin_deg[4][7] = {{-2.0, 0.0, 0.0, 0.0, -160, -90, -160},
							 {-2.0, 0.0, 0.0, 0.0, -160, -90, -160},
							 {-90, -90, -90, -90, -80, -80},
							 {-90, -90, -90, -90, -30, -30}};

// 各节点最大运动速度限制	deg/s
double VelocityLimit_deg[4][7] = {{60, 60, 60, 60, 60, 60, 60},
							  	  {60, 60, 60, 60, 60, 60, 60},
							  	  {60, 60, 60, 60, 40, 40},
							  	  {60, 60, 60, 60, 20, 20}};
// 各节点运动错误指示
// 1——正向位置超限
// -1——反向位置超限
// 2——正向速度超限
// -2——反向速度超限
// 0——正常
int JointError[4][7] = {{0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0}};

// 各节点电机侧弧度与码盘计数换算关系（1rad对应码盘数）
double Rad2Count[4][7] = {{7.00281748,636.619772,636.619772,636.619772,2607.5945876176,2607.5945876176,2607.5945876176},
						  {7.00281748,636.619772,636.619772,636.619772,2607.5945876176,2607.5945876176,2607.5945876176},
 						  {2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,1303.7973,1303.7973,1000.0},
 						  {2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,651.8986,651.8986,0.0}};//651.898;

// 各节点减速比
double reduction_ratio[4][7] = {{34.0,100.0,100.0,100.0,160.0,160.0,160.0},
								{34.0,100.0,100.0,100.0,160.0,160.0,160.0},
								{160.0,160.0,160.0,160.0,21.0,21.0,100.0},
								{160.0,160.0,160.0,160.0,120.0,120.0}};
double zero_comp[4][7] = {0.0};
// 各节点CAN工作状态
int can_work_states[27] = {0};
// 各节点电流值
double motor_current[4][7] = {0.0};
// 各节点电流参考值
double motor_current_refrence[4][7] = {{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
									   {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
									   {5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
									   {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};
// 各节点速度限制设定值
long motor_sp[4][7] = {{40, 	5500,	5500,	5500, 	50000,	50000,	50000},
					   {40, 	5500,	5500,	5500, 	50000,	50000,	50000},
					   {50000, 	50000, 	50000,	50000, 	500,	500},
					   {50000, 	50000,	50000,	50000, 	500,	500}};

// 各节点角度反馈弧度值
double Joint_Angle_FB[4][7] = {0.0};
// 各节点角度期望弧度值
double Joint_Angle_EP[4][7] = {0.0};
// 各节点角度反馈角度值
double Joint_Angle_FB_degree[4][7] = {0.0};
// 各节点角度期望角度值
double Joint_Angle_EP_degree[4][7] = {0.0};
// 各节点角度误差角度值
double Joint_Angle_ER_degree[4][7] = {0.0};
// 各节点前一次期望角度弧度值
double Joint_Angle_LastEP[4][7] = {0.0};
// 各节点前一次期望角度角度值
double Joint_Angle_LastEP_degree[4][7] = {0.0};

//
struct RealRobot_Struct RobotAngleFB;
struct RealRobot_Struct RobotAngleEP;
struct RealRobot_Struct RobotAngleFBDeg;
struct RealRobot_Struct RobotAngleEPDeg;
struct RealRobot_Struct RobotAngleERDeg;
struct RealRobot_Struct RobotRealCurrent;

// 头部绝对码盘角度弧度值
double Head_Angle_FB[2] = {0.0};
// 头部绝对码盘角度角度值
double Head_Angle_FB_degree[2] = {0.0};
// 供电电源电压反馈
double VoltageFB = 0.0;
// 供电电源电流反馈
double CurrentFB = 0.0;

double T_hand_end[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -hand2base},
						{0.0, 0.0, 0.0, 1.0}};

double T_hand_end_inv[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, hand2base},
						{0.0, 0.0, 0.0, 1.0}};

// 标志量
int power_on_flag = 0;
int servo_on_flag = 0;
int motion_enable_flag= 0;
int RemoteMotion_enable_flag = 0;

// UDP通信数据
float JointMoveData = 0.0;
float RemoteMotionData[14] = {0.0};
float RemoteMotionDataLast[14] = {0.0};
short rockerL = 0;
short rockerR = 0;
int RemoteNewData = 0;
int NoCollisionFlag = 1;
int RemoteStep = 0;

float HandAngleL = 0.0;
float HandAngleR = 0.0;
int HandSelect = 0;
int HandNewData = 0;

int ArmSelect = 0;
float One_arm_Data[7] = {0.0};

/********************** 力控制相关 Start ***************************/
double A6D_ep[6] = {1.1, 1.1, 1.1, 1.1, 1.1, 1.1};				/*阻尼比*/
double A6D_m[6] = {180, 180, 300, 15, 15, 15};				/*质量*/
double A6D_K[6] = {150, 150, 200, 3, 3, 3};			/*刚度A6D_wn.^2.*A6D_m*/
double A6D_C[6] = {1, 1, 1, 1, 1, 1};			/*阻尼系数 2.*A6D_ep.*A6D_wn.*A6D_m      ep*2*sqrt(k*m)*/
int A6D_enable[6] = {1, 1, 1, 1, 1, 1};		/*control end velocity enable*/

double FDeltaT = 0.006;		/*力控周期*/
double AccL[6] = {0.0};
double AccR[6] = {0.0};
double A6D_D_totalL[6] = {0.0};
double A6D_D_totalR[6] = {0.0};
double A6D_VL[6] = {0.0};
double A6D_VR[6] = {0.0};
double A6D_PL[6] = {0.0};
double A6D_PR[6] = {0.0};
double A6D_Joint_PL[7] = {0.0};
double A6D_Joint_PR[7] = {0.0};
double A6D_Joint_VL[7] = {0.0};
double A6D_Joint_VR[7] = {0.0};
double force_velocity_limit[6] = {20.0, 20.0, 20.0, 0.1, 0.1, 0.1};
/********************** 力控制相关 End ***************************/

int DutyStep = 0;
int DutyForceFlag = 0;
/********************** UDP通讯相关 Start ***************************/
int UDPTimes = 0;	// UDP 周期计数

/********************** UDP通讯相关 End ***************************/

struct Cubic_Struct cubic[14];
/**********************  VisionControl Start ***********************/
float DeltaMatrix[4][4] = {0.0};
/**********************  VisionControl End ***********************/

int AllJointMove(struct RealRobot_Struct RealPos, double time, int joint, int hand, int head, int waist)
{
	static double start_position[4][7];
	double TargetPos[4][7];
	double PlanPos[4][7];
	int i, j;
	static int first_time_AllJointMove = 0;
	double move_time;
	static double t = 0.0;
	struct timespec sleeptime;

	memset(&PlanPos,0,sizeof(PlanPos));
	memset(&TargetPos,0,sizeof(TargetPos));
	RealRobot2CanDef(RealPos, TargetPos);
	move_time = time;

	if(first_time_AllJointMove == 0)
	{
		for(i=0; i<4; i++)
		{
			for(j=0; j<7; j++)
			{
				start_position[i][j] = Joint_Angle_FB[i][j];

			}
		}
		first_time_AllJointMove = 1;
		t = 0.0;
		return 1;
	}
	else
	{
		if(t <= move_time)
		{
			t = t + time_interval;
			for(i=0; i<4; i++)
			{
				for(j=0; j<7; j++)
				{
					PlanPos[i][j] = Five_Interpolation(start_position[i][j], 0, 0, TargetPos[i][j], 0, 0, move_time,t);

					if(t > move_time)
					{
						PlanPos[i][j] = TargetPos[i][j];
					}

				}
			}

			if (joint)
			{
				for(i=0; i<2; i++)
				{
					Joint_Angle_EP[i][4] = JointDetect(i, 4, PlanPos[i][4]);
					Joint_Angle_EP[i][5] = JointDetect(i, 5, PlanPos[i][5]);
					Joint_Angle_EP[i][6] = JointDetect(i, 6, PlanPos[i][6]);
					Joint_Angle_EP[i+2][0] = JointDetect(i+2, 0, PlanPos[i+2][0]);
					Joint_Angle_EP[i+2][1] = JointDetect(i+2, 1, PlanPos[i+2][1]);
					Joint_Angle_EP[i+2][2] = JointDetect(i+2, 2, PlanPos[i+2][2]);
					Joint_Angle_EP[i+2][3] = JointDetect(i+2, 3, PlanPos[i+2][3]);
				}
			}

			if(hand)
			{
				for (i=0; i<2; i++)
				{
					Joint_Angle_EP[i][0] = JointDetect(i, 0, PlanPos[i][0]);
					Joint_Angle_EP[i][1] = JointDetect(i, 1, PlanPos[i][1]);
					Joint_Angle_EP[i][2] = JointDetect(i, 2, PlanPos[i][2]);
					Joint_Angle_EP[i][3] = JointDetect(i, 3, PlanPos[i][3]);
				}
			}

			if (head)
			{
				Joint_Angle_EP[2][4] = JointDetect(2, 4, PlanPos[2][4]);
				Joint_Angle_EP[2][5] = JointDetect(2, 5, PlanPos[2][5]);
			}

			if (waist)
			{
				Joint_Angle_EP[3][4] = JointDetect(3, 4, PlanPos[3][4]);
				Joint_Angle_EP[3][5] = JointDetect(3, 5, PlanPos[3][5]);
			}

			if(motion_enable_flag == 1)
			{
				for(j=0; j<6; j++)
				{
					for(i=0; i<4; i++)
					{
						rad_send(i, j, Joint_Angle_EP[i][j]);
						sleeptime.tv_nsec = 5000;
						sleeptime.tv_sec = 0;
						nanosleep(&sleeptime,NULL);
					}
					sleeptime.tv_nsec = 200000;
					sleeptime.tv_sec = 0;
					nanosleep(&sleeptime,NULL);
				}
				rad_send(0, 6, Joint_Angle_EP[0][6]);
				sleeptime.tv_nsec = 5000;
				sleeptime.tv_sec = 0;
				nanosleep(&sleeptime,NULL);
				rad_send(1, 6, Joint_Angle_EP[1][6]);
				sleeptime.tv_nsec = 100000;
				sleeptime.tv_sec = 0;
				nanosleep(&sleeptime,NULL);
			}

			return 1;
		}
		else
		{
			first_time_AllJointMove = 0;
			return 0;
		}
	}

}


void send_event()
{
	XClientMessageEvent xevent;

	// Send a ClientMessage
	xevent.type = ClientMessage;
	xevent.window = MyWindow;

	// Use the Atom we got for our custom message
	xevent.message_type = MyAtom;
	// I don't really use these fields, but here's an example
	// of setting them
	xevent.format = 32;
	xevent.data.l[0] = 0;

	// Send the ClientMessage event
	XSendEvent(MyDisplay, MyWindow, 0, NoEventMask, (XEvent *)&xevent);
	XFlush(MyDisplay);
}


void view (void *n)
{
	struct sched_param the_priority;

	XEvent event;
	sem_post(&low_go);
	char ch[1];
     int num;
	int view_running = 1;
	long can_content[8] = {0x11, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	long zero_content[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	long pause_order1[8] = {0xD1, 0x11, 0x55, 0x00, 0x00, 0x00, 0x00, 0x37};
	long pause_order2[8] = {0xD1, 0x22, 0x55, 0x00, 0x00, 0x00, 0x00, 0x48};
	long pause_order3[8] = {0xD1, 0x11, 0x55, 0x00, 0x00, 0x00, 0x00, 0x37};
	long pause_order4[8] = {0xD1, 0x22, 0x55, 0x00, 0x00, 0x00, 0x00, 0x48};
	long pause_order5[8] = {0xD1, 0x11, 0x55, 0x00, 0x00, 0x00, 0x00, 0x37};

	int j,k, i = 0;
	int DispHead = 30;

	while(view_running)
	{
		XNextEvent(MyDisplay, &event);


		if(event.type == KeyPress)
		{

			KeySym keysym;
			XLookupString(&event.xkey,ch,1,&keysym,NULL);



			printf("keypress  %c\n", ch[0]);

			switch(ch[0])
			{
				case 's':	// 关伺服
				case 'S':
					printf("servo off\n");
					servo_off();
					servo_on_flag  = 0;
				break;

				case '1':	// 开伺服
         			printf("servo on\n");
         			servo_on();
					servo_on_flag = 1;
				break;

				case 'i':
				case 'I':
					elmo_init();
					printf("elmo init\n");
				break;

				case '2':	// get current joint pos
					printf("power on\n");
					power_on();
					power_on_flag = 1;

				break;

				case '3':
					printf("power off\n");
					power_off();
					power_on_flag = 0;

				break;

				case '4':
					printf("clear zero\n");
	 				for(i=0; i<4; i++)
	 				{
	 					for(j=0; j<4; j++)
	 					{
	 						zero_comp[i][j] = home_offset[i][j] * joint_direction[i][j];
	 					}
	 				}

				break;

				case 'e':	// 关程序
				case 'E':
					//add code to stop the motors
					XCloseDisplay(MyDisplay);
					view_running = 0;
					canrv_running = 0;
					rt_task_delete(&rt_task_view);
					rt_task_delete(&demo_task_rvcan);

				break;

				case '5':
					motion_enable_flag =1;
					printf("motion_enable_flag = 1\n");

				break;

				case '6':
					motion_enable_flag = 0;
					printf("motion_enable_flag = 0\n");
				break;

				case '8':
					motion_mode = PREPARE_FIND_HOME;
					printf("prepare find home\n");
				break;

				case '9':
					motion_mode = RETURN_ORIGIN_POSITION;
					printf("return_origin_position\n");
				break;

			}

		}

		if(event.type == ClientMessage)
		{
			XClientMessageEvent *evt;
			evt = (XClientMessageEvent *)&event;
			// User wants to close the window?
			if (evt->message_type == wm_protocols && evt->data.l[0] == wm_delete_window)
			{
				XCloseDisplay(MyDisplay);
				rt_task_delete(&rt_task_view);
				rt_task_delete(&demo_task_rvcan);
			}
			// Is it my custom message?
			if (evt->message_type == MyAtom)
			{
			// Here I would do something about my custom message
				XClearWindow(MyDisplay, MyWindow);

				sprintf(buf, "Loop time : %ldus   runtime: %5.2fs", (long)period, runtime);
				if(power_on_flag == 1)
					sprintf(buf1, "power on");
				else
					sprintf(buf1, "power off");
				if(servo_on_flag == 1)
					sprintf(bufs1, "servo on");
				else
					sprintf(bufs1, "servo off");
				if(motion_enable_flag == 1)
					sprintf(bufs2, "motion enable");
				else
					sprintf(bufs2, "motion disable");

				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead, buf, strlen(buf));
				XDrawString(MyDisplay, MyWindow, MyGC, 400, DispHead, buf1, strlen(buf1));
				XDrawString(MyDisplay, MyWindow, MyGC, 520, DispHead, bufs1, strlen(bufs1));
				XDrawString(MyDisplay, MyWindow, MyGC, 640, DispHead, bufs2, strlen(bufs2));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+25, buf2, strlen(buf2));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+50, buf3, strlen(buf3));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+75, buf4, strlen(buf4));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+100, buf5, strlen(buf5));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+125, buf6, strlen(buf6));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+150, buf7, strlen(buf7));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+175, buf8, strlen(buf8));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+200, buf9, strlen(buf9));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+225, buf10, strlen(buf10));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+250, buf11, strlen(buf11));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+275, buf12, strlen(buf12));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+300, buf13, strlen(buf13));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+325, buf14, strlen(buf14));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+350, buf15, strlen(buf15));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+375, buf16, strlen(buf16));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+400, buf17, strlen(buf17));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+425, buf18, strlen(buf18));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+450, buf19, strlen(buf19));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+475, buf20, strlen(buf20));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+500, buf21, strlen(buf21));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+525, buf22, strlen(buf22));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+550, buf23, strlen(buf23));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+575, buf24, strlen(buf24));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+600, buf25, strlen(buf25));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+625, buf26, strlen(buf26));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+650, buf27, strlen(buf27));
			}
		}
	}


	XCloseDisplay(MyDisplay);
	printf("close windows\n");
//	timer_delete(timerid);

}


int can_send(int channel, long can_id, int can_mode, long can_content[8], int can_lenth)
{

    int i, j, ret, opt;
    int can_id_new = 0;
    int can_send_flag = 0;

    can_id_new = (can_id & 0x00F) - 1;
    if(can_id_new < 0)
    		can_send_flag = 1;
    else if( can_id_new < 7)
    {
   	 	if(can_switch[channel][can_id_new] == 1)
		{
			can_send_flag = 1;
		}
    }


	if(can_send_flag == 1)
	{
		frame.can_id = can_id;

		if (can_mode == 0)
		{
			frame.can_dlc = can_lenth;
		}

		if (can_mode == 1)
		{
			frame.can_id |= CAN_RTR_FLAG;
		}

		if (can_mode == 2)
		{
			frame.can_id |= CAN_EFF_FLAG;
		}


		for(i=0;i<can_lenth;i++)
		{
			frame.data[i] = can_content[i];
		}

	    /* Note: sendto avoids the definiton of a receive filter list */


		to_addr.can_ifindex = ifr[channel].ifr_ifindex;
		to_addr.can_family = AF_CAN;

		ret = rt_dev_sendto(txsock[channel], (void *)&frame, sizeof(can_frame_t), 0,
		                   (struct sockaddr *)&to_addr, sizeof(to_addr));
	//  printf("txsock[%d]=%d\n",channel,txsock[channel]);
	//	printf("ret = %d\n", ret);
		if (ret < 0)
		{
			switch (ret) {
			case -ETIMEDOUT:
				if (verbose)
			//		printf("rt_dev_send(to): timed out\n");
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_send(to): aborted because socket was closed\n");
				break;
			default:
				fprintf(stderr, "rt_dev_send: %s\n", strerror(-ret));
				break;
			}

		}
		return 0;
	}
}

void SYNC_Receive(void)
{
	struct timespec sleeptime;
	long count_out[8] = {0};
	long can_id = 0;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;
	int i, j;
	int canRecvStatus[4];
	int motor_current_feedback[4][7] = {0};
	int Encoder_Count_FB[4][7] = {0};
	now2 = rt_timer_read();

	// receive Motor Driver data
	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;

	for(i=0; i<4; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, count_out, 0);   //SYNC
		nanosleep(&sleeptime,NULL);
	}

	sleeptime.tv_nsec = 900000;		//moyang602  sleeptime.tv_nsec = 2000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;

	// 保存上次状态
	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			Joint_Angle_LastEP[i][j] = Joint_Angle_EP[i][j];
			Joint_Angle_LastEP_degree[i][j] = Joint_Angle_EP_degree[i][j];
		}
	}

	// 获取最新状态
	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
			nanosleep(&sleeptime,NULL);
			if (ret < 0)
			{
				switch (ret)
				{
					case -ETIMEDOUT:
						if (verbose)
					//		printf("rt_dev_recv: timed out\n");
							canRecvStatus[i] = -1;
						break;
					case -EBADF:
						if (verbose)
							printf("rt_dev_recv: aborted because socket was closed");
							canRecvStatus[i] = -2;
						break;
					default:
						fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
						canRecvStatus[i] = -3;
						break;
				}
			}
			else
			{
				Encoder_Count_FB[i][(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB[i][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[i][(frame.can_id & 0xf)-1])/Rad2Count[i][(frame.can_id & 0xf)-1]/reduction_ratio[i][(frame.can_id & 0xf)-1] - zero_comp[i][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1];
				Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1]*Rad2Degree;

				if( (frame.data[7] & 0x80) == 0x80)
					motor_current_feedback[i][(frame.can_id & 0xf)-1] = -(~((int)frame.data[7]*256 + (int)frame.data[6] -1) & 0x0000FFFF);
				else
					motor_current_feedback[i][(frame.can_id & 0xf)-1] = frame.data[6] + frame.data[7]*256;

				motor_current[i][(frame.can_id & 0xf)-1] = motor_current_feedback[i][(frame.can_id & 0xf)-1]*0.001*motor_current_refrence[i][(frame.can_id & 0xf)-1];
			}
		}
	}

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			Joint_Angle_EP_degree[i][j] = Joint_Angle_EP[i][j] * Rad2Degree;
			Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];
		}
	}


	// Change Format
	memset(&RobotAngleFB,0,sizeof(RobotAngleFB));
	memset(&RobotAngleEP,0,sizeof(RobotAngleEP));
	memset(&RobotAngleFBDeg,0,sizeof(RobotAngleFBDeg));
	memset(&RobotAngleEPDeg,0,sizeof(RobotAngleEPDeg));
	memset(&RobotAngleERDeg,0,sizeof(RobotAngleERDeg));

	CanDef2RealRobot(Joint_Angle_FB, &RobotAngleFB);
	CanDef2RealRobot(Joint_Angle_EP, &RobotAngleEP);
	CanDef2RealRobot(Joint_Angle_FB_degree, &RobotAngleFBDeg);
	CanDef2RealRobot(Joint_Angle_EP_degree, &RobotAngleEPDeg);
	CanDef2RealRobot(Joint_Angle_ER_degree, &RobotAngleERDeg);

	previous2 = rt_timer_read();
	period2 = (previous2 - now2) / 1000;   //us

/*
	// receive Head Encoder data
	sleeptime.tv_nsec = 30000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	count_out[0] = 0x3;
	count_out[1] = 0x3;
	can_send(2, 0x7, 0, count_out, 2);

	sleeptime.tv_nsec = 40000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);
	ret = rt_dev_recvfrom(rxsock[2], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	if (ret < 0)
	{
		switch (ret)
		{
			case -ETIMEDOUT:
				if (verbose)
			//		printf("rt_dev_recv: timed out\n");
					canRecvStatus[2] = -1;
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_recv: aborted because socket was closed");
					canRecvStatus[2] = -2;
				break;
			default:
				fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
				canRecvStatus[2] = -3;
				break;
		}
	}
	else
	{
		int HeadEncoder_Count_FB[2] = {0, 0};
		HeadEncoder_Count_FB[0] = frame.data[0]*256 + frame.data[1];
		HeadEncoder_Count_FB[1] = frame.data[2]*256 + frame.data[3];
		Head_Angle_FB[0] = (double)HeadEncoder_Count_FB[0]/Rad2Count[2][6];		// moyang602  zero
		Head_Angle_FB[1] = (double)HeadEncoder_Count_FB[1]/Rad2Count[2][6];		// moyang602  zero
		Head_Angle_FB_degree[0] = Head_Angle_FB[0]*Rad2Degree;
		Head_Angle_FB_degree[1] = Head_Angle_FB[1]*Rad2Degree;
	}

	// receive PowerController data
	sleeptime.tv_nsec = 30000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	count_out[0] = 0x49;
	count_out[1] = 0x55;
	can_send(3, 0x7, 0, count_out, 2);

	sleeptime.tv_nsec = 40000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);
	ret = rt_dev_recvfrom(rxsock[3], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	if (ret < 0)
	{
		switch (ret)
		{
			case -ETIMEDOUT:
				if (verbose)
			//		printf("rt_dev_recv: timed out\n");
					canRecvStatus[3] = -1;
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_recv: aborted because socket was closed");
					canRecvStatus[3] = -2;
				break;
			default:
				fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
				canRecvStatus[3] = -3;
				break;
		}
	}
	else
	{
		VoltageFB = (frame.data[1]*256 + frame.data[0])/1000.0;
		CurrentFB = (frame.data[3]*256 + frame.data[2])/1000.0;
	}

*/
}


void rad_send(int can_channel_num, int id, double angle_in)
{
	double Encoder_Count_EP2 = 0.0;
	long Encoder_Count_EP_Send2 = 0.0;
	struct timespec sleeptime;
	long count_out[8] = {0};
	long can_id = 0;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;
	int can_node_num = 4;
	int i, j;
	int canRecvStatus[4];

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	Encoder_Count_EP2 = (angle_in * joint_direction[can_channel_num][id] + zero_comp[can_channel_num][id])* Rad2Count[can_channel_num][id] * reduction_ratio[can_channel_num][id];

	Encoder_Count_EP_Send2 = Encoder_Count_EP2;

	count_out[0] = 0X1F;
	count_out[1] = 0X00;
	count_out[2] = Encoder_Count_EP_Send2 & 0x000000FF;
	count_out[3] = (Encoder_Count_EP_Send2 & 0x0000FF00)>>8;
	count_out[4] = (Encoder_Count_EP_Send2 & 0x00FF0000)>>16;
	count_out[5] = (Encoder_Count_EP_Send2 & 0xFF000000)>>24;

	can_id = 0x200 + id +1;

	can_send(can_channel_num, can_id, 0, count_out, 6);

}

void find_home(int channel_num, int id_num)
{
	struct timespec sleeptime;

	long can_id = 0x600 + id_num + 1;

	long can_initiates_OS[8] = {0x23, 0x24, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
	long can_Segmented_SDO[8] = {0x21, 0x23, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00};
	long can_XQ[5] = { 0x07, 0x58, 0x51, 0x23, 0x23};
	int canRecvStatus[4];

	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;

	if(id_num == 9)
	{
		can_id = 0x600;
	}

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us
	can_send(channel_num, can_id, 0, can_initiates_OS, 8);   //SYNC

	printf("send can can_initiates_OS\n");

	sleeptime.tv_nsec = 700000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 400us

	ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	if (ret < 0)
	{
		switch (ret)
		{
			case -ETIMEDOUT:
				if (verbose)
					printf("rt_dev_recv: timed out\n");
					canRecvStatus[channel_num] = -1;
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_recv: aborted because socket was closed");
					canRecvStatus[channel_num] = -2;
				break;
			default:
				fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
				canRecvStatus[channel_num] = -3;
				break;
		}
	}
	else
	{
		if(frame.data[0] != 0x60)
			printf("client initiate OS ERROR");
	}

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us
	can_send(channel_num, can_id, 0, can_Segmented_SDO, 8);   //SYNC

	printf("send can_Segmented_SDO\n");

	sleeptime.tv_nsec = 700000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 400us

	ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	if (ret < 0)
	{
		switch (ret)
		{
			case -ETIMEDOUT:
				if (verbose)
					printf("rt_dev_recv: timed out\n");
					canRecvStatus[channel_num] = -1;
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_recv: aborted because socket was closed");
					canRecvStatus[channel_num] = -2;
				break;
			default:
				fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
				canRecvStatus[channel_num] = -3;
				break;
		}
	}
	else
	{
		if(frame.data[0] != 0x60)
			printf("client initiate segmented SDO ERROR");
	}

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us
	can_send(channel_num, can_id, 0, can_XQ, 5);   //SYNC

	printf("send can_XQ\n");

	sleeptime.tv_nsec = 750000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 400us

	ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	if (ret < 0)
	{
		switch (ret)
		{
			case -ETIMEDOUT:
				if (verbose)
					printf("rt_dev_recv: timed out\n");
					canRecvStatus[channel_num] = -1;
				break;
			case -EBADF:
				if (verbose)
					printf("rt_dev_recv: aborted because socket was closed");
					canRecvStatus[channel_num] = -2;
				break;
			default:
				fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
				canRecvStatus[channel_num] = -3;
				break;
		}
	}
	else
	{
		if(frame.data[0] != 0x20)
			printf("client send XQ ERROR");
	}

}

void rt_can_recv(void *arg)
{
	// trajectory planning var
	int i,j = 0;
	struct timespec sleeptime;

	double t = 0.0;

	double start_position[4][7] = {0.0};

	struct RealRobot_Struct RemoteRobotPos_deg;
	struct RealRobot_Struct RemoteCrtPos_deg;
	double RemotePlanPos_deg[14];

	struct RealRobot_Struct OneArmStart;
	struct RealRobot_Struct RealTargetPos;

	double end_position[4][4] = {{-0.510793, 0, 0.701553, 0.0},{0.510793, 0.0, 0.371256, 0.0},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};

	double T_END_main[4][4] = {{1.0, 0.0, 0.0, 600.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -24.0},
						{0.0, 0.0, 0.0, 1.0}};
	double T_END2[4][4] = {{1.0, 0.0, 0.0, 600.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -24.0},
						{0.0, 0.0, 0.0, 1.0}};

	int fisrt_time_SINGLE_JOINT_MOTION = 0;
	int fisrt_time_ONE_ARM_MOTION = 0;
	int first_time_TWO_ARMS_MOTION = 0;
	int first_time_HOMEBACK = 0;
	int first_time_VISION_MOTION = 0;

	int first_time_HANDCMD_ZERO = 0;
	int first_time_HANDCMD_CYLINDER = 0;
	int first_time_HANDCMD_CYLINDER_PRE = 0;
	int first_time_HANDCMD_SPHERE = 0;
	int first_time_HANDCMD_SPHERE_PRE = 0;

	double singe_joint_time = 5.0;
	double one_arm_time = 15.0;
	double two_arms_time = 30.0;
	double homeback_time = 10.0;
	double vision_motion_time = 3.0;

	double T_Now[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, 0.0},
						{0.0, 0.0, 0.0, 1.0}};;
	double T_Now2[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, 0.0},
						{0.0, 0.0, 0.0, 1.0}};

	double T_Now1[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, 0.0},
						{0.0, 0.0, 0.0, 1.0}};
	double anglein[8] = {0.5, 0.0, 0.2, 0.0, -0.5, 0.0, -0.2, 0.0};
	double angle_out2[8];

	int moving_flag = 0;   //when moving_flag == 0; motion_mode can be changed

	double angle_2leg[6] = {0.0};
	double delta_matrix2[4][4];

	int return_value = 1;

	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;

	int first_move_flag = 1;	// used for move CMD
	rt_task_set_periodic(NULL, TM_NOW, 6000000);
	RTIME LastTime, NowTime;

/**********************************************************************************/
	//    开始循环
/**********************************************************************************/
	FILE *fp;
	fp = fopen("current.txt","w");
	double Posture[3]={0.0, 0.0, 0.0};
	double latitude = 0;
	double longitude = 0;
	int n_gps;
	while(canrv_running)
	{
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		period = (now - previous) / 1000;   //us
		previous = now;

		if(view_time > 10)
		{
			send_event();
			view_time = 0;
		}
		view_time++;

		runtime = runtime + time_interval;

		return_value = JY901_GetData(Posture);
		if (return_value > 0)
		{
		//	printf("rtn = %d, Pos1 = %lf, Pos2 = %lf, Pos3 = %lf\n", return_value, Posture[0], Posture[1], Posture[2]);
		}
		n_gps++;
		if(n_gps>333)
		{
			return_value = GPS_GetData(&latitude, &longitude);
		//	printf("==   纬度 : 北纬:%d度%d分%d秒                              \n", ((int)latitude) / 100, (int)(latitude - ((int)latitude / 100 * 100)), (int)(((latitude - ((int)latitude / 100 * 100)) - ((int)latitude - ((int)latitude / 100 * 100))) * 60.0));
   		//	printf("==   经度 : 东经:%d度%d分%d秒                              \n", ((int)longitude) / 100, (int)(longitude - ((int)longitude / 100 * 100)), (int)(((	longitude - ((int)longitude / 100 * 100)) - ((int)longitude - ((int)longitude / 100 * 100))) * 60.0));
			n_gps = 0;
		}

		switch (control_mode)
		{
			case CMD_POWER_ON:
				printf("power on by UDP\n");
				power_on();
				power_on_flag = 1;
				control_mode = 0;
			break;

			case CMD_POWER_OFF:
				printf("power off by UDP\n");
				power_off();
				power_on_flag = 0;
				control_mode = 0;
			break;

			case CMD_ELMO_INIT:
				elmo_init();
				printf("elmo init by UDP\n");
				control_mode = 0;
			break;

			case CMD_SERVO_ON:
				printf("servo on by UDP\n");
     			servo_on();
				servo_on_flag = 1;
				control_mode = 0;
			break;

			case CMD_SERVO_OFF:
				printf("servo off by UDP\n");
				servo_off();
				servo_on_flag  = 0;
				control_mode = 0;
			break;

			case CMD_CTR_ENABLE:
			{
				motion_enable_flag =1;
				// 避免仿真运动切换到真实运动时出现大幅度运动的现象
				for(i=0; i<can_channel_number; i++)
				{
					for(j=0; j< can_node_number[i]; j++)
					{
						Joint_Angle_LastEP[i][j] = Joint_Angle_FB[i][j];
						Joint_Angle_EP[i][j] = Joint_Angle_FB[i][j];
						Joint_Angle_LastEP_degree[i][j] = Joint_Angle_FB_degree[i][j];
						Joint_Angle_EP_degree[i][j] = Joint_Angle_FB_degree[i][j];
					}
				}
				printf("motion_enable_flag = 1 by UDP\n");
				control_mode = 0;
			}
			break;

			case CMD_CTR_DISENABLE:
				motion_enable_flag = 0;
				printf("motion_enable_flag = 0 by UDP\n");
				control_mode = 0;
			break;

			case CMD_HOMEPREPARE:
				motion_mode = PREPARE_FIND_HOME;
				printf("find home prepare\n");
				control_mode = 0;
			break;

			case CMD_HOMEZERO:
				// 避免仿真运动切换到真实运动时出现大幅度运动的现象
				for(i=0; i<can_channel_number; i++)
				{
					for(j=0; j< can_node_number[i]; j++)
					{
						Joint_Angle_LastEP[i][j] = Joint_Angle_FB[i][j];
						Joint_Angle_EP[i][j] = Joint_Angle_FB[i][j];
						Joint_Angle_LastEP_degree[i][j] = Joint_Angle_FB_degree[i][j];
						Joint_Angle_EP_degree[i][j] = Joint_Angle_FB_degree[i][j];
					}
				}
				motion_mode = HOMEBACK;
				printf("home offset\n");
				control_mode = 0;
			break;

			case CMD_RESET:
				motion_mode = RETURN_ZERO;
				printf("return to zero postion\n");
				control_mode = 0;
			break;

			case CMD_BACKORIGIN:
				motion_mode = RETURN_ORIGIN_POSITION;
				printf("return origin position\n");
				control_mode = 0;
			break;

			case CMD_MOVEPRE:
				motion_mode = MOVE_PRE_POSITION;
				printf("move to prepare position\n");
				control_mode = 0;
			break;

			default:
			break;
		}

		if(servo_on_flag == 1) // when servo on, the motion control is able
		{
/************************* 四通道CAN数据接收**************************/
			switch (motion_mode)
			{
				case SINGLE_JOINT_MOTION:

					if(fisrt_time_SINGLE_JOINT_MOTION == 0)
					{
						start_position[can_channel_main][can_id_main] = Joint_Angle_FB[can_channel_main][can_id_main];
						fisrt_time_SINGLE_JOINT_MOTION = 1;
						t = 0;
						printf("MOTION_MODE: SINGLE_JOINT_MOTION\n");
						moving_flag = 1;
					}
					else
					{
						if(t <= singe_joint_time)
						{
							t = t+time_interval;
							float angleplan = 0.0;
							if(t > singe_joint_time)
							{
								angleplan = start_position[can_channel_main][can_id_main]+JointMoveData;
							}
							else
							{
								angleplan = Five_Interpolation(start_position[can_channel_main][can_id_main],0,0,start_position[can_channel_main][can_id_main]+JointMoveData,0,0,singe_joint_time,t);
							}

							Joint_Angle_EP[can_channel_main][can_id_main] = JointDetect(can_channel_main, can_id_main, angleplan);



							if(motion_enable_flag == 1)
							{
								rad_send(can_channel_main,can_id_main,Joint_Angle_EP[can_channel_main][can_id_main]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							fisrt_time_SINGLE_JOINT_MOTION = 0;
							moving_flag =0;
						}

					}


				break;

				case REMOTE_MOTION:
				{
					int i_R;
					static int firsttimeflag = 1;
					if(RemoteMotion_enable_flag == 0)
					{
						memcpy(&RemoteRobotPos_deg,&RobotAngleFBDeg,sizeof(RemoteRobotPos_deg));
						memset(&RemoteCrtPos_deg,0,sizeof(RemoteCrtPos_deg));
						for(i_R=0;i_R<7;i_R++)
						{
							RemoteCrtPos_deg.LeftArm[i_R] = RemoteMotionData[i_R];
							RemoteCrtPos_deg.RightArm[i_R] = RemoteMotionData[i_R+7];
							RemoteMotionDataLast[i_R] = RemoteMotionData[i_R];
							RemoteMotionDataLast[i_R+7] = RemoteMotionData[i_R+7];

							RemotePlanPos_deg[i_R] = RemoteRobotPos_deg.LeftArm[i_R];
							RemotePlanPos_deg[i_R+7] = RemoteRobotPos_deg.RightArm[i_R];
						}

						for (i_R = 0; i_R < 14; i_R++)
						{
							memset(&cubic[i_R],0,sizeof(cubic[i_R]));
							cubic[i_R].needNextPoint = 1;
							cubic[i_R].segmentTime = 0.24;
							cubic[i_R].interpolationRate = 0.24/time_interval + 1;
							cubic[i_R].interpolationIncrement = 0.24/(double)(cubic[i_R].interpolationRate - 1);
						}
						firsttimeflag = 1;
					}
					else
					{
						while(cubic[1].needNextPoint)
						{
							if (RemoteNewData==1&&firsttimeflag==0)
							{
								for (i_R = 0; i_R < 14; i_R++)
								{
									if (RemotePlanPos_deg[i_R] - RemoteMotionData[i_R]>10*0.24)
									{
										RemotePlanPos_deg[i_R] = RemotePlanPos_deg[i_R] - 10*0.24;
									}
									else if (RemotePlanPos_deg[i_R] - RemoteMotionData[i_R]<-10*0.24)
									{
										RemotePlanPos_deg[i_R] = RemotePlanPos_deg[i_R] + 10*0.24;
									}
									else
									{
										RemotePlanPos_deg[i_R] = RemoteMotionData[i_R];
									}

									cubicAddPoint(i_R,RemotePlanPos_deg[i_R]);
								}

								RemoteNewData = 0;
							}
							else
							{
								for (i_R = 0; i_R < 14; i_R++)
								{
									cubicAddPoint(i_R,RemotePlanPos_deg[i_R]);
								}
								firsttimeflag = 0;
							}

						}

						for (i_R = 0; i_R < 7; i_R++)
						{
							RemoteRobotPos_deg.LeftArm[i_R] = cubicInterpolate(i_R);
							RemoteRobotPos_deg.RightArm[i_R] = cubicInterpolate(i_R+7);
						}

						Joint_Angle_EP[2][0] = JointDetect(2, 0, RemoteRobotPos_deg.LeftArm[0]*Degree2Rad);
						Joint_Angle_EP[3][0] = JointDetect(3, 0, RemoteRobotPos_deg.LeftArm[1]*Degree2Rad);
						Joint_Angle_EP[3][1] = JointDetect(3, 1, RemoteRobotPos_deg.LeftArm[2]*Degree2Rad);
						Joint_Angle_EP[2][1] = JointDetect(2, 1, RemoteRobotPos_deg.LeftArm[3]*Degree2Rad);
						Joint_Angle_EP[0][4] = JointDetect(0, 4, RemoteRobotPos_deg.LeftArm[4]*Degree2Rad);
						Joint_Angle_EP[0][5] = JointDetect(0, 5, RemoteRobotPos_deg.LeftArm[5]*Degree2Rad);
						Joint_Angle_EP[0][6] = JointDetect(0, 6, RemoteRobotPos_deg.LeftArm[6]*Degree2Rad);

						Joint_Angle_EP[2][2] = JointDetect(2, 2, RemoteRobotPos_deg.RightArm[0]*Degree2Rad);
						Joint_Angle_EP[3][2] = JointDetect(3, 2, RemoteRobotPos_deg.RightArm[1]*Degree2Rad);
						Joint_Angle_EP[3][3] = JointDetect(3, 3, RemoteRobotPos_deg.RightArm[2]*Degree2Rad);
						Joint_Angle_EP[2][3] = JointDetect(2, 3, RemoteRobotPos_deg.RightArm[3]*Degree2Rad);
						Joint_Angle_EP[1][4] = JointDetect(1, 4, RemoteRobotPos_deg.RightArm[4]*Degree2Rad);
						Joint_Angle_EP[1][5] = JointDetect(1, 5, RemoteRobotPos_deg.RightArm[5]*Degree2Rad);
						Joint_Angle_EP[1][6] = JointDetect(1, 6, RemoteRobotPos_deg.RightArm[6]*Degree2Rad);

				//		fprintf(fp, "%8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n", Joint_Angle_EP[2][0]*Rad2Degree, Joint_Angle_EP[3][0]*Rad2Degree, Joint_Angle_EP[3][1]*Rad2Degree, Joint_Angle_EP[2][1]*Rad2Degree, Joint_Angle_EP[0][4]*Rad2Degree, Joint_Angle_EP[0][5]*Rad2Degree, Joint_Angle_EP[0][6]*Rad2Degree);

						int rtnn = 0;
						struct RealRobot_Struct SendAngle;
						memset(&SendAngle,0,sizeof(SendAngle));
						CanDef2RealRobot(Joint_Angle_EP, &SendAngle);
						rtnn = CollisionDetection(SendAngle.LeftArm, SendAngle.RightArm, SendAngle.Waist);
						if (rtnn == 0)
						{
							printf("Detect collision!\n");
							NoCollisionFlag = 1;
							RemoteMotion_enable_flag = 0;
						}
						else
						{
						//	printf("No collision!\n");
							NoCollisionFlag = 1;
						}

						if(motion_enable_flag == 1&&NoCollisionFlag==1)
						{
							rad_send(0, 4, Joint_Angle_EP[0][4]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(0, 5, Joint_Angle_EP[0][5]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(0, 6, Joint_Angle_EP[0][6]);
							sleeptime.tv_nsec = 5000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(2, 0, Joint_Angle_EP[2][0]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(2, 1, Joint_Angle_EP[2][1]);
							sleeptime.tv_nsec = 5000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(3, 0, Joint_Angle_EP[3][0]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(3, 1, Joint_Angle_EP[3][1]);
							sleeptime.tv_nsec = 5000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);

							rad_send(1, 4, Joint_Angle_EP[1][4]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(1, 5, Joint_Angle_EP[1][5]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(1, 6, Joint_Angle_EP[1][6]);
							sleeptime.tv_nsec = 5000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(2, 2, Joint_Angle_EP[2][2]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(2, 3, Joint_Angle_EP[2][3]);
							sleeptime.tv_nsec = 5000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(3, 2, Joint_Angle_EP[3][2]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
							rad_send(3, 3, Joint_Angle_EP[3][3]);
							sleeptime.tv_nsec = 200000;
							sleeptime.tv_sec = 0;
							nanosleep(&sleeptime,NULL);
						}

						double AngleH[2][4] = {{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0}};
						control_handL(rockerL, motor_current[0][1], motor_current[0][2], motor_current[0][3], AngleH[0]);
						control_handR(rockerR, motor_current[1][1], motor_current[1][1], motor_current[1][1], AngleH[1]);

						for (i_R=0; i_R<2; i_R++)
						{
							Joint_Angle_EP[i_R][0] = JointDetect(i_R, 0, AngleH[i_R][0]*Degree2Rad);
							Joint_Angle_EP[i_R][1] = JointDetect(i_R, 1, AngleH[i_R][1]*Degree2Rad);
							Joint_Angle_EP[i_R][2] = JointDetect(i_R, 2, AngleH[i_R][2]*Degree2Rad);
							Joint_Angle_EP[i_R][3] = JointDetect(i_R, 3, AngleH[i_R][3]*Degree2Rad);
						}

						if(motion_enable_flag == 1)
						{
							for (i_R = 0; i_R < 4; i_R++)
							{
								rad_send(0,i_R,Joint_Angle_EP[0][i_R]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,i_R,Joint_Angle_EP[1][i_R]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}


						NowTime = rt_timer_read();
						double period1 = 0;
						period1 = (NowTime - LastTime) / 1000;   //us
						LastTime = NowTime;
						printf("period1 = %f\n", period1);
					}
				}
				break;

				case HANDCMD_ZERO:
				{
					int i_H = 0;
					if(first_time_HANDCMD_ZERO == 0)
					{
						for (i_H = 0; i_H < 4; i_H++)
						{
							start_position[0][i_H] = Joint_Angle_FB[0][i_H];
							start_position[1][i_H] = Joint_Angle_FB[1][i_H];
						}
						first_time_HANDCMD_ZERO = 1;
						t = 0;
						moving_flag = 1;
					}
					else
					{
						if(t <= 5.0)
						{
							t = t+time_interval;
							float angleplan[2][4] = {0.0};
							if(t > 5.0)
							{
								for (i_H = 0; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = 0.0;
									angleplan[1][i_H] = 0.0;
								}
							}
							else
							{
								for (i_H = 0; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = Five_Interpolation(start_position[0][i_H],0,0, 0,0,0, 5.0,t);
									angleplan[1][i_H] = Five_Interpolation(start_position[1][i_H],0,0, 0,0,0, 5.0,t);
								}

							}

							for (i_H = 0; i_H < 4; i_H++)
							{
								Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
								Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
							}

							if(motion_enable_flag == 1)
							{
								for (i_H = 0; i_H < 4; i_H++)
								{
									rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_HANDCMD_ZERO = 0;
							moving_flag =0;
						}

					}

				}
				break;

				case HANDCMD_CYLINDER:
				{
					int i_H = 0;
					if(first_time_HANDCMD_CYLINDER == 0)
					{
						for (i_H = 0; i_H < 4; i_H++)
						{
							start_position[0][i_H] = Joint_Angle_FB[0][i_H];
							start_position[1][i_H] = Joint_Angle_FB[1][i_H];
						}
						first_time_HANDCMD_CYLINDER = 1;
						t = 0;
						moving_flag = 1;
					}
					else
					{
						if(t <= 5.0)
						{
							t = t+time_interval;
							float angleplan[2][4] = {0.0};
							if(t > 5.0)
							{
								angleplan[0][0] = 0.0;
								angleplan[1][0] = 0.0;
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = HandAngleL;
									angleplan[1][i_H] = HandAngleR;
								}
							}
							else
							{
								angleplan[0][0] = Five_Interpolation(start_position[0][0],0,0, 0,0,0, 5.0,t);
								angleplan[1][0] = Five_Interpolation(start_position[1][0],0,0, 0,0,0, 5.0,t);
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = Five_Interpolation(start_position[0][i_H],0,0, HandAngleL,0,0, 5.0,t);
									angleplan[1][i_H] = Five_Interpolation(start_position[1][i_H],0,0, HandAngleR,0,0, 5.0,t);
								}

							}


							switch(HandSelect)
							{
								case 0xA0:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
										printf("%lf %lf %lf %lf \n",Joint_Angle_EP[0][0],Joint_Angle_EP[0][1],Joint_Angle_EP[0][2],Joint_Angle_EP[0][3]);
									}

								break;

								case 0x0A:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0xAA:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								default:
								break;
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_HANDCMD_CYLINDER = 0;
							moving_flag =0;
						}

					}

				}
				break;

				case HANDCMD_CYLINDER_PRE:
				{
					int i_H = 0;
					if(first_time_HANDCMD_CYLINDER_PRE == 0)
					{
						for (i_H = 0; i_H < 4; i_H++)
						{
							start_position[0][i_H] = Joint_Angle_FB[0][i_H];
							start_position[1][i_H] = Joint_Angle_FB[1][i_H];
						}
						first_time_HANDCMD_CYLINDER_PRE = 1;
						t = 0;
						moving_flag = 1;
					}
					else
					{
						if(t <= 5.0)
						{
							t = t+time_interval;
							float angleplan[2][4] = {0.0};

							if(t > 5.0)
							{
								for (i_H = 0; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = 0;
									angleplan[1][i_H] = 0;
								}
							}
							else
							{
								for (i_H = 0; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = Five_Interpolation(start_position[0][i_H],0,0, 0,0,0, 5.0,t);
									angleplan[1][i_H] = Five_Interpolation(start_position[1][i_H],0,0, 0,0,0, 5.0,t);
								}

							}

							switch(HandSelect)
							{
								case 0xA0:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
									}

									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0x0A:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0xAA:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								default:
								break;
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_HANDCMD_CYLINDER_PRE = 0;
							moving_flag =0;
						}

					}

				}
				break;

				case HANDCMD_SPHERE:
				{
					int i_H = 0;
					if(first_time_HANDCMD_SPHERE == 0)
					{
						for (i_H = 0; i_H < 4; i_H++)
						{
							start_position[0][i_H] = Joint_Angle_FB[0][i_H];
							start_position[1][i_H] = Joint_Angle_FB[1][i_H];
						}
						first_time_HANDCMD_SPHERE = 1;
						t = 0;
						moving_flag = 1;
					}
					else
					{
						if(t <= 5.0)
						{
							t = t+time_interval;
							float angleplan[2][4] = {0.0};
							if(t > 5.0)
							{
								angleplan[0][0] = 60.0*Degree2Rad;
								angleplan[1][0] = 60.0*Degree2Rad;
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = HandAngleL;
									angleplan[1][i_H] = HandAngleR;
								}
							}
							else
							{
								angleplan[0][0] = Five_Interpolation(start_position[0][0],0,0, 60*Degree2Rad,0,0, 5.0,t);
								angleplan[1][0] = Five_Interpolation(start_position[1][0],0,0, 60*Degree2Rad,0,0, 5.0,t);
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = Five_Interpolation(start_position[0][i_H],0,0, HandAngleL,0,0, 5.0,t);
									angleplan[1][i_H] = Five_Interpolation(start_position[1][i_H],0,0, HandAngleR,0,0, 5.0,t);
								}

							}


							switch(HandSelect)
							{
								case 0xA0:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0x0A:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0xAA:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								default:
								break;
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_HANDCMD_SPHERE = 0;
							moving_flag =0;
						}

					}

				}
				break;

				case HANDCMD_SPHERE_PRE:
				{
					int i_H = 0;
					if(first_time_HANDCMD_SPHERE_PRE == 0)
					{
						for (i_H = 0; i_H < 4; i_H++)
						{
							start_position[0][i_H] = Joint_Angle_FB[0][i_H];
							start_position[1][i_H] = Joint_Angle_FB[1][i_H];
						}
						first_time_HANDCMD_SPHERE_PRE = 1;
						t = 0;
						moving_flag = 1;
					}
					else
					{
						if(t <= 5.0)
						{
							t = t+time_interval;
							float angleplan[2][4] = {0.0};
							if(t > 5.0)
							{
								angleplan[0][0] = 60.0*Degree2Rad;
								angleplan[1][0] = 60.0*Degree2Rad;
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = 0;
									angleplan[1][i_H] = 0;
								}
							}
							else
							{
								angleplan[0][0] = Five_Interpolation(start_position[0][i_H],0,0, 60.0*Degree2Rad,0,0, 5.0,t);
								angleplan[1][0] = Five_Interpolation(start_position[1][i_H],0,0, 60.0*Degree2Rad,0,0, 5.0,t);
								for (i_H = 1; i_H < 4; i_H++)
								{
									angleplan[0][i_H] = Five_Interpolation(start_position[0][i_H],0,0, HandAngleL,0,0, 5.0,t);
									angleplan[1][i_H] = Five_Interpolation(start_position[1][i_H],0,0, HandAngleR,0,0, 5.0,t);
								}

							}


							switch(HandSelect)
							{
								case 0xA0:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0x0A:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								case 0xAA:
									for (i_H = 0; i_H < 4; i_H++)
									{
										Joint_Angle_EP[0][i_H] = JointDetect(0, i_H, angleplan[0][i_H]);
										Joint_Angle_EP[1][i_H] = JointDetect(1, i_H, angleplan[1][i_H]);
									}
									if(motion_enable_flag == 1)
									{
										for (i_H = 0; i_H < 4; i_H++)
										{
											rad_send(0,i_H,Joint_Angle_EP[0][i_H]);
											rad_send(1,i_H,Joint_Angle_EP[1][i_H]);
											sleeptime.tv_nsec = 200000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime,NULL);
										}
									}
								break;

								default:
								break;
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_HANDCMD_SPHERE_PRE = 0;
							moving_flag =0;
						}

					}

				}
				break;

				case FIND_HOME_MOTION:
					if (can_id_main == 9)
					{
						find_home(0,9);
						sleeptime.tv_nsec = 5000;
						sleeptime.tv_sec = 0;
						nanosleep(&sleeptime,NULL);
						find_home(1,9);
						sleeptime.tv_nsec = 5000;
						sleeptime.tv_sec = 0;
						nanosleep(&sleeptime,NULL);
						find_home(2,9);
						sleeptime.tv_nsec = 5000;
						sleeptime.tv_sec = 0;
						nanosleep(&sleeptime,NULL);
						find_home(3,9);
					}
					else
					{
						find_home(can_channel_main,can_id_main);
					}
					motion_mode = 100;
					printf("MOTION_MODE: FIND_HOME_MOTION\n");
				break;

				case ONE_ARM_MOTION:
				{
					int i_OA =0;
					if(fisrt_time_ONE_ARM_MOTION == 0)
					{
						memcpy(&OneArmStart,&RobotAngleFB,sizeof(OneArmStart));
						fisrt_time_ONE_ARM_MOTION = 1;
						t = 0;
						printf("MOTION_MODE: ONE_ARM_MOTION\n");
						moving_flag = 1;
					}
					else
					{
						if(t <= one_arm_time)
						{
							t = t+time_interval;
							switch(ArmSelect)
							{
								case 1:
								{
									float plan[7]= {0.0};
									for(i_OA=0;i_OA<7;i_OA++)
									{
										plan[i_OA] = Five_Interpolation(OneArmStart.LeftArm[i_OA], 0, 0, OneArmStart.LeftArm[i_OA] + One_arm_Data[i_OA], 0, 0, one_arm_time,t);
									}
									if (t>one_arm_time)
									{
										for(i_OA=0;i_OA<7;i_OA++)
										{
											plan[i_OA] = OneArmStart.LeftArm[i_OA] + One_arm_Data[i_OA];
										}
									}

									Joint_Angle_EP[2][0] = JointDetect(2, 0, plan[0]);
									Joint_Angle_EP[3][0] = JointDetect(3, 0, plan[1]);
									Joint_Angle_EP[3][1] = JointDetect(3, 1, plan[2]);
									Joint_Angle_EP[2][1] = JointDetect(2, 1, plan[3]);
									Joint_Angle_EP[0][4] = JointDetect(0, 4, plan[4]);
									Joint_Angle_EP[0][5] = JointDetect(0, 5, plan[5]);
									Joint_Angle_EP[0][6] = JointDetect(0, 6, plan[6]);

									if(motion_enable_flag == 1)
									{
										rad_send(0, 4, Joint_Angle_EP[0][4]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(0, 5, Joint_Angle_EP[0][5]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(0, 6, Joint_Angle_EP[0][6]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(2, 0, Joint_Angle_EP[2][0]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(2, 1, Joint_Angle_EP[2][1]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(3, 0, Joint_Angle_EP[3][0]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(3, 1, Joint_Angle_EP[3][1]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
									}

								}
								break;

								case 2:
								{
									float plan[7]= {0.0};
									for(i_OA=0;i_OA<7;i_OA++)
									{
										plan[i_OA] = Five_Interpolation(OneArmStart.RightArm[i_OA], 0, 0, OneArmStart.RightArm[i_OA] + One_arm_Data[i_OA], 0, 0, one_arm_time,t);
									}
									if (t>one_arm_time)
									{
										for(i_OA=0;i_OA<7;i_OA++)
										{
											plan[i_OA] = OneArmStart.RightArm[i_OA] + One_arm_Data[i_OA];
										}
									}

									Joint_Angle_EP[2][2] = JointDetect(2, 2, plan[0]);
									Joint_Angle_EP[3][2] = JointDetect(3, 2, plan[1]);
									Joint_Angle_EP[3][3] = JointDetect(3, 3, plan[2]);
									Joint_Angle_EP[2][3] = JointDetect(2, 3, plan[3]);
									Joint_Angle_EP[1][4] = JointDetect(1, 4, plan[4]);
									Joint_Angle_EP[1][5] = JointDetect(1, 5, plan[5]);
									Joint_Angle_EP[1][6] = JointDetect(1, 6, plan[6]);

									if(motion_enable_flag == 1)
									{
										rad_send(1, 4, Joint_Angle_EP[1][4]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(1, 5, Joint_Angle_EP[1][5]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(1, 6, Joint_Angle_EP[1][6]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(2, 2, Joint_Angle_EP[2][2]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(2, 3, Joint_Angle_EP[2][3]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(3, 2, Joint_Angle_EP[3][2]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(3, 3, Joint_Angle_EP[3][3]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
									}
								}
								break;
								default:
								break;
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							fisrt_time_ONE_ARM_MOTION = 0;
							moving_flag	= 0;
						}
					}
				}
				break;

				case HOMEBACK:
				{
					int i_j;
					if(first_time_HOMEBACK == 0)
		 			{
						memcpy(&OneArmStart,&RobotAngleFB,sizeof(OneArmStart));
		 				first_time_HOMEBACK = 1;
						t = 0.0;
						moving_flag	= 1;
						printf("MOTION_MODE = HOMEBACK\n");
		 			}
					else
					{
						if(t < homeback_time)
						{
							t = t + time_interval;
							float plan[2][7]= {0.0};
							struct RealRobot_Struct RobotHomeSet;
							memset(&RobotHomeSet,0,sizeof(RobotHomeSet));
							CanDef2RealRobot(home_offset, &RobotHomeSet);

							for(i_j=0;i_j<7;i_j++)
							{
								plan[0][i_j] = Five_Interpolation(OneArmStart.LeftArm[i_j], 0, 0, RobotHomeSet.LeftArm[i_j], 0, 0, homeback_time,t);
								plan[1][i_j] = Five_Interpolation(OneArmStart.RightArm[i_j], 0, 0, RobotHomeSet.RightArm[i_j], 0, 0, homeback_time,t);
							}
							if (t>homeback_time)
							{
								for(i_j=0;i_j<7;i_j++)
								{
									plan[0][i_j] = RobotHomeSet.LeftArm[i_j];
									plan[1][i_j] = RobotHomeSet.RightArm[i_j];
								}
							}

							Joint_Angle_EP[2][0] = JointDetect(2, 0, plan[0][0]);
							Joint_Angle_EP[3][0] = JointDetect(3, 0, plan[0][1]);
							Joint_Angle_EP[3][1] = JointDetect(3, 1, plan[0][2]);
							Joint_Angle_EP[2][1] = JointDetect(2, 1, plan[0][3]);
							Joint_Angle_EP[0][4] = JointDetect(0, 4, plan[0][4]);
							Joint_Angle_EP[0][5] = JointDetect(0, 5, plan[0][5]);
							Joint_Angle_EP[0][6] = JointDetect(0, 6, plan[0][6]);

							Joint_Angle_EP[2][2] = JointDetect(2, 2, plan[1][0]);
							Joint_Angle_EP[3][2] = JointDetect(3, 2, plan[1][1]);
							Joint_Angle_EP[3][3] = JointDetect(3, 3, plan[1][2]);
							Joint_Angle_EP[2][3] = JointDetect(2, 3, plan[1][3]);
							Joint_Angle_EP[1][4] = JointDetect(1, 4, plan[1][4]);
							Joint_Angle_EP[1][5] = JointDetect(1, 5, plan[1][5]);
							Joint_Angle_EP[1][6] = JointDetect(1, 6, plan[1][6]);


							if(motion_enable_flag == 1)
							{
								rad_send(0, 4, Joint_Angle_EP[0][4]);		//moyang602 	可根据通道发送减少时间
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0, 5, Joint_Angle_EP[0][5]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0, 6, Joint_Angle_EP[0][6]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 0, Joint_Angle_EP[2][0]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 1, Joint_Angle_EP[2][1]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 0, Joint_Angle_EP[3][0]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 1, Joint_Angle_EP[3][1]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);

								rad_send(1, 4, Joint_Angle_EP[1][4]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1, 5, Joint_Angle_EP[1][5]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1, 6, Joint_Angle_EP[1][6]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 2, Joint_Angle_EP[2][2]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 3, Joint_Angle_EP[2][3]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 2, Joint_Angle_EP[3][2]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 3, Joint_Angle_EP[3][3]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							first_time_HOMEBACK = 0;
							t = 0;
							motion_mode = 100;
							moving_flag	= 0;

							printf("clear zero\n");
			 				for(i=0; i<4; i++)
			 				{
			 					for(j=0; j<7; j++)
			 					{
			 						zero_comp[i][j] = home_offset[i][j] * joint_direction[i][j];
			 					}
			 				}

			 				for(i=0; i<can_channel_number; i++)
							{
								for(j=0; j< can_node_number[i]; j++)
								{
									Joint_Angle_LastEP[i][j] = 0.0;
									Joint_Angle_EP[i][j] = 0.0;
									Joint_Angle_LastEP_degree[i][j] = 0.0;
									Joint_Angle_EP_degree[i][j] = 0.0;
								}
							}
						}
					}
				}

		 		break;

		 		case PREPARE_FIND_HOME:
		 		{
		 			if (first_move_flag)
		 			{
		 				first_move_flag = 0;
		 				moving_flag	= 1;

						memcpy(&RealTargetPos,&RobotAngleFB,sizeof(RealTargetPos));

						RealTargetPos.LeftArm[0] = -8*Degree2Rad;
						RealTargetPos.LeftArm[1] = -65*Degree2Rad;
						RealTargetPos.LeftArm[2] = 8*Degree2Rad;
						RealTargetPos.LeftArm[3] = 8*Degree2Rad;
						RealTargetPos.LeftArm[4] = 10*Degree2Rad;
						RealTargetPos.LeftArm[5] = 8*Degree2Rad;
						RealTargetPos.LeftArm[6] = 0*Degree2Rad;
						RealTargetPos.RightArm[0] = -8*Degree2Rad;
						RealTargetPos.RightArm[1] = 85*Degree2Rad;
						RealTargetPos.RightArm[2] = 12*Degree2Rad;
						RealTargetPos.RightArm[3] = 8*Degree2Rad;
						RealTargetPos.RightArm[4] = -16*Degree2Rad;
						RealTargetPos.RightArm[5] = 8*Degree2Rad;
						RealTargetPos.RightArm[6] = 0*Degree2Rad;
		 			}
		 			else
		 			{
			 			return_value = AllJointMove(RealTargetPos,10,1,0,0,0);
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 				moving_flag	= 0;
			 				first_move_flag = 1;
			 			}
		 			}

		 		}
		 		break;

		 		case RETURN_ORIGIN_POSITION:
		 		{
		 			if (first_move_flag)
		 			{
		 				first_move_flag = 0;
		 				moving_flag	= 1;

						memcpy(&RealTargetPos,&RobotAngleFB,sizeof(RealTargetPos));
						for(i=0; i<7; i++)
						{
							RealTargetPos.LeftArm[i] = 0.0*Degree2Rad;
							RealTargetPos.RightArm[i] = 0.0*Degree2Rad;
						}
						RealTargetPos.LeftArm[1] = 73.5*Degree2Rad;
						RealTargetPos.RightArm[1] = -73.5*Degree2Rad;
		 			}
		 			else
		 			{
			 			return_value = AllJointMove(RealTargetPos,12,1,0,0,0);
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 				moving_flag	= 0;
			 				first_move_flag = 1;
			 			}
		 			}

		 		}
		 		break;

				case RETURN_ZERO:
				{
					if (first_move_flag)
		 			{
		 				first_move_flag = 0;
		 				moving_flag	= 1;

		 				memset(&RealTargetPos,0,sizeof(RealTargetPos));
		 			}
		 			else
		 			{
			 			return_value = AllJointMove(RealTargetPos,15,1,0,0,0);
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 				moving_flag	= 0;
			 				first_move_flag = 1;
			 			}
		 			}
				}
		 		break;

		 		case MOVE_PRE_POSITION:
	 			{
	 				if (first_move_flag)
		 			{
		 				first_move_flag = 0;
		 				moving_flag	= 1;

						memcpy(&RealTargetPos,&RobotAngleFB,sizeof(RealTargetPos));

						RealTargetPos.LeftArm[0] = -45*Degree2Rad;
						RealTargetPos.LeftArm[1] = 60*Degree2Rad;
						RealTargetPos.LeftArm[2] = 0*Degree2Rad;
						RealTargetPos.LeftArm[3] = 30*Degree2Rad;
						RealTargetPos.LeftArm[4] = 0*Degree2Rad;
						RealTargetPos.LeftArm[5] = 30*Degree2Rad;
						RealTargetPos.LeftArm[6] = 0*Degree2Rad;
						RealTargetPos.RightArm[0] = 45*Degree2Rad;
						RealTargetPos.RightArm[1] = -60*Degree2Rad;
						RealTargetPos.RightArm[2] = 0*Degree2Rad;
						RealTargetPos.RightArm[3] = -30*Degree2Rad;
						RealTargetPos.RightArm[4] = 0*Degree2Rad;
						RealTargetPos.RightArm[5] = -30*Degree2Rad;
						RealTargetPos.RightArm[6] = 0*Degree2Rad;
		 			}
		 			else
		 			{
			 			return_value = AllJointMove(RealTargetPos,12,1,0,0,0);
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 				moving_flag	= 0;
			 				first_move_flag = 1;
			 			}
		 			}
				}
		 		break;

				case VISION_MOTION:

					if(first_time_VISION_MOTION == 0)
					{
						first_time_VISION_MOTION = 1;
						moving_flag	= 1;
						t = 0.0;
						printf("MOTION_MODE = VISION_MOTION\n");

						for(i=0; i<3; i++)
						{
							for(j=0; j<4; j++)
							{
								start_position[i][j] = Joint_Angle_FB[i][j];
							}
						}

						printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f    %f   %f   %f   %f\n", start_position[0][0]*Rad2Degree,start_position[0][1]*Rad2Degree,start_position[0][2]*Rad2Degree,start_position[0][3]*Rad2Degree,start_position[1][0]*Rad2Degree, start_position[1][1]*Rad2Degree, start_position[1][2]*Rad2Degree,start_position[1][3]*Rad2Degree, start_position[2][0]*Rad2Degree, start_position[2][1]*Rad2Degree, start_position[2][2]*Rad2Degree,start_position[2][3]*Rad2Degree);

						if(End_Numb == ARM1)
						{
							angle_2leg[0] = start_position[1][0];
							angle_2leg[1] = start_position[1][1];
							angle_2leg[2] = start_position[1][2];
							angle_2leg[3] = start_position[1][3];

							angle_2leg[4] = start_position[0][0];
							angle_2leg[5] = start_position[0][1];
							angle_2leg[6] = start_position[0][2];
							angle_2leg[7] = start_position[0][3];

							printf("ARM1 end\n");
							printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f\n", angle_2leg[0]*Rad2Degree, angle_2leg[1]*Rad2Degree, angle_2leg[2]*Rad2Degree, angle_2leg[3]*Rad2Degree, angle_2leg[4]*Rad2Degree, angle_2leg[5]*Rad2Degree, angle_2leg[6]*Rad2Degree, angle_2leg[7]*Rad2Degree);
						}

						KinTwoLegR(angle_2leg,  T_Now);

						for(i=0; i<4; i++)
						{
							for(j=0; j<4;j++)
								delta_matrix2[i][j] = DeltaMatrix[i][j];
						}

						matrix_multiply(T_Now, T_hand_end_inv, T_Now1);
						matrix_multiply(T_Now1, delta_matrix2, T_Now2);
						matrix_multiply(T_Now2, T_hand_end, T_END2);

						printf("\nT_Now1 = ");
						for(i=0; i<4; i++)
						{
							for(j=0; j<4;j++)
								printf("%f   ",T_Now1[i][j]);
							printf("\n");
						}

						printf("\nDeltaMatrix = ");
						for(i=0; i<4; i++)
						{
							for(j=0; j<4;j++)
								printf("%f   ", DeltaMatrix[i][j]);
							printf("\n");
						}


						printf("\nT_Now2 = ");
						for(i=0; i<4; i++)
						{
							for(j=0; j<4;j++)
								printf("%f   ",T_Now2[i][j]);
							printf("\n");
						}

						printf("\nT_END2 = ");
						for(i=0; i<4; i++)
						{
							for(j=0; j<4;j++)
								printf("%f   ",T_END2[i][j]);
							printf("\n");
						}


						printf("angle_in= %f  %f   %f  %f  %f  %f  %f  %f\n", angle_2leg[0]*180.0/pi, angle_2leg[1]*180.0/pi,angle_2leg[2]*180.0/pi,angle_2leg[3]*180.0/pi,angle_2leg[4]*180.0/pi,angle_2leg[5]*180.0/pi,angle_2leg[6]*180.0/pi,angle_2leg[7]*180.0/pi);

						InvCham(T_END2, angle_2leg, 0.0, angle_out2);

						if(End_Numb == ARM1)
						{

							end_position[1][0] = angle_out2[0];
							end_position[1][1] = angle_out2[1];
							end_position[1][2] = angle_out2[2];
							end_position[1][3] = angle_out2[3];

							end_position[0][0] = angle_out2[4];
							end_position[0][1] = angle_out2[5];
							end_position[0][2] = angle_out2[6];
							end_position[0][3] = angle_out2[7];

							end_position[2][0] = start_position[2][0];
							end_position[2][1] = start_position[2][1];
							end_position[2][2] = start_position[2][2];
							end_position[2][3] = start_position[2][3];

							printf("ARM1 end\n");
							printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree, end_position[0][3]*Rad2Degree, end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree, end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree, end_position[2][3]*Rad2Degree);
						}
					}
					else
					{
						if(t < vision_motion_time)
						{

							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{

									Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position[i][j],0,0,vision_motion_time,t);

									if(t > vision_motion_time)
									{
										Joint_Angle_EP[i][j] = end_position[i][j];
									}

									if(motion_enable_flag == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}

									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_VISION_MOTION = 0;
							moving_flag	= 0;
						}
					}

				break;

				case FORCE_MOTION:
				{
					int ForceNewDataL = 0;
					double ForceDataL[6] = {0.0};
					double ForceDataRAWL[6] = {0.0};
					int ForceNewDataR = 0;
					double ForceDataR[6] = {0.0};
					double ForceDataRAWR[6] = {0.0};
					// 左臂读数
					if(ForceTCPFlagL <= 0)
					{
						ForceTCPFlagL = ForceSensorTCP_init(0x01);	//0x01 代表左臂
						printf("ForceTCPFlagL is %d\n",ForceTCPFlagL);
					}
					else
					{
						switch(force_stepL)
						{
							case 1:
							{
								int Cfg = 0;
								Cfg = ConfigSystemL(&nStatusL, &bIsSendFlagL, &bReceivedL);
								if(Cfg == 2)
									force_stepL = 2;

								ForceTCPRecv(1);
							}
							break;

							case 2:
							{
								ForceNewDataL = GetData(1,ForceDataRAWL);				// twice get a data
								double Angle[7];
								int i;

								for (i = 0; i < 7; i++)
								{
									Angle[i] = RobotAngleFB.LeftArm[i];
								}
								ForceCompensationL(ForceDataRAWL, Angle, ForceDataL);

								sprintf(buf21, "*********************************  Robot Force *********************************");

								sprintf(buf22, "ForceL:   %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",ForceDataL[0],ForceDataL[1],ForceDataL[2],ForceDataL[3],ForceDataL[4],ForceDataL[5]);

							}
							break;
							default:
							break;
						}

					}
					// 右臂读数
					if(ForceTCPFlagR <= 0)
					{
						ForceTCPFlagR = ForceSensorTCP_init(0x02);	//0x01 代表左臂
						printf("ForceTCPFlagR is %d\n",ForceTCPFlagR);
					}
					else
					{
						switch(force_stepR)
						{
							case 1:
							{
								int Cfg = 0;
								Cfg = ConfigSystemR(&nStatusR, &bIsSendFlagR, &bReceivedR);
								if(Cfg == 2)
									force_stepR = 2;

								ForceTCPRecv(2);
							}
							break;

							case 2:
							{
								ForceNewDataR = GetData(2,ForceDataRAWR);				// twice get a data
								double Angle[7];
								int i;

								for (i = 0; i < 7; i++)
								{
									Angle[i] = RobotAngleFB.RightArm[i];
								}
								ForceCompensationR(ForceDataRAWR, Angle, ForceDataR);

								sprintf(buf23, "ForceR:   %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",ForceDataR[0],ForceDataR[1],ForceDataR[2],ForceDataR[3],ForceDataR[4],ForceDataR[5]);
							}
							break;
							default:
							break;
						}

					}

					int i_f = 0;
					double force_limit[6] = {80.0, 80.0, 100.0, 7.0, 7.0, 7.0};

					if(ForceControl_flagL == 1&& ForceNewDataL==1)	//左臂control
					{
						int position_change_flagL = 1;

						for(i_f=0; i_f<6; i_f++)
						{
							if(fabs(ForceDataL[i_f]) > force_limit[i_f])
							{
								ForceControl_flagL = 0;
								position_change_flagL = 0;
								printf("forceL limited\n");
								break;
							}
						}

						if(position_change_flagL == 1)
						{
							for(i_f=0;i_f<6;i_f++)
							{
								A6D_C[i_f] = 2*A6D_ep[i_f]*sqrt(A6D_K[i_f]*A6D_m[i_f]);
							}

							for(i_f=0;i_f<6;i_f++)
							{
								AccL[i_f] = (ForceDataL[i_f] - A6D_K[i_f]*A6D_D_totalL[i_f] - A6D_C[i_f]*A6D_VL[i_f])/A6D_m[i_f];
								A6D_D_totalL[i_f] = A6D_D_totalL[i_f] + AccL[i_f]*FDeltaT*FDeltaT/2.0 + A6D_VL[i_f]*FDeltaT ;
								A6D_VL[i_f] = A6D_VL[i_f] + AccL[i_f]*FDeltaT;
							}

							for(i_f=0;i_f<6;i_f++)
							{
								if(A6D_enable[i_f] == 0)
									A6D_VL[i_f] = 0;
							}


							if(FirstForceControlL == 1)
							{
								memcpy(&OneArmStart,&RobotAngleFBDeg,sizeof(OneArmStart));
							//	A6D_Joint_PL[0] = -45.0;
							//	A6D_Joint_PL[1] = 60.0;
							//	A6D_Joint_PL[2] = 0.0;
							//	A6D_Joint_PL[3] = 30.0;
							//	A6D_Joint_PL[4] = 0.0;
							//	A6D_Joint_PL[5] = 30.0;
							//	A6D_Joint_PL[6] = 0.0;
								for(i_f=0;i_f<7;i_f++)
								{
									A6D_Joint_PL[i_f] = OneArmStart.LeftArm[i_f];
									A6D_Joint_VL[i_f] = 0;
								}

								FirstForceControlL = 0;
							}
							double InvJacobinNowL[7][6] = {0.0};
							InvJacobianL(A6D_Joint_PL, InvJacobinNowL);		// 雅可比奇异情况呢
							double A6D_VmmL[6];
							A6D_VmmL[0] = A6D_VL[0]*1000;		// change into mm/s
							A6D_VmmL[1] = A6D_VL[1]*1000;		// change into mm/s
							A6D_VmmL[2] = A6D_VL[2]*1000;		// change into mm/s
							A6D_VmmL[3] = A6D_VL[3];
							A6D_VmmL[4] = A6D_VL[4];
							A6D_VmmL[5] = A6D_VL[5];

							for(i_f = 0; i_f<6; i_f++)
							{
								if(A6D_VmmL[i_f] > force_velocity_limit[i_f])
								{
									A6D_VmmL[i_f] = force_velocity_limit[i_f];
									printf("velocityL limited\n");
								}
								else if (A6D_VmmL[i_f] < -force_velocity_limit[i_f])
								{
									A6D_VmmL[i_f] = -force_velocity_limit[i_f];
									printf("velocityL limited\n");
								}
							}

							sprintf(buf24, "ExpEndVL: %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f", A6D_VmmL[0],A6D_VmmL[1],A6D_VmmL[2],A6D_VmmL[3]*Rad2Degree,A6D_VmmL[4]*Rad2Degree,A6D_VmmL[5]*Rad2Degree);

							Matrix_Multiply(7,6,1,*InvJacobinNowL,A6D_VmmL,A6D_Joint_VL);

							sprintf(buf25, "ExpJointVL:%7.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",A6D_Joint_VL[0]*Rad2Degree,A6D_Joint_VL[1]*Rad2Degree,A6D_Joint_VL[2]*Rad2Degree,A6D_Joint_VL[3]*Rad2Degree,A6D_Joint_VL[4]*Rad2Degree,A6D_Joint_VL[5]*Rad2Degree,A6D_Joint_VL[6]*Rad2Degree);

							for(i_f=0;i_f<7;i_f++)
							{
								A6D_Joint_PL[i_f] += A6D_Joint_VL[i_f]*FDeltaT*Rad2Degree;
							}


							Joint_Angle_EP[2][0] = JointDetect(2, 0, A6D_Joint_PL[0]*Degree2Rad);
							Joint_Angle_EP[3][0] = JointDetect(3, 0, A6D_Joint_PL[1]*Degree2Rad);
							Joint_Angle_EP[3][1] = JointDetect(3, 1, A6D_Joint_PL[2]*Degree2Rad);
							Joint_Angle_EP[2][1] = JointDetect(2, 1, A6D_Joint_PL[3]*Degree2Rad);
							Joint_Angle_EP[0][4] = JointDetect(0, 4, A6D_Joint_PL[4]*Degree2Rad);
							Joint_Angle_EP[0][5] = JointDetect(0, 5, A6D_Joint_PL[5]*Degree2Rad);
							Joint_Angle_EP[0][6] = JointDetect(0, 6, A6D_Joint_PL[6]*Degree2Rad);

							// 缺少末端限位

							if(motion_enable_flag == 1)
							{
								rad_send(0, 4, Joint_Angle_EP[0][4]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0, 5, Joint_Angle_EP[0][5]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0, 6, Joint_Angle_EP[0][6]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 0, Joint_Angle_EP[2][0]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 1, Joint_Angle_EP[2][1]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 0, Joint_Angle_EP[3][0]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 1, Joint_Angle_EP[3][1]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);

							}
						}
					}
					else if(ForceControl_flagL == 0)
					{
						for(i_f=0;i_f<6;i_f++)
						{
							AccL[i_f] = 0.0;
							A6D_VL[i_f] = 0.0;
							A6D_D_totalL[i_f] = 0.0;
						}
						FirstForceControlL = 1;
					}


					if(ForceControl_flagR == 1&& ForceNewDataR==1)	//右臂control
					{
						int position_change_flagR = 1;

						for(i_f=0; i_f<6; i_f++)
						{
							if(fabs(ForceDataR[i_f]) > force_limit[i_f])
							{
								ForceControl_flagR = 0;
								position_change_flagR = 0;
								printf("force limited\n");
								break;
							}
						}

						if(position_change_flagR == 1)
						{
							for(i_f=0;i_f<6;i_f++)
							{
								A6D_C[i_f] = 2*A6D_ep[i_f]*sqrt(A6D_K[i_f]*A6D_m[i_f]);
							}

							for(i_f=0;i_f<6;i_f++)
							{
								AccR[i_f] = (ForceDataR[i_f] - A6D_K[i_f]*A6D_D_totalR[i_f] - A6D_C[i_f]*A6D_VR[i_f])/A6D_m[i_f];
								A6D_D_totalR[i_f] = A6D_D_totalR[i_f] + AccR[i_f]*FDeltaT*FDeltaT/2.0 + A6D_VR[i_f]*FDeltaT ;
								A6D_VR[i_f] = A6D_VR[i_f] + AccR[i_f]*FDeltaT;
							}

							for(i_f=0;i_f<6;i_f++)
							{
								if(A6D_enable[i_f] == 0)
									A6D_VR[i_f] = 0;
							}


							if(FirstForceControlR == 1)
							{
								memcpy(&OneArmStart,&RobotAngleFBDeg,sizeof(OneArmStart));
								/*A6D_Joint_PR[0] = 45.0;
								A6D_Joint_PR[1] = -60.0;
								A6D_Joint_PR[2] = 0.0;
								A6D_Joint_PR[3] = -30.0;
								A6D_Joint_PR[4] = 0.0;
								A6D_Joint_PR[5] = -30.0;
								A6D_Joint_PR[6] = 0.0;*/
								for(i_f=0;i_f<7;i_f++)
								{
									A6D_Joint_PR[i_f] = OneArmStart.RightArm[i_f];
									A6D_Joint_VR[i_f] = 0;
								}

								FirstForceControlR = 0;
							}
							double InvJacobinNowR[7][6] = {0.0};
							InvJacobianR(A6D_Joint_PR, InvJacobinNowR);		// 雅可比奇异情况呢
							double A6D_VmmR[6];
							A6D_VmmR[0] = A6D_VR[0]*1000;		// change into mm/s
							A6D_VmmR[1] = A6D_VR[1]*1000;		// change into mm/s
							A6D_VmmR[2] = A6D_VR[2]*1000;		// change into mm/s
							A6D_VmmR[3] = A6D_VR[3];
							A6D_VmmR[4] = A6D_VR[4];
							A6D_VmmR[5] = A6D_VR[5];

							for(i_f = 0; i_f<6; i_f++)
							{
								if(A6D_VmmR[i_f] > force_velocity_limit[i_f])
								{
									A6D_VmmR[i_f] = force_velocity_limit[i_f];
									printf("velocity limited\n");
								}
								else if (A6D_VmmR[i_f] < -force_velocity_limit[i_f])
								{
									A6D_VmmR[i_f] = -force_velocity_limit[i_f];
									printf("velocity limited\n");
								}
							}

							sprintf(buf24, "ExpEndV:  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f", A6D_VmmR[0],A6D_VmmR[1],A6D_VmmR[2],A6D_VmmR[3]*Rad2Degree,A6D_VmmR[4]*Rad2Degree,A6D_VmmR[5]*Rad2Degree);

							Matrix_Multiply(7,6,1,*InvJacobinNowR,A6D_VmmR,A6D_Joint_VR);

							sprintf(buf25, "ExpJointV:%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",A6D_Joint_VR[0]*Rad2Degree,A6D_Joint_VR[1]*Rad2Degree,A6D_Joint_VR[2]*Rad2Degree,A6D_Joint_VR[3]*Rad2Degree,A6D_Joint_VR[4]*Rad2Degree,A6D_Joint_VR[5]*Rad2Degree,A6D_Joint_VR[6]*Rad2Degree);

							for(i_f=0;i_f<7;i_f++)
							{
								A6D_Joint_PR[i_f] += A6D_Joint_VR[i_f]*FDeltaT*Rad2Degree;
							}


							Joint_Angle_EP[2][2] = JointDetect(2, 2, A6D_Joint_PR[0]*Degree2Rad);
							Joint_Angle_EP[3][2] = JointDetect(3, 2, A6D_Joint_PR[1]*Degree2Rad);
							Joint_Angle_EP[3][3] = JointDetect(3, 3, A6D_Joint_PR[2]*Degree2Rad);
							Joint_Angle_EP[2][3] = JointDetect(2, 3, A6D_Joint_PR[3]*Degree2Rad);
							Joint_Angle_EP[1][4] = JointDetect(1, 4, A6D_Joint_PR[4]*Degree2Rad);
							Joint_Angle_EP[1][5] = JointDetect(1, 5, A6D_Joint_PR[5]*Degree2Rad);
							Joint_Angle_EP[1][6] = JointDetect(1, 6, A6D_Joint_PR[6]*Degree2Rad);

							// 缺少末端限位

							if(motion_enable_flag == 1)
							{
								rad_send(1, 4, Joint_Angle_EP[1][4]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1, 5, Joint_Angle_EP[1][5]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1, 6, Joint_Angle_EP[1][6]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 2, Joint_Angle_EP[2][2]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2, 3, Joint_Angle_EP[2][3]);
								sleeptime.tv_nsec = 5000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 2, Joint_Angle_EP[3][2]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3, 3, Joint_Angle_EP[3][3]);
								sleeptime.tv_nsec = 200000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
					}
					else if(ForceControl_flagR == 0)
					{
						for(i_f=0;i_f<6;i_f++)
						{
							AccR[i_f] = 0.0;
							A6D_VR[i_f] = 0.0;
							A6D_D_totalR[i_f] = 0.0;
						}
						FirstForceControlR = 1;
					}

				}
			 	break;

			 	case DUTY_MOTION:
			 	{
			 		//**********************    读取力传感器数据   *****************************//
					int ForceNewDataL = 0;

					double ForceDataL[6] = {0.0};
					double ForceDataRAWL[6] = {0.0};
					int ForceNewDataR = 0;
					double ForceDataR[6] = {0.0};
					double ForceDataRAWR[6] = {0.0};
					// 左臂读数
					if(ForceTCPFlagL <= 0)
					{
						ForceTCPFlagL = ForceSensorTCP_init(0x01);	//0x01 代表左臂
						printf("ForceTCPFlagL is %d\n",ForceTCPFlagL);
					}
					else
					{
						switch(force_stepL)
						{
							case 1:
							{
								int Cfg = 0;
								Cfg = ConfigSystemL(&nStatusL, &bIsSendFlagL, &bReceivedL);
								if(Cfg == 2)
									force_stepL = 2;

								ForceTCPRecv(1);
							}
							break;

							case 2:
							{
								ForceNewDataL = GetData(1,ForceDataRAWL);				// twice get a data
								double Angle[7];
								int i;

								for (i = 0; i < 7; i++)
								{
									Angle[i] = RobotAngleFB.LeftArm[i];
								}
								ForceCompensationL(ForceDataRAWL, Angle, ForceDataL);

								sprintf(buf21, "*********************************  Robot Force *********************************");

								sprintf(buf22, "ForceL:   %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",ForceDataL[0],ForceDataL[1],ForceDataL[2],ForceDataL[3],ForceDataL[4],ForceDataL[5]);

							}
							break;
							default:
							break;
						}

					}
					// 右臂读数
					if(ForceTCPFlagR <= 0)
					{
						ForceTCPFlagR = ForceSensorTCP_init(0x02);	//0x01 代表左臂
						printf("ForceTCPFlagR is %d\n",ForceTCPFlagR);
					}
					else
					{
						switch(force_stepR)
						{
							case 1:
							{
								int Cfg = 0;
								Cfg = ConfigSystemR(&nStatusR, &bIsSendFlagR, &bReceivedR);
								if(Cfg == 2)
									force_stepR = 2;

								ForceTCPRecv(2);
							}
							break;

							case 2:
							{
								ForceNewDataR = GetData(2,ForceDataRAWR);				// twice get a data
								double Angle[7];
								int i;

								for (i = 0; i < 7; i++)
								{
									Angle[i] = RobotAngleFB.RightArm[i];
								}
								ForceCompensationR(ForceDataRAWR, Angle, ForceDataR);

								sprintf(buf23, "ForceR:   %8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f",ForceDataR[0],ForceDataR[1],ForceDataR[2],ForceDataR[3],ForceDataR[4],ForceDataR[5]);
							}
							break;
							default:
							break;
						}

					}

					//**********************   任务序列   ***********************//
					switch(DutyStep)
					{
						case 1:
						{
							if (first_move_flag)
							{
								first_move_flag = 0;
								moving_flag	= 1;

								memcpy(&RealTargetPos,&RobotAngleFB,sizeof(RealTargetPos));
								RealTargetPos.LeftArm[0] = -45*Degree2Rad;
								RealTargetPos.LeftArm[1] = 60*Degree2Rad;
								RealTargetPos.LeftArm[2] = 0*Degree2Rad;
								RealTargetPos.LeftArm[3] = 30*Degree2Rad;
								RealTargetPos.LeftArm[4] = 0*Degree2Rad;
								RealTargetPos.LeftArm[5] = 30*Degree2Rad;
								RealTargetPos.LeftArm[6] = 0*Degree2Rad;
								RealTargetPos.RightArm[0] = 45*Degree2Rad;
								RealTargetPos.RightArm[1] = -60*Degree2Rad;
								RealTargetPos.RightArm[2] = 0*Degree2Rad;
								RealTargetPos.RightArm[3] = -30*Degree2Rad;
								RealTargetPos.RightArm[4] = 0*Degree2Rad;
								RealTargetPos.RightArm[5] = -30*Degree2Rad;
								RealTargetPos.RightArm[6] = 0*Degree2Rad;
							}
							else
							{
								return_value = AllJointMove(RealTargetPos,5,1,0,0,0);
								if(return_value == 0)
								{
									DutyStep = 2;
									moving_flag	= 0;
									first_move_flag = 1;
								}
							}
						}
						break;

						case 2:
						{
							int i_R;
							static int firsttimeflag = 1;		// 第一次进行遥操作标志

							if(RemoteMotion_enable_flag == 0)// 遥操作指令未开启时
							{
								firsttimeflag = 1;
								// 遥操作控制器输出关节角设置为当前关节角
								memcpy(&RemoteRobotPos_deg,&RobotAngleFBDeg,sizeof(RemoteRobotPos_deg));
								// 遥操作规划器输入关节角设置为当前关节角
								for(i_R=0;i_R<7;i_R++)
								{
									RemotePlanPos_deg[i_R] = RemoteRobotPos_deg.LeftArm[i_R];
									RemotePlanPos_deg[i_R+7] = RemoteRobotPos_deg.RightArm[i_R];
								}
								// 清空规划器
								for (i_R = 0; i_R < 14; i_R++)
								{
									memset(&cubic[i_R],0,sizeof(cubic[i_R]));
									cubic[i_R].needNextPoint = 1;
									cubic[i_R].segmentTime = 0.24;
									cubic[i_R].interpolationRate = 0.24/time_interval + 1;
									cubic[i_R].interpolationIncrement = 0.24/(double)(cubic[i_R].interpolationRate - 1);
								}

								// 清空阻抗控制器
								for(i_R=0;i_R<6;i_R++)
								{
									AccL[i_R] = 0.0;
									A6D_VL[i_R] = 0.0;
									A6D_D_totalL[i_R] = 0.0;
									AccR[i_R] = 0.0;
									A6D_VR[i_R] = 0.0;
									A6D_D_totalR[i_R] = 0.0;
								}
								FirstForceControlL = 1;
								FirstForceControlR = 1;

							}
							else// 遥操作指令开启时
							{
								// 添加遥操作规划点
								while(cubic[1].needNextPoint)
								{
									if (RemoteNewData==1&&firsttimeflag==0)
									{
										for (i_R = 0; i_R < 14; i_R++)
										{
											if (RemotePlanPos_deg[i_R] - RemoteMotionData[i_R]>10*0.24)
											{
												RemotePlanPos_deg[i_R] = RemotePlanPos_deg[i_R] - 10*0.24;
											}
											else if (RemotePlanPos_deg[i_R] - RemoteMotionData[i_R]<-10*0.24)
											{
												RemotePlanPos_deg[i_R] = RemotePlanPos_deg[i_R] + 10*0.24;
											}
											else
											{
												RemotePlanPos_deg[i_R] = RemoteMotionData[i_R];
											}

											cubicAddPoint(i_R,RemotePlanPos_deg[i_R]);
										}

										RemoteNewData = 0;
									}
									else
									{
										for (i_R = 0; i_R < 14; i_R++)
										{
											cubicAddPoint(i_R,RemotePlanPos_deg[i_R]);
										}
										firsttimeflag = 0;
									}

								}
								// 获取遥操作期望位置
								for (i_R = 0; i_R < 7; i_R++)
								{
									RemoteRobotPos_deg.LeftArm[i_R] = cubicInterpolate(i_R);
									RemoteRobotPos_deg.RightArm[i_R] = cubicInterpolate(i_R+7);
								}

								static double testangle = 0;
								testangle-= FDeltaT*1;
								RemoteRobotPos_deg.LeftArm[0] = -45+testangle;
								RemoteRobotPos_deg.LeftArm[1] = 60;
								RemoteRobotPos_deg.LeftArm[2] = 0;
								RemoteRobotPos_deg.LeftArm[3] = 30;
								RemoteRobotPos_deg.LeftArm[4] = 0;
								RemoteRobotPos_deg.LeftArm[5] = 30;
								RemoteRobotPos_deg.LeftArm[6] = 0;
								RemoteRobotPos_deg.RightArm[0] = 45-testangle;
								RemoteRobotPos_deg.RightArm[1] = -60;
								RemoteRobotPos_deg.RightArm[2] = 0;
								RemoteRobotPos_deg.RightArm[3] = -30;
								RemoteRobotPos_deg.RightArm[4] = 0;
								RemoteRobotPos_deg.RightArm[5] = -30;
								RemoteRobotPos_deg.RightArm[6] = 0;

								double force_limit[6] = {80.0, 80.0, 100.0, 7.0, 7.0, 7.0};
								if(DutyForceFlag==0)	// 任务模式关闭力控时清空力传感器数值
								{
									for(i_R=0; i_R<6; i_R++)
									{
										ForceDataL[i_R] = 0;
										ForceDataR[i_R] = 0;
									}
									FirstForceControlL == 1;
									FirstForceControlR == 1;
								}


								static double TotalDeltaTL[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
								// 计算遥操作输入末端位姿
								double Angle[7];	// unit: rad
								for(i_R=0;i_R<7;i_R++)
								{
									Angle[i_R] = RemoteRobotPos_deg.LeftArm[i_R]*Degree2Rad;
								}
								double RemoteTendL[4][4];	// 遥操作末端位姿
								KinL(Angle, RemoteTendL);	// input: rad  output: mm

								static double TotalDeltaTR[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
								// 计算遥操作输入末端位姿
								for(i_R=0;i_R<7;i_R++)
								{
									Angle[i_R] = RemoteRobotPos_deg.RightArm[i_R]*Degree2Rad;
								}
								double RemoteTendR[4][4];	// 遥操作末端位姿
								KinR(Angle, RemoteTendR);	// input: rad  output: mm

								// 当采集到力传感器数据时进行力控制
								if(ForceNewDataL==1)	//左臂control 利用逆运动学进行融合
								{
									int position_change_flagL = 1;
									// 当末端力超过六维力传感器限制时关闭运动控制
									for(i_R=0; i_R<6; i_R++)
									{
										if(fabs(ForceDataL[i_R]) > force_limit[i_R])
										{
											position_change_flagL = 0;
											motion_enable_flag = 0;
											printf("forceL limited\n");
											break;
										}
									}

									if(position_change_flagL == 1)
									{
										// 第一次进入力控制时
										if(FirstForceControlL == 1)
										{
											// 力控输出角等于当前关节角
											memcpy(&OneArmStart,&RobotAngleFBDeg,sizeof(OneArmStart));
											A6D_Joint_PL[0] = -45.0;
											A6D_Joint_PL[1] = 60.0;
											A6D_Joint_PL[2] = 0.0;
											A6D_Joint_PL[3] = 30.0;
											A6D_Joint_PL[4] = 0.0;
											A6D_Joint_PL[5] = 30.0;
											A6D_Joint_PL[6] = 0.0;
											for(i_R=0;i_R<7;i_R++)
											{
											//	A6D_Joint_PL[i_R] = OneArmStart.LeftArm[i_R];
												A6D_Joint_VL[i_R] = 0;
											}
											// 初始化总位移矩阵
											memset(TotalDeltaTL,0,sizeof(TotalDeltaTL));
											TotalDeltaTL[0][0] = 1;
											TotalDeltaTL[1][1] = 1;
											TotalDeltaTL[2][2] = 1;
											TotalDeltaTL[3][3] = 1;

											FirstForceControlL = 0;
										}

										// 计算阻抗控制下，此步移动量
										for(i_R=0;i_R<6;i_R++)
										{
											A6D_C[i_R] = 2*A6D_ep[i_R]*sqrt(A6D_K[i_R]*A6D_m[i_R]);
										}
										for(i_R=0;i_R<6;i_R++)
										{
											AccL[i_R] = (ForceDataL[i_R] - A6D_K[i_R]*A6D_D_totalL[i_R] - A6D_C[i_R]*A6D_VL[i_R])/A6D_m[i_R];
											A6D_D_totalL[i_R] = A6D_D_totalL[i_R] + AccL[i_R]*FDeltaT*FDeltaT/2.0 + A6D_VL[i_R]*FDeltaT;
											A6D_VL[i_R] = A6D_VL[i_R] + AccL[i_R]*FDeltaT;
											A6D_PL[i_R] = A6D_VL[i_R]*FDeltaT;
										}

										// 对特定方向关闭控制
										for(i_R=0;i_R<6;i_R++)
										{
											if(A6D_enable[i_R] == 0)
											{
												A6D_VL[i_R] = 0;
												A6D_PL[i_R] = 0.0;
											}
										}
										A6D_PL[3] = 0.0;
										A6D_PL[4] = 0.0;
										A6D_PL[5] = 0.0;

										// 计算阻抗控制末端变换矩阵
										double deltaTRaw[4][4],deltaT[4][4];

										delta2tr(A6D_PL, deltaTRaw);
										Schmidt(deltaTRaw, deltaT);

										deltaT[0][3] = deltaT[0][3]*1000;
										deltaT[1][3] = deltaT[1][3]*1000;
										deltaT[2][3] = deltaT[2][3]*1000;  // mm

										// 计算阻抗控制总变换矩阵
										double TempT[4][4];
										matrix_multiply(TotalDeltaTL, deltaT, TempT);
										memcpy(TotalDeltaTL,TempT,sizeof(TempT));
										//printf("TotalDeltaTL:%lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf\n",TotalDeltaTL[0][0],TotalDeltaTL[0][1],TotalDeltaTL[0][2],TotalDeltaTL[0][3],TotalDeltaTL[1][0],TotalDeltaTL[1][1],TotalDeltaTL[1][2],TotalDeltaTL[1][3],TotalDeltaTL[2][0],TotalDeltaTL[2][1],TotalDeltaTL[2][2],TotalDeltaTL[2][3]);

									}
								}
								double ControlTL[4][4];
								// 计算叠加阻抗控制后末端矩阵
								matrix_multiply(RemoteTendL,TotalDeltaTL,ControlTL);

								// 通过逆运动学计算当前位姿下关节角
								double AngleLNow[7];
								double AngleLBeta[7];
								for(i_R=0; i_R<7; i_R++)
								{
								//	AngleLNow[i_R] = RobotAngleFB.LeftArm[i_R];
									AngleLNow[i_R] = A6D_Joint_PL[i_R]*Degree2Rad;
									AngleLBeta[i_R] = RemoteRobotPos_deg.LeftArm[i_R]*Degree2Rad;
								}
								double betaL;
								betaL = Beta_CalL(AngleLBeta);	// input: rad
								invKinL(AngleLNow, ControlTL, betaL, A6D_Joint_PL);	// input:
							//	printf("AngleLNow = %f, %f, %f, %f, %f, %f, %f\n", AngleLNow[0],AngleLNow[1],AngleLNow[2],AngleLNow[3],AngleLNow[4],AngleLNow[5],AngleLNow[6]);

								Joint_Angle_EP[2][0] = JointDetect(2, 0, A6D_Joint_PL[0]*Degree2Rad);
								Joint_Angle_EP[3][0] = JointDetect(3, 0, A6D_Joint_PL[1]*Degree2Rad);
								Joint_Angle_EP[3][1] = JointDetect(3, 1, A6D_Joint_PL[2]*Degree2Rad);
								Joint_Angle_EP[2][1] = JointDetect(2, 1, A6D_Joint_PL[3]*Degree2Rad);
								Joint_Angle_EP[0][4] = JointDetect(0, 4, A6D_Joint_PL[4]*Degree2Rad);
								Joint_Angle_EP[0][5] = JointDetect(0, 5, A6D_Joint_PL[5]*Degree2Rad);
								Joint_Angle_EP[0][6] = JointDetect(0, 6, A6D_Joint_PL[6]*Degree2Rad);

								if(ForceNewDataR==1)	//右臂control 利用逆运动学进行融合
								{
									int position_change_flagR = 1;
									// 当末端力超过六维力传感器限制时关闭运动控制
									for(i_R=0; i_R<6; i_R++)
									{
										if(fabs(ForceDataR[i_R]) > force_limit[i_R])
										{
											position_change_flagR = 0;
											motion_enable_flag = 0;
											printf("forceR limited\n");
											break;
										}
									}

									if(position_change_flagR == 1)
									{
										// 第一次进入力控制时
										if(FirstForceControlR == 1)
										{
											// 力控输出角等于当前关节角
											memcpy(&OneArmStart,&RobotAngleFBDeg,sizeof(OneArmStart));
											A6D_Joint_PR[0] = 45.0;
											A6D_Joint_PR[1] = -60.0;
											A6D_Joint_PR[2] = 0.0;
											A6D_Joint_PR[3] = -30.0;
											A6D_Joint_PR[4] = 0.0;
											A6D_Joint_PR[5] = -30.0;
											A6D_Joint_PR[6] = 0.0;
											for(i_R=0;i_R<7;i_R++)
											{
											//	A6D_Joint_PR[i_R] = OneArmStart.RightArm[i_R];
												A6D_Joint_VR[i_R] = 0;
											}
											// 初始化总位移矩阵
											memset(TotalDeltaTR,0,sizeof(TotalDeltaTR));
											TotalDeltaTR[0][0] = 1;
											TotalDeltaTR[1][1] = 1;
											TotalDeltaTR[2][2] = 1;
											TotalDeltaTR[3][3] = 1;

											FirstForceControlR = 0;
										}

										// 计算阻抗控制下，此步移动量
										for(i_R=0;i_R<6;i_R++)
										{
											A6D_C[i_R] = 2*A6D_ep[i_R]*sqrt(A6D_K[i_R]*A6D_m[i_R]);
										}
										for(i_R=0;i_R<6;i_R++)
										{
											AccR[i_R] = (ForceDataR[i_R] - A6D_K[i_R]*A6D_D_totalR[i_R] - A6D_C[i_R]*A6D_VR[i_R])/A6D_m[i_R];
											A6D_D_totalR[i_R] = A6D_D_totalR[i_R] + AccR[i_R]*FDeltaT*FDeltaT/2.0 + A6D_VR[i_R]*FDeltaT;
											A6D_VR[i_R] = A6D_VR[i_R] + AccR[i_R]*FDeltaT;
											A6D_PR[i_R] = A6D_VR[i_R]*FDeltaT;
										}

										// 对特定方向关闭控制
										for(i_R=0;i_R<6;i_R++)
										{
											if(A6D_enable[i_R] == 0)
											{
												A6D_VR[i_R] = 0;
												A6D_PR[i_R] = 0.0;
											}
										}
										A6D_PR[3] = 0.0;
										A6D_PR[4] = 0.0;
										A6D_PR[5] = 0.0;

										// 计算阻抗控制末端变换矩阵
										double deltaTRaw[4][4],deltaT[4][4];

										delta2tr(A6D_PR, deltaTRaw);
										Schmidt(deltaTRaw, deltaT);

										deltaT[0][3] = deltaT[0][3]*1000;
										deltaT[1][3] = deltaT[1][3]*1000;
										deltaT[2][3] = deltaT[2][3]*1000;  // mm

										// 计算阻抗控制总变换矩阵
										double TempT[4][4];
										matrix_multiply(TotalDeltaTR, deltaT, TempT);
										memcpy(TotalDeltaTR,TempT,sizeof(TempT));
										//printf("TotalDeltaTR:%lf %lf %lf %lf\n %lf %lf %lf %lf\n %lf %lf %lf %lf\n",TotalDeltaTR[0][0],TotalDeltaTR[0][1],TotalDeltaTR[0][2],TotalDeltaTR[0][3],TotalDeltaTR[1][0],TotalDeltaTR[1][1],TotalDeltaTR[1][2],TotalDeltaTR[1][3],TotalDeltaTR[2][0],TotalDeltaTR[2][1],TotalDeltaTR[2][2],TotalDeltaTR[2][3]);

									}
								}

								double ControlTR[4][4];
								// 计算叠加阻抗控制后末端矩阵
								matrix_multiply(RemoteTendR,TotalDeltaTR,ControlTR);

								// 通过逆运动学计算当前位姿下关节角
								double AngleRNow[7];
								double AngleRBeta[7];
								for(i_R=0; i_R<7; i_R++)
								{
								//	AngleRNow[i_R] = RobotAngleFB.RightArm[i_R];
									AngleRNow[i_R] = A6D_Joint_PR[i_R]*Degree2Rad;
									AngleRBeta[i_R] = RemoteRobotPos_deg.RightArm[i_R]*Degree2Rad;
								}
								double betaR;
								betaR = Beta_CalR(AngleRBeta);	// input: rad
								invKinR(AngleRNow, ControlTR, betaR, A6D_Joint_PR);	// input:
							//	printf("AngleRNow = %f, %f, %f, %f, %f, %f, %f\n", AngleRNow[0],AngleRNow[1],AngleRNow[2],AngleRNow[3],AngleRNow[4],AngleRNow[5],AngleRNow[6]);

								Joint_Angle_EP[2][2] = JointDetect(2, 2, A6D_Joint_PR[0]*Degree2Rad);
								Joint_Angle_EP[3][2] = JointDetect(3, 2, A6D_Joint_PR[1]*Degree2Rad);
								Joint_Angle_EP[3][3] = JointDetect(3, 3, A6D_Joint_PR[2]*Degree2Rad);
								Joint_Angle_EP[2][3] = JointDetect(2, 3, A6D_Joint_PR[3]*Degree2Rad);
								Joint_Angle_EP[1][4] = JointDetect(1, 4, A6D_Joint_PR[4]*Degree2Rad);
								Joint_Angle_EP[1][5] = JointDetect(1, 5, A6D_Joint_PR[5]*Degree2Rad);
								Joint_Angle_EP[1][6] = JointDetect(1, 6, A6D_Joint_PR[6]*Degree2Rad);

								// 加入碰撞检测
								int rtnn = 0;
								struct RealRobot_Struct SendAngle;
								memset(&SendAngle,0,sizeof(SendAngle));
								CanDef2RealRobot(Joint_Angle_EP, &SendAngle);
								rtnn = CollisionDetection(SendAngle.LeftArm, SendAngle.RightArm, SendAngle.Waist);
								if (rtnn == 0)
								{
									printf("Detect collision!\n");
									NoCollisionFlag = 1;
									RemoteMotion_enable_flag = 0;
								}
								else
								{
								//	printf("No collision!\n");
									NoCollisionFlag = 1;
								}

								if(motion_enable_flag == 1)
								{
									rad_send(0, 4, Joint_Angle_EP[0][4]);		//moyang602 	可根据通道发送减少时间
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(0, 5, Joint_Angle_EP[0][5]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(0, 6, Joint_Angle_EP[0][6]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(2, 0, Joint_Angle_EP[2][0]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(2, 1, Joint_Angle_EP[2][1]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(3, 0, Joint_Angle_EP[3][0]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(3, 1, Joint_Angle_EP[3][1]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);

									rad_send(1, 4, Joint_Angle_EP[1][4]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(1, 5, Joint_Angle_EP[1][5]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(1, 6, Joint_Angle_EP[1][6]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(2, 2, Joint_Angle_EP[2][2]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(2, 3, Joint_Angle_EP[2][3]);
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(3, 2, Joint_Angle_EP[3][2]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
									rad_send(3, 3, Joint_Angle_EP[3][3]);
									sleeptime.tv_nsec = 200000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}

								double AngleH[2][4] = {{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0}};
								control_handL(rockerL, motor_current[0][1], motor_current[0][2], motor_current[0][3], AngleH[0]);
								control_handR(rockerR, motor_current[1][1], motor_current[1][1], motor_current[1][1], AngleH[1]);


								for (i_R=0; i_R<2; i_R++)
								{
									Joint_Angle_EP[i_R][0] = JointDetect(i_R, 0, AngleH[i_R][0]*Degree2Rad);
									Joint_Angle_EP[i_R][1] = JointDetect(i_R, 1, AngleH[i_R][1]*Degree2Rad);
									Joint_Angle_EP[i_R][2] = JointDetect(i_R, 2, AngleH[i_R][2]*Degree2Rad);
									Joint_Angle_EP[i_R][3] = JointDetect(i_R, 3, AngleH[i_R][3]*Degree2Rad);
								}


								if(motion_enable_flag == 1)
								{
									for (i_R = 0; i_R < 4; i_R++)
									{
										rad_send(0,i_R,Joint_Angle_EP[0][i_R]);
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
										rad_send(1,i_R,Joint_Angle_EP[1][i_R]);
										sleeptime.tv_nsec = 200000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
									}
								}
							}
						}
						break;
						case 3:
						{

						}
						break;
						default:
						break;

					}

			 	}
			 	break;

				default:
				break;

			}

			// 数据同步接收
			SYNC_Receive();
		}

		/********************** UDP通讯相关 Start ***************************/
		UDPTimes++;
		if(UDPTimes >= UDPCYCLE)
		{
			UDPTimes = 0;
			struct RobotDataUDP_Struct UploadData;
			memset(&UploadData,0,sizeof(UploadData));

			UploadData.TrustFlag = 0x5555;
			if(motion_enable_flag)
			{
				CanDef2RealRobot(Joint_Angle_FB, &UploadData.RobotAngle);
			}
			else
			{
				CanDef2RealRobot(Joint_Angle_EP, &UploadData.RobotAngle);
			}

			CanDef2RealRobot(motor_current, &UploadData.RobotCurrent);

			CanDef2RealRobot(Joint_Angle_EP, &UploadData.RobotAngleEX);

			RobotFBSend(UploadData);
			int rtnMode = UDPRecv();
			if(moving_flag == 0)	// 运动已完成
			{
				switch(rtnMode)
				{
					case SINGLE_JOINT_MOTION:	//单关节运动模式
					{
						GetSingleJointData(&can_channel_main,&can_id_main,&JointMoveData,&singe_joint_time);
						motion_mode = SINGLE_JOINT_MOTION;
					}
					break;

					case ONE_ARM_MOTION:
					{
						GetSingleArmData(&ArmSelect,One_arm_Data, &one_arm_time);
						motion_mode = ONE_ARM_MOTION;
					}
					break;

					case ROBOT_CONTROL:		//机器人控制模式
					{
						control_mode = GetControlCMD();
					}
					break;

					case REMOTE_DATA:
					{
						GetRemoteData(&rockerL, &rockerR, RemoteMotionData);
						RemoteNewData = 1;
					}
					break;

					case REMOTE_CONTROL:
					{
						int RemoteMode = 0;
						RemoteMode = GetRemoteCMD();
						switch(RemoteMode)
						{
							case REMOTE_START:
								motion_mode = REMOTE_MOTION;
							break;
							case REMOTE_STOP:
								motion_mode = 100;
							break;
							case REMOTE_ENABLE:
								RemoteMotion_enable_flag = 1;
							break;
							case REMOTE_DISABLE:
								RemoteMotion_enable_flag = 0;
							break;
							default:
							break;
						}
					}
					break;

					case HAND_MOTION:
					{
						motion_mode = GetHandCMD(&HandSelect, &HandAngleL, &HandAngleR);
						HandAngleL = HandAngleL*Degree2Rad;
						HandAngleR = HandAngleR*Degree2Rad;
						HandNewData = 1;

					}
					break;

					case FIND_HOME_MOTION:
					{
						GetFindHomeData(&can_channel_main,&can_id_main);
						motion_mode = FIND_HOME_MOTION;
					}
					break;

					case FORCE_CONTROL:
					{
						int ForceMode = 0;
						int ParamType = 0;
						float ForceParam[6] = {0.0};
						ForceMode = GetForceCMD(&ParamType, ForceParam);
						switch(ForceMode)
						{
							case FORCE_START:
								motion_mode = FORCE_MOTION;
								printf("ForceControl Start\n");
							break;
							case FORCE_ENABLE:
								ForceControl_flagL = 1;
								ForceControl_flagR = 1;
								printf("ForceControl Enable\n");
							break;
							case FORCE_DISABLE:
								ForceControl_flagL = 0;
								ForceControl_flagR = 0;
								printf("ForceControl Disable\n");
								memset(buf24,0,sizeof(buf24));
								memset(buf25,0,sizeof(buf25));
							break;
							case FORCE_STOP:
								motion_mode = 100;
								force_stepL = 1;
								nStatusL = 0;
								bIsSendFlagL = 1;
								bReceivedL = 0;
								ForceTCPFlagL = 0;

								force_stepR = 1;
								nStatusR = 0;
								bIsSendFlagR = 1;
								bReceivedR = 0;
								ForceTCPFlagR = 0;
								printf("ForceControl Stop\n");
								memset(buf21,0,sizeof(buf21));
								memset(buf22,0,sizeof(buf22));
								memset(buf23,0,sizeof(buf23));
								memset(buf24,0,sizeof(buf24));
								memset(buf25,0,sizeof(buf25));
								ForceSensorTCP_end();
							break;
							case FORCE_CLEAR:
								First_ForceL = 1;
								First_ForceR = 1;
							break;
							case FORCE_SETPARAM:
							{
								int ii = 0;
								if(ParamType == 1)
								{
									for(ii=0;ii<6;ii++)
										A6D_m[ii] = ForceParam[ii];
									printf("Set A6D_M: %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n",A6D_m[0],A6D_m[1],A6D_m[2],A6D_m[3],A6D_m[4],A6D_m[5]);
								}
								else if(ParamType == 2)
								{
									for(ii=0;ii<6;ii++)
										force_velocity_limit[ii] = ForceParam[ii];
									printf("Set force_velocity_limit: %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n",force_velocity_limit[0],force_velocity_limit[1],force_velocity_limit[2],force_velocity_limit[3],force_velocity_limit[4],force_velocity_limit[5]);
								}
								else if(ParamType == 3)
								{
									for(ii=0;ii<6;ii++)
										A6D_K[ii] = ForceParam[ii];
									printf("Set A6D_K: %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n",A6D_K[0],A6D_K[1],A6D_K[2],A6D_K[3],A6D_K[4],A6D_K[5]);
								}
								else if(ParamType == 4)
								{
									for(ii=0;ii<6;ii++)
										A6D_enable[ii] = ForceParam[ii];
									printf("Set A6D_enable: %d   %d   %d   %d   %d   %d\n",A6D_enable[0],A6D_enable[1],A6D_enable[2],A6D_enable[3],A6D_enable[4],A6D_enable[5]);
								}
							}
							break;
							default:
							break;
						}
					}
					break;

					case DUTY_MOTION:
					{
						int DutyFlag = 0;
						DutyFlag = GetDutyCMD();
						switch(DutyFlag)
						{
							case DUTY_START:
								motion_mode = DUTY_MOTION;
								DutyStep = 0;
							break;
							case DUTY_STOP:
								DutyStep = 0;
								motion_mode = 100;

								force_stepL = 1;
								nStatusL = 0;
								bIsSendFlagL = 1;
								bReceivedL = 0;
								ForceTCPFlagL = 0;

								force_stepR = 1;
								nStatusR = 0;
								bIsSendFlagR = 1;
								bReceivedR = 0;
								ForceTCPFlagR = 0;
								printf("ForceControl Stop\n");
								memset(buf21,0,sizeof(buf21));
								memset(buf22,0,sizeof(buf22));
								memset(buf23,0,sizeof(buf23));
								memset(buf24,0,sizeof(buf24));
								memset(buf25,0,sizeof(buf25));
								ForceSensorTCP_end();

							break;
							case FORCE_CLEAR:
								First_ForceL = 1;
								First_ForceR = 1;
							break;
							case DUTY_RUN:
								DutyStep = 1;
							break;
							case REMOTE_ENABLE:
								RemoteMotion_enable_flag = 1;
							break;
							case REMOTE_DISABLE:
								RemoteMotion_enable_flag = 0;
							break;
							case DUTY_FORCESTART:
								DutyForceFlag = 1;
							break;
							case DUTY_FORCESTOP:
								DutyForceFlag = 0;
							break;

							default:
							break;
						}
					}
					break;

					default:
					break;
				}

			}
			else if(moving_flag&&rtnMode)
			{
				printf("Motion has not completed!\n");
			}

		}

		/********************** UDP通讯相关 End ***************************/

		/************************* 界面显示 **************************/

		memset(&RobotRealCurrent,0,sizeof(RobotRealCurrent));

		CanDef2RealRobot(motor_current, &RobotRealCurrent);

		sprintf(buf2, "Voltage:  %8.3f    Current:  %8.3f",VoltageFB,CurrentFB);
		sprintf(buf3, "******************************  Robot Joint Angle  ******************************");

		sprintf(buf4, "LArmRecv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleFBDeg.LeftArm[0], RobotAngleFBDeg.LeftArm[1], RobotAngleFBDeg.LeftArm[2], RobotAngleFBDeg.LeftArm[3], RobotAngleFBDeg.LeftArm[4], RobotAngleFBDeg.LeftArm[5], RobotAngleFBDeg.LeftArm[6]);
		sprintf(buf5, "LArmSend  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleEPDeg.LeftArm[0], RobotAngleEPDeg.LeftArm[1], RobotAngleEPDeg.LeftArm[2], RobotAngleEPDeg.LeftArm[3], RobotAngleEPDeg.LeftArm[4], RobotAngleEPDeg.LeftArm[5], RobotAngleEPDeg.LeftArm[6]);
		sprintf(buf6, "LArmErr   %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleERDeg.LeftArm[0], RobotAngleERDeg.LeftArm[1], RobotAngleERDeg.LeftArm[2], RobotAngleERDeg.LeftArm[3], RobotAngleERDeg.LeftArm[4], RobotAngleERDeg.LeftArm[5], RobotAngleERDeg.LeftArm[6]);

		sprintf(buf7, "RArmRecv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleFBDeg.RightArm[0], RobotAngleFBDeg.RightArm[1], RobotAngleFBDeg.RightArm[2], RobotAngleFBDeg.RightArm[3], RobotAngleFBDeg.RightArm[4], RobotAngleFBDeg.RightArm[5], RobotAngleFBDeg.RightArm[6]);
		sprintf(buf8, "RArmSend  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleEPDeg.RightArm[0], RobotAngleEPDeg.RightArm[1], RobotAngleEPDeg.RightArm[2], RobotAngleEPDeg.RightArm[3], RobotAngleEPDeg.RightArm[4], RobotAngleEPDeg.RightArm[5], RobotAngleEPDeg.RightArm[6]);
		sprintf(buf9, "RArmErr   %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotAngleERDeg.RightArm[0], RobotAngleERDeg.RightArm[1], RobotAngleERDeg.RightArm[2], RobotAngleERDeg.RightArm[3], RobotAngleERDeg.RightArm[4], RobotAngleERDeg.RightArm[5], RobotAngleERDeg.RightArm[6]);

		sprintf(buf10, "LHandRecv %8.3f  %8.3f  %8.3f  %8.3f    HeadRecv:%8.3f  %8.3f",
		RobotAngleFBDeg.LeftHand[0], RobotAngleFBDeg.LeftHand[1], RobotAngleFBDeg.LeftHand[2], RobotAngleFBDeg.LeftHand[3], RobotAngleFBDeg.Head[0], RobotAngleFBDeg.Head[1]);
		sprintf(buf11, "LHandSend %8.3f  %8.3f  %8.3f  %8.3f    HeadSend:%8.3f  %8.3f",
		RobotAngleEPDeg.LeftHand[0], RobotAngleEPDeg.LeftHand[1], RobotAngleEPDeg.LeftHand[2], RobotAngleEPDeg.LeftHand[3], RobotAngleEPDeg.Head[0], RobotAngleEPDeg.Head[1]);
		sprintf(buf12, "LHandErr  %8.3f  %8.3f  %8.3f  %8.3f    HeadErr: %8.3f  %8.3f",
		RobotAngleERDeg.LeftHand[0], RobotAngleERDeg.LeftHand[1], RobotAngleERDeg.LeftHand[2], RobotAngleERDeg.LeftHand[3], RobotAngleERDeg.Head[0], RobotAngleERDeg.Head[1]);

		sprintf(buf13, "RHandRecv %8.3f  %8.3f  %8.3f  %8.3f    WaistRecv:%7.3f  %8.3f",
		RobotAngleFBDeg.RightHand[0], RobotAngleFBDeg.RightHand[1], RobotAngleFBDeg.RightHand[2], RobotAngleFBDeg.RightHand[3], RobotAngleFBDeg.Waist[0], RobotAngleFBDeg.Waist[1]);
		sprintf(buf14, "RHandSend %8.3f  %8.3f  %8.3f  %8.3f    WaistSend:%7.3f  %8.3f",
		RobotAngleEPDeg.RightHand[0], RobotAngleEPDeg.RightHand[1], RobotAngleEPDeg.RightHand[2], RobotAngleEPDeg.RightHand[3], RobotAngleEPDeg.Waist[0], RobotAngleEPDeg.Waist[1]);
		sprintf(buf15, "RHandErr  %8.3f  %8.3f  %8.3f  %8.3f    WaistErr: %7.3f  %8.3f",
		RobotAngleERDeg.RightHand[0], RobotAngleERDeg.RightHand[1], RobotAngleERDeg.RightHand[2], RobotAngleERDeg.RightHand[3], RobotAngleERDeg.Waist[0], RobotAngleERDeg.Waist[1]);

		sprintf(buf16, "*********************************  Robot current *********************************");
		sprintf(buf17, "LArmCur   %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotRealCurrent.LeftArm[0], RobotRealCurrent.LeftArm[1], RobotRealCurrent.LeftArm[2], RobotRealCurrent.LeftArm[3], RobotRealCurrent.LeftArm[4], RobotRealCurrent.LeftArm[5], RobotRealCurrent.LeftArm[6]);
		sprintf(buf18, "RArmCur   %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		RobotRealCurrent.RightArm[0], RobotRealCurrent.RightArm[1], RobotRealCurrent.RightArm[2], RobotRealCurrent.RightArm[3], RobotRealCurrent.RightArm[4], RobotRealCurrent.RightArm[5], RobotRealCurrent.RightArm[6]);
		sprintf(buf19, "LHandCur  %8.3f  %8.3f  %8.3f  %8.3f    HeadCur: %8.3f  %8.3f",
		RobotRealCurrent.LeftHand[0], RobotRealCurrent.LeftHand[1], RobotRealCurrent.LeftHand[2], RobotRealCurrent.LeftHand[3], RobotRealCurrent.Head[0], RobotRealCurrent.Head[1]);
		sprintf(buf20, "RHandCur  %8.3f  %8.3f  %8.3f  %8.3f    WaistCur:%8.3f  %8.3f",
		RobotRealCurrent.RightHand[0], RobotRealCurrent.RightHand[1], RobotRealCurrent.RightHand[2], RobotRealCurrent.RightHand[3], RobotRealCurrent.Waist[0], RobotRealCurrent.Waist[1]);

	}
	GPS_end();
	JY901_end();
	fclose(fp);
}



int can_rt_init(char channal[4][16], long baudrate_set)
{
//	char    ifname[16];
	int can_fd[4] = {-1, -1, -1, -1};
	long     new_baudrate = -1;
	int     new_mode = -1;
	//    int     verbose = 0;

	can_baudrate_t *baudrate;
	can_ctrlmode_t *ctrlmode;
	can_mode_t *mode;
	struct can_bittime *bittime;
	int opt, ret;
	char* ptr;
	int i = 0;

	/////////////////////////config/////////////////////////
	for(i=0; i<4; i++)
	{
		strncpy(ifr[i].ifr_name, channal[i], IFNAMSIZ);
		printf("%s\n", ifr[i].ifr_name);
	}


	for(i=0; i<4; i++)
	{
		can_fd[i] = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (can_fd[i] < 0)
		{
			fprintf(stderr, "Cannot open RTDM CAN socket. Maybe driver not loaded? \n");
			return can_fd[i];
		}

		ret = rt_dev_ioctl(can_fd[i], SIOCGIFINDEX, &ifr[i]);
		if (ret)
		{
			fprintf(stderr,"Can't get interface index for %s, code = %d\n", ifr[i].ifr_name, ret);
			return ret;
		}

		baudrate = (can_baudrate_t *)&ifr[i].ifr_ifru;
		new_baudrate = baudrate_set;
		*baudrate = new_baudrate;

		ret = rt_dev_ioctl(can_fd[i], SIOCSCANBAUDRATE, &ifr[i]);
		if(ret <0)
		{
			fprintf(stderr, "baudrate error\n");
			goto abort;
    	}

		new_mode = CAN_MODE_START;
		mode = (can_mode_t *)&ifr[i].ifr_ifru;
        *mode = new_mode;
        ret = rt_dev_ioctl(can_fd[i], SIOCSCANMODE, &ifr[i]);
        if (ret<0)
		{
            goto abort;
        }
		rt_dev_close(can_fd[i]);
	}

	/////////////////// receive config/////////////////////////
	/* Get CAN interface name */

	for(i=0; i<4; i++)
	{
		rxsock[i] = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (rxsock[i] < 0)
		{
			fprintf(stderr, "Cannot open RTDM CAN socket. Maybe driver not loaded? \n");
			goto abort;
		}

		ret = rt_dev_ioctl(rxsock[i], SIOCGIFINDEX, &ifr[i]);
		if (ret<0)
		{
			fprintf(stderr,"Can't get interface index for %s, code = %d\n", ifr[i].ifr_name, ret);
			goto abort;
		}

		memset(&recv_addr, 0, sizeof(recv_addr));
		recv_addr.can_family = AF_CAN;
		recv_addr.can_ifindex = ifr[i].ifr_ifindex;
		ret = rt_dev_bind(rxsock[i], (struct sockaddr *)&recv_addr, sizeof(struct sockaddr_can));
		if (ret < 0)
		{
			fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
			goto abort;
		}

		ret = rt_dev_ioctl(rxsock[i], RTCAN_RTIOC_RCV_TIMEOUT, &timeout_rv);
		if (ret < 0)
		{
			fprintf(stderr, "rt_dev_ioctl RCV_TIMEOUT: %s\n", strerror(-ret));
			goto abort;
		}
	}

	for(i=0; i<4; i++)
	{
		txsock[i] = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (txsock[i] < 0)
		{
			fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
			return -1;
		}
		printf("txsock=%d, ifr_name=%s\n", txsock[i], ifr[i].ifr_name);

		ret = rt_dev_ioctl(txsock[i], SIOCGIFINDEX, &ifr[i]);
		if (ret < 0)
		{
			fprintf(stderr,"Can't get interface index for %s, code = %d\n", ifr[i].ifr_name, ret);
			return ret;
		}

		memset(&to_addr, 0, sizeof(to_addr));
		to_addr.can_ifindex = ifr[i].ifr_ifindex;
		to_addr.can_family = AF_CAN;

	/*
		ret = rt_dev_setsockopt(txsock, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
		if (ret < 0)
		{
			fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
			goto abort;
		}
	*/
		ret = rt_dev_bind(txsock[i], (struct sockaddr *)&to_addr, sizeof(to_addr));
		printf("to_addr.can_ifindex = %d\n",to_addr.can_ifindex);

		if (ret < 0)
		{
			fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
			goto abort;
		}

		ret = rt_dev_ioctl(txsock[i], RTCAN_RTIOC_SND_TIMEOUT, &timeout_send);
		if (ret<0)
		{
			fprintf(stderr, "rt_dev_ioctl SND_TIMEOUT: %s\n", strerror(-ret));
			goto abort;
		}
	}
	return 0;
//////////////////////////////send init/////////////////////////////////
//	frame.can_id = 1;



 abort:
	printf("abort error\n");
	for(i=0; i<4; i++)
	{
	    rt_dev_close(can_fd[i]);
	}
    return ret;
}

int main(int argc, char* argv[])
{
 //   signal(SIGTERM, catch_signal);
//	signal(SIGINT, catch_signal);
	/* Avoids memory swapping for this program */
	Font Myfont;

	int ret = -1;
	char name[32];
	mlockall(MCL_CURRENT|MCL_FUTURE);
	char channal[4][16] = {"rtcan0","rtcan1","rtcan2","rtcan3"};
	int i =0;
	int s1;

	RobotUDPComm_init();
	JY901_init();
	GPS_init();
	can_rt_init(channal, 1000000);


// Open screen/window
	MyDisplay = XOpenDisplay(0);
	s1 = DefaultScreen(MyDisplay);
	MyWindow = XCreateSimpleWindow(MyDisplay, DefaultRootWindow(MyDisplay),
	50, 50, 1000, 750, 0, BlackPixel(MyDisplay, s1),
	WhitePixel(MyDisplay, s1));

	// We want to know about the user clicking on the close window button
	wm_delete_window = XInternAtom(MyDisplay, "WM_DELETE_WINDOW", 0);
	XSetWMProtocols(MyDisplay, MyWindow, &wm_delete_window, 1);
	wm_protocols = XInternAtom(MyDisplay, "WM_PROTOCOLS", 0);

	MyGC = XCreateGC(MyDisplay, MyWindow,0,0);
	Myfont = XLoadFont(MyDisplay, "10x20");
	XSetFont(MyDisplay, MyGC, Myfont);

	// We want to know about ClientMessages
//	XSelectInput(MyDisplay, MyWindow, StructureNotifyMask|OwnerGrabButtonMask);
	XSelectInput(MyDisplay, MyWindow, KeyPressMask);

	// Display the window
	XMapWindow(MyDisplay, MyWindow);
	// Get an Atom to use for my custom message. Arbitrarily name the atom
	// "MyAtom"

	MyAtom = XInternAtom(MyDisplay, "MyAtom", 0);

	// Send my window my custom ClientMessage event
//	XEvent event;

	sem_init (&low_go,0,0);
/*
	pthread_attr_init(&my_attr);
	pthread_attr_setdetachstate(&my_attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setstacksize(&my_attrr, PTHREAD_STACK_MIN);
	ret = pthread_create(&th2, &my_attrr, &low, NULL);
*/
//	pthread_attr_setschedparam(&my_attr, &param2);
//	pthread_create(&th2, &my_attr, low, 0);

//	pthread_attr_destroy(&my_attr);
//	pthread_join(th2, NULL);




	snprintf(name, sizeof(name), "rtcansend-%d", getpid());
	ret = rt_task_shadow(&rt_task_desc, name, 1, 0);
	if (ret<0) {
		fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
	//	goto failure;
	}


	rt_task_create(&rt_task_view, "view", 0, 30, 0);


	rt_task_create(&demo_task_rvcan, "can_rv", 0, 99, 0);
        /*
         * Arguments: &task,
         *            task function,
         *            function argument
         */
	rt_task_start(&rt_task_view, &view, NULL);
	rt_task_start(&demo_task_rvcan, &rt_can_recv, NULL);

	pause();
	printf("close\n");
	rt_task_delete(&rt_task_view);
	rt_task_delete(&demo_task_rvcan);
//	pthread_join(th2, NULL);

	return 0;
}


int elmo_init(void)
{
	long can_id = 1;
	long can_content[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[27] = {0};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
//	int k=0;

	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;

	for(i=0; i<can_channel_number; i++)
	{

		can_id = 0x00;
		can_content_order[0] = 0x82;
		can_content_order[1] = 0x00;
		can_send(i, can_id, 0, can_content_order, 2);  //RESET nodes parameters
	}

	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret >= 0)
			{
				can_connection_status[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
			}
		}
	}

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x16;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //disable RPDO1
	}

	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x23;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x16;
		can_content_order[3] = 0x01;
		can_content_order[4] = 0x10;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x40;
		can_content_order[7] = 0x60;
		can_send(i, can_id, 0, can_content_order, 8);   //MAP CONTROLWORD TO RPDO1
	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x23;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x16;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0x20;
		can_content_order[5] = 0x01;
		can_content_order[6] = 0xC1;
		can_content_order[7] = 0x60;
		can_send(i, can_id, 0, can_content_order, 8);   //MAP IP data to RPDO1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x14;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0x01;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //RPDO1 become valid every sync

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x16;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x02;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //enable 2 mapping object

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x1A;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //disable TPDO1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x23;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x1A;
		can_content_order[3] = 0x01;
		can_content_order[4] = 0x20;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x64;
		can_content_order[7] = 0x60;
		can_send(i, can_id, 0, can_content_order, 8);   //MAP actual position to TPDO1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x23;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x1A;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0x10;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x41;
		can_content_order[7] = 0x60;
		can_send(i, can_id, 0, can_content_order, 8);   //MAP Statusword to TPDO1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x23;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x1A;
		can_content_order[3] = 0x03;
		can_content_order[4] = 0x10;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x78;
		can_content_order[7] = 0x60;
		can_send(i, can_id, 0, can_content_order, 8);   //MAP Statusword to TPDO1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x18;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0x01;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //map current to tpdo1

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x1A;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x03;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //enable 3 mapping objects

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x60;
		can_content_order[2] = 0x60;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x07;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //operation mode：IP

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0xC2;
		can_content_order[2] = 0x60;
		can_content_order[3] = 0x01;
		can_content_order[4] = 0x06;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //INTERPOLATION TIME PERIOD:2 MESC
	}

	nanosleep(&sleeptime,NULL);
	driver_init_status();


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0xC2;
		can_content_order[2] = 0x60;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0xFD;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //INTERPOLATION TIME index：10e·-3

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x600;
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0xC4;
		can_content_order[2] = 0x60;
		can_content_order[3] = 0x06;
		can_content_order[4] = 0x01;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //INTERPOLATION buffer clear

	}
	nanosleep(&sleeptime,NULL);
	driver_init_status();


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x00;
		can_content_order[0] = 0x01;
		can_content_order[1] = 0x00;
		can_send(i, can_id, 0, can_content_order, 2);   //start remote node (enter operational)

	}
	nanosleep(&sleeptime,NULL);

	printf("leg1 to leg3 elmo init finish\n");

	return 0;
}



void driver_init_status(void)
{
//	int k = 0;
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{
				can_work_states[4*i + j] = 0;
			}
			else
			{
				if(frame.data[0] == 0x60)
				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}
				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}
		}
	}

	sprintf(buf27, "CAN init        %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1],  can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14], can_work_states[15], can_work_states[16], can_work_states[17], can_work_states[18], can_work_states[19], can_work_states[20], can_work_states[21], can_work_states[22], can_work_states[23], can_work_states[24], can_work_states[25], can_work_states[26]);
}



void servo_off(void)
{
	long can_id =1;
	long can_content[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[27] = {0};
//	int k=0;

	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x200;

		can_content_order[0] = 0x06;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x00;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;

		can_content_order[5] = 0x00;
		can_send(i, can_id, 0, can_content_order, 6);   //controlword: enable operation
	}

	nanosleep(&sleeptime,NULL);


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC

	}

	sleeptime.tv_nsec = 200000000;

	sleeptime.tv_sec = 0;

	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{
				can_work_states[4*i + j] = 0;
			}
			else
			{
				if((frame.data[4] == 0x33) && (frame.data[5] == 0x02))
				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}
				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}
		}
	}
	sleeptime.tv_nsec = 2000000;
	sleeptime.tv_sec = 0;


}

void power_on(void)
{
	int can_ch = 3;
	long can_id = 7;
	long can_content[2] = {0x4E, 0x50};
	struct timespec sleeptime;
	int ret;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);

	can_send(can_ch, can_id, 0, can_content, 2);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;

	nanosleep(&sleeptime,NULL);

	ret = rt_dev_recvfrom(rxsock[can_ch], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
	if(ret < 0)
	{
		can_work_states[27] = 0;
		printf("power on receive failed\n");
	}
	else
	{
		if((frame.data[0] == 0x4E) && (frame.data[1] == 0x4E))
		{
			can_work_states[27] = 1;
			printf("power on success\n");
		}

		else
		{
			can_work_states[27] = 0;
			printf("power on failed\n");
		}
	}
}

void power_off(void)
{
	int can_ch = 3;
	long can_id = 7;
	long can_content[2] = {0x46, 0x50};
	struct timespec sleeptime;
	int ret;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);

	can_send(can_ch, can_id, 0, can_content, 2);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;

	nanosleep(&sleeptime,NULL);

	ret = rt_dev_recvfrom(rxsock[can_ch], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
	if(ret < 0)
	{
		can_work_states[27] = 0;
		printf("power off receive failed\n");
	}
	else
	{
		if((frame.data[0] == 0x46) && (frame.data[1] == 0x46))
		{
			can_work_states[27] = 1;
			printf("power off success\n");
		}

		else
		{
			can_work_states[27] = 0;
			printf("power off failed\n");
		}
	}
}


void servo_on(void)
{
//	int i=0;
	long can_id =1;
	long can_content[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
//	int k=0;
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[27] = {0};


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x200;
		can_content_order[0] = 0x00;

		can_content_order[1] = 0x00;
		can_content_order[2] = 0x00;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;
		can_content_order[5] = 0x00;

		can_send(i, can_id, 0, can_content_order, 6);   //controlword: disable volatage
	}
	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC
	}


	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{
				can_work_states[4*i + j] = 0;
			}
			else
			{
				if((frame.data[4] == 0x40) && (frame.data[5] == 0x06))
				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}

				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}
		}

	}

	for(i=0; i<can_channel_number; i++)
	{

		can_id = 0x200;
		can_content_order[0] = 0x06;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x00;

		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;
		can_content_order[5] = 0x00;
		can_send(i, can_id, 0, can_content_order, 6);   //controlword: shutdown (ready to switch on)
	}

	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC
	}
	nanosleep(&sleeptime,NULL);


	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)

		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{

				can_work_states[4*i + j] = 0;
			}
			else
			{

				if((frame.data[4] == 0x40) && (frame.data[5] == 0x06))
				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}

				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}
		}
	}


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x200;
		can_content_order[0] = 0x07;

		can_content_order[1] = 0x00;
		can_content_order[2] = 0x00;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;

		can_content_order[5] = 0x00;
		can_send(i, can_id, 0, can_content_order, 6);   //controlword: switch on
	}


	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{

		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC
		sleeptime.tv_nsec = 10000000;
		sleeptime.tv_sec = 0;

	}

	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{

			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{
				can_work_states[4*i + j] = 0;
			}
			else
			{
				if((frame.data[4] == 0x31) && (frame.data[5] == 0x02))

				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}
				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}

		}
	}

	for(i=0; i<can_channel_number; i++)

	{
		can_id = 0x200;
		can_content_order[0] = 0x0f;
		can_content_order[1] = 0x00;

		can_content_order[2] = 0x00;
		can_content_order[3] = 0x00;
		can_content_order[4] = 0x00;
		can_content_order[5] = 0x00;
		can_send(i, can_id, 0, can_content_order, 6);   //controlword: enable operation
	}

		nanosleep(&sleeptime,NULL);


	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC

	}

	sleeptime.tv_nsec = 200000000;
	sleeptime.tv_sec = 0;


	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
	{
		for(j=0; j< can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier boot can massage
			if(ret < 0)
			{
				can_work_states[4*i + j] = 0;
			}
			else
			{
				if((frame.data[4] == 0x33) && (frame.data[5] == 0x02))
				{
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 1;
				}
				else
					can_work_states[((frame.can_id & 0x00F) - 1) + 4*i] = 0;
			}
		}
	}
/*		sleeptime.tv_nsec = 2000000;
		sleeptime.tv_sec = 0;

		for(i=0; i<can_channel_number; i++)
		{
			can_id = 0x80;
			can_send(i, can_id, 0, can_content_order, 0);   //SYNC
		//	can_send(i, can_id, 0, can_content_order, 0);   //SYNC
		}
		nanosleep(&sleeptime,NULL);
*/

	printf("ret = %d\n", ret);
	printf("servo on %02x  %02x	 %02x  %02x  %02x  %02x  %02x  %02x\n", frame.data[0], frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

	sprintf(buf26, "CAN connection  %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d", can_connection_status[0],can_connection_status[1], can_connection_status[2],can_connection_status[3],can_connection_status[4],can_connection_status[5],can_connection_status[6],can_connection_status[7], can_connection_status[8],can_connection_status[9],can_connection_status[10],can_connection_status[11],can_connection_status[12],can_connection_status[13], can_connection_status[14],can_connection_status[15],can_connection_status[16],can_connection_status[17], can_connection_status[18],can_connection_status[19], can_connection_status[20],can_connection_status[21], can_connection_status[22],can_connection_status[23],can_connection_status[24],can_connection_status[25],can_connection_status[26]);

	sprintf(buf27, "CAN init        %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1],  can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14], can_work_states[15], can_work_states[16], can_work_states[17], can_work_states[18], can_work_states[19], can_work_states[20], can_work_states[21], can_work_states[22], can_work_states[23], can_work_states[24], can_work_states[25], can_work_states[26]);
}

// CAN定义到实际机器人数据结构转换
void CanDef2RealRobot(double CanDef[4][7], struct RealRobot_Struct* RealRobot)
{
	RealRobot->LeftArm[0] = CanDef[2][0];
	RealRobot->LeftArm[1] = CanDef[3][0];
	RealRobot->LeftArm[2] = CanDef[3][1];
	RealRobot->LeftArm[3] = CanDef[2][1];
	RealRobot->LeftArm[4] = CanDef[0][4];
	RealRobot->LeftArm[5] = CanDef[0][5];
	RealRobot->LeftArm[6] = CanDef[0][6];

	RealRobot->RightArm[0] = CanDef[2][2];
	RealRobot->RightArm[1] = CanDef[3][2];
	RealRobot->RightArm[2] = CanDef[3][3];
	RealRobot->RightArm[3] = CanDef[2][3];
	RealRobot->RightArm[4] = CanDef[1][4];
	RealRobot->RightArm[5] = CanDef[1][5];
	RealRobot->RightArm[6] = CanDef[1][6];

	RealRobot->LeftHand[0] = CanDef[0][0];
	RealRobot->LeftHand[1] = CanDef[0][1];
	RealRobot->LeftHand[2] = CanDef[0][2];
	RealRobot->LeftHand[3] = CanDef[0][3];

	RealRobot->RightHand[0] = CanDef[1][0];
	RealRobot->RightHand[1] = CanDef[1][1];
	RealRobot->RightHand[2] = CanDef[1][2];
	RealRobot->RightHand[3] = CanDef[1][3];

	RealRobot->Head[0] = CanDef[2][4];
	RealRobot->Head[1] = CanDef[2][5];

	RealRobot->Waist[0] = CanDef[3][4];
	RealRobot->Waist[1] = CanDef[3][5];
}

// 实际机器人数据结构到CAN定义转换
void RealRobot2CanDef(struct RealRobot_Struct RealRobot, double CanDef[4][7])
{
	CanDef[2][0] = RealRobot.LeftArm[0];
	CanDef[3][0] = RealRobot.LeftArm[1];
	CanDef[3][1] = RealRobot.LeftArm[2];
	CanDef[2][1] = RealRobot.LeftArm[3];
	CanDef[0][4] = RealRobot.LeftArm[4];
	CanDef[0][5] = RealRobot.LeftArm[5];
	CanDef[0][6] = RealRobot.LeftArm[6];

	CanDef[2][2] = RealRobot.RightArm[0];
	CanDef[3][2] = RealRobot.RightArm[1];
	CanDef[3][3] = RealRobot.RightArm[2];
	CanDef[2][3] = RealRobot.RightArm[3];
	CanDef[1][4] = RealRobot.RightArm[4];
	CanDef[1][5] = RealRobot.RightArm[5];
	CanDef[1][6] = RealRobot.RightArm[6];

	CanDef[0][0] = RealRobot.LeftHand[0];
	CanDef[0][1] = RealRobot.LeftHand[1];
	CanDef[0][2] = RealRobot.LeftHand[2];
	CanDef[0][3] = RealRobot.LeftHand[3];

	CanDef[1][0] = RealRobot.RightHand[0];
	CanDef[1][1] = RealRobot.RightHand[1];
	CanDef[1][2] = RealRobot.RightHand[2];
	CanDef[1][3] = RealRobot.RightHand[3];

	CanDef[2][4] = RealRobot.Head[0];
	CanDef[2][5] = RealRobot.Head[1];

	CanDef[3][4] = RealRobot.Waist[0];
	CanDef[3][5] = RealRobot.Waist[1];
}

double JointDetect(int Can_CH, int Can_ID, double angle_in)		// input: rad    output: rad
{
	double out = 100;
	// 发送数据位置限位
	if (angle_in>AngleMax_deg[Can_CH][Can_ID]*Degree2Rad)
	{
		out = AngleMax_deg[Can_CH][Can_ID]*Degree2Rad;
		JointError[Can_CH][Can_ID] = 1;
		return out;
	}
	else if(angle_in<AngleMin_deg[Can_CH][Can_ID]*Degree2Rad)
	{
		out = AngleMin_deg[Can_CH][Can_ID]*Degree2Rad;
		JointError[Can_CH][Can_ID] = -1;
		return out;
	}
	else
	{
		out = angle_in;
	}


	// 发送数据速度限制
	if ((angle_in - Joint_Angle_LastEP[Can_CH][Can_ID])/time_interval*Rad2Degree > VelocityLimit_deg[Can_CH][Can_ID])
	{
		printf("over speed ++++++++2  %f %f %f\n",angle_in,Joint_Angle_LastEP[Can_CH][Can_ID],(angle_in - Joint_Angle_LastEP[Can_CH][Can_ID])/time_interval*Rad2Degree);
		out=Joint_Angle_LastEP[Can_CH][Can_ID] + VelocityLimit_deg[Can_CH][Can_ID]*time_interval*Degree2Rad;
		JointError[Can_CH][Can_ID] = 2;
		return out;

	}
	else if ((angle_in - Joint_Angle_LastEP[Can_CH][Can_ID])/time_interval*Rad2Degree < -VelocityLimit_deg[Can_CH][Can_ID])
	{
		printf("over speed --------2  %f %f %f\n",angle_in,Joint_Angle_LastEP[Can_CH][Can_ID],(angle_in - Joint_Angle_LastEP[Can_CH][Can_ID])/time_interval*Rad2Degree);
		out=Joint_Angle_LastEP[Can_CH][Can_ID] - VelocityLimit_deg[Can_CH][Can_ID]*time_interval*Degree2Rad;
		JointError[Can_CH][Can_ID] = -2;
		return out;

	}
	else
	{
		out = angle_in;
		return out;
	}

}




//第一个输入参数为16为无符号整形u16_sig，各个位上为1代表相应按键有按下
//第二-四个参数为三个手指采回来的电流
//最后的参数为保存输出结果的数组1*4，数组前三个分别是三个手指的弯曲角度，最后一个为旋转角度
int control_handL(short u16_sig, double I_f1, double I_f2, double I_f3,double * output)
{
//解析信号
	int finger_sig = 0, theta_sig = 0;//用于解析手指弯曲、旋转角度的变化信号
	static int s_current_mag_sig_inc = 0;//该变量用于记录当前的档位调节信号，必须是static，因为得考虑上一次的值
	static int s_current_mag_sig_des = 0;//上一个是记录挡位增加的按钮，这个是记录挡位减小的按钮
	static int s_last_mag_sig_inc;
	static int s_last_mag_sig_des;//同理，一个记录增加按钮，一个记录减小按钮
	int mag_sig = 0;//该变量用于此次档位的变化
	static double sd_f_alpha[3] = { f_alpha_default,f_alpha_default,f_alpha_default };
	static double theta = theta_default;//四个角度变量


	I_f1 = fabs(I_f1);
	I_f2 = fabs(I_f2);
	I_f3 = fabs(I_f3);
	s_last_mag_sig_inc = s_current_mag_sig_inc;//此时变量s_current_mag_sig记录的是上一次的档位调节信号
	s_last_mag_sig_des = s_current_mag_sig_des;

	//检查各位，设定相应的信号值
	if ((u16_sig & 0x01) != 0) //手指张合与旋转
	{
		finger_sig = -1;
	}
	if ((u16_sig & 0x02) != 0) //if ((u16_sig || key_number<<1 )!=0)
	{
		theta_sig = 1;
	}
	if ((u16_sig & 0x04) != 0)
	{
		theta_sig = -1;
	}
	if ((u16_sig & 0x08) != 0)
	{
		finger_sig = 1;
	}

	//电流挡位的设置
	if ((u16_sig & 0x10) != 0)//挡位增大键
	{
		s_current_mag_sig_inc = 1;
	}
	else
	{
		s_current_mag_sig_inc = 0;
	}
	if ((u16_sig & 0x100) != 0)//挡位减小键
	{
		s_current_mag_sig_des = 1;
	}
	else
	{
		s_current_mag_sig_des = 0;
	}

	//电流档位调节的按键检测
	if (s_last_mag_sig_inc == 1 && s_current_mag_sig_inc == 0)//如果按键上一次处于按下状态，这一次处于弹起状态
	{
		mag_sig = 1;//此时电流档位才可以改变
	}
	if (s_last_mag_sig_des == 1 && s_current_mag_sig_des == 0)//如果按键上一次处于按下状态，这一次处于弹起状态
	{
		mag_sig = -1;//此时电流档位才可以改变
	}





//以下定义计算平均电流用的变量
	double d_I[3] = { 0 };//三个手指平均电流值
	static double sd_I_sink[3][num_sample] = { 0 };
	static int s_pointer = 0, s_period_flag = 0;
	static double sd_last_If1 = 0, sd_last_If2 = 0, sd_last_If3 = 0;//用于记录上一次的电流值


	int i, j, k;//循环用的数
	double td_Itemp_sum[3] = { 0 };

    //第一阶段：计算平均电流
	{
		if (I_f1 == 0)
		{
			I_f1 = sd_last_If1;
		}
		if (I_f2 == 0)
		{
			I_f2 = sd_last_If2;
		}
		if (I_f3 == 0)
		{
			I_f3 = sd_last_If3;
		}

		sd_I_sink[0][s_pointer] = I_f1;//填入输入的电流值
		sd_I_sink[1][s_pointer] = I_f2;
		sd_I_sink[2][s_pointer] = I_f3;

		sd_last_If1 = I_f1;//保存函数这次调用时的电流值
		sd_last_If2 = I_f2;
		sd_last_If3 = I_f3;

		for (j = 0; j < 3; j++)//求一段时间内的电流和
		{
			for (td_Itemp_sum[j] = 0, i = 0; i < num_sample; i++)
			{
				td_Itemp_sum[j] += sd_I_sink[j][i];
			}
		}

		//求平均得3个手指电流值
		if (s_period_flag == 1)//电流的采样数是否大于num_sample
		{
			for (k = 0; k < 3; k++)
			{
				d_I[k] = td_Itemp_sum[k] / num_sample;
			}
		}
		else
		{
			for (k = 0; k < 3; k++)
			{
				d_I[k] = td_Itemp_sum[k] / (s_pointer + 1);
			}
		}

		s_pointer++;
		if (s_pointer == num_sample)
		{
			s_pointer = 0;
			s_period_flag = 1;
		}
	}

	//第二阶段

	//先定义相关变量
	//static double sd_Ic[Ic_mark_max] = { Ic1,Ic2,Ic3,Ic4,Ic5 };
	//static int si_Ic_mark = 1;//默认一档
	static double sd_Ic = Ic_default;
	double d_I0 = I0_benchmark;
	static double sd_last_falpha[3];

	//上一次手指的期望角度
	for (i = 0; i < 3; i++)
	{
		sd_last_falpha[i] = sd_f_alpha[i];
	}

//处理档位变化信号，旋转角度变化信号
	sd_Ic += mag_sig * Ic_delta;//根据信号，档位增加、不变或减少
	if (sd_Ic >= Ic_max)
	{
		sd_Ic = Ic_max;
	}
	else if (sd_Ic <= Ic_mini)
	{
		sd_Ic = Ic_mini;
	}


//模式设置键
	static int cur_modelKeyInc = 0;
	static int last_modelKeyInc = 0;
	static int cur_modelKeyDes = 0;
	static int last_modelKeyDes = 0;
	int model_sig = 0;//模式变化的最终信号
	static int model = 1;
	static int thumb_Disable = 0;//拇指失效标志
	static int f23_Disable = 0;
	static int modelChange_Flag = 0;
	static double f_alpha_Set[3] = { f_alpha_default, f_alpha_default, f_alpha_default };//设定模式时角度的理想值
	static double theta_Set = theta_default;


	last_modelKeyInc = cur_modelKeyInc;//记录上个调用周期的按键信号
	last_modelKeyDes = cur_modelKeyDes;

	//信号解析
	if ((u16_sig & 0x20) != 0)//模式增
	{
		cur_modelKeyInc = 1;
	}
	else
	{
		cur_modelKeyInc = 0;
	}
	if ((u16_sig & 0x200) != 0)//模式减
	{
		cur_modelKeyDes = 1;
	}
	else
	{
		cur_modelKeyDes = 0;
	}

	if (last_modelKeyInc == 1 && cur_modelKeyInc == 0)//按键的升降沿检测
	{
		modelChange_Flag = 2;//该标志设为1，开始模式改变的流程
		model_sig = 1;
		model = model + model_sig;
		if (model == 0)				//model不能超出范围,限于1-4
		{
			model = 4;
		}
		else if (model == 5)
		{
			model = 1;
		}
		//模式设置生效
		switch (model)
		{
		case 1:
			theta_Set = theta_default;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 2:
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 3:							//该模式下仅食指动，拇指保持张开
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_min;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 1;
			f23_Disable = 0;
			break;
		case 4:							//该模式下仅拇指动，食指收拢
			theta_Set = theta_min;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 1;
			break;
		default:
			break;
		}
	}
	//模式减的按键升降沿检测
	if (last_modelKeyDes == 1 && cur_modelKeyDes == 0)
	{
		modelChange_Flag = 2;//该标志设为1，开始模式改变的流程
		model_sig = -1;
		model = model + model_sig;
		if (model == 0)				//model不能超出范围,限于1-4
		{
			model = 4;
		}
		else if (model == 5)
		{
			model = 1;
		}
		//模式设置生效
		switch (model)
		{
		case 1:
			theta_Set = theta_default;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 2:
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 3:							//该模式下仅食指动，拇指保持张开
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_min;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 1;
			f23_Disable = 0;
			break;
		case 4:							//该模式下仅拇指动，食指收拢
			theta_Set = theta_min;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 1;
			break;
		default:
			break;
		}
	}

	if (modelChange_Flag>0)	//开始模式改变，该状态下，此if语句一下的其他语句不执行
	{
		if (modelChange_Flag==2)//先把手指张到最大位置
		{
			for (i = 0; i < 3; i++)
			{
				if (fabs(f_alpha_default - sd_f_alpha[i]) > delta_falpha)//实际角度值与设定值有误差的话就要减小误差，直至一致
				{
					sd_f_alpha[i] += (f_alpha_default - sd_f_alpha[i]) / fabs(f_alpha_default - sd_f_alpha[i])*delta_falpha;
				}
				else
				{
					sd_f_alpha[i] = f_alpha_default;
				}
			}

			if ((fabs(f_alpha_default - sd_f_alpha[0]) <= 0.000001) && (fabs(f_alpha_default - sd_f_alpha[1]) <= 0.000001) && (fabs(f_alpha_default - sd_f_alpha[2]) <= 0.000001))
			{
				modelChange_Flag = 1;
			}
		}
		else //然后，先旋转，再弯曲，这样就不会干涉了
		{
			if (fabs(theta_Set - theta) > delta_theta)//实际旋转角度值与设定值有误差的话就要减小误差，直至一致
			{
				theta += (theta_Set - theta) / fabs(theta_Set - theta)*delta_theta;
			}
			else
			{
				theta = theta_Set;
			}

			if ((fabs(theta_Set - theta) <= 0.000001))
			{
				if (fabs(f_alpha_Set[i] - sd_f_alpha[i]) > delta_falpha)//实际角度值与设定值有误差的话就要减小误差，直至一致
				{
					sd_f_alpha[i] += (f_alpha_Set[i] - sd_f_alpha[i]) / fabs(f_alpha_Set[i] - sd_f_alpha[i])*delta_falpha;
				}
				else
				{
					sd_f_alpha[i] = f_alpha_Set[i];
				}
			}

			//如果角度的实际值与设定值一致，则modelChange_flag置0
			if ((fabs(f_alpha_Set[0] - sd_f_alpha[0]) <= 0.000001) && (fabs(f_alpha_Set[1] - sd_f_alpha[1]) <= 0.000001) && (fabs(f_alpha_Set[2] - sd_f_alpha[2]) <= 0.000001) && (fabs(theta_Set - theta) <= 0.000001))
			{
				modelChange_Flag = 0;
			}
		}

		output[1] = sd_f_alpha[1];
		output[2] = sd_f_alpha[2];
		output[3] = sd_f_alpha[0];
		output[0] = theta;

		return 0;
	}



//处理手指的旋转
	if (theta + delta_theta * theta_sig > theta_max)//将要超过限制
	{
		theta = theta_max;
	}
	else if (theta + delta_theta * theta_sig < theta_min)//将要低于限制
	{
		theta = theta_min;
	}
	else
	{
		theta += delta_theta * theta_sig;//正常变化
	}

	//暂时旋转角度保持为默认值
	//theta = theta_default;


//由于手指旋转角度的问题，需要对单个手指的实际电流档位进行修正
	//默认以拇指为基准
	double Ic_amend[3];
	double cof = cos(theta * pi / 180) * 2;
	Ic_amend[0] = sd_Ic;
	Ic_amend[1] = sd_Ic / cof;
	Ic_amend[2] = sd_Ic / cof;//食指的电流得到修正
											  //if (Ic_amend[0]<Ic[0])//档位过小的话,直接设为最小档位.	暂时取消此句，还得考虑到拇指参不参与抓取
											  //{
											  //	Ic_amend[0] = Ic[0];
											  //}
	switch (model)
	{
	case 1:
		if (Ic_amend[1]>=2.5)
		{
			Ic_amend[1] = 2.5;
		}
		break;
	case 2:
		Ic_amend[1] = sd_Ic;
		Ic_amend[2] = sd_Ic;
		break;
	case 3:
		Ic_amend[1] = sd_Ic;
		Ic_amend[2] = sd_Ic;
		break;
	case  4:
		//无需改变
		break;
	default:
		break;
	}


//处理手指的张合
	if (finger_sig == -1)//张开手指的处理
	{
		for (i = 0; i < 3; i++)//三个手指单独考虑
		{
			if ((i==0)&&thumb_Disable==1)	//拇指失效
			{
				continue;
			}
			if ((i==1||i==2)&&f23_Disable==1)//食指失效
			{
				continue;
			}

			sd_f_alpha[i] += finger_sig * delta_falpha;
			if (sd_f_alpha[i] < f_alpha_min)
			{
				sd_f_alpha[i] = f_alpha_min;
			}
		}
	}
	else
	{
		for (i = 0; i < 3; i++)//三个手指单独考虑
		{
			if ((i == 0) && thumb_Disable == 1)	//拇指失效
			{
				continue;
			}
			if ((i == 1 || i == 2) && f23_Disable == 1)//食指失效
			{
				continue;
			}

			if (d_I[i] <= d_I0)//小于I0表示没碰到物体
			{
				sd_f_alpha[i] += finger_sig * delta_falpha;//继续闭合手指
				if (sd_f_alpha[i] > f_alpha_max)
				{
					sd_f_alpha[i] = f_alpha_max;
				}
			}
			else if (d_I[i] < Ic_amend[i])//小于Ic，接触到物体，但是未到设定的最大值
			{
				sd_f_alpha[i] += finger_sig * minidelta_falpha;//继续闭合手指,但是现在要缓慢
				if (sd_f_alpha[i] > f_alpha_max)
				{
					sd_f_alpha[i] = f_alpha_max;
				}
			}
			else 		//达到Ic后
			{
				sd_f_alpha[i] = sd_last_falpha[i];				//此段以后可根据需要更改为阻抗控制等
			}
		}
	}


	//输出参数，第一个是旋转角度，后三个分别是三个手指的弯曲角度
	output[1] = sd_f_alpha[1];
	output[2] = sd_f_alpha[2];
	output[3] = sd_f_alpha[0];
	output[0] = theta;

	return 0;
}

//第一个输入参数为16为无符号整形u16_sig，各个位上为1代表相应按键有按下
//第二-四个参数为三个手指采回来的电流
//最后的参数为保存输出结果的数组1*4，数组前三个分别是三个手指的弯曲角度，最后一个为旋转角度
int control_handR(short u16_sig, double I_f1, double I_f2, double I_f3,double * output)
{
//解析信号
	int finger_sig = 0, theta_sig = 0;//用于解析手指弯曲、旋转角度的变化信号
	static int s_current_mag_sig_inc = 0;//该变量用于记录当前的档位调节信号，必须是static，因为得考虑上一次的值
	static int s_current_mag_sig_des = 0;//上一个是记录挡位增加的按钮，这个是记录挡位减小的按钮
	static int s_last_mag_sig_inc;
	static int s_last_mag_sig_des;//同理，一个记录增加按钮，一个记录减小按钮
	int mag_sig = 0;//该变量用于此次档位的变化
	static double sd_f_alpha[3] = { f_alpha_default,f_alpha_default,f_alpha_default };
	static double theta = theta_default;//四个角度变量


	I_f1 = fabs(I_f1);
	I_f2 = fabs(I_f2);
	I_f3 = fabs(I_f3);
	s_last_mag_sig_inc = s_current_mag_sig_inc;//此时变量s_current_mag_sig记录的是上一次的档位调节信号
	s_last_mag_sig_des = s_current_mag_sig_des;

	//检查各位，设定相应的信号值
	if ((u16_sig & 0x01) != 0) //手指张合与旋转
	{
		finger_sig = -1;
	}
	if ((u16_sig & 0x02) != 0) //if ((u16_sig || key_number<<1 )!=0)
	{
		theta_sig = 1;
	}
	if ((u16_sig & 0x04) != 0)
	{
		theta_sig = -1;
	}
	if ((u16_sig & 0x08) != 0)
	{
		finger_sig = 1;
	}

	//电流挡位的设置
	if ((u16_sig & 0x10) != 0)//挡位增大键
	{
		s_current_mag_sig_inc = 1;
	}
	else
	{
		s_current_mag_sig_inc = 0;
	}
	if ((u16_sig & 0x100) != 0)//挡位减小键
	{
		s_current_mag_sig_des = 1;
	}
	else
	{
		s_current_mag_sig_des = 0;
	}

	//电流档位调节的按键检测
	if (s_last_mag_sig_inc == 1 && s_current_mag_sig_inc == 0)//如果按键上一次处于按下状态，这一次处于弹起状态
	{
		mag_sig = 1;//此时电流档位才可以改变
	}
	if (s_last_mag_sig_des == 1 && s_current_mag_sig_des == 0)//如果按键上一次处于按下状态，这一次处于弹起状态
	{
		mag_sig = -1;//此时电流档位才可以改变
	}





//以下定义计算平均电流用的变量
	double d_I[3] = { 0 };//三个手指平均电流值
	static double sd_I_sink[3][num_sample] = { 0 };
	static int s_pointer = 0, s_period_flag = 0;
	static double sd_last_If1 = 0, sd_last_If2 = 0, sd_last_If3 = 0;//用于记录上一次的电流值


	int i, j, k;//循环用的数
	double td_Itemp_sum[3] = { 0 };

    //第一阶段：计算平均电流
	{
		if (I_f1 == 0)
		{
			I_f1 = sd_last_If1;
		}
		if (I_f2 == 0)
		{
			I_f2 = sd_last_If2;
		}
		if (I_f3 == 0)
		{
			I_f3 = sd_last_If3;
		}

		sd_I_sink[0][s_pointer] = I_f1;//填入输入的电流值
		sd_I_sink[1][s_pointer] = I_f2;
		sd_I_sink[2][s_pointer] = I_f3;

		sd_last_If1 = I_f1;//保存函数这次调用时的电流值
		sd_last_If2 = I_f2;
		sd_last_If3 = I_f3;

		for (j = 0; j < 3; j++)//求一段时间内的电流和
		{
			for (td_Itemp_sum[j] = 0, i = 0; i < num_sample; i++)
			{
				td_Itemp_sum[j] += sd_I_sink[j][i];
			}
		}

		//求平均得3个手指电流值
		if (s_period_flag == 1)//电流的采样数是否大于num_sample
		{
			for (k = 0; k < 3; k++)
			{
				d_I[k] = td_Itemp_sum[k] / num_sample;
			}
		}
		else
		{
			for (k = 0; k < 3; k++)
			{
				d_I[k] = td_Itemp_sum[k] / (s_pointer + 1);
			}
		}

		s_pointer++;
		if (s_pointer == num_sample)
		{
			s_pointer = 0;
			s_period_flag = 1;
		}
	}

	//第二阶段

	//先定义相关变量
	//static double sd_Ic[Ic_mark_max] = { Ic1,Ic2,Ic3,Ic4,Ic5 };
	//static int si_Ic_mark = 1;//默认一档
	static double sd_Ic = Ic_default;
	double d_I0 = I0_benchmark;
	static double sd_last_falpha[3];

	//上一次手指的期望角度
	for (i = 0; i < 3; i++)
	{
		sd_last_falpha[i] = sd_f_alpha[i];
	}

//处理档位变化信号，旋转角度变化信号
	sd_Ic += mag_sig * Ic_delta;//根据信号，档位增加、不变或减少
	if (sd_Ic >= Ic_max)
	{
		sd_Ic = Ic_max;
	}
	else if (sd_Ic <= Ic_mini)
	{
		sd_Ic = Ic_mini;
	}


//模式设置键
	static int cur_modelKeyInc = 0;
	static int last_modelKeyInc = 0;
	static int cur_modelKeyDes = 0;
	static int last_modelKeyDes = 0;
	int model_sig = 0;//模式变化的最终信号
	static int model = 1;
	static int thumb_Disable = 0;//拇指失效标志
	static int f23_Disable = 0;
	static int modelChange_Flag = 0;
	static double f_alpha_Set[3] = { f_alpha_default, f_alpha_default, f_alpha_default };//设定模式时角度的理想值
	static double theta_Set = theta_default;


	last_modelKeyInc = cur_modelKeyInc;//记录上个调用周期的按键信号
	last_modelKeyDes = cur_modelKeyDes;

	//信号解析
	if ((u16_sig & 0x20) != 0)//模式增
	{
		cur_modelKeyInc = 1;
	}
	else
	{
		cur_modelKeyInc = 0;
	}
	if ((u16_sig & 0x200) != 0)//模式减
	{
		cur_modelKeyDes = 1;
	}
	else
	{
		cur_modelKeyDes = 0;
	}

	if (last_modelKeyInc == 1 && cur_modelKeyInc == 0)//按键的升降沿检测
	{
		modelChange_Flag = 2;//该标志设为1，开始模式改变的流程
		model_sig = 1;
		model = model + model_sig;
		if (model == 0)				//model不能超出范围,限于1-4
		{
			model = 4;
		}
		else if (model == 5)
		{
			model = 1;
		}
		//模式设置生效
		switch (model)
		{
		case 1:
			theta_Set = theta_default;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 2:
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 3:							//该模式下仅食指动，拇指保持张开
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_min;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 1;
			f23_Disable = 0;
			break;
		case 4:							//该模式下仅拇指动，食指收拢
			theta_Set = theta_min;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 1;
			break;
		default:
			break;
		}
	}
	//模式减的按键升降沿检测
	if (last_modelKeyDes == 1 && cur_modelKeyDes == 0)
	{
		modelChange_Flag = 2;//该标志设为1，开始模式改变的流程
		model_sig = -1;
		model = model + model_sig;
		if (model == 0)				//model不能超出范围,限于1-4
		{
			model = 4;
		}
		else if (model == 5)
		{
			model = 1;
		}
		//模式设置生效
		switch (model)
		{
		case 1:
			theta_Set = theta_default;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 2:
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 0;
			break;
		case 3:							//该模式下仅食指动，拇指保持张开
			theta_Set = 180;
			f_alpha_Set[0] = f_alpha_min;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 1;
			f23_Disable = 0;
			break;
		case 4:							//该模式下仅拇指动，食指收拢
			theta_Set = theta_min;
			f_alpha_Set[0] = f_alpha_default;
			f_alpha_Set[1] = f_alpha_default;
			f_alpha_Set[2] = f_alpha_default;
			thumb_Disable = 0;
			f23_Disable = 1;
			break;
		default:
			break;
		}
	}

	if (modelChange_Flag>0)	//开始模式改变，该状态下，此if语句一下的其他语句不执行
	{
		if (modelChange_Flag==2)//先把手指张到最大位置
		{
			for (i = 0; i < 3; i++)
			{
				if (fabs(f_alpha_default - sd_f_alpha[i]) > delta_falpha)//实际角度值与设定值有误差的话就要减小误差，直至一致
				{
					sd_f_alpha[i] += (f_alpha_default - sd_f_alpha[i]) / fabs(f_alpha_default - sd_f_alpha[i])*delta_falpha;
				}
				else
				{
					sd_f_alpha[i] = f_alpha_default;
				}
			}

			if ((fabs(f_alpha_default - sd_f_alpha[0]) <= 0.000001) && (fabs(f_alpha_default - sd_f_alpha[1]) <= 0.000001) && (fabs(f_alpha_default - sd_f_alpha[2]) <= 0.000001))
			{
				modelChange_Flag = 1;
			}
		}
		else //然后，先旋转，再弯曲，这样就不会干涉了
		{
			if (fabs(theta_Set - theta) > delta_theta)//实际旋转角度值与设定值有误差的话就要减小误差，直至一致
			{
				theta += (theta_Set - theta) / fabs(theta_Set - theta)*delta_theta;
			}
			else
			{
				theta = theta_Set;
			}

			if ((fabs(theta_Set - theta) <= 0.000001))
			{
				if (fabs(f_alpha_Set[i] - sd_f_alpha[i]) > delta_falpha)//实际角度值与设定值有误差的话就要减小误差，直至一致
				{
					sd_f_alpha[i] += (f_alpha_Set[i] - sd_f_alpha[i]) / fabs(f_alpha_Set[i] - sd_f_alpha[i])*delta_falpha;
				}
				else
				{
					sd_f_alpha[i] = f_alpha_Set[i];
				}
			}

			//如果角度的实际值与设定值一致，则modelChange_flag置0
			if ((fabs(f_alpha_Set[0] - sd_f_alpha[0]) <= 0.000001) && (fabs(f_alpha_Set[1] - sd_f_alpha[1]) <= 0.000001) && (fabs(f_alpha_Set[2] - sd_f_alpha[2]) <= 0.000001) && (fabs(theta_Set - theta) <= 0.000001))
			{
				modelChange_Flag = 0;
			}
		}

		output[1] = sd_f_alpha[1];
		output[2] = sd_f_alpha[2];
		output[3] = sd_f_alpha[0];
		output[0] = theta;

		return 0;
	}



//处理手指的旋转
	if (theta + delta_theta * theta_sig > theta_max)//将要超过限制
	{
		theta = theta_max;
	}
	else if (theta + delta_theta * theta_sig < theta_min)//将要低于限制
	{
		theta = theta_min;
	}
	else
	{
		theta += delta_theta * theta_sig;//正常变化
	}

	//暂时旋转角度保持为默认值
	//theta = theta_default;


//由于手指旋转角度的问题，需要对单个手指的实际电流档位进行修正
	//默认以拇指为基准
	double Ic_amend[3];
	double cof = cos(theta * pi / 180) * 2;
	Ic_amend[0] = sd_Ic;
	Ic_amend[1] = sd_Ic / cof;
	Ic_amend[2] = sd_Ic / cof;//食指的电流得到修正
											  //if (Ic_amend[0]<Ic[0])//档位过小的话,直接设为最小档位.	暂时取消此句，还得考虑到拇指参不参与抓取
											  //{
											  //	Ic_amend[0] = Ic[0];
											  //}
	switch (model)
	{
	case 1:
		if (Ic_amend[1]>=2.5)
		{
			Ic_amend[1] = 2.5;
		}
		break;
	case 2:
		Ic_amend[1] = sd_Ic;
		Ic_amend[2] = sd_Ic;
		break;
	case 3:
		Ic_amend[1] = sd_Ic;
		Ic_amend[2] = sd_Ic;
		break;
	case  4:
		//无需改变
		break;
	default:
		break;
	}


//处理手指的张合
	if (finger_sig == -1)//张开手指的处理
	{
		for (i = 0; i < 3; i++)//三个手指单独考虑
		{
			if ((i==0)&&thumb_Disable==1)	//拇指失效
			{
				continue;
			}
			if ((i==1||i==2)&&f23_Disable==1)//食指失效
			{
				continue;
			}

			sd_f_alpha[i] += finger_sig * delta_falpha;
			if (sd_f_alpha[i] < f_alpha_min)
			{
				sd_f_alpha[i] = f_alpha_min;
			}
		}
	}
	else
	{
		for (i = 0; i < 3; i++)//三个手指单独考虑
		{
			if ((i == 0) && thumb_Disable == 1)	//拇指失效
			{
				continue;
			}
			if ((i == 1 || i == 2) && f23_Disable == 1)//食指失效
			{
				continue;
			}

			if (d_I[i] <= d_I0)//小于I0表示没碰到物体
			{
				sd_f_alpha[i] += finger_sig * delta_falpha;//继续闭合手指
				if (sd_f_alpha[i] > f_alpha_max)
				{
					sd_f_alpha[i] = f_alpha_max;
				}
			}
			else if (d_I[i] < Ic_amend[i])//小于Ic，接触到物体，但是未到设定的最大值
			{
				sd_f_alpha[i] += finger_sig * minidelta_falpha;//继续闭合手指,但是现在要缓慢
				if (sd_f_alpha[i] > f_alpha_max)
				{
					sd_f_alpha[i] = f_alpha_max;
				}
			}
			else 		//达到Ic后
			{
				sd_f_alpha[i] = sd_last_falpha[i];				//此段以后可根据需要更改为阻抗控制等
			}
		}
	}


	//输出参数，第一个是旋转角度，后三个分别是三个手指的弯曲角度
	output[1] = sd_f_alpha[1];
	output[2] = sd_f_alpha[2];
	output[3] = sd_f_alpha[0];
	output[0] = theta;

	return 0;
}
