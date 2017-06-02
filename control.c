
 /*
 * Program to Control Robot
 *
 * Copyright (C) 2017 Beijing Institute of Technology
 *
 * Copyright (C) 2017 Institute of Intelligent Robot 
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
#include <errno.h>
#include <getopt.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rtdm/rtcan.h>
#include "trajectory.h"


//函数声明
void send_event();
void timespec_add_us(struct timespec *t, long us);
int timespec_cmp(struct timespec *a, struct timespec *b);
int can_send(int channel, long can_id, int can_mode, long can_content[8], int can_lenth);
void rt_can_recv(void *arg);
void driver_init_status(void);
int elmo_init(void);
void servo_off(void);
void servo_on(void);
void current_position(int channel_num, double Joint_Angle_FB[4]);
void find_home(int channel_num, int id_num);
void rad_send(int i, int j, double angle_in);


//varable declear

RT_TASK demo_task_rvcan;
RT_TASK rt_task_desc;
RT_TASK rt_task_view;
RTIME now, previous, period;

GC MyGC;

extern int optind, opterr, optopt;
static int txsock[4], rxsock[4];

RT_TASK rt_task_desc;
static int s=-1, dlc=0, rtr=0, extended=0, verbose=1, loops=1;
static SRTIME delay=1000000;
static int count=0, print=1, use_send=0, loopback=-1;
static nanosecs_rel_t timeout_rv = 100;
static nanosecs_rel_t timeout_send = 1000;
static struct can_frame frame;
static struct sockaddr_can to_addr;
//socklen_t addrlen = sizeof(addr);
int can_fd = -1;

#define BUF_SIZ 255
#define MAX_FILTER 16
struct sockaddr_can recv_addr;
struct can_filter recv_filter[MAX_FILTER];
static int filter_count = 0;
struct  ifreq ifr[4];

double acangle[30];

Display *MyDisplay;
Window MyWindow;
Atom MyAtom;
char buf[200] = {0};
char buf1[200] = {0};
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

int can_node_number[4] = {4,4,4,3};
int can_channel_number = 4;
int can_switch[4][4] = {{1, 1, 1, 1},
					{1, 1, 1, 1},
					{1, 1, 1, 1},
					{1, 1, 1, 1}};
					
double joint_direction[4][4] = {{-1, 1, -1, 1},
					{-1, 1, -1, 1},
					{-1, 1, -1, 1},
					{1, 1, 1, 1}};
double zero_comp[4][4] = {0.0};
int can_work_states[15] = {0};

double T_END[4][4] = {{1.0, 0.0, 0.0, 600.0},
						{0.0, 1.0, 0.0, 100.0},
						{0.0, 0.0, 1.0, -50.0},
						{0.0, 0.0, 0.0, 1.0}};
int print_flag = 1;

long data1[8];
Atom wm_delete_window;
Atom wm_protocols;

struct timespec t1; 
double time_start = 0;
double time_end;
double time_diff = 0.0;
int jjj = 0;
double max = 0;
double average =0.0;
double numb = 1.0;
int s1;
int can1=0;
int can2=1;
int can3=2;
int can4=3;
long handvel=0;
sem_t low_go;

int offset_home = 3;


int SendFlag = 1;

int TestStep = 0;
int BallStep = 0;
int ButtonStep = 0;
int HandleStep = 0;
int HandStep = 0;
int InverseHandStep = 0;
int Prg1Step = 0;
int Prg2Step = 0;
int Prg3Step = 0;
int Prg4Step = 0;

int PlanningPath = 0;
double JointAngle_new[7] = {0.0, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0};
double JointAngle_last[7];
double JointAngle_Start[7];	
double JointPos_offset[7] = {51.426, 127.293, 21.124, 162.42, 231.78, 90.0, 0.0};

// 发送关节角限位
double JointMax[7] = {128.574,85.507,158.876,96.38,89.61,0,13};
double JointMin[7] ={-231.426,-82.093,-201.124,-101.92,-270.39,-120,-13};
int JointError[7];
double JointSendtoDsp[7];	// 发送给DSP角度值
double JointReal[7];		// 实际关节角 运动坐标系下表示
double EndReal[6];			// 实际末端位姿
double EndPos_last[3];
double EndPos_Start[3];	  
int Encoder_Count_FB[4][4] = {0};
int Encoder_Count_EP[4][4] = {0};
int Encoder_Count_EP_Send[4][4] = {0};
double Joint_Angle_FB[4][4] = {0.0};
double Joint_Angle_EP[4][4] = {0.0};
double Joint_Angle_FB_degree[4][4] = {0.0};
double Joint_Angle_EP_degree[4][4] = {0.0};
double Joint_Angle_ER_degree[4][4] = {0.0};

double home_offset[4][4] = {{0.1,0.1,0.1,0.1},{0.1,0.1,0.1,0.1},{0.1,0.1,0.1,0.1},{0.0,0.0,0.0,0.0}};
//{{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};
double home_time = 5.0;

double Joint_Current_Angle[4][4] = {0.0};


//double pi = 3.1415926535898;
int servo_on_flag = 0;
int begin_flag =0;
int motion_mode = 100;
double line_2count = 500.0;


/**********************  UDP通讯 Start ***********************/
#pragma pack(push)
#pragma pack(1)
struct DataUDP_Struct	  
{
	unsigned int TrustFlag;	
	float Arm1Angle[5];
	float Arm2Angle[5];
	float Arm3Angle[5];
	unsigned int CheckSum;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
struct CommandUDP_Struct
{
	unsigned char Mode;
	unsigned char Arm1;
	unsigned char Arm2;
	unsigned char Arm3;
	float Data;
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

int UDP_Sock;	
struct sockaddr_in DestAddr;
socklen_t nAddrLen;
int UDPComm_init(void);

unsigned char sendbuff[100];
unsigned char recvbuff[100];

int UDPTimes = 0;
/**********************  UDP通讯 End ***********************/
// 运动关节定义
float JointMoveData = 0.0;


double high_time = 0.0;

long time_inteval = 1000;
struct sched_param the_priority_main, the_priority5;

pthread_t th1, th2;
int prio;
int high_stop_flag = 0;
struct timespec start, end; 
int unm100 = 0;
int canrv_running = 1;

//int ordermode;
int view_time = 1;
double Rad2Count = 2607.5945876176;//651.898;
double Rad2Degree = 57.29577951308;
double degree2rad = 0.0174532925;
double reduction_ratio = 160.0;//4.8;

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

	int j, i = 0;
	int DispHead = 40;
	
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
			//	case 'T':
         				printf("servo on\n");
         				servo_on();	
					servo_on_flag = 1;					
				break;
				
				case '3':
					printf("home offset\n");
         				motion_mode = 21;					
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
				
				case 'b':	// 抓小球
				case 'B':
				
				//	begin_flag = 1;
					motion_mode = 20;
					
				break;
				
				case '2':	// 按按钮
				//case 'C':
					for(i=0; i<4; i++)
					{
						current_position(i, Joint_Angle_FB[i]);
					}
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

				
				sprintf(buf, "Loop time : %ldus", (long)period);
				if(servo_on_flag == 1)
					sprintf(buf1, "servo on");
				else
					sprintf(buf1, "servo off");

					
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead, buf, strlen(buf));
				XDrawString(MyDisplay, MyWindow, MyGC, 300, DispHead, buf1, strlen(buf1));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+30, buf2, strlen(buf2));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+60, buf3, strlen(buf3));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+90, buf4, strlen(buf4));
				
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+120, buf5, strlen(buf5));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+150, buf10, strlen(buf10));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+180, buf6, strlen(buf6));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+210, buf7, strlen(buf7));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+240, buf11, strlen(buf11));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+270, buf8, strlen(buf8));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+300, buf9, strlen(buf9));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+330, buf12, strlen(buf12));
		//		XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+300, buf11, strlen(buf11));
		//		XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+330, buf12, strlen(buf12));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+360, buf13, strlen(buf13));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+390, buf14, strlen(buf14));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+420, buf15, strlen(buf15));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+450, buf16, strlen(buf16));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+480, buf17, strlen(buf17));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+510, buf18, strlen(buf18));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+540, buf19, strlen(buf19));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+570, buf20, strlen(buf20));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+600, buf22, strlen(buf22));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+630, buf23, strlen(buf23));
			}
		}
	}
	

	XCloseDisplay(MyDisplay);
	printf("close windows\n");
//	timer_delete(timerid);
//	high_stop_flag = 1;
}


int can_send(int channel, long can_id, int can_mode, long can_content[8], int can_lenth)
//can_mode = 0 normal can   can_mode =1 rtr can frame  can_mode = 2 extend can frame
//channel = 0 can0  channel = 1 can1  channel = 2 can2  channel = 3 can3 
{

    int i, j, ret, opt;
    int can_id_new = 0;
    int can_send_flag = 0;
   
    can_id_new = (can_id & 0x00F) - 1;
    if(can_id_new < 0)
    		can_send_flag = 1;
    else if( can_id_new < 4)
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
	
	//	ret = rt_dev_send(txsock, (void *)&frame, sizeof(can_frame_t), 0);
		ret = rt_dev_sendto(txsock[channel], (void *)&frame, sizeof(can_frame_t), 0,
		                   (struct sockaddr *)&to_addr, sizeof(to_addr));
	 //   printf("txsock[%d]=%d\n",channel,txsock[channel]);
	//	printf("ret = %d\n", ret);
		if (ret < 0) 
		{
			switch (ret) {
			case -ETIMEDOUT:
				if (verbose)
					printf("rt_dev_send(to): timed out\n");
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
	int can_node_num =4;
	int i, j;
	int canRecvStatus[4];
	
	sleeptime.tv_nsec = 500000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL); 
	
//	printf("angle_in = %f\n", angle_in*57.2);
	
	Encoder_Count_EP2 = angle_in * Rad2Count * reduction_ratio;
	Encoder_Count_EP_Send2 = Encoder_Count_EP2 * joint_direction[can_channel_num][id];						

	count_out[0] = 0X1F;
	count_out[1] = 0X00;
	count_out[2] = Encoder_Count_EP_Send2 & 0x000000FF;
	count_out[3] = (Encoder_Count_EP_Send2 & 0x0000FF00)>>8;
	count_out[4] = (Encoder_Count_EP_Send2 & 0x00FF0000)>>16;
	count_out[5] = (Encoder_Count_EP_Send2 & 0xFF000000)>>24;

	can_id = 0x200 + id +1;						
	can_send(can_channel_num, can_id, 0, count_out, 6);
//	can_send(can_channel_num, 0x202, 0, count_out, 6);
//	printf("cansend can_channel_num=%d, can_id=%x\n", can_channel_num, can_id);
	
	sleeptime.tv_nsec = 500000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);
	can_id = 0x80;	
	can_send(can_channel_num, can_id, 0, count_out, 0);   //SYNC
	
	sleeptime.tv_nsec = 1500000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);
	
	if(can_channel_num == 3)
		can_node_num = 3;
		
	for(i=0; i<can_node_num; i++)
	{
		ret = rt_dev_recvfrom(rxsock[can_channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
		if (ret < 0) 
		{
			switch (ret) 
			{
				case -ETIMEDOUT:
					if (verbose)
						printf("rt_dev_recv: timed out\n");
						canRecvStatus[can_channel_num] = -1;
					break;
				case -EBADF:
					if (verbose)
						printf("rt_dev_recv: aborted because socket was closed");
						canRecvStatus[can_channel_num] = -2;
					break;
				default:
					fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
					canRecvStatus[can_channel_num] = -3;
					break;
			}
		}
		else
		{
			if(i < 3)
			{
				Encoder_Count_FB[can_channel_num][(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB[can_channel_num][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[can_channel_num][(frame.can_id & 0xf)-1])/Rad2Count/reduction_ratio - zero_comp[can_channel_num][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1] ;
				Joint_Angle_FB_degree[can_channel_num][(frame.can_id & 0xf)-1] = Joint_Angle_FB[can_channel_num][(frame.can_id & 0xf)-1]*Rad2Degree;
			}
			else
			{
				Encoder_Count_FB[can_channel_num][(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB[can_channel_num][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[can_channel_num][(frame.can_id & 0xf)-1])/line_2count - zero_comp[can_channel_num][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1];
				Joint_Angle_FB_degree[can_channel_num][(frame.can_id & 0xf)-1] = Joint_Angle_FB[can_channel_num][(frame.can_id & 0xf)-1];
			}	
		}
		
	}				
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
	
	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us
	can_send(channel_num, can_id, 0, can_initiates_OS, 8);   //SYNC
	
	sleeptime.tv_nsec = 600000;
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
	
	sleeptime.tv_nsec = 600000;
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
	
	sleeptime.tv_nsec = 600000;
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

void current_position(int channel_num, double Joint_Angle_FB2[4])
{
	struct timespec sleeptime;
	long can_id = 0x80;
	long can_content[8] = {0x23, 0x24, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
//	long can_Segmented_SDO[8] = {0x21, 0x23, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00};
//	long can_XQ[6] = {0x01, 0x07, 0x58, 0x51, 0x23, 0x23};
	int i;
	int can_node_num = 4;
	int canRecvStatus[4];
	
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;
	double Encoder_Count_FB[4]; 
	
	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us
	
	for(i=0; i<12; i++)
	{
		ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}
	
//	can_id = 0x80;		
	can_send(channel_num, can_id, 0, can_content, 0);   //SYNC
	
	sleeptime.tv_nsec = 1000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 400us
	
	if(channel_num == 3)
		can_node_num =3;
		
	for(i=0; i<can_node_num; i++)
	{
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
			{
				Encoder_Count_FB[(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB2[(frame.can_id & 0xf)-1] =(((double)Encoder_Count_FB[(frame.can_id & 0xf)-1])/Rad2Count/reduction_ratio - zero_comp[channel_num][(frame.can_id & 0xf)-1]) * joint_direction[channel_num][(frame.can_id & 0xf)-1] ;
		//		Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1]*Rad2Degree/reduction_ratio;
				Joint_Angle_FB_degree[channel_num][(frame.can_id & 0xf)-1] = Joint_Angle_FB2[(frame.can_id & 0xf)-1]*Rad2Degree;	
			}	
			
		}
		
	}
//	printf("ret = %d\n", ret);
//	printf("current position %02x  %02x	 %02x  %02x  %02x  %02x  %02x  %02x\n", frame.data[0], frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
//	printf("current position is %f\n", Joint_Angle_FB2[0]*57.2);
//	return Joint_Angle_FB[i][id_num];
}


void rt_can_recv(void *arg)
{
	// trajectory planning var
	int index;
	double VDataTemp[3];

	int GetDataTimes = 0;
	int PlanningStep = 0;

	int i,j, ret, count = 0;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	struct msghdr msg;
	struct iovec iov;
	long int acjiaodu[30];
	struct timespec sleeptime;
	long can_id = 1;

	nanosecs_abs_t timestamp, timestamp_prev = 0;
	long can_content[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	long Encoder_Count_EP_BYTE[15][6] = {0};
	int canRecvStatus[4];

	double a = 0.0;
	double t = 0.0;
	double time_interval = 0.003;
	
	int can_connection_status[15] = {0};
	int k = 0;
	int can_id_i=0;
	int can_id_j =0;
	int can_node_number_max = 0;
	int fisttime_real = 0;
	double half_time = 10.0;
	double start_position[4][4] = {0.0};
//	double increased_position[4][4] = {{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};
	
	double end_position[4][4] = {{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};
	double offset_time = 5.0;
	double end_position_hand = 2.0;
//	double hand_position = 11.0;
	
	static int first_time_flag = 0;
	static int first_time_flag_mode21 = 0;
	static int first_time_flag_mode20 = 0;
//	double T_END[4][4] = {{1.0, 0.0, 0.0, 600.0},
//						{0.0, 1.0, 0.0, 0.0},
//						{0.0, 0.0, 1.0, -50.0},
//						{0.0, 0.0, 0.0, 1.0}};
	double anglein[8] = {0.0};
	double angle_out[4][8];
	double beta;
	
	int motion_mode_control = 1;   //when motion_mode_control == 0; motion_mode can be changed
	double timecost = 0;
	int current_position_flag = 0;
	int offset_cycle_num = 0;
	
	
	
	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;
	
	

	elmo_init();
	for(i=0;  i<can_channel_number; i++)
	{
		if(can_node_number[i] > can_node_number_max)
			can_node_number_max = can_node_number[i];
	}
	
//	can_node_number_max =1;
	
	rt_task_set_periodic(NULL, TM_NOW, 3000000);
/**********************************************************************************/
	//    开始循环
/**********************************************************************************/
	RTIME timetest1, timetest2;
	
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
		
		if(servo_on_flag == 1)
		{	
/************************* 四通道CAN数据接收**************************/

			switch (motion_mode)   //switch one joint, or all joint move
			{
				case 1:
					if(offset_home == 0)
					{
						find_home(0,0);	
						offset_home = 3;					
					}
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(0, Joint_Angle_FB[0]);						
								start_position[0][0] = Joint_Angle_FB[0][0];
				
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{																
									Joint_Angle_EP[0][0] = Five_Interpolation(start_position[0][0],0,0,start_position[0][0]+JointMoveData,0,0,offset_time,t);			
									rad_send(0,0,Joint_Angle_EP[0][0]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[0][0] = start_position[0][0]+JointMoveData;
									}
																	
								}					
								else
								{
					
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[0], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
								
										}
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);  //wait for 5us
							
										offset_cycle_num = 0;
								
									}
								}		
								break;	
						}
						
					}	
				break;
				
				case 2:				
					if(offset_home == 0)
					{
						find_home(0,1);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(0, Joint_Angle_FB[0]);						
								start_position[0][1] = Joint_Angle_FB[0][1];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[0][1] = Five_Interpolation(start_position[0][1],0,0,start_position[0][1]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(0,1,Joint_Angle_EP[0][1]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[0][1] = start_position[0][1]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[0], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}	
				break;
				
				case 3:
					if(offset_home == 0)
					{
						find_home(0,2);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(0, Joint_Angle_FB[0]);						
								start_position[0][2] = Joint_Angle_FB[0][2];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[0][2] = Five_Interpolation(start_position[0][2],0,0,start_position[0][2]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(0,2,Joint_Angle_EP[0][2]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[0][2] = start_position[0][2]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[0], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
											sleeptime.tv_nsec = 5000;
											sleeptime.tv_sec = 0;
											nanosleep(&sleeptime, NULL);  //wait for 5us
										}
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}	
					
				break;
				
				case 4:
				
					if(offset_home == 0)
					{
						find_home(0,3);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(0, Joint_Angle_FB[0]);						
								start_position[0][3] = Joint_Angle_FB[0][3];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[0][3] = Five_Interpolation(start_position[0][3],0,0,start_position[0][3]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(0,3,Joint_Angle_EP[0][3]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[0][3] = start_position[0][3]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[0], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
					
				break;
				
				case 5:
				
					if(offset_home == 0)
					{
						find_home(1,0);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(1, Joint_Angle_FB[1]);						
								start_position[1][0] = Joint_Angle_FB[1][0];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[1][0] = Five_Interpolation(start_position[1][0],0,0,start_position[1][0]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(1, 0,Joint_Angle_EP[1][0]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[1][0] = start_position[1][0]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[1], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
					
				break;
				
				case 6:
					if(offset_home == 0)
					{
						find_home(1,1);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(1, Joint_Angle_FB[1]);						
								start_position[1][1] = Joint_Angle_FB[1][1];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[1][1] = Five_Interpolation(start_position[1][1],0,0,start_position[1][1]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(1, 1,Joint_Angle_EP[1][1]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[1][1] = start_position[1][1]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[1], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
				
					
				break;
				
				case 7:
					if(offset_home == 0)
					{
						find_home(1,2);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(1, Joint_Angle_FB[1]);						
								start_position[1][2] = Joint_Angle_FB[1][2];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[1][2] = Five_Interpolation(start_position[1][2],0,0,start_position[1][2]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(1, 2,Joint_Angle_EP[1][2]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[1][2] = start_position[1][2]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[1], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
					
				break;
				
				case 8:
					if(offset_home == 0)
					{
						find_home(1,3);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(1, Joint_Angle_FB[1]);						
								start_position[1][3] = Joint_Angle_FB[1][3];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[1][3] = Five_Interpolation(start_position[1][3],0,0,start_position[1][3]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(1, 3,Joint_Angle_EP[1][3]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[1][3] = start_position[1][3]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[1], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
				
					
				break;
				
				case 9:
					if(offset_home == 0)
					{
						find_home(2, 0);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(2, Joint_Angle_FB[2]);						
								start_position[2][0] = Joint_Angle_FB[2][0];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[2][0] = Five_Interpolation(start_position[2][0],0,0,start_position[2][0]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(2, 0,Joint_Angle_EP[2][0]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[2][0] = start_position[2][0]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[2], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
						
				break;
				
				case 10:
					if(offset_home == 0)
					{
						find_home(2,1);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(2, Joint_Angle_FB[2]);						
								start_position[2][1] = Joint_Angle_FB[2][1];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[2][1] = Five_Interpolation(start_position[2][1],0,0,start_position[2][1]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(2, 1,Joint_Angle_EP[2][1]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[2][1] = start_position[2][1]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[2], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
					
				break;
				
				case 11:
				
					if(offset_home == 0)
					{
						find_home(2,2);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(2, Joint_Angle_FB[2]);						
								start_position[2][2] = Joint_Angle_FB[2][2];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t =0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[2][2] = Five_Interpolation(start_position[2][2],0,0,start_position[2][2]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(2, 2,Joint_Angle_EP[2][2]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[2][2] = start_position[2][2]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[2], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
						
				break;
				
				case 12:
					if(offset_home == 0)
					{
						find_home(2,3);	
						offset_home = 3;					
					}
					
					if(offset_home == 1)
					{
						switch (first_time_flag)
						{
							case 0:
						
								current_position(2, Joint_Angle_FB[2]);						
								start_position[2][3] = Joint_Angle_FB[2][3];
					
								first_time_flag = 1;
								motion_mode_control = 0;	
								t = 0;
							break;	
						
							case 1:
								
								if(t <= offset_time)
								{	
															
									Joint_Angle_EP[2][3] = Five_Interpolation(start_position[2][3],0,0,start_position[2][3]+JointMoveData,0,0,offset_time,t);			
				
									rad_send(2, 3,Joint_Angle_EP[2][3]);
									t = t+time_interval;
									
									if(t+time_interval > offset_time)
									{
										Joint_Angle_EP[2][3] = start_position[2][3]+JointMoveData;
									}
																	
								}					
								else
								{
									offset_cycle_num++;
									if(offset_cycle_num == 30)
									{
										t = 0;
										motion_mode = 100;
										first_time_flag = 0;
										motion_mode_control = 1;
								
										for(k-0; k<8;k++)
										{
											ret = rt_dev_recvfrom(rxsock[2], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
										}
						
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime, NULL);  //wait for 5us
							
										offset_cycle_num = 0;
									
									}
								}		
								break;	
						}						
					}
					
				break;
				
				case 21:
				
					if(first_time_flag_mode21 == 1)
					{
						for(i=0; i<3; i++)
						{
							for(j=0; j< can_node_number[i]; j++)
							{
								ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
								if (ret < 0) 
								{
									switch (ret) 
									{
										case -ETIMEDOUT:
											if (verbose)
												printf("rt_dev_recv: timed out\n");
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
									if(i < 3)
									{
										Encoder_Count_FB[i][(frame.can_id & 0xf)-1] = *(int*)frame.data;
										Joint_Angle_FB[i][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[i][(frame.can_id & 0xf)-1])/Rad2Count/reduction_ratio - zero_comp[i][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1] ;
										Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1]*Rad2Degree;
									}
									else
									{
										Encoder_Count_FB[i][(frame.can_id & 0xf)-1] = *(int*)frame.data;
										Joint_Angle_FB[i][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[i][(frame.can_id & 0xf)-1])/line_2count- zero_comp[i][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1] ;
										Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1];
									}
								}								
							}
						}
						
						if(t > home_time)
						{
							first_time_flag_mode21 = 0;
							motion_mode = 100;
							t = 0.0;
							break;
						}
						
					}
				
					if(t <= home_time)
					{
						sleeptime.tv_nsec = 5000;
						sleeptime.tv_sec = 0;
						nanosleep(&sleeptime,NULL);  //wait for 50us
				
						sleeptime.tv_nsec = 150000;
						sleeptime.tv_sec = 0;
				
						//leg expect trajactory		
						for(j=0; j<can_node_number_max; j++)
						{
							for(i=0; i<3; i++)
							{	
								if(t < half_time)
								{
									Joint_Angle_EP[i][j] = Five_Interpolation(0,0,0,home_offset[i][j],0,0,home_time,t);
								}							
							}
						}
					
					
			
						for(j=0; j<can_node_number_max; j++)
						{
							for(i=0; i<3; i++)
							{	
								if(i < 3)	   //the joint motion
								{						
									if(j < can_node_number[i])
									{
										Encoder_Count_EP[i][j] = (Joint_Angle_EP[i][j] * joint_direction[i][j] + zero_comp[i][j]) * Rad2Count * reduction_ratio;
										Encoder_Count_EP_Send[i][j] = Encoder_Count_EP[i][j] ;						
										Joint_Angle_EP_degree[i][j] = (Joint_Angle_EP[i][j] ) * Rad2Degree;
								
										Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];
								
										can_content_order[0] = 0X1F;
										can_content_order[1] = 0X00;
										can_content_order[2] = Encoder_Count_EP_Send[i][j] & 0x000000FF;
										can_content_order[3] = (Encoder_Count_EP_Send[i][j] & 0x0000FF00)>>8;
										can_content_order[4] = (Encoder_Count_EP_Send[i][j] & 0x00FF0000)>>16;
										can_content_order[5] = (Encoder_Count_EP_Send[i][j] & 0xFF000000)>>24;
						
										can_id = 0x200 + j+1;						
										can_send(i, can_id, 0, can_content_order, 6);
									
									}		
								}	
						
							}
					
							nanosleep(&sleeptime,NULL);
						}		
					
						can_id = 0x80;		
						for(i=0; i<can_channel_number; i++)
						{	
							can_send(i, can_id, 0, can_content_order, 0);   //SYNC				
						}
					
						first_time_flag_mode21 = 1;	
						t = t + time_interval;						
					}
							
					break;
					
					case 20:
					
					if(first_time_flag_mode20 == 1)
					{
					//	first_time_flag = 1;					
						for(i=0; i<can_channel_number; i++)
						{
							for(j=0; j< can_node_number[i]; j++)
							{
								{
									ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
									if (ret < 0) 
									{
										switch (ret) 
										{
											case -ETIMEDOUT:
												if (verbose)
													printf("rt_dev_recv: timed out\n");
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
										if(i < 3)
										{
											Encoder_Count_FB[i][(frame.can_id & 0xf)-1] = *(int*)frame.data;
											Joint_Angle_FB[i][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[i][(frame.can_id & 0xf)-1])/Rad2Count/reduction_ratio - zero_comp[i][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1] ;
											Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1]*Rad2Degree;
										}
										else
										{
											Encoder_Count_FB[i][(frame.can_id & 0xf)-1] = *(int*)frame.data;
											Joint_Angle_FB[i][(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[i][(frame.can_id & 0xf)-1])/line_2count- zero_comp[i][(frame.can_id & 0xf)-1]) * joint_direction[i][(frame.can_id & 0xf)-1]  ;
											Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1];
										}
									}	
								}
							}
						}
					}
				
					sleeptime.tv_nsec = 5000;
					sleeptime.tv_sec = 0;
					nanosleep(&sleeptime,NULL);  //wait for 50us
				
					sleeptime.tv_nsec = 150000;
					sleeptime.tv_sec = 0;
				
					//leg expect trajactory		
					for(j=0; j<can_node_number_max; j++)
					{
						for(i=0; i<3; i++)
						{	
							if(t < half_time)
							{
								Joint_Angle_EP[i][j] = Five_Interpolation(0,0,0,end_position[i][j],0,0,half_time,t);
							}
							else
							{
								if(t < half_time+1)
								{
								}
								else
								{
									if(t < 2*half_time +1)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(end_position[i][j],0,0,0,0,0,half_time, t - half_time-1);
									}
									else
									{
									}
								}
							}
						}
					}
					
				
					i = 3;
					
					for(j=0; j<can_node_number_max; j++)
					{
						
						if(t < half_time)
						{
							Joint_Angle_EP[i][j] = Five_Interpolation(0,0,0,end_position_hand,0,0,half_time,t);
					
						}
						else
						{
							if(t < half_time+1)
							{
							}
							else
							{
								if(t < 2*half_time +1)
								{
									Joint_Angle_EP[i][j] = Five_Interpolation(end_position_hand,0,0,0,0,0,half_time, t-half_time-1);
								}
								else
								{
								}
							}
						}
					}
				
							

					for(j=0; j<can_node_number_max; j++)
					{
						for(i=0; i<can_channel_number; i++)
						{	
							if(i < 3)	   //the joint motion
							{						
								if(j < can_node_number[i])
								{
									Encoder_Count_EP[i][j] = (Joint_Angle_EP[i][j] * joint_direction[i][j] + zero_comp[i][j]) * Rad2Count * reduction_ratio;
									Encoder_Count_EP_Send[i][j] = Encoder_Count_EP[i][j] ;						
									Joint_Angle_EP_degree[i][j] = (Joint_Angle_EP[i][j]) * Rad2Degree;
								
									Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];
								
									can_content_order[0] = 0X1F;
									can_content_order[1] = 0X00;
									can_content_order[2] = Encoder_Count_EP_Send[i][j] & 0x000000FF;
									can_content_order[3] = (Encoder_Count_EP_Send[i][j] & 0x0000FF00)>>8;
									can_content_order[4] = (Encoder_Count_EP_Send[i][j] & 0x00FF0000)>>16;
									can_content_order[5] = (Encoder_Count_EP_Send[i][j] & 0xFF000000)>>24;
						
									can_id = 0x200 + j+1;						
									can_send(i, can_id, 0, can_content_order, 6);
								//	printf("t=%f i=%d j=%d\n", t,i,j);
								}		
							}	
							else   //the hand motion
							{
								if(j < can_node_number[i])
								{
									Encoder_Count_EP[i][j] = (Joint_Angle_EP[i][j] * joint_direction[i][j] + zero_comp[i][j]) * line_2count;
									Encoder_Count_EP_Send[i][j] = Encoder_Count_EP[i][j];
									Joint_Angle_EP_degree[i][j] = Joint_Angle_EP[i][j];
								
									Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];
								
									can_content_order[0] = 0X1F;
									can_content_order[1] = 0X00;
									can_content_order[2] = Encoder_Count_EP_Send[i][j] & 0x000000FF;
									can_content_order[3] = (Encoder_Count_EP_Send[i][j] & 0x0000FF00)>>8;
									can_content_order[4] = (Encoder_Count_EP_Send[i][j] & 0x00FF0000)>>16;
									can_content_order[5] = (Encoder_Count_EP_Send[i][j] & 0xFF000000)>>24;
						
									can_id = 0x200 + j+1;						
									can_send(i, can_id, 0, can_content_order, 6);
							//		printf("t=%f i=%d j=%d\n", t,i,j);
								}								
							}
						}
					
						nanosleep(&sleeptime,NULL);
					}		
					
								
				
					can_id = 0x80;		
					for(i=0; i<can_channel_number; i++)
					{	
						can_send(i, can_id, 0, can_content_order, 0);   //SYNC				
					}
					
					first_time_flag_mode20 = 1;
					
					t = t + time_interval;
					if(t >= 2.0*half_time+2)
					{
						t = 0;
				//		first_time_flag = 0;
					}					
					
					break;
					
				default:
					
					break;
		
			}
			
			for(j=0; j<can_node_number_max; j++)
			{
				for(i=0; i<can_channel_number; i++)
				{	
					if(i < 3)	   //the joint motion
					{						
						if(j < can_node_number[i])
						{											
							Joint_Angle_EP_degree[i][j] = (Joint_Angle_EP[i][j]) * Rad2Degree;				
							Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];				
						}		
					}	
					else   //the hand motion
					{
						if(j < can_node_number[i])
						{						
							Joint_Angle_EP_degree[i][j] = (Joint_Angle_EP[i][j]);				
							Joint_Angle_ER_degree[i][j] = Joint_Angle_EP_degree[i][j] - Joint_Angle_FB_degree[i][j];
						}								
					}
				}
	
			}
			
		}
				



		/************************* UDP数据传输 Start **************************/		
		UDPTimes++;
		if(UDPTimes == 33)
		{	
				UDPTimes = 0;
				// 发送关节角
				UploadData.TrustFlag = 0x5555;
				for(i=0; i<5; i++)
				{
					UploadData.Arm1Angle[i] = 0.0;
					UploadData.Arm2Angle[i] = 0.0;
					UploadData.Arm3Angle[i] = 0.0;
				}
				memcpy(sendbuff,&UploadData,sizeof(UploadData));
		
				int n;

				nAddrLen = sizeof(DestAddr);
				n = sendto(UDP_Sock, sendbuff, sizeof(UploadData), 0, (struct sockaddr *)&DestAddr, nAddrLen);
		
				// 接收指令
				timetest1 = rt_timer_read();
				memset(recvbuff,0,sizeof(recvbuff));
				n = recvfrom(UDP_Sock, recvbuff, sizeof(recvbuff), MSG_DONTWAIT, (struct sockaddr *)&DestAddr, &nAddrLen);
				if(n == sizeof(DownloadData))
				{
					memcpy(&DownloadData,recvbuff,sizeof(DownloadData));
					
					if(motion_mode_control == 1)
					{			
						if(DownloadData.Arm1 == 0x01)
							motion_mode = 1;
						else if(DownloadData.Arm1 == 0x02)
							motion_mode = 2;
						else if(DownloadData.Arm1 == 0x04)
							motion_mode = 3;
						else if(DownloadData.Arm1 == 0x08)
							motion_mode = 4;
						else if(DownloadData.Arm2 == 0x01)
							motion_mode = 5;
						else if(DownloadData.Arm2 == 0x02)
							motion_mode = 6;
						else if(DownloadData.Arm2 == 0x04)
							motion_mode = 7;
						else if(DownloadData.Arm2 == 0x08)
							motion_mode = 8;
						else if(DownloadData.Arm3 == 0x01)
							motion_mode = 9;
						else if(DownloadData.Arm3 == 0x02)
							motion_mode = 10;
						else if(DownloadData.Arm3 == 0x04)
							motion_mode = 11;
						else if(DownloadData.Arm3 == 0x08)
							motion_mode = 12;
						else
							motion_mode = 0;	
					}			
				
					if(DownloadData.Mode == 0x01)
					{
						offset_home = 0;
						JointMoveData = 0.0;
					}
					else if(DownloadData.Mode == 0x02)
					{
						offset_home = 1;
						JointMoveData = DownloadData.Data * degree2rad; 
					}
							
					timetest2 = rt_timer_read();
					timecost = (timetest2 - timetest1)/1000;//us
					printf("Spend %lf us, Joint %d, Mode %d, Data %f\n",timecost,motion_mode,offset_home,JointMoveData);
					first_time_flag = 0;
				}
				else if(n>0)
				{
					printf("Recv %d Byte data\n",n);

				}
				timetest1 = 0;
				timetest2 = 0;
		}
		
			
		
		/************************* UDP数据传输 End **************************/	
		
		/************************* 界面显示 **************************/			
		sprintf(buf3, "********************  Robot Joint Angle  **********************");	
		sprintf(buf4, "ARM1 Recv %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_FB_degree[0][0], Joint_Angle_FB_degree[0][1], Joint_Angle_FB_degree[0][2], Joint_Angle_FB_degree[0][3], Joint_Angle_FB_degree[3][0]);
		
		sprintf(buf5, "ARM1 Send %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_EP_degree[0][0], Joint_Angle_EP_degree[0][1], Joint_Angle_EP_degree[0][2], Joint_Angle_EP_degree[0][3], Joint_Angle_EP_degree[3][0]);
		
		sprintf(buf6, "ARM2 Recv %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_FB_degree[1][0], Joint_Angle_FB_degree[1][1], Joint_Angle_FB_degree[1][2], Joint_Angle_FB_degree[1][3], Joint_Angle_FB_degree[3][1]);
		
		sprintf(buf7, "ARM2 Send %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_EP_degree[1][0], Joint_Angle_EP_degree[1][1], Joint_Angle_EP_degree[1][2], Joint_Angle_EP_degree[1][3], Joint_Angle_EP_degree[3][1]);

		sprintf(buf8, "ARM3 Recv %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_FB_degree[2][0], Joint_Angle_FB_degree[2][1], Joint_Angle_FB_degree[2][2], Joint_Angle_FB_degree[2][3], Joint_Angle_FB_degree[3][2]);
		
		sprintf(buf9, "ARM3 Send %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_EP_degree[2][0], Joint_Angle_EP_degree[2][1], Joint_Angle_EP_degree[2][2], Joint_Angle_EP_degree[2][3], Joint_Angle_EP_degree[3][2]);	
		
		sprintf(buf10, "ARM1 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_ER_degree[0][0], Joint_Angle_ER_degree[0][1], Joint_Angle_ER_degree[0][2], Joint_Angle_ER_degree[0][3], Joint_Angle_ER_degree[3][0]);
		sprintf(buf11, "ARM2 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_ER_degree[1][0], Joint_Angle_ER_degree[1][1], Joint_Angle_ER_degree[1][2], Joint_Angle_ER_degree[1][3], Joint_Angle_ER_degree[3][1]);
		sprintf(buf12, "ARM3 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f", 
		Joint_Angle_ER_degree[2][0], Joint_Angle_ER_degree[2][1], Joint_Angle_ER_degree[2][2], Joint_Angle_ER_degree[2][3], Joint_Angle_ER_degree[3][2]);
		
//		sprintf(buf13, "*****************************  Robot Status  *****************************");
//		sprintf(buf14, "CAN Recv Status  %d  %d  %d  %d",canRecvStatus[0],canRecvStatus[1],canRecvStatus[2],canRecvStatus[3]);	
	}
}

int UDPComm_init(void)
{
	
    	if ( (UDP_Sock=socket(AF_INET, SOCK_DGRAM, 0)) <0)
	{
		perror("socket created failed!");
	}
	struct sockaddr_in HostAddr;
	bzero(&HostAddr,sizeof(HostAddr));
	HostAddr.sin_family = AF_INET;
	HostAddr.sin_port = htons(HOST_PORT);
	HostAddr.sin_addr.s_addr = inet_addr(HOST_IP);
	if (bind(UDP_Sock, &HostAddr, sizeof(HostAddr)) < 0)
	{
		perror("socket binded failed!");
	}

	DestAddr.sin_family = AF_INET;
	DestAddr.sin_port = htons(DEST_PORT);
	DestAddr.sin_addr.s_addr = inet_addr(DEST_IP);
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
	//    strncpy(ifname, argv[optind], IFNAMSIZ);task
	
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
	
	UDPComm_init();
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


void cleanup(void)
{
    	int ret, i;
    	if (verbose)
        	printf("Cleaning up...\n");

	for(i=0; i<4; i++)
	{
   		usleep(1000);

		if (txsock[i] >= 0) 
		{
			ret = rt_dev_close(txsock[i]);
			txsock[i] = -1;
			if (ret) 
			{
			    fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
			}
			exit(EXIT_SUCCESS);
		}

		if (rxsock[i] >= 0) 
		{
			ret = rt_dev_close(rxsock[i]);
			rxsock[i] = -1;
			if (ret) 
			{
			    fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
			}
			exit(EXIT_SUCCESS);
		}
	}
}

int add_filter(u_int32_t id, u_int32_t mask)
{
    if (filter_count >= MAX_FILTER)
        return -1;
    recv_filter[filter_count].can_id = id;
    recv_filter[filter_count].can_mask = mask;
    printf("Filter #%d: id=0x%08x mask=0x%08x\n", filter_count, id, mask);
    filter_count++;
    return 0;
}

int elmo_init(void)
{
	long can_id = 1;
	long can_content[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[15] = {0};
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
		can_content_order[0] = 0x2F;
		can_content_order[1] = 0x00;
		can_content_order[2] = 0x18;
		can_content_order[3] = 0x02;
		can_content_order[4] = 0x01;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //TPDO1 transmission every sync
		
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
		can_content_order[4] = 0x02;
		can_content_order[5] = 0x00;
		can_content_order[6] = 0x00;
		can_content_order[7] = 0x00;
		can_send(i, can_id, 0, can_content_order, 8);   //enable 2 mapping objects
		
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
		can_content_order[4] = 0x03;
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
	
	sprintf(buf23, "CAN init        %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1],  can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], 	can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14]);
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
	int can_connection_status[15] = {0};
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

	for(i=0; i<can_channel_number; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, can_content_order, 0);   //SYNC
	//	can_send(i, can_id, 0, can_content_order, 0);   //SYNC
	}	
	nanosleep(&sleeptime,NULL);
	
	sprintf(buf22, "CAN connection  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d", can_connection_status[0],can_connection_status[1], can_connection_status[2],can_connection_status[3],can_connection_status[4],can_connection_status[5],can_connection_status[6],can_connection_status[7], can_connection_status[8],can_connection_status[9],can_connection_status[10],can_connection_status[11],can_connection_status[12],can_connection_status[13], can_connection_status[14]);
	
	sprintf(buf23, "CAN init        %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1], can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14]);
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
	
	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[15] = {0};
	
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
	
		sprintf(buf22, "CAN connection  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d", can_connection_status[0],can_connection_status[1], can_connection_status[2],can_connection_status[3],can_connection_status[4],can_connection_status[5],can_connection_status[6],can_connection_status[7], can_connection_status[8],can_connection_status[9],can_connection_status[10],can_connection_status[11],can_connection_status[12],can_connection_status[13], can_connection_status[14]);
	
		sprintf(buf23, "CAN init        %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1], can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14]);
}

