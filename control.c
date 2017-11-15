/* Program to Control Robot
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
#include "global_def.h"
#include "communication.h"

#define hand2base 300.0

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

void servo_on_control(int can_channel, int id);
void servo_off_control(int can_channel, int id);
void hand_current_set(int id, float cl);
void emlo_can_init(void);
void SYNC_Receive(void);
void servo_on_off(int on_off, int channel, int id);
int find_home_new(void);
int return_origin_position(void);
int prepare_find_home(void);
//varable declear

RT_TASK demo_task_rvcan;
RT_TASK rt_task_desc;
RT_TASK rt_task_view;
RTIME now, previous, period, now2;

GC MyGC;

double time_interval = 0.006;

extern int optind, opterr, optopt;
static int txsock[4], rxsock[4];

RT_TASK rt_task_desc;
static int s=-1, dlc=0, rtr=0, extended=0, verbose=1, loops=1;
//static SRTIME delay=1000000;
//static int count=0, print=1, use_send=0, loopback=-1;
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

double runtime = 0.0;
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
char buf24[200]={0};

double One_arm_main[4] = {0.0};
int motion_mode = 0;
long can_channel_main = 0;
long can_id_main = 0;
int End_Numb = 0;
int can_node_number[4] = {7,7,6,6};
int can_channel_number = 4;
int can_switch[4][7] = {{1, 1, 1, 1, 1, 1, 1},
						{1, 1, 1, 1, 1, 1, 1},
						{1, 1, 1, 1, 1, 1, 1},
						{1, 1, 1, 1, 1, 1, 0}};


double joint_direction[4][7] = {{1, -1, 1, 1, 1, -1, 1},		// 456 OK
								{1, 1, 1, 1, 1, 1, 1},
								{1, 1, 1, 1, 1, 1},			// 12 OK
								{1, 1, 1, 1, 1, 1}};
double zero_comp[4][7] = {0.0};
int can_work_states[27] = {0};
double motor_current[4][7] = {0.0};
int motor_current_feedback[4][7] = {0};
double motor_current_refrence[4][7] = {	{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
										{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
										{5.0, 5.0, 5.0, 5.0, 5.0, 5.0},
										{5.0, 5.0, 5.0, 5.0, 5.0, 5.0}};


double T_hand_end[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -hand2base},
						{0.0, 0.0, 0.0, 1.0}};

double T_hand_end_inv[4][4] = {{1.0, 0.0, 0.0, 0.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, hand2base},
						{0.0, 0.0, 0.0, 1.0}};

int servo_on_off_flag = 0;
//double v_hand_max = 0.2;
//double a_hand_max = 0.5;
//double hand_current_max = 1.0;
//int print_flag = 1;
int hand_id = 0;
float hand_current_expect = 0.0;
double sin15 = 0.2588190451025;
double cos15 = 0.965925826289;

//long data1[8];
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
//long handvel=0;
sem_t low_go;

int offset_home = 3;
double turn_time = 15.0;


int Encoder_Count_FB[4][7] = {0};
int Encoder_Count_EP[4][7] = {0};
int Encoder_Count_EP_Send[4][7] = {0};
double Joint_Angle_FB[4][7] = {0.0};
double Joint_Angle_EP[4][7] = {0.0};
double Joint_Angle_FB_degree[4][7] = {0.0};
double Joint_Angle_EP_degree[4][7] = {0.0};
double Joint_Angle_ER_degree[4][7] = {0.0};

int HeadEncoder_Count_FB[2] = {0};
double Head_Angle_FB[2] = {0.0};
double Head_Angle_FB_degree[2] = {0.0};

double VoltageAll = 0.0;
double CurrentAll = 0.0;

long motor_sp[4][7] = {{40, 	5500,	5500,	5500, 	50000,	50000,	50000},
					   {40, 	5500,	5500,	5500, 	50000,	50000,	50000},
					   {50000, 	50000, 	50000,	50000, 	500,	500},
					   {50000, 	50000,	50000,	50000, 	500,	500}};

float motor_cl[4][7] = {{0.75, 1.0, 1.0, 1.0, 7.0, 7.0, 7.0},
						{0.75, 1.0, 1.0, 1.0, 7.0, 7.0, 7.0},
						{7.0, 7.0, 7.0, 7.0, 1.0, 1.0},
						{7.0, 7.0, 7.0, 7.0, 1.0, 1.0}};

float motor_pl[4][7] = {{0.8, 1.2, 1.2, 1.2, 10.0, 10.0, 10.0},
						{0.8, 1.2, 1.2, 1.2, 10.0, 10.0, 10.0},
						{10.0, 10.0, 10.0, 10.0, 1.2, 1.2},
						{10.0, 10.0, 10.0, 10.0, 1.2, 1.2}};

double home_offset[4][7] = {{-0.043633, 	-0.366619137, 	0.0261799, 0.19198622,0.0,0.0,0.0},
							{-0.02617994,	-0.383972436,  -0.017453293, -0.102617994,0.0,0.0,0.0},
							{-0.01745329,	0.453785603,   -0.0174532925, -0.0959931068,0.0,0.0,0.0},
							{0.0,			0.0,			0.0, 0.0, 0.0, 0.0, 0.0}};

double joint_angle_last[8] = {0.5, 0.0, 0.2, 0.0, -0.5, 0.0, -0.2, 0.0};

double joint_max_angle[4] = {0.0};
double joint_min_angle[4] = {0.0};
double max_joint_speed[4] = {0.0};

double home_time = 5.0;

double Joint_Current_Angle[4][7] = {0.0};


//double pi = 3.1415926535898;
int servo_on_flag = 0;
int begin_flag =0;
//int MOTION_MODE = 100;
double line_2count[3] = {1344.0, 1344.0, 1344.0};
//double line_2count[3] = {240.0, 720.0, 240.0};
int send_flag_99= 0;


/********************** UDP通讯相关 Start ***************************/
int UDPTimes = 0;	// UDP 周期计数

/********************** UDP通讯相关 End ***************************/


/**********************  VisionControl Start ***********************/
float DeltaMatrix[4][4] = {0.0};

/**********************  VisionControl End ***********************/


// 关节运动量
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
double Rad2Count[4][7] = {{7.00281748,636.619772,636.619772,636.619772,2607.5945876176,2607.5945876176,2607.5945876176},
						  {7.00281748,636.619772,636.619772,636.619772,2607.5945876176,2607.5945876176,2607.5945876176},
 						  {2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176},
 						  {2607.5945876176,2607.5945876176,2607.5945876176,2607.5945876176,651.8986,651.8986,0.0}};//651.898;
double Rad2Degree = 57.29577951308;
double degree2rad = 0.0174532925;
double reduction_ratio[4][7] = {{34.0,100.0,100.0,100.0,160.0,160.0,160.0},
								{34.0,100.0,100.0,100.0,160.0,160.0,160.0},
								{160.0,160.0,160.0,160.0,100.0,100.0,100.0},
								{160.0,160.0,160.0,160.0,120.0,120.0}};//4.8;


void servo_off_control(int can_channel, int id)
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

	sleeptime.tv_nsec = 1000000;
	sleeptime.tv_sec = 0;

	nanosleep(&sleeptime,NULL);

	can_id = 0x200 + id;

	can_content_order[0] = 0x06;
	can_content_order[1] = 0x00;
	can_content_order[2] = 0x00;
	can_content_order[3] = 0x00;
	can_content_order[4] = 0x00;
	can_content_order[5] = 0x00;
	can_send(can_channel, can_id, 0, can_content_order, 6);   //SYNC
	nanosleep(&sleeptime,NULL);

	can_id = 0x80;
	can_send(can_channel, can_id, 0, can_content_order, 0);   //SYNC
	printf("0x80\n");

	sleeptime.tv_nsec = 200000000;

	sleeptime.tv_sec = 0;

	nanosleep(&sleeptime,NULL);

	for(i=0; i<4; i++)
	{
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
	}

}


int find_home_new(void)
{
	static double start_position[3][4];

	double prepare_position[3][4] = {{8.0/Rad2Degree, 10.0/Rad2Degree, -85.0/Rad2Degree, -12.0/Rad2Degree},
									{12.0/Rad2Degree, 10.0/Rad2Degree, 95.0/Rad2Degree, 0.0/Rad2Degree},
									{7.0/Rad2Degree, -30.0/Rad2Degree, 7.0/Rad2Degree, 0.0/Rad2Degree}};
	int i, j;
	static int fisrt_time_find_home_new = 0;
	double move_time = 10.0;
	static double t = 0.0;
	struct timespec sleeptime;

	if(fisrt_time_find_home_new == 0)
	{
		for(i=0; i<3; i++)
		{
			for(j=0; j<4; j++)
			{
				start_position[i][j] = Joint_Angle_FB[i][j];
				fisrt_time_find_home_new = 1;
				t = 0;
			}
		}
		return 1;
	}
	else
	{

		if(t <= move_time)
		{
			t = t + time_interval;
			i = 0;
			for(j=0; j<4; j++)
			{
				Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j], 0, 0, prepare_position[i][j], 0, 0, move_time,t);

				if(t > move_time)
				{
					Joint_Angle_EP[i][j] = prepare_position[i][j];
				}

				if(send_flag_99 == 1)
				{
					rad_send(i,j,Joint_Angle_EP[i][j]);
				}
				sleeptime.tv_nsec = 250000;
				sleeptime.tv_sec = 0;
				nanosleep(&sleeptime,NULL);
			}
	//		printf("Joint_Angle_EP[0][3] = %f    start_position[0][0] = %f\n",Joint_Angle_EP[0][3], start_position[0][3]);
			return 1;
		}
		else if(t <= 2.0*move_time)
			{
				t = t + time_interval;
				i = 1;
				for(j=0; j<4; j++)
				{
					Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0,prepare_position[i][j],0,0,move_time,t-move_time);

					if((t - move_time) > move_time)
					{
						Joint_Angle_EP[i][j] = prepare_position[i][j];
					}

					if(send_flag_99 == 1)
					{
						rad_send(i,j,Joint_Angle_EP[i][j]);
					}
					sleeptime.tv_nsec = 250000;
					sleeptime.tv_sec = 0;
					nanosleep(&sleeptime,NULL);
				}
				return 1;
			}
			else if (t <= 3.0*move_time)
			{
				t = t + time_interval;
				i =  2;
				for(j=0; j<4; j++)
				{
					Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0,prepare_position[i][j],0,0,move_time,t - 2.0*move_time);

					if((t-2.0*move_time) > move_time)
					{
						Joint_Angle_EP[i][j] = prepare_position[i][j];
					}

					if(send_flag_99 == 1)
					{
						rad_send(i,j,Joint_Angle_EP[i][j]);
					}
					sleeptime.tv_nsec = 250000;
					sleeptime.tv_sec = 0;
					nanosleep(&sleeptime,NULL);
				}
				return 1;
			}
			else
			{
				return 0;
			}

	}
}



int prepare_find_home(void)
{
	double angle[2] = {-3.5/Rad2Degree, 3.5/Rad2Degree};
	static double start_position[3][4];
	int i, j;
	static int fisrt_time_prepare_find_home = 0;
	double move_time = 5.0;
	static double t = 0.0;
	struct timespec sleeptime;

	if(fisrt_time_prepare_find_home == 0)
	{
		for(i=0; i<3; i++)
		{
			for(j=0; j<4; j++)
			{
				start_position[i][j] = Joint_Angle_FB[i][j];
				fisrt_time_prepare_find_home = 1;
				t = 0;
			}
		}
		return 1;
	}
	else
	{

		if(t <= move_time)
		{
			t = t+ time_interval;
			Joint_Angle_EP[0][0] = Five_Interpolation(start_position[0][0],0,0, angle[0],0,0,move_time,t);
			Joint_Angle_EP[1][0] = Five_Interpolation(start_position[1][0],0,0, angle[1],0,0,move_time,t);
			if(t > move_time)
			{
				Joint_Angle_EP[0][0] = angle[0];
				Joint_Angle_EP[1][0] = angle[1];
			}

			if(send_flag_99 == 1)
			{
				rad_send(0,0,Joint_Angle_EP[0][0]);
				sleeptime.tv_nsec = 5000;
				sleeptime.tv_sec = 0;
				nanosleep(&sleeptime,NULL);
				rad_send(1,0,Joint_Angle_EP[1][0]);
			}

	//		printf("Joint_Angle_EP[0][3] = %f    start_position[0][0] = %f\n",Joint_Angle_EP[0][3], start_position[0][3]);
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

int return_origin_position(void)
{
	double origin_position[3][4] = {{0.0, 0.0, 1.5708, 0.0},
									{0.0, 0.0, -1.5708, 0.0},
									{0.0, 0.0, 0.0, 0.0}};
	static double start_position[3][4];
	int i, j;
	static int fisrt_time_return_origin_position = 0;
	double move_time = 10.0;
	static double t = 0.0;
	struct timespec sleeptime;

	if(fisrt_time_return_origin_position == 0)
	{
		for(i=0; i<3; i++)
		{
			for(j=0; j<4; j++)
			{
				start_position[i][j] = Joint_Angle_FB[i][j];
				fisrt_time_return_origin_position = 1;
				t = 0;
			}
		}
	//	printf("test1\n");
		return 1;
	}
	else
	{

		if(t <= move_time)
		{
			t = t + time_interval;
			for(j=0; j<4; j++)
			{
				for(i=0; i<3; i++)
				{
					Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0,origin_position[i][j],0,0,move_time,t);

					if(t > move_time)
					{
						Joint_Angle_EP[i][j] = origin_position[i][j];
					}

					if(send_flag_99 == 1)
					{
						rad_send(i,j,Joint_Angle_EP[i][j]);
					}
					sleeptime.tv_nsec = 5000;
					sleeptime.tv_sec = 0;
					nanosleep(&sleeptime,NULL);
				}
				sleeptime.tv_nsec = 250000;
				sleeptime.tv_sec = 0;
				nanosleep(&sleeptime,NULL);
			}

			return 1;
		}
		else
		{

			return 0;
		}
	}
//	return 1;
}


void servo_on_control(int can_channel, int id)
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
	int can_connection_status[27] = {0};


	can_id = 0x200 + id;
	can_content_order[0] = 0x07;
	can_content_order[1] = 0x00;
	can_content_order[2] = 0x00;
	can_content_order[3] = 0x00;
	can_content_order[4] = 0x00;
	can_content_order[5] = 0x00;
	can_send(can_channel, can_id, 0, can_content_order, 6);   //controlword: switch on
	nanosleep(&sleeptime,NULL);


	can_id = 0x80;
	can_send(can_channel, can_id, 0, can_content_order, 0);   //SYNC
	sleeptime.tv_nsec = 10000000;
	sleeptime.tv_sec = 0;


	nanosleep(&sleeptime,NULL);

	for(i=0; i<4; i++)
	{
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
		ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier
	}


	can_id = 0x200 + id;
//		can_id = 0x200;
	can_content_order[0] = 0x0f;
	can_content_order[1] = 0x00;

	can_content_order[2] = 0x00;
	can_content_order[3] = 0x00;
	can_content_order[4] = 0x00;
	can_content_order[5] = 0x00;
	can_send(can_channel, can_id, 0, can_content_order, 6);   //controlword: enable operation
	nanosleep(&sleeptime,NULL);



	can_id = 0x80;
	can_send(can_channel, can_id, 0, can_content_order, 0);   //SYNC


	sleeptime.tv_nsec = 200000000;
	sleeptime.tv_sec = 0;


	nanosleep(&sleeptime,NULL);


		ret = rt_dev_recvfrom(rxsock[can_channel], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);  //recive the drvier

}


void servo_on_off(int on_off, int channel, int id)
{
	long can_id =1;
	long can_content[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
//	unsigned char *can_byte;

	sleeptime.tv_nsec = 500000;
	sleeptime.tv_sec = 0;
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[27] = {0};



	can_id = 0x300 + id;
	can_content_order[0] = 0x4D; //C
	can_content_order[1] = 0x4F; //L
	can_content_order[2] = 0x00;
	can_content_order[3] = 0x00;

//	can_byte = (unsigned char *)&cl;
	if(on_off == 1)
	{
		can_content_order[4] = 0x01;
	}
	else if(on_off == 0)
	{
		can_content_order[4] = 0x00;
	}

	can_content_order[5] = 0x00;
	can_content_order[6] = 0x00;
	can_content_order[7] = 0x00;

	can_send(channel, can_id, 0, can_content_order, 8);   //controlword: disable volatage
	printf("channel = %ld, can_id = %ld id=%d\n", channel, can_id, id);
	nanosleep(&sleeptime,NULL);

	for(i=0; i<3; i++)
	{
		ret = rt_dev_recvfrom(rxsock[channel], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}
}


void emlo_can_init(void)
{
	long can_id =1;
	long can_content[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	unsigned char *can_byte;

	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			can_id = 0x300 + j + 1;
			can_content_order[0] = 0x43; //C
			can_content_order[1] = 0x4C; //L
			can_content_order[2] = 0x01;
			can_content_order[3] = 0x80;

			can_byte = (unsigned char *)& motor_cl[i][j];
			can_content_order[4] = can_byte[0];
			can_content_order[5] = can_byte[1];
			can_content_order[6] = can_byte[2];
			can_content_order[7] = can_byte[3];

			can_send(i, can_id, 0, can_content_order, 8);   //controlword: disable volatage
			nanosleep(&sleeptime,NULL);
		}
	}

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
		}
	}
	nanosleep(&sleeptime,NULL);

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			can_id = 0x300 + j + 1;
			can_content_order[0] = 0x50; //P
			can_content_order[1] = 0x4C; //L
			can_content_order[2] = 0x01;
			can_content_order[3] = 0x80;

			can_byte = (unsigned char *)& motor_pl[i][j];
			can_content_order[4] = can_byte[0];
			can_content_order[5] = can_byte[1];
			can_content_order[6] = can_byte[2];
			can_content_order[7] = can_byte[3];

			can_send(i, can_id, 0, can_content_order, 8);   //controlword: disable volatage
			nanosleep(&sleeptime,NULL);
		}
	}

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
		}
	}

	nanosleep(&sleeptime,NULL);

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			can_id = 0x300 + j + 1;
			can_content_order[0] = 0x53; //S
			can_content_order[1] = 0x50; //p
			can_content_order[2] = 0x00;
			can_content_order[3] = 0x00;

			can_content_order[4] = (motor_sp[i][j] & 0xFF);
			can_content_order[5] = (motor_sp[i][j] & 0xFF00) >>8;
			can_content_order[6] = (motor_sp[i][j] & 0xFF0000) >>16;
			can_content_order[7] = (motor_sp[i][j] & 0xFF000000) >>24;

			can_send(i, can_id, 0, can_content_order, 8);   //controlword: disable volatage
			nanosleep(&sleeptime,NULL);
		}
	}

	for(i=0; i<4; i++)
	{
		for(j=0; j<can_node_number[i]; j++)
		{
			ret = rt_dev_recvfrom(rxsock[i], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
		}
	}
}


void hand_current_set(int id, float cl)
{
	long can_id =1;
	long can_content[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i, j, ret;
	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	unsigned char *can_byte;
//	int k=0;

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	long can_content_order[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int can_connection_status[27] = {0};

	can_id = 0x300 + id;
	can_content_order[0] = 0x43; //C
	can_content_order[1] = 0x4C; //L
	can_content_order[2] = 0x01;
	can_content_order[3] = 0x80;

	can_byte = (unsigned char *)&cl;
	can_content_order[4] = can_byte[0];
	can_content_order[5] = can_byte[1];
	can_content_order[6] = can_byte[2];
	can_content_order[7] = can_byte[3];

	can_send(3, can_id, 0, can_content_order, 8);   //controlword: disable volatage
	nanosleep(&sleeptime,NULL);

	for(i=0; i<3; i++)
	{
		ret = rt_dev_recvfrom(rxsock[3], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
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
         				printf("servo on\n");
         				servo_on();
						servo_on_flag = 1;
				break;

				case 't':
				case 'T':
					printf("Temp motion\n");
         			motion_mode = SINGLE_JOINT_MOTION;
				break;

				case '2':	// get current joint pos
				//	for(i=0; i<4; i++)
				//	{
				//		current_position(i, Joint_Angle_FB[i]);
				//	}
				break;

				case '3':

					printf("home offset\n");
         			motion_mode = HOMEBACK;
         		//	home_flag = 1;

				break;

				case '4':
		//			if(home_flag == 1)
		//			{
						printf("clear zero\n");
		 				for(i=0; i<4; i++)
		 				{
		 					for(j=0; j<4; j++)
		 					{
		 						zero_comp[i][j] = home_offset[i][j] * joint_direction[i][j];
		 					}
		 				}
     	//			}

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

					send_flag_99 =1;
					printf("send_flag_99 = 1\n");

				break;

				case '6':
					send_flag_99 = 0;
					printf("send_flag_99 = 0\n");
				break;

				case '8':
					motion_mode = FIND_HOME_NEW;
					printf("find_home_new\n");
				break;

				case '9':
					motion_mode = RETURN_ORIGIN_POSITION;
					printf("return_origin_position\n");
				break;

				case '7':
					motion_mode = PREPARE_FIND_HOME;
					printf("prepare_find_home\n");
				break;

				case '0':
					motion_mode = TURN_MOTION;
					printf("turn motion\n");
				break;

				case 'a':
				case 'A':
					motion_mode = TURN_MOTION2;
					printf("turn motion\n");
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
				if(servo_on_flag == 1)
					sprintf(buf1, "servo on");
				else
					sprintf(buf1, "servo off");


				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead, buf, strlen(buf));
				XDrawString(MyDisplay, MyWindow, MyGC, 400, DispHead, buf1, strlen(buf1));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+30, buf2, strlen(buf2));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+60, buf3, strlen(buf3));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+90, buf4, strlen(buf4));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+120, buf5, strlen(buf5));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+150, buf6, strlen(buf6));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+180, buf7, strlen(buf7));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+210, buf8, strlen(buf8));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+240, buf9, strlen(buf9));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+270, buf10, strlen(buf10));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+300, buf11, strlen(buf11));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+330, buf12, strlen(buf12));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+360, buf13, strlen(buf13));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+390, buf14, strlen(buf14));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+420, buf15, strlen(buf15));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+450, buf16, strlen(buf16));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+480, buf17, strlen(buf17));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+510, buf18, strlen(buf18));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+540, buf19, strlen(buf19));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+570, buf20, strlen(buf20));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+600, buf21, strlen(buf21));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+630, buf22, strlen(buf22));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+660, buf23, strlen(buf23));
				XDrawString(MyDisplay, MyWindow, MyGC, 0, DispHead+690, buf24, strlen(buf24));
			}
		}
	}


	XCloseDisplay(MyDisplay);
	printf("close windows\n");
//	timer_delete(timerid);
//	high_stop_flag = 1;
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
	double Encoder_Count_EP2 = 0.0;
	long Encoder_Count_EP_Send2 = 0.0;
	struct timespec sleeptime;
	long count_out[8] = {0};
	long can_id = 0;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;
	int i, j;
	int canRecvStatus[4];


	// receive Motor Driver data
	sleeptime.tv_nsec = 300000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);
	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;

	for(i=0; i<4; i++)
	{
		can_id = 0x80;
		can_send(i, can_id, 0, count_out, 0);   //SYNC
		nanosleep(&sleeptime,NULL);
	}

	sleeptime.tv_nsec = 800000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	for(i=0; i<can_channel_number; i++)
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

	count_out[0] = 0x4;
	count_out[1] = 0x4;
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
		VoltageAll = ((frame.data[0]& 0x1f)*32 + frame.data[1])/1.0;
		CurrentAll = ((frame.data[2]& 0x1f)*32 + frame.data[3])/1.0;
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

void current_position(int channel_num, double Joint_Angle_FB2[4])
{
	struct timespec sleeptime;
	long can_id = 0x80;
	long can_content[8] = {0x23, 0x24, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i;
	int can_node_num = 4;
	int canRecvStatus[4];

	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);
	int ret;
	int Encoder_Count_FB[4];

	sleeptime.tv_nsec = 5000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 5us

	for(i=0; i<12; i++)
	{
		ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}

	can_send(channel_num, can_id, 0, can_content, 0);   //SYNC

	sleeptime.tv_nsec = 1000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);  //wait for 400us

	if(channel_num == 3)
		can_node_num =3;

//		printf("test2\n");

	for(i=0; i<can_node_num; i++)
	{
//		printf("test3\n");
		ret = rt_dev_recvfrom(rxsock[channel_num], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
//		printf("ret= %d\n",ret);
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
			if(channel_num <3)
			{
		//		printf("test4\n");
				Encoder_Count_FB[(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB2[(frame.can_id & 0xf)-1] =(((double)Encoder_Count_FB[(frame.can_id & 0xf)-1])/Rad2Count[channel_num][(frame.can_id & 0xf)-1]/reduction_ratio[channel_num][(frame.can_id & 0xf)-1] - zero_comp[channel_num][(frame.can_id & 0xf)-1]) * joint_direction[channel_num][(frame.can_id & 0xf)-1] ;
		//		Joint_Angle_FB_degree[i][(frame.can_id & 0xf)-1] = Joint_Angle_FB[i][(frame.can_id & 0xf)-1]*Rad2Degree/reduction_ratio;
				Joint_Angle_FB_degree[channel_num][(frame.can_id & 0xf)-1] = Joint_Angle_FB2[(frame.can_id & 0xf)-1]*Rad2Degree;
			//	printf("test5\n");
			}
			else
			{
			//	printf("test6\n");
				Encoder_Count_FB[(frame.can_id & 0xf)-1] = *(int*)frame.data;
				Joint_Angle_FB2[(frame.can_id & 0xf)-1] = (((double)Encoder_Count_FB[(frame.can_id & 0xf)-1])/line_2count[(frame.can_id & 0xf)-1] - zero_comp[channel_num][(frame.can_id & 0xf)-1]) * joint_direction[channel_num][(frame.can_id & 0xf)-1];
				Joint_Angle_FB_degree[channel_num][(frame.can_id & 0xf)-1] = Joint_Angle_FB2[(frame.can_id & 0xf)-1];
			//	printf("test7\n");
			}

			printf("current position %d %d is %02x  %02x	 %02x  %02x  %02x  %02x  %02x  %02x    %f   %d\n",channel_num, (frame.can_id & 0xf)-1, frame.data[0], frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7],  Joint_Angle_FB2[(frame.can_id & 0xf)-1], Encoder_Count_FB[(frame.can_id & 0xf)-1]);
		//	printf("size of int = %d\n", sizeof(int));
		}
		nanosleep(&sleeptime,NULL);
	}
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
	unsigned char * can_byte;
	long Encoder_Count_EP_BYTE[27][6] = {0};
	int canRecvStatus[4];

	double t = 0.0;
//	double time_interval = 0.003;

	int can_connection_status[27] = {0};
	double half_time = 10.0;
	double start_position[4][7] = {0.0};

	double end_position[4][4] = {{-0.510793, 0, 0.701553, 0.0},{0.510793, 0.0, 0.371256, 0.0},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};
	double end_position2[4][4] = {{-0.510793, 0, 0.701553, 0.0},{0.510793, 0.0, 0.371256, 0.0},{pi/10, pi/10, pi/10, pi/10},{pi/10, pi/10, pi/10, pi/10}};


	double T_END_main[4][4] = {{1.0, 0.0, 0.0, 600.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -24.0},
						{0.0, 0.0, 0.0, 1.0}};
	double T_END2[4][4] = {{1.0, 0.0, 0.0, 600.0},
						{0.0, 1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, -24.0},
						{0.0, 0.0, 0.0, 1.0}};

	double T_TURN[4][4] = {{-1.0, 0.0, 0.0, -500.0},
						{0.0, -1.0, 0.0, 0.0},
						{0.0, 0.0, 1.0, 250.0},
						{0.0, 0.0, 0.0, 1.0}};

	int fisrt_time_SINGLE_JOINT_MOTION = 0;
	int fisrt_time_ONE_ARM_MOTION = 0;
	int first_time_TWO_ARMS_MOTION = 0;
	int first_time_HOMEBACK = 0;
	int first_time_VISION_MOTION = 0;
	int first_time_TURN_MOTION = 0;
	int first_time_TURN_MOTION2 = 0;

	double singe_joint_time = 5.0;
	double singe_joint_time2 = 5.0;
	double hand_acc = 0.4;//0.5;
	double hand_vb = 1.5;//2.0;
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
	double anglein2[8] = {0.698, 0.0, 0.151, -pi, -0.698, 0.0, -0.546, 0.0};
	double anglein3[8] = {0.956, 0.0, -0.151, -pi/2.0, -0.956, 0.0, -0.369, 0.0};
	double anglein4[8] = {41.671/Rad2Degree, -1.175/Rad2Degree, 6.661/Rad2Degree, 2.501/Rad2Degree, -41.671/Rad2Degree, 0.025/Rad2Degree, -28.799/Rad2Degree, 1.139/Rad2Degree};
	double angle_out[4][8];
	double angle_out2[8];
	double beta;

	int motion_mode_control = 0;   //when motion_mode_control == 0; motion_mode can be changed

	double angle_2leg[6] = {0.0};
	double delta_matrix2[4][4];
	double timecost = 0.0;

	int return_value = 1;

	sleeptime.tv_nsec = 20000000;
	sleeptime.tv_sec = 0;

	double turn_angle[4] = {0, 10.0/Rad2Degree, 0.0, 5.0/Rad2Degree};

	elmo_init();

	rt_task_set_periodic(NULL, TM_NOW, 6000000);
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
	//	printf("zhouqi = %ld\n", (long)period);
		runtime = runtime + time_interval;

		if(servo_on_flag == 1)
		{
/************************* 四通道CAN数据接收**************************/

			switch (motion_mode)   //switch one joint, or all joint move
			{
				case SINGLE_JOINT_MOTION:

					if(fisrt_time_SINGLE_JOINT_MOTION == 0)
					{
						start_position[can_channel_main][can_id_main] = Joint_Angle_FB[can_channel_main][can_id_main];
						fisrt_time_SINGLE_JOINT_MOTION = 1;
						t = 0;
						printf("MOTION_MODE: SINGLE_JOINT_MOTION\n");
						motion_mode_control = 1;
					}
					else
					{
						if(t <= singe_joint_time)
						{
						/*	t = t+time_interval;
							Joint_Angle_EP[0][0] = 1*sin(0.1256637*t);
							Joint_Angle_EP[0][1] = 0.2*sin(0.1256637*t);
							Joint_Angle_EP[0][2] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[0][3] = 0.25*sin(0.1256637*t);
							Joint_Angle_EP[0][4] = 0.21*sin(0.1256637*t);
							Joint_Angle_EP[0][5] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[0][6] = 0.23*sin(0.1256637*t);
							printf("time = %f\n",t);
							if(t > singe_joint_time)
							{
								Joint_Angle_EP[0][0] = 0;
								Joint_Angle_EP[0][1] = 0;
								Joint_Angle_EP[0][2] = 0;
								Joint_Angle_EP[0][3] = 0;
								Joint_Angle_EP[0][4] = 0;
								Joint_Angle_EP[0][5] = 0;
								Joint_Angle_EP[0][6] = 0;
							}

							if(send_flag_99 == 1)
							{
								rad_send(0,0,Joint_Angle_EP[0][0]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,1,Joint_Angle_EP[0][1]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,2,Joint_Angle_EP[0][2]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,3,Joint_Angle_EP[0][3]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,4,Joint_Angle_EP[0][4]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,5,Joint_Angle_EP[0][5]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(0,6,Joint_Angle_EP[0][6]);
							}	*/

						/*	t = t+time_interval;
							Joint_Angle_EP[1][0] = 1*sin(0.1256637*t);
							Joint_Angle_EP[1][1] = 0.2*sin(0.1256637*t);
							Joint_Angle_EP[1][2] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[1][3] = 0.25*sin(0.1256637*t);
							Joint_Angle_EP[1][4] = 0.21*sin(0.1256637*t);
							Joint_Angle_EP[1][5] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[1][6] = 0.23*sin(0.1256637*t);
							printf("time = %f\n",t);
							if(t > singe_joint_time)
							{
								Joint_Angle_EP[1][0] = 0;
								Joint_Angle_EP[1][1] = 0;
								Joint_Angle_EP[1][2] = 0;
								Joint_Angle_EP[1][3] = 0;
								Joint_Angle_EP[1][4] = 0;
								Joint_Angle_EP[1][5] = 0;
								Joint_Angle_EP[1][6] = 0;
							}

							if(send_flag_99 == 1)
							{
								rad_send(1,0,Joint_Angle_EP[1][0]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,1,Joint_Angle_EP[1][1]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,2,Joint_Angle_EP[1][2]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,3,Joint_Angle_EP[1][3]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,4,Joint_Angle_EP[1][4]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,5,Joint_Angle_EP[1][5]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(1,6,Joint_Angle_EP[1][6]);

							}	*/

						/*	t = t+time_interval;
							Joint_Angle_EP[2][0] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[2][1] = 0.2*sin(0.1256637*t);
							Joint_Angle_EP[2][2] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[2][3] = 0.25*sin(0.1256637*t);
							Joint_Angle_EP[2][4] = 0.21*sin(0.1256637*t);
							Joint_Angle_EP[2][5] = 0.3*sin(0.1256637*t);
							printf("time = %f\n",t);
							if(t > singe_joint_time)
							{
								Joint_Angle_EP[2][0] = 0;
								Joint_Angle_EP[2][1] = 0;
								Joint_Angle_EP[2][2] = 0;
								Joint_Angle_EP[2][3] = 0;
								Joint_Angle_EP[2][4] = 0;
								Joint_Angle_EP[2][5] = 0;
							}

							if(send_flag_99 == 1)
							{
								rad_send(2,0,Joint_Angle_EP[2][0]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2,1,Joint_Angle_EP[2][1]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2,2,Joint_Angle_EP[2][2]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2,3,Joint_Angle_EP[2][3]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2,4,Joint_Angle_EP[2][4]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(2,5,Joint_Angle_EP[2][5]);

							}	*/

						/*	t = t+time_interval;
							Joint_Angle_EP[3][0] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[3][1] = 0.2*sin(0.1256637*t);
							Joint_Angle_EP[3][2] = 0.3*sin(0.1256637*t);
							Joint_Angle_EP[3][3] = 0.25*sin(0.1256637*t);
							Joint_Angle_EP[3][4] = 0.21*sin(0.1256637*t);
							Joint_Angle_EP[3][5] = 0.3*sin(0.1256637*t);
							printf("time = %f\n",t);
							if(t > singe_joint_time)
							{
								Joint_Angle_EP[3][0] = 0;
								Joint_Angle_EP[3][1] = 0;
								Joint_Angle_EP[3][2] = 0;
								Joint_Angle_EP[3][3] = 0;
								Joint_Angle_EP[3][4] = 0;
								Joint_Angle_EP[3][5] = 0;
							}

							if(send_flag_99 == 1)
							{
								rad_send(3,0,Joint_Angle_EP[3][0]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3,1,Joint_Angle_EP[3][1]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3,2,Joint_Angle_EP[3][2]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3,3,Joint_Angle_EP[3][3]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3,4,Joint_Angle_EP[3][4]);
								sleeptime.tv_nsec = 400000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
								rad_send(3,5,Joint_Angle_EP[3][5]);

							}	*/

							t = t+time_interval;
							Joint_Angle_EP[can_channel_main][can_id_main] = Five_Interpolation(start_position[can_channel_main][can_id_main],0,0,start_position[can_channel_main][can_id_main]+JointMoveData,0,0,singe_joint_time,t);

							if(t > singe_joint_time)
							{
								Joint_Angle_EP[can_channel_main][can_id_main] = start_position[can_channel_main][can_id_main]+JointMoveData;
							}

							if(send_flag_99 == 1)
							{
								rad_send(can_channel_main,can_id_main,Joint_Angle_EP[can_channel_main][can_id_main]);
							}

						}
						else
						{
							t = 0;
							motion_mode = 100;
							fisrt_time_SINGLE_JOINT_MOTION = 0;
							motion_mode_control =0;
						}

					}


				break;

				case FIND_HOME_MOTION:

					find_home(can_channel_main,can_id_main);
					motion_mode = 100;
					printf("MOTION_MODE: FIND_HOME_MOTION\n");
			//		canrv_running = 0;

				break;

				case ONE_ARM_MOTION:

					if(fisrt_time_ONE_ARM_MOTION == 0)
					{
						for(i=0; i<4; i++)
						{
							start_position[can_channel_main][i] = Joint_Angle_FB[can_channel_main][i];
						}
						fisrt_time_ONE_ARM_MOTION = 1;
						t = 0;
						printf("MOTION_MODE: ONE_ARM_MOTION\n");
						motion_mode_control = 1;
					}
					else
					{
						if(t <= one_arm_time)
						{
							t = t+time_interval;
							for(i=0; i<4; i++)
							{

								if(can_channel_main <3)
								{
									Joint_Angle_EP[can_channel_main][i] = Five_Interpolation(start_position[can_channel_main][i], 0, 0, start_position[can_channel_main][i] + One_arm_main[i], 0, 0, one_arm_time,t);
								}

								if(t > one_arm_time)
								{
									Joint_Angle_EP[can_channel_main][i] = start_position[can_channel_main][i]+One_arm_main[i];
								}

								if(send_flag_99 == 1)
								{
									rad_send(can_channel_main, i, Joint_Angle_EP[can_channel_main][i]);
								}

								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							t = 0;
							motion_mode = 100;
							fisrt_time_ONE_ARM_MOTION = 0;
							motion_mode_control	= 0;
						}
					}
				break;

				case TWO_ARMS_MOTION:

					if(first_time_TWO_ARMS_MOTION == 0)
					{
				/*		Joint_Angle_FB[2][0] = 0.5;
						Joint_Angle_FB[2][1] = 0.0;
						Joint_Angle_FB[2][2] = 0.2;
						Joint_Angle_FB[2][3] = 0.0;

						Joint_Angle_FB[0][0] = 0.0;
						Joint_Angle_FB[0][1] = 0.0;
						Joint_Angle_FB[0][2] = 0.0;
						Joint_Angle_FB[0][3] = 0.0;

						Joint_Angle_FB[1][0] = -0.5;
						Joint_Angle_FB[1][1] = 0.0;
						Joint_Angle_FB[1][2] = -0.2;
						Joint_Angle_FB[1][3] = 0.0;
					*/
						for(i=0; i<3; i++)
						{
							for(j=0; j<4; j++)
							{
								start_position[i][j] = Joint_Angle_FB[i][j];
							}
						}

						printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f    %f   %f   %f   %f\n", start_position[0][0]*Rad2Degree,start_position[0][1]*Rad2Degree,start_position[0][2]*Rad2Degree,start_position[0][3]*Rad2Degree,start_position[1][0]*Rad2Degree, start_position[1][1]*Rad2Degree, start_position[1][2]*Rad2Degree,start_position[1][3]*Rad2Degree, start_position[2][0]*Rad2Degree, start_position[2][1]*Rad2Degree, start_position[2][2]*Rad2Degree,start_position[2][3]*Rad2Degree);

						motion_mode_control	= 1;

						printf("MOTION_MODE: TWO_ARMS_MOTION\n");


						matrix_multiply(T_END_main, T_hand_end, T_Now2);
				//		InvCham(T_Now2, anglein, 0.0, angle_out);
				//		joint_angle_last[i] =

				//		InvCham(T_Now2, joint_angle_last, 0.0, angle_out2);

					//	InvCham(T_END_main, anglein, 0.0, angle_out);

						printf("T_END_main = \n");

						for(i=0;i<4;i++)
						{
							for(j=0; j<4; j++)
							{
								printf("%f   ", T_END_main[i][j]);
							}
							printf("\n");
						}

						printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f\n", angle_out2[0]*Rad2Degree, angle_out2[1]*Rad2Degree, angle_out2[2]*Rad2Degree, angle_out2[3]*Rad2Degree, angle_out2[4]*Rad2Degree, angle_out2[5]*Rad2Degree, angle_out2[6]*Rad2Degree, angle_out2[7]*Rad2Degree);

						printf("\n");

						if(End_Numb == ARM1)
						{
					/*		anglein[0] =  start_position[1][0];
							anglein[1] =  start_position[1][1];
							anglein[2] =  start_position[1][2];
							anglein[3] =  start_position[1][3];
							anglein[4] =  start_position[0][0];
							anglein[5] =  start_position[0][1];
							anglein[6] =  start_position[0][2];
							anglein[7] =  start_position[0][3];
				*/
							InvCham(T_Now2, anglein, 0.0, angle_out2);


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
							printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree, end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);
						}
						else if(End_Numb == ARM2)
						{
					/*		anglein[0] =  start_position[2][0];
							anglein[1] =  start_position[2][1];
							anglein[2] =  start_position[2][2];
							anglein[3] =  start_position[2][3];
							anglein[4] =  start_position[1][0];
							anglein[5] =  start_position[1][1];
							anglein[6] =  start_position[1][2];
							anglein[7] =  start_position[1][3];
				*/
							InvCham(T_Now2, anglein, 0.0, angle_out2);

							end_position[2][0] = angle_out2[0];
							end_position[2][1] = angle_out2[1];
							end_position[2][2] = angle_out2[2];
							end_position[2][3] = angle_out2[3];

							end_position[1][0] = angle_out2[4];
							end_position[1][1] = angle_out2[5];
							end_position[1][2] = angle_out2[6];
							end_position[1][3] = angle_out2[7];

							end_position[0][0] = start_position[0][0];
							end_position[0][1] = start_position[0][1];
							end_position[0][2] = start_position[0][2];
							end_position[0][3] = start_position[0][3];
							printf("ARM2 end\n");
							printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);
						}
						else if(End_Numb == ARM3)
						{

					/*		anglein[0] =  start_position[0][0];
							anglein[1] =  start_position[0][1];
							anglein[2] =  start_position[0][2];
							anglein[3] =  start_position[0][3];
							anglein[4] =  start_position[2][0];
							anglein[5] =  start_position[2][1];
							anglein[6] =  start_position[2][2];
							anglein[7] =  start_position[2][3];
				*/
							InvCham(T_Now2, anglein, 0.0, angle_out2);

							end_position[0][0] = angle_out2[0];
							end_position[0][1] = angle_out2[1];
							end_position[0][2] = angle_out2[2];
							end_position[0][3] = angle_out2[3];

							end_position[2][0] = angle_out2[4];
							end_position[2][1] = angle_out2[5];
							end_position[2][2] = angle_out2[6];
							end_position[2][3] = angle_out2[7];

							end_position[1][0] = start_position[1][0];
							end_position[1][1] = start_position[1][1];
							end_position[1][2] = start_position[1][2];
							end_position[1][3] = start_position[1][3];
							printf("ARM3 end\n");
							printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);
						}

						first_time_TWO_ARMS_MOTION = 1;
						t = 0;
					}
					else
					{
						if(t <= two_arms_time)
						{
							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{

									if(i <3)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position[i][j],0,0,two_arms_time,t);
									}

									if(t > two_arms_time)
									{
										Joint_Angle_EP[i][j] = end_position[i][j];
									}

									if(send_flag_99 == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_TWO_ARMS_MOTION = 0;
							motion_mode_control	= 0;
						}
					}

					break;

					case TURN_MOTION:

					if(first_time_TURN_MOTION == 0)
					{
						for(i=0; i<3; i++)
						{
							for(j=0; j<4; j++)
							{
								start_position[i][j] = Joint_Angle_FB[i][j];
							}
						}

						printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f    %f   %f   %f   %f\n", start_position[0][0]*Rad2Degree,start_position[0][1]*Rad2Degree,start_position[0][2]*Rad2Degree,start_position[0][3]*Rad2Degree,start_position[1][0]*Rad2Degree, start_position[1][1]*Rad2Degree, start_position[1][2]*Rad2Degree,start_position[1][3]*Rad2Degree, start_position[2][0]*Rad2Degree, start_position[2][1]*Rad2Degree, start_position[2][2]*Rad2Degree,start_position[2][3]*Rad2Degree);

						motion_mode_control	= 1;
				/*		matrix_multiply(T_TURN, T_hand_end, T_Now2);

						printf("\nT_Now2 = ");
							for(i=0; i<4; i++)
							{
								for(j=0; j<4;j++)
									printf("%f   ",T_Now2[i][j]);
								printf("\n");
							}
						*/
				/*
						start_position[2][0] = 0.70763;
						start_position[2][1] = -0.0;
						start_position[2][2] = 0.2;
						start_position[2][3] = -0.0;
						start_position[1][0] = -0.70763;
						start_position[1][1] = -0;
						start_position[1][2] = -0.478664;
						start_position[1][3] = -0.0;
						start_position[0][1] = 20.0/Rad2Degree;
				*/
						printf("MOTION_MODE: TURN_MOTION\n");


				//		InvCham(T_Now2, anglein2, 0.0, angle_out2);

						end_position[2][0] = anglein2[0];
						end_position[2][1] = anglein2[1];
						end_position[2][2] = anglein2[2];
						end_position[2][3] = anglein2[3];

						end_position[1][0] = anglein2[4];
						end_position[1][1] = anglein2[5];
						end_position[1][2] = anglein2[6];
						end_position[1][3] = anglein2[7];

						end_position2[2][0] = anglein3[0];
						end_position2[2][1] = anglein3[1];
						end_position2[2][2] = anglein3[2];
						end_position2[2][3] = anglein3[3] ;

						end_position2[1][0] = anglein3[4];
						end_position2[1][1] = anglein3[5] ;
						end_position2[1][2] = anglein3[6] ;
						end_position2[1][3] = anglein3[7] ;


						end_position[0][0] = start_position[0][0];
						end_position[0][1] = start_position[0][1];
						end_position[0][2] = start_position[0][2];
						end_position[0][3] = start_position[0][3];

						end_position2[0][0] = start_position[0][0];
						end_position2[0][1] = start_position[0][1];
						end_position2[0][2] = start_position[0][2];
						end_position2[0][3] = start_position[0][3];


						printf("ARM2 end\n");
						printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);


						first_time_TURN_MOTION = 1;
						t = 0;
					}
					else
					{
						if(t <= turn_time/2.0)
						{
							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{


									Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position2[i][j],0,0,turn_time/2.0,t);
									if(i==2 && j==3)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position2[i][j],-pi/turn_time,0,turn_time/2.0,t);
									}


									if(t > turn_time/2.0)
									{
										Joint_Angle_EP[i][j] = end_position2[i][j];
									}

									if(send_flag_99 == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else if(t < turn_time)
						{
							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{


										Joint_Angle_EP[i][j] = Five_Interpolation(end_position2[i][j],0,0, end_position[i][j],0,0,turn_time/2.0,t-turn_time/2.0);
									if(i==2 && j==3)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(end_position2[i][j],-pi/turn_time,0, end_position[i][j],0,0,turn_time/2.0,t-turn_time/2.0);
									}

									if(t > turn_time)
									{
										Joint_Angle_EP[i][j] = end_position[i][j];
									}

									if(send_flag_99 == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_TURN_MOTION = 0;
							motion_mode_control	= 0;
						}
					}

					break;


					case TURN_MOTION2:

					if(first_time_TURN_MOTION2 == 0)
					{
						for(i=0; i<3; i++)
						{
							for(j=0; j<4; j++)
							{
								start_position[i][j] = Joint_Angle_FB[i][j];
							}
						}

						printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f    %f   %f   %f   %f\n", start_position[0][0]*Rad2Degree,start_position[0][1]*Rad2Degree,start_position[0][2]*Rad2Degree,start_position[0][3]*Rad2Degree,start_position[1][0]*Rad2Degree, start_position[1][1]*Rad2Degree, start_position[1][2]*Rad2Degree,start_position[1][3]*Rad2Degree, start_position[2][0]*Rad2Degree, start_position[2][1]*Rad2Degree, start_position[2][2]*Rad2Degree,start_position[2][3]*Rad2Degree);

						motion_mode_control	= 1;
				/*		matrix_multiply(T_TURN, T_hand_end, T_Now2);

						printf("\nT_Now2 = ");
							for(i=0; i<4; i++)
							{

								for(j=0; j<4;j++)
									printf("%f   ",T_Now2[i][j]);
								printf("\n");
							}

						*/
				/*
						start_position[2][0] = 0.698;
						start_position[2][1] = -0.0;
						start_position[2][2] = 0.151;
						start_position[2][3] = -pi;

						start_position[1][0] = -0.698;
						start_position[1][1] = -0;
						start_position[1][2] = -0.546;
						start_position[1][3] = -0.0;

						start_position[0][0] = 20.0/Rad2Degree;
					*/

						printf("MOTION_MODE: TURN_MOTION2\n");


				//		InvCham(T_Now2, anglein2, 0.0, angle_out2);

						end_position[2][0] = anglein4[0];
						end_position[2][1] = anglein4[1];
						end_position[2][2] = anglein4[2];
						end_position[2][3] = anglein4[3];

						end_position[1][0] = anglein4[4];
						end_position[1][1] = anglein4[5];
						end_position[1][2] = anglein4[6];
						end_position[1][3] = anglein4[7];

						end_position2[2][0] = anglein3[0];
						end_position2[2][1] = anglein3[1];
						end_position2[2][2] = anglein3[2];
						end_position2[2][3] = anglein3[3] ;

						end_position2[1][0] = anglein3[4];
						end_position2[1][1] = anglein3[5] ;
						end_position2[1][2] = anglein3[6] ;
						end_position2[1][3] = anglein3[7] ;


						end_position[0][0] = start_position[0][0];
						end_position[0][1] = start_position[0][1];
						end_position[0][2] = start_position[0][2];
						end_position[0][3] = start_position[0][3];

						end_position2[0][0] = start_position[0][0];
						end_position2[0][1] = start_position[0][1];
						end_position2[0][2] = start_position[0][2];
						end_position2[0][3] = start_position[0][3];


						printf("ARM2 end\n");
						printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);


						first_time_TURN_MOTION2 = 1;
						t = 0;
					}
					else
					{
						if(t <= turn_time/2.0)
						{
							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{


									Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position2[i][j],0,0,turn_time/2.0,t);
									if(i==2 && j==3)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0, end_position2[i][j],pi/turn_time,0,turn_time/2.0,t);
									}


									if(t > turn_time/2.0)
									{
										Joint_Angle_EP[i][j] = end_position2[i][j];
									}

									if(send_flag_99 == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else if(t < turn_time)
						{
							t = t + time_interval;
							for(j=0; j<4; j++)
							{
								for(i=0; i<3; i++)
								{


										Joint_Angle_EP[i][j] = Five_Interpolation(end_position2[i][j],0,0, end_position[i][j],0,0,turn_time/2.0,t-turn_time/2.0);
									if(i==2 && j==3)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(end_position2[i][j],pi/turn_time,0, end_position[i][j],0,0,turn_time/2.0,t-turn_time/2.0);
									}

									if(t > turn_time)
									{
										Joint_Angle_EP[i][j] = end_position[i][j];
									}

									if(send_flag_99 == 1)
									{
										rad_send(i,j,Joint_Angle_EP[i][j]);
									}
									sleeptime.tv_nsec = 5000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
								sleeptime.tv_nsec = 250000;
								sleeptime.tv_sec = 0;
								nanosleep(&sleeptime,NULL);
							}
						}
						else
						{
							t = 0;
							motion_mode = 100;
							first_time_TURN_MOTION2 = 0;
							motion_mode_control	= 0;
						}
					}

					break;



					case HOMEBACK:

						if(first_time_HOMEBACK == 0)
			 			{
			 				first_time_HOMEBACK = 1;
							t = 0.0;
							motion_mode_control	= 1;

							for(i=0; i<3; i++)
							{
								for(j=0; j<4; j++)
								{
									start_position[i][j] = Joint_Angle_FB[i][j];
								}
							}
							printf("MOTION_MODE = HOMEBACK\n");
			 			}
						else
						{
							if(t < homeback_time)
							{


								t = t + time_interval;
								for(j=0; j<4; j++)
								{
									for(i=0; i<3; i++)
									{
										Joint_Angle_EP[i][j] = Five_Interpolation(start_position[i][j],0,0,home_offset[i][j],0,0,homeback_time,t);

										if(t > homeback_time)
										{
											Joint_Angle_EP[i][j] = home_offset[i][j];
										}

										if(send_flag_99 == 1)
										{
											rad_send(i,j,Joint_Angle_EP[i][j]);
										}
										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);  //wait for 50us
									}
									sleeptime.tv_nsec = 250000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
							}
							else
							{
								first_time_HOMEBACK = 0;
								t = 0;
								motion_mode = 100;
								motion_mode_control	= 0;

								printf("clear zero\n");
				 				for(i=0; i<4; i++)
				 				{
				 					for(j=0; j<4; j++)
				 					{
				 						zero_comp[i][j] = home_offset[i][j] * joint_direction[i][j];
				 					}
				 				}
							}
						}
			 		break;

			 		case FIND_HOME_NEW:
			 			return_value = find_home_new();
			 	//		printf("return_value = %d\n", return_value );
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 			}
			 		break;

			 		case RETURN_ORIGIN_POSITION:

			 			return_value = return_origin_position();
			 		//	printf("return_value = %d\n", return_value);
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 			}
			 		break;

			 		case PREPARE_FIND_HOME:

			 			return_value = prepare_find_home();
			 			if(return_value == 0)
			 			{
			 				motion_mode = 100;
			 			}
			 		break;


					case VISION_MOTION:

						if(first_time_VISION_MOTION == 0)
						{
							first_time_VISION_MOTION = 1;
							motion_mode_control	= 1;
							t = 0.0;
							printf("MOTION_MODE = VISION_MOTION\n");

					/*		Joint_Angle_FB[2][0] = 40.793/Rad2Degree;
							Joint_Angle_FB[2][1] = 0.0;
							Joint_Angle_FB[2][2] = 13.496/Rad2Degree;
							Joint_Angle_FB[2][3] = 0.0;

							Joint_Angle_FB[0][0] = 0;
							Joint_Angle_FB[0][1] = 0.0;
							Joint_Angle_FB[0][2] = 0;
							Joint_Angle_FB[0][3] = 0.0;

							Joint_Angle_FB[1][0] = -40.793/Rad2Degree;
							Joint_Angle_FB[1][1] = 0.0;
							Joint_Angle_FB[1][2] = -24.917/Rad2Degree;
							Joint_Angle_FB[1][3] = 0.0;
						*/
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
							else if(End_Numb == ARM2)
							{
								angle_2leg[0] = start_position[2][0];
								angle_2leg[1] = start_position[2][1];
								angle_2leg[2] = start_position[2][2];
								angle_2leg[3] = start_position[2][3];

								angle_2leg[4] = start_position[1][0];
								angle_2leg[5] = start_position[1][1];
								angle_2leg[6] = start_position[1][2];
								angle_2leg[7] = start_position[1][3];
								printf("ARM2 end\n");
								printf("present agnle  %f   %f   %f   %f   %f   %f   %f   %f\n", angle_2leg[0]*Rad2Degree, angle_2leg[1]*Rad2Degree, angle_2leg[2]*Rad2Degree, angle_2leg[3]*Rad2Degree, angle_2leg[4]*Rad2Degree, angle_2leg[5]*Rad2Degree, angle_2leg[6]*Rad2Degree, angle_2leg[7]*Rad2Degree);
							}
							else if(End_Numb == ARM3)
							{
								angle_2leg[0] = start_position[0][0];
								angle_2leg[1] = start_position[0][1];
								angle_2leg[2] = start_position[0][2];
								angle_2leg[3] = start_position[0][3];

								angle_2leg[4] = start_position[2][0];
								angle_2leg[5] = start_position[2][1];
								angle_2leg[6] = start_position[2][2];
								angle_2leg[7] = start_position[2][3];

								printf("ARM3 end\n");
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
					//		matrix_multiply(T_Now, delta_matrix2, T_END2);

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

					//		InvCham(T_END2, anglein, 0.0, angle_out);
					//		InvCham(T_Now2, joint_angle_last, 0.0, angle_out2);

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
							else if(End_Numb == ARM2)
							{


								end_position[2][0] = angle_out2[0];
								end_position[2][1] = angle_out2[1];
								end_position[2][2] = angle_out2[2];
								end_position[2][3] = angle_out2[3];

								end_position[1][0] = angle_out2[4];
								end_position[1][1] = angle_out2[5];
								end_position[1][2] = angle_out2[6];
								end_position[1][3] = angle_out2[7];

								end_position[0][0] = start_position[0][0];
								end_position[0][1] = start_position[0][1];
								end_position[0][2] = start_position[0][2];
								end_position[0][3] = start_position[0][3];

								printf("ARM2 end\n");
								printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree,end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree,end_position[2][3]*Rad2Degree);

								angle_2leg[0] = angle_out2[0];
								angle_2leg[1] = angle_out2[1];
								angle_2leg[2] = angle_out2[2];
								angle_2leg[3] = angle_out2[3];

								angle_2leg[4] = angle_out2[4];
								angle_2leg[5] = angle_out2[5];
								angle_2leg[6] = angle_out2[6];
								angle_2leg[7] = angle_out2[7];

								KinTwoLegR(angle_2leg,  T_Now);
								matrix_multiply(T_Now, T_hand_end_inv, T_Now1);

								printf("\nT_expcet = ");
								for(i=0; i<4; i++)
								{
									for(j=0; j<4;j++)
										printf("%f   ",T_Now1[i][j]);
									printf("\n");
								}

							}
							else if(End_Numb == ARM3)
							{
								end_position[0][0] = angle_out2[0];
								end_position[0][1] = angle_out2[1];
								end_position[0][2] = angle_out2[2];
								end_position[0][3] = angle_out2[3];

								end_position[2][0] = angle_out2[4];
								end_position[2][1] = angle_out2[5];
								end_position[2][2] = angle_out2[6];
								end_position[2][3] = angle_out2[7];

								end_position[1][0] = start_position[1][0];
								end_position[1][1] = start_position[1][1];
								end_position[1][2] = start_position[1][2];
								end_position[1][3] = start_position[1][3];

								printf("ARM3 end\n");
								printf("expect agnle  %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", end_position[0][0]*Rad2Degree,end_position[0][1]*Rad2Degree,end_position[0][2]*Rad2Degree,end_position[0][3]*Rad2Degree,end_position[1][0]*Rad2Degree, end_position[1][1]*Rad2Degree, end_position[1][2]*Rad2Degree, end_position[1][3]*Rad2Degree, end_position[2][0]*Rad2Degree, end_position[2][1]*Rad2Degree, end_position[2][2]*Rad2Degree, end_position[2][3]*Rad2Degree);
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

										if(send_flag_99 == 1)
										{
											rad_send(i,j,Joint_Angle_EP[i][j]);
										}

										sleeptime.tv_nsec = 5000;
										sleeptime.tv_sec = 0;
										nanosleep(&sleeptime,NULL);
									}
									sleeptime.tv_nsec = 250000;
									sleeptime.tv_sec = 0;
									nanosleep(&sleeptime,NULL);
								}
							}
							else
							{
								t = 0;
								motion_mode = 100;
								first_time_VISION_MOTION = 0;
								motion_mode_control	= 0;
							}
						}

					break;

				default:
					break;

			}

			if(servo_on_off_flag == 0)
			{
				SYNC_Receive();
		//		printf("servo_on_off_flag = %d\n", servo_on_off_flag);
		//		printf("sync 0x80\n");

			}
		}

		/********************** UDP通讯相关 Start ***************************/
		UDPTimes++;
		if(UDPTimes >= UDPCYCLE)
		{
			UDPTimes = 0;

			// 发送数据帧赋值
			UploadData.TrustFlag = 0x5555;
			if(send_flag_99)
			{
				UploadData.LeftArmAngle[0] = Joint_Angle_FB[2][0];
				UploadData.LeftArmAngle[1] = Joint_Angle_FB[3][0];
				UploadData.LeftArmAngle[2] = Joint_Angle_FB[3][1];
				UploadData.LeftArmAngle[3] = Joint_Angle_FB[2][1];
				UploadData.LeftArmAngle[4] = Joint_Angle_FB[0][4];
				UploadData.LeftArmAngle[5] = Joint_Angle_FB[0][5];
				UploadData.LeftArmAngle[6] = Joint_Angle_FB[0][6];

				UploadData.RightArmAngle[0] = Joint_Angle_FB[2][2];
				UploadData.RightArmAngle[1] = Joint_Angle_FB[3][2];
				UploadData.RightArmAngle[2] = Joint_Angle_FB[3][3];
				UploadData.RightArmAngle[3] = Joint_Angle_FB[2][3];
				UploadData.RightArmAngle[4] = Joint_Angle_FB[1][4];
				UploadData.RightArmAngle[5] = Joint_Angle_FB[1][5];
				UploadData.RightArmAngle[6] = Joint_Angle_FB[1][6];

				UploadData.LeftHandAngle[0] = Joint_Angle_FB[0][0];
				UploadData.LeftHandAngle[1] = Joint_Angle_FB[0][1];
				UploadData.LeftHandAngle[2] = Joint_Angle_FB[0][2];
				UploadData.LeftHandAngle[3] = Joint_Angle_FB[0][3];

				UploadData.RightHandAngle[0] = Joint_Angle_FB[1][0];
				UploadData.RightHandAngle[1] = Joint_Angle_FB[1][1];
				UploadData.RightHandAngle[2] = Joint_Angle_FB[1][2];
				UploadData.RightHandAngle[3] = Joint_Angle_FB[1][3];

				UploadData.HeadAngle[0] = Joint_Angle_FB[2][4];
				UploadData.HeadAngle[1] = Joint_Angle_FB[2][5];

				UploadData.WaistAngle[0] = Joint_Angle_FB[3][4];
				UploadData.WaistAngle[1] = Joint_Angle_FB[3][5];
			}
			else
			{
				UploadData.LeftArmAngle[0] = Joint_Angle_EP[2][0];
				UploadData.LeftArmAngle[1] = Joint_Angle_EP[3][0];
				UploadData.LeftArmAngle[2] = Joint_Angle_EP[3][1];
				UploadData.LeftArmAngle[3] = Joint_Angle_EP[2][1];
				UploadData.LeftArmAngle[4] = Joint_Angle_EP[0][4];
				UploadData.LeftArmAngle[5] = Joint_Angle_EP[0][5];
				UploadData.LeftArmAngle[6] = Joint_Angle_EP[0][6];

				UploadData.RightArmAngle[0] = Joint_Angle_EP[2][2];
				UploadData.RightArmAngle[1] = Joint_Angle_EP[3][2];
				UploadData.RightArmAngle[2] = Joint_Angle_EP[3][3];
				UploadData.RightArmAngle[3] = Joint_Angle_EP[2][3];
				UploadData.RightArmAngle[4] = Joint_Angle_EP[1][4];
				UploadData.RightArmAngle[5] = Joint_Angle_EP[1][5];
				UploadData.RightArmAngle[6] = Joint_Angle_EP[1][6];

				UploadData.LeftHandAngle[0] = Joint_Angle_EP[0][0];
				UploadData.LeftHandAngle[1] = Joint_Angle_EP[0][1];
				UploadData.LeftHandAngle[2] = Joint_Angle_EP[0][2];
				UploadData.LeftHandAngle[3] = Joint_Angle_EP[0][3];

				UploadData.RightHandAngle[0] = Joint_Angle_EP[1][0];
				UploadData.RightHandAngle[1] = Joint_Angle_EP[1][1];
				UploadData.RightHandAngle[2] = Joint_Angle_EP[1][2];
				UploadData.RightHandAngle[3] = Joint_Angle_EP[1][3];

				UploadData.HeadAngle[0] = Joint_Angle_EP[2][4];
				UploadData.HeadAngle[1] = Joint_Angle_EP[2][5];

				UploadData.WaistAngle[0] = Joint_Angle_EP[3][4];
				UploadData.WaistAngle[1] = Joint_Angle_EP[3][5];
			}


			UploadData.LeftArmCurrent[0] = motor_current[2][0];
			UploadData.LeftArmCurrent[1] = motor_current[3][0];
			UploadData.LeftArmCurrent[2] = motor_current[3][1];
			UploadData.LeftArmCurrent[3] = motor_current[2][1];
			UploadData.LeftArmCurrent[4] = motor_current[0][4];
			UploadData.LeftArmCurrent[5] = motor_current[0][5];
			UploadData.LeftArmCurrent[6] = motor_current[0][6];

			UploadData.RightArmCurrent[0] = motor_current[2][2];
			UploadData.RightArmCurrent[1] = motor_current[3][2];
			UploadData.RightArmCurrent[2] = motor_current[3][3];
			UploadData.RightArmCurrent[3] = motor_current[2][3];
			UploadData.RightArmCurrent[4] = motor_current[1][4];
			UploadData.RightArmCurrent[5] = motor_current[1][5];
			UploadData.RightArmCurrent[6] = motor_current[1][6];

			UploadData.LeftHandCurrent[0] = motor_current[0][0];
			UploadData.LeftHandCurrent[1] = motor_current[0][1];
			UploadData.LeftHandCurrent[2] = motor_current[0][2];
			UploadData.LeftHandCurrent[3] = motor_current[0][3];

			UploadData.RightHandCurrent[0] = motor_current[1][0];
			UploadData.RightHandCurrent[1] = motor_current[1][1];
			UploadData.RightHandCurrent[2] = motor_current[1][2];
			UploadData.RightHandCurrent[3] = motor_current[1][3];

			UploadData.HeadCurrent[0] = motor_current[2][4];
			UploadData.HeadCurrent[1] = motor_current[2][5];

			UploadData.WaistCurrent[0] = motor_current[3][4];
			UploadData.WaistCurrent[1] = motor_current[3][5];


			UploadData.LeftArmCurrent[0] = motor_current[2][0];
			UploadData.LeftArmCurrent[1] = motor_current[3][0];
			UploadData.LeftArmCurrent[2] = motor_current[3][1];
			UploadData.LeftArmCurrent[3] = motor_current[2][1];
			UploadData.LeftArmCurrent[4] = motor_current[0][4];
			UploadData.LeftArmCurrent[5] = motor_current[0][5];
			UploadData.LeftArmCurrent[6] = motor_current[0][6];

			UploadData.RightArmCurrent[0] = motor_current[2][2];
			UploadData.RightArmCurrent[1] = motor_current[3][2];
			UploadData.RightArmCurrent[2] = motor_current[3][3];
			UploadData.RightArmCurrent[3] = motor_current[2][3];
			UploadData.RightArmCurrent[4] = motor_current[1][4];
			UploadData.RightArmCurrent[5] = motor_current[1][5];
			UploadData.RightArmCurrent[6] = motor_current[1][6];

			UploadData.LeftHandCurrent[0] = motor_current[0][0];
			UploadData.LeftHandCurrent[1] = motor_current[0][1];
			UploadData.LeftHandCurrent[2] = motor_current[0][2];
			UploadData.LeftHandCurrent[3] = motor_current[0][3];

			UploadData.RightHandCurrent[0] = motor_current[1][0];
			UploadData.RightHandCurrent[1] = motor_current[1][1];
			UploadData.RightHandCurrent[2] = motor_current[1][2];
			UploadData.RightHandCurrent[3] = motor_current[1][3];

			UploadData.HeadCurrent[0] = motor_current[2][4];
			UploadData.HeadCurrent[1] = motor_current[2][5];

			UploadData.WaistCurrent[0] = motor_current[3][4];
			UploadData.WaistCurrent[1] = motor_current[3][5];


			UploadData.LeftArmAngleEX[0] = Joint_Angle_EP[2][0];
			UploadData.LeftArmAngleEX[1] = Joint_Angle_EP[3][0];
			UploadData.LeftArmAngleEX[2] = Joint_Angle_EP[3][1];
			UploadData.LeftArmAngleEX[3] = Joint_Angle_EP[2][1];
			UploadData.LeftArmAngleEX[4] = Joint_Angle_EP[0][4];
			UploadData.LeftArmAngleEX[5] = Joint_Angle_EP[0][5];
			UploadData.LeftArmAngleEX[6] = Joint_Angle_EP[0][6];

			UploadData.RightArmAngleEX[0] = Joint_Angle_EP[2][2];
			UploadData.RightArmAngleEX[1] = Joint_Angle_EP[3][2];
			UploadData.RightArmAngleEX[2] = Joint_Angle_EP[3][3];
			UploadData.RightArmAngleEX[3] = Joint_Angle_EP[2][3];
			UploadData.RightArmAngleEX[4] = Joint_Angle_EP[1][4];
			UploadData.RightArmAngleEX[5] = Joint_Angle_EP[1][5];
			UploadData.RightArmAngleEX[6] = Joint_Angle_EP[1][6];

			UploadData.LeftHandAngleEX[0] = Joint_Angle_EP[0][0];
			UploadData.LeftHandAngleEX[1] = Joint_Angle_EP[0][1];
			UploadData.LeftHandAngleEX[2] = Joint_Angle_EP[0][2];
			UploadData.LeftHandAngleEX[3] = Joint_Angle_EP[0][3];

			UploadData.RightHandAngleEX[0] = Joint_Angle_EP[1][0];
			UploadData.RightHandAngleEX[1] = Joint_Angle_EP[1][1];
			UploadData.RightHandAngleEX[2] = Joint_Angle_EP[1][2];
			UploadData.RightHandAngleEX[3] = Joint_Angle_EP[1][3];

			UploadData.HeadAngleEX[0] = Joint_Angle_EP[2][4];
			UploadData.HeadAngleEX[1] = Joint_Angle_EP[2][5];

			UploadData.WaistAngleEX[0] = Joint_Angle_EP[3][4];
			UploadData.WaistAngleEX[1] = Joint_Angle_EP[3][5];

			UDPSend();
			UDPRecv(motion_mode_control,&can_channel_main,&can_id_main,&JointMoveData);

		}

				

		/********************** UDP通讯相关 End ***************************/

		/************************* 界面显示 **************************/
		sprintf(buf2, "Voltage:  %8.3f    Current:  %8.3f",VoltageAll,CurrentAll);
		sprintf(buf3, "******************************  Robot Joint Angle  ******************************");
		sprintf(buf4, "CH1 Recv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_FB_degree[0][0], Joint_Angle_FB_degree[0][1], Joint_Angle_FB_degree[0][2], Joint_Angle_FB_degree[0][3], Joint_Angle_FB_degree[0][4], Joint_Angle_FB_degree[0][5], Joint_Angle_FB_degree[0][6]);

		sprintf(buf5, "CH1 Send  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_EP_degree[0][0], Joint_Angle_EP_degree[0][1], Joint_Angle_EP_degree[0][2], Joint_Angle_EP_degree[0][3], Joint_Angle_EP_degree[0][4], Joint_Angle_EP_degree[0][5], Joint_Angle_EP_degree[0][6]);

		sprintf(buf6, "CH1 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_ER_degree[0][0], Joint_Angle_ER_degree[0][1], Joint_Angle_ER_degree[0][2], Joint_Angle_ER_degree[0][3], Joint_Angle_ER_degree[0][4], Joint_Angle_ER_degree[0][5], Joint_Angle_ER_degree[0][6]);

		sprintf(buf7, "CH2 Recv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_FB_degree[1][0], Joint_Angle_FB_degree[1][1], Joint_Angle_FB_degree[1][2], Joint_Angle_FB_degree[1][3], Joint_Angle_FB_degree[1][4], Joint_Angle_FB_degree[1][5], Joint_Angle_FB_degree[1][6]);

		sprintf(buf8, "CH2 Send  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_EP_degree[1][0], Joint_Angle_EP_degree[1][1], Joint_Angle_EP_degree[1][2], Joint_Angle_EP_degree[1][3], Joint_Angle_EP_degree[1][4], Joint_Angle_EP_degree[1][5], Joint_Angle_EP_degree[1][6]);

		sprintf(buf9, "CH2 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		Joint_Angle_ER_degree[1][0], Joint_Angle_ER_degree[1][1], Joint_Angle_ER_degree[1][2], Joint_Angle_ER_degree[1][3], Joint_Angle_ER_degree[1][4], Joint_Angle_ER_degree[1][5], Joint_Angle_ER_degree[1][6]);

		sprintf(buf10, "CH3 Recv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f  %8.3f",
		Joint_Angle_FB_degree[2][0], Joint_Angle_FB_degree[2][1], Joint_Angle_FB_degree[2][2], Joint_Angle_FB_degree[2][3], Joint_Angle_FB_degree[2][4], Joint_Angle_FB_degree[2][5], Head_Angle_FB_degree[0], Head_Angle_FB_degree[1]);

		sprintf(buf11, "CH3 Send  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		Joint_Angle_EP_degree[2][0], Joint_Angle_EP_degree[2][1], Joint_Angle_EP_degree[2][2], Joint_Angle_EP_degree[2][3], Joint_Angle_EP_degree[2][4], Joint_Angle_EP_degree[2][5]);

		sprintf(buf12, "CH3 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		Joint_Angle_ER_degree[2][0], Joint_Angle_ER_degree[2][1], Joint_Angle_ER_degree[2][2], Joint_Angle_ER_degree[2][3], Joint_Angle_ER_degree[2][4], Joint_Angle_ER_degree[2][5]);

		sprintf(buf13, "CH4 Recv  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		Joint_Angle_FB_degree[3][0], Joint_Angle_FB_degree[3][1], Joint_Angle_FB_degree[3][2], Joint_Angle_FB_degree[3][3], Joint_Angle_FB_degree[3][4], Joint_Angle_FB_degree[3][5]);

		sprintf(buf14, "CH4 Send  %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		Joint_Angle_EP_degree[3][0], Joint_Angle_EP_degree[3][1], Joint_Angle_EP_degree[3][2], Joint_Angle_EP_degree[3][3], Joint_Angle_EP_degree[3][4], Joint_Angle_EP_degree[3][5]);

		sprintf(buf15, "CH4 Error %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		Joint_Angle_ER_degree[3][0], Joint_Angle_ER_degree[3][1], Joint_Angle_ER_degree[3][2], Joint_Angle_ER_degree[3][3], Joint_Angle_ER_degree[3][4], Joint_Angle_ER_degree[3][5]);

		sprintf(buf16, "*********************************  Robot current *********************************");
		sprintf(buf17, "CH1 Current %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		motor_current[0][0], motor_current[0][1], motor_current[0][2], motor_current[0][3], motor_current[0][4], motor_current[0][5], motor_current[0][6]);
		sprintf(buf18, "CH2 Current %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f  %8.3f",
		motor_current[1][0], motor_current[1][1], motor_current[1][2], motor_current[1][3], motor_current[1][4], motor_current[1][5], motor_current[1][6]);
		sprintf(buf19, "CH3 Current %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		motor_current[2][0], motor_current[2][1], motor_current[2][2], motor_current[2][3], motor_current[2][4], motor_current[2][5]);
		sprintf(buf20, "CH4 Current %8.3f  %8.3f  %8.3f  %8.3f   %8.3f  %8.3f",
		motor_current[3][0], motor_current[3][1], motor_current[3][2], motor_current[3][3], motor_current[3][4], motor_current[3][5]);

	}
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

int elmo_init_leg4(void)
{
	long can_id = 0;
	int i=0;
	long can_content_order8200[8] = {0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	long can_content_order8000[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	long can_content_order0100[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	long can_content_order_UM[8] = {0x55, 0x4D, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
	long can_content_order_BG[8] = {0x4D, 0x4F, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
	int ret;
//	long can_content_order_TC[8] = {0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	struct timespec sleeptime;
//	struct timespec sleeptime;
	struct can_frame frame;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);

	sleeptime.tv_nsec = 10000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	can_send(3, can_id, 0, can_content_order8200, 2);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	for(i=0; i<3; i++)
	{
		ret = rt_dev_recvfrom(rxsock[3], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	can_send(3, can_id, 0, can_content_order8000, 2);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	can_send(3, can_id, 0, can_content_order0100, 2);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	can_id = 0x300;
	can_send(3, can_id, 0, can_content_order_UM, 8);
	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	for(i=0; i<3; i++)
	{
		ret = rt_dev_recvfrom(rxsock[3], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}


	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	can_send(3, can_id, 0, can_content_order_BG, 8);

	sleeptime.tv_nsec = 5000000;
	sleeptime.tv_sec = 0;
	nanosleep(&sleeptime,NULL);

	for(i=0; i<3; i++)
	{
		ret = rt_dev_recvfrom(rxsock[3], (void *)&frame, sizeof(can_frame_t), 0, (struct sockaddr *)&addr, &addrlen);
	}
	printf("hand initate finished\n");
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

	sprintf(buf23, "CAN init        %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1],  can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14], can_work_states[15], can_work_states[16], can_work_states[17], can_work_states[18], can_work_states[19], can_work_states[20], can_work_states[21], can_work_states[22], can_work_states[23], can_work_states[24], can_work_states[25], can_work_states[26]);
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

	sprintf(buf22, "CAN connection  %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d", can_connection_status[0],can_connection_status[1], can_connection_status[2],can_connection_status[3],can_connection_status[4],can_connection_status[5],can_connection_status[6],can_connection_status[7], can_connection_status[8],can_connection_status[9],can_connection_status[10],can_connection_status[11],can_connection_status[12],can_connection_status[13], can_connection_status[14],can_connection_status[15],can_connection_status[16],can_connection_status[17], can_connection_status[18],can_connection_status[19], can_connection_status[20],can_connection_status[21], can_connection_status[22],can_connection_status[23],can_connection_status[24],can_connection_status[25],can_connection_status[26]);

	sprintf(buf23, "CAN init        %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d  %d . %d  %d  %d  %d  %d  %d",can_work_states[0],can_work_states[1],  can_work_states[2],can_work_states[3],can_work_states[4],can_work_states[5],can_work_states[6],can_work_states[7], can_work_states[8],can_work_states[9],can_work_states[10],can_work_states[11],can_work_states[12],can_work_states[13], can_work_states[14], can_work_states[15], can_work_states[16], can_work_states[17], can_work_states[18], can_work_states[19], can_work_states[20], can_work_states[21], can_work_states[22], can_work_states[23], can_work_states[24], can_work_states[25], can_work_states[26]);
}

