#ifndef GLOABL_DEF

#include "trajectory.h"

#define Rad2Degree 	57.29577951308
#define Degree2Rad 	0.0174532925
#define sin15	0.2588190451025
#define cos15 	0.965925826289

// 机器人运动模式
#define SINGLE_JOINT_MOTION	101
#define ONE_ARM_MOTION		102
#define END_MOTION 			103
#define VISION_MOTION   	104
#define FIND_HOME_MOTION  	106
#define ROBOT_CONTROL 		107
#define REMOTE_MOTION		109
#define REMOTE_CONTROL		110
#define REMOTE_DATA			111
#define FORCE_MOTION		112
#define FORCE_CONTROL		113
#define HAND_MOTION			115
#define HOMEBACK			80
#define PREPARE_FIND_HOME 	122
#define RETURN_ORIGIN_POSITION	 121
#define RETURN_ZERO			119
#define MOVE_PRE_POSITION	 123
#define DUTY_MOTION			124
#define FETCH_MOTION		125
#define PLACE_MOTION		126  //0x7e

#define ARM1 0
#define ARM2 1
#define ARM3 2



//工控机控制指令
#define CMD_POWER_ON 		0xA1
#define CMD_POWER_OFF 		0xA2
#define CMD_ELMO_INIT 		0xA3
#define CMD_SERVO_ON 		0xA4
#define CMD_SERVO_OFF 		0xA5
#define CMD_CTR_ENABLE 		0xA6
#define CMD_CTR_DISENABLE 	0xA7
#define CMD_HOMEPREPARE		0xA8
#define CMD_HOMEZERO 		0xA9
#define CMD_RESET			0xAA
#define CMD_BACKORIGIN		0xAB
#define CMD_MOVEPRE			0xAC

//力传感器控制指令
#define FORCE_START   		0xF1
#define FORCE_STOP   		0xF2
#define FORCE_ENABLE   		0xF3
#define FORCE_DISABLE  		0xF4
#define FORCE_CLEAR   		0xF5
#define FORCE_SETPARAM 		0xF6

// 遥操作控制指令
#define REMOTE_START		0xC1
#define REMOTE_STOP			0xC2
#define REMOTE_ENABLE		0xC3
#define REMOTE_DISABLE		0xC4

// 手部操作控制指令
#define HANDCMD_ZERO			0xD1
#define HANDCMD_CYLINDER		0xD2
#define HANDCMD_SPHERE			0xD3
#define HANDCMD_CYLINDER_PRE	0xD4
#define HANDCMD_SPHERE_PRE		0xD5

// 开门任务指令
#define DUTY_START			0xB1
#define DUTY_STOP			0xB2
#define DUTY_RUN			0xB3
#define DUTY_FORCESTART		0xB4
#define DUTY_FORCESTOP		0xB5

// 自控指令
#define SELFCONTROL_START	0xE1
#define SELFCONTROL_STOP	0xE2

// 取工具指令
#define FETCH_START			0x91
#define FETCH_STOP			0x92
#define FETCH_MOVEPRE1		0x8a
#define FETCH_MOVEPRE2		0x8b
#define FETCH_MOVEPRE3		0x8c
#define FETCH_DOCK			0x94
#define FETCH_ADJUST		0x95
#define FETCH_MOVEOUT		0x96
#define FETCH_BACK			0x97

// Place tool
#define PLACE_START			0x98
#define PLACE_STOP			0x99
#define PLACE_MOVEPRE1		0x8d
#define PLACE_MOVEPRE2		0x8e
#define PLACE_MOVEPRE3		0x8f
#define PLACE_DOCK			0x9b
#define PLACE_DOCK2			0x9c
#define PLACE_MOVEIN		0x9d
#define PLACE_UNDOCK		0x9e
#define PLACE_BACK			0x9f

#define GLOABL_DEF
#endif
