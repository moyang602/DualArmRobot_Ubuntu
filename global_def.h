#ifndef GLOABL_DEF

#define Rad2Degree 	57.29577951308
#define Degree2Rad 	0.0174532925
#define sin15	0.2588190451025
#define cos15 	0.965925826289

// 机器人运动模式
#define SINGLE_JOINT_MOTION	101
#define ONE_ARM_MOTION		102
#define TWO_ARMS_MOTION 	103
#define VISION_MOTION   	104
#define FIND_HOME_MOTION  	106
#define ROBOT_CONTROL 		107
#define REMOTE_MOTION		109
#define REMOTE_CONTROL		110
#define REMOTE_DATA			111
#define HAND_MOTION			115
#define HOMEBACK			80
#define PREPARE_FIND_HOME 	122
#define RETURN_ORIGIN_POSITION	 121
#define RETURN_ZERO			119
#define MOVE_PRE_POSITION	 123

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
#define FORCE_INIT   		0xB1

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

#define GLOABL_DEF
#endif
