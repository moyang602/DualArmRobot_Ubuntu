#ifndef GLOABL_DEF

#define Rad2Degree 	57.29577951308
#define Degree2Rad 	0.0174532925
#define sin15	0.2588190451025
#define cos15 	0.965925826289

#define ARM1_JOINT1_MOVE 1
#define ARM1_JOINT2_MOVE 2
#define ARM1_JOINT3_MOVE 3
#define ARM1_JOINT4_MOVE 4
#define ARM2_JOINT1_MOVE 5
#define ARM2_JOINT2_MOVE 6
#define ARM2_JOINT3_MOVE 7
#define ARM2_JOINT4_MOVE 8
#define ARM3_JOINT1_MOVE 9
#define ARM3_JOINT2_MOVE 10
#define ARM3_JOINT3_MOVE 11
#define ARM3_JOINT4_MOVE 12

#define ARM1_CLAW_MOVE 13
#define ARM2_CLAW_MOVE 14
#define ARM3_CLAW_MOVE 15

#define ARM2_ARM1_MOVE 20
#define ARM1_ARM3_MOVE 21
#define ARM3_ARM2_MOVE 22

#define ARM1_VISION_MOVE		31
#define ARM2_VISION_MOVE		32
#define ARM3_VISION_MOVE		33

#define ARM1_SERVO_ON   50
#define ARM1_SERVO_OFF   60
#define ARM2_SERVO_ON   51
#define ARM2_SERVO_OFF   61
#define ARM3_SERVO_ON   52
#define ARM3_SERVO_OFF   62

#define HAND1_SERVO_ON   53
#define HAND1_SERVO_OFF   63
#define HAND2_SERVO_ON   54
#define HAND2_SERVO_OFF   64
#define HAND3_SERVO_ON   55
#define HAND3_SERVO_OFF   65



#define ARM1_ENDZ_MOVE 99
#define ARM2_ENDZ_MOVE 98
#define ARM3_ENDZ_MOVE 97

#define HAND_CURRENT 70
#define HAND1_AUTOMATIC_CLOSE 71

// 机器人运动模式
#define SINGLE_JOINT_MOTION	101
#define ONE_ARM_MOTION		102
#define TWO_ARMS_MOTION 	103
#define VISION_MOTION   	104
#define MANIPULATE_MOTION 	105
#define FIND_HOME_MOTION  	106
#define ROBOT_CONTROL 		107
#define REMOTE_MOTION		109
#define REMOTE_CONTROL		110
#define REMOTE_DATA			111
#define HAND_MOTION			115
#define HOMEBACK			80
#define FIND_HOME_NEW      	120
#define RETURN_ORIGIN_POSITION	 121

#define ARM1 0
#define ARM2 1
#define ARM3 2



#define PREPARE_FIND_HOME 122

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
