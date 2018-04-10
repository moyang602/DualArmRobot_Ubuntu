#ifndef HANDCONTROL_H
#define HANDCONTROL_H

#define pi				3.1415926

#define num_sample		50   //采样数，调用周期为6ms，求平均值得时间为60ms
#define I0_benchmark	0.2   //电流的基准值，大于这个表示手指接触到物体了

#define f_alpha_default 0		//手指角度默认值
#define f_alpha_max		90		//默认-15-30度之间
#define f_alpha_min		0
#define delta_falpha	0.045//0.18	//手指弯曲速度delta；相应的速度为30度/S
#define minidelta_falpha	0.025//0.01//手指缓慢弯曲速度minidelta,接触到物体时的

#define theta_default	0		//角度制；范围最大到平角
#define theta_max		90
#define theta_min		0
#define delta_theta		0.09//0.18		//旋转速度60度/S

#define Ic_mark_max		5		//最大档位数,档位从1开始
#define Ic1				0.8		//档位值
#define Ic2				0.9
#define Ic3				1.0
#define Ic4				1.2
#define Ic5				1.4

#endif //
