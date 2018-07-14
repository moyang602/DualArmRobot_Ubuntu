#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define Rad2Deg 57.29578
#define Deg2Rad 0.0174533


int UnitVector(double vec[3],double unitvec[3]);
void cross(double in1[3], double in2[3], double out[3]);
void matrix_multiply(double in1[4][4], double in2[4][4], double out[4][4]);
void inv_matrix(double in[4][4], double out[4][4]);

double Five_Interpolation(double x0,double v0,double a0, double x1,double v1,double a1,double tf,double t);
double parabolic_interpolation(double x0, double xf, double vb, double acc, double t);
double parabolic_interpolation_time(double x0, double xf, double vb, double acc);

void JacobianL(double anglein[7], double JT[6][7]);	// input: deg
void JacobianR(double anglein[7], double JT[6][7]);	// input: deg
void InvJacobianL(double anglein[7], double InvJ[7][6]);	// input: deg
void InvJacobianR(double anglein[7], double InvJ[7][6]);	// input: deg

// 运动学
void KinL(double Angle[7], double TransMatrix[4][4]);
void KinR(double Angle[7], double TransMatrix[4][4]);

//
void Matrix2Pose(double TransMatrix[4][4], double Pose[6]);
void Pose2Matrix(double Pose[6], double TransMatrix[4][4]);
// 冗余自由度规划计算
double Beta_CalL(double Angle[7]);	// input: rad
double Beta_CalR(double Angle[7]);	// input: rad
// 逆运动学
int invKinL(double Angle_now[7], double TransMatrix[4][4], double Beta, double Angle_cal[7]);	// input:
int invKinR(double Angle_now[7], double TransMatrix[4][4], double Beta, double Angle_cal[7]);
int ChooseSolve(double Angle_now[7], double Solve[8][7]);	// input: rad

// 差分运动转换为矩阵
void delta2tr(double delta[6], double tr[4][4]);
// 矩阵转换为差分运动
//void tr2delta(double T0[4][4], double T1[4][4], double delta[6]);
void tr2delta(double R[3][3], double P[3], double delta[6]);

void tr2rt(double Tr[4][4], double R[3][3], double T[3]);
void rt2tr(double R[3][3], double T[3], double Tr[4][4]);

// 将旋转矩阵正交单位化
void Schmidt(double Raw[4][4], double Out[4][4]);
void Matrix_Trans3(double MA[3][3], double MB[3][3]);
int Matrix_Reverse(int iNum, double* pSourceR, double* pDestR);
//
struct Cubic_Struct
{
	int filled;
	int needNextPoint;
	float segmentTime;
	int interpolationRate;
	float interpolationTime;
	float interpolationIncrement;
	float x0;
	float x1;
	float x2;
	float x3;
	float wp0;
	float wp1;
	float vel0;
	float vel1;
	float coeffa;
	float coeffb;
	float coeffc;
	float coeffd;
};
int cubicAddPoint(int index, double point);
double cubicInterpolate(int index);
int cubicAddPoint_WaistHead(int index, double point);
double cubicInterpolate_WaistHead(int index);

//3维向量叉乘点乘
void VecCross(double va[3],double vb[3],double vre[3]);
double VecMulti(double va[3],double vb[3]);
// 矩阵相乘
void Matrix_Multiply(int iRow1, int iCol1, int iCol2, double* MA, double* MB, double* MC);
// 碰撞检测相关
struct OBB_struct
{
// 	double x_axis[3];
// 	double y_axis[3];
// 	double z_axis[3];
	double Rt[3][3];
	double pos[3];
	double lwh[3];
};
void OBB_Update(float AngleL[7],float AngleR[7],float AngleW[2]);
int OBB_Collision(struct OBB_struct OStr1,struct OBB_struct OStr2);
int CollisionDetection(float jL[7],float jR[7],float jW[2]);	// left_arm right_arm waist_rotate waist_pitch

#endif