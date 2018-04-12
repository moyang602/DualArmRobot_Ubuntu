#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define Rad2Deg 57.29578
#define Deg2Rad 0.0174533


int UnitVector(double vec[3],double unitvec[3]);
void cross(double in1[3], double in2[3], double out[3]);
void matrix_multiply(double in1[4][4], double in2[4][4], double out[4][4]);
void inv_matrix(double in[4][4], double out[4][4]);

int KinOneLeg(double angle4[4],double Kin_T14[4][4]);
int KinTwoLegR(double angle8[8],double T18[4][4]);
int KinTwoLegL(double angle8[8],double T18[4][4]);
double betaCal(double x[8]);
double betaCal_Left(double x[8]);
int InvCham(double T08[4][4],double anglein[8],double beta,double angle_out2[8]);
int InvCham_Left(double T08[4][4],double anglein[8],double beta,double angle_out2[8]);
double Five_Interpolation(double x0,double v0,double a0, double x1,double v1,double a1,double tf,double t);
double parabolic_interpolation(double x0, double xf, double vb, double acc, double t);
double parabolic_interpolation_time(double x0, double xf, double vb, double acc);
void select_inv(double angle_last[8], double angle_in[4][8], double angle_out[8]);

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
int cubicAddPoint(int index, float point);
float cubicInterpolate(int index);

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
void OBB_Update(double AngleL[7],double AngleR[7],double AngleW[2]);
int OBB_Collision(struct OBB_struct OStr1,struct OBB_struct OStr2);
int CollisionDetection(double jL[7],double jR[7],double jW[2]);	// left_arm right_arm waist_rotate waist_pitch

#endif