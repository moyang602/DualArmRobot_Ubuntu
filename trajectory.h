#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define Rad2Deg 57.29578
#define Deg2Rad 0.0174533

extern double L1;
extern double L2;
extern double pi;

extern double sqrt3c2;

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

#endif
