#include "trajectory.h"
#include <math.h>


double L1 = 290.5;
double L2 = 311.357;
double pi = 3.1415926;

double sqrt3c2 = 0.8660254037844;
double beta_L1 = 270.0;
double Minimum_Value = 1e-6;
//int i=0;

//单位向量
int UnitVector(double vec[3],double unitvec[3])
{
	int i=0;
	double length = 0;

	for(i=0;i<3;i++)
		length = length + vec[i]*vec[i];

	length = sqrt(length);

	for(i=0;i<3;i++)
		unitvec[i] = vec[i]/length;

	return 0;
}

//向量叉乘
void cross(double in1[3], double in2[3], double out[3])
{
	out[0] = in1[1]*in2[2] - in1[2]*in2[1];
	out[1] = in1[2]*in2[0] - in1[0]*in2[2];
	out[2] = in1[0]*in2[1] - in1[1]*in2[0];
	return;
}

// 新矩阵相乘
void matrix_multiply(double in1[4][4], double in2[4][4], double out[4][4])
{
	int i, j; 

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			out[i][j] = in1[i][0]*in2[0][j] + in1[i][1]*in2[1][j] + in1[i][2]*in2[2][j] + in1[i][3]*in2[3][j]; 
		}
	}
}

// 新矩阵求逆
void inv_matrix(double in[4][4], double out[4][4])
{
	int i, j;
	//	double temp[3][3];
	for(i=0; i<3; i++)
	{
		for (j=0; j<3; j++)
		{
			out[i][j] = in[j][i];
			//	temp[i][j] = in[i][i];
		}
	}

	for(i=0; i<3; i++)
	{
		out[i][3] = -(out[i][0]*in[0][3] + out[i][1]*in[1][3] + out[i][2]*in[2][3]);
	}

	out[3][0] = 0.0;
	out[3][1] = 0.0;
	out[3][2] = 0.0;
	out[3][3] = 1.0;
}

int KinOneLeg(double angle4[4],double Kin_T14[4][4])
{
	double Kin_T1[4][4] = {{cos(angle4[0]),-sin(angle4[0]),0,0},{sin(angle4[0]),cos(angle4[0]),0,0},{0,0,1,0},{0,0,0,1}};
	double Kin_T2[4][4] = {{cos(angle4[1]),-sin(angle4[1]),0,0},{0,0,-1,-L1},{sin(angle4[1]),cos(angle4[1]),0,0},{0,0,0,1}};
	double Kin_T3[4][4] = {{cos(angle4[2]),-sin(angle4[2]),0,0},{0,0,1,0},{-sin(angle4[2]),-cos(angle4[2]),0,0},{0,0,0,1}};
	double Kin_T4[4][4] = {{cos(angle4[3]),-sin(angle4[3]),0,0},{0,0,-1,0},{sin(angle4[3]),cos(angle4[3]),0,0},{0,0,0,1}};

	double Kin_T12[4][4];
	double Kin_T13[4][4];

	matrix_multiply(Kin_T1, Kin_T2, Kin_T12);
	matrix_multiply(Kin_T12, Kin_T3, Kin_T13);
	matrix_multiply(Kin_T13, Kin_T4, Kin_T14);

	return 0;
}

int KinTwoLegR(double angle8[8],double T18[4][4])
{
	double T11p[4][4] = {{-0.5,-sqrt3c2,0,L2/2},{sqrt3c2,-0.5,0,sqrt3c2*L2},{0,0,1,0},{0,0,0,1}};

	double angle14[4] = {angle8[0],angle8[1],angle8[2],angle8[3]};
	double angle58[4] = {angle8[4],angle8[5],angle8[6],angle8[7]};

	double twoleg_T14[4][4];
	double twoleg_T41[4][4];
	double twoleg_T58[4][4];

	double T41p[4][4];

	KinOneLeg(angle14,twoleg_T14);
	inv_matrix(twoleg_T14,twoleg_T41);

	KinOneLeg(angle58,twoleg_T58);

	matrix_multiply(twoleg_T41,T11p,T41p);

	matrix_multiply(T41p,twoleg_T58,T18);

	return 0;
}

double betaCal(double x[8])
{
	int i;

	double beta_T2[4][4] = {{cos(x[1]),-sin(x[1]),0,0},{0,0,-1,-beta_L1},{sin(x[1]),cos(x[1]),0,0},{0,0,0,1}};
	double beta_T3[4][4] = {{cos(x[2]),-sin(x[2]),0,0},{0,0,1,0},{-sin(x[2]),-cos(x[2]),0,0},{0,0,0,1}};
	double beta_T4[4][4] = {{cos(x[3]),-sin(x[3]),0,0},{0,0,-1,0},{sin(x[3]),cos(x[3]),0,0},{0,0,0,1}};

	double beta_T23[4][4] = {0};
	double beta_T24[4][4] = {0};
	double beta_T42[4][4] = {0};
	double beta_z1[3];
	double beta_T08[4][4] = {0};
	double beta_p4p[3];

	double beta_x_unit[3] = {1,0,0};
	double beta_p4p_unit[3];

	double beta_k[3];
	double beta_k_unit[3];
	double beta_x_vertical[3];
	double beta_x_vertical_unit[3];

	double tanAlphaX;
	double tanAlphaY;
	double beta_alpha;
	double beta_z1_origin_unit[3];

	double beta_z1_origin_vertical[3];
	double beta_z1_origin_vertical_unit[3];

	double beta_x1;
	double beta_y1;

	double beta;

	matrix_multiply(beta_T2, beta_T3, beta_T23);
	matrix_multiply(beta_T23, beta_T4, beta_T24);

	inv_matrix(beta_T24,beta_T42);

	beta_z1[0] = beta_T42[0][2];
	beta_z1[1] = beta_T42[1][2];
	beta_z1[2] = beta_T42[2][2];

	KinTwoLegR(x,beta_T08);

	beta_p4p[0] = beta_T08[0][3];
	beta_p4p[1] = beta_T08[1][3];
	beta_p4p[2] = beta_T08[2][3];
	
	UnitVector(beta_p4p,beta_p4p_unit);

	cross(beta_x_unit,beta_p4p_unit,beta_k);
	UnitVector(beta_k,beta_k_unit);
	
	cross(beta_k_unit,beta_x_unit,beta_x_vertical);
	UnitVector(beta_x_vertical,beta_x_vertical_unit);

	tanAlphaX = 0;
	tanAlphaY = 0;

	for(i=0;i<3;i++)
		tanAlphaY = tanAlphaY + beta_p4p_unit[i]*beta_x_vertical_unit[i];

	for(i=0;i<3;i++)
		tanAlphaX = tanAlphaX + beta_p4p_unit[i]*beta_x_unit[i];

	beta_alpha = atan2(tanAlphaY,tanAlphaX);

	beta_z1_origin_unit[0] = beta_k_unit[0]*beta_k_unit[1]*(1-cos(beta_alpha))-beta_k_unit[2]*sin(beta_alpha);
	beta_z1_origin_unit[1] = beta_k_unit[1]*beta_k_unit[1]*(1-cos(beta_alpha))+cos(beta_alpha);
	beta_z1_origin_unit[2] = beta_k_unit[1]*beta_k_unit[2]*(1-cos(beta_alpha))-beta_k_unit[0]*sin(beta_alpha);

	cross(beta_z1_origin_unit,beta_p4p_unit,beta_z1_origin_vertical);
	UnitVector(beta_z1_origin_vertical,beta_z1_origin_vertical_unit);

	beta_x1 = 0;
	beta_y1 = 0;

	for(i=0;i<3;i++)
		beta_x1 = beta_x1 + beta_z1[i]*beta_z1_origin_unit[i];

	for(i=0;i<3;i++)
		beta_y1 = beta_y1 + beta_z1[i]*beta_z1_origin_vertical_unit[i];

	
	beta = atan2(beta_y1,beta_x1);

	return beta;
}

int InvCham(double T08[4][4],double anglein[8],double beta,double angle_out[4][8])
{
	int i;
	int j;
	double T11p[4][4] = {{-0.5,-sqrt3c2,0,L2/2},{sqrt3c2,-0.5,0,sqrt3c2*L2},{0,0,1,0},{0,0,0,1}};

	double px = T08[0][3];
	double py = T08[1][3];
	double pz = T08[2][3];

	double L;
	double an1;
	double L3;
	double an2_1;
	double an2_2;
	double an2;
	double p4p[3];

	double x_unit[3] = {1,0,0};
	double p4p_unit[3];
	double k[3];
	double k_unit[3];
	double x_vertical[3];
	double x_vertical_unit[3];

	double tanAlphaX = 0;
	double tanAlphaY = 0;
	double alpha;
	double z1_origin_unit[3];
	double z1_origin_vertical[3];
	double z1_origin_vertical_unit[3];
	double z1[3];
	double z1_unit[3];

	double an3_1;
	double an3_2;
	double an3;
	double p4p_vertical_unit[3];
	double p1_unit[3];
	double x1_unit[3];
	double y1_unit[3];
	double R41[3][3];
	double Inv_angle1[4];
	double T14[4][4];
	double T1p[4][4];
	double T41[4][4];
	double Ttemp1[4][4];
	double T15[4][4];
	double Ttemp2[4][4];
	double T68_1[4][4];
	double Inv_angle3[4];
	double T2p[4][4];
	double T68_2[4][4];
	double k_value;

	L = px*px + py*py + pz*pz;
	L = sqrt(L);

//	for (i=0;i<4;i++)
//		angle_out[i][0] = anglein[0];
	
	an1 = asin((L-L2)/(2.0*L1)) + pi/2.0;
	an2 = an1;

//	an1 = 5*pi/6 - anglein[0];
	L3 = sqrt(L1*L1 + L2*L2 - 2*L1*L2*cos(an1));
//	an2_1 = acos((L2*L2 + L3*L3 - L1*L1)/(2.0*L2*L3));
//	an2_2 = acos((L1*L1 + L3*L3 - L*L)/(2.0*L1*L3));
//	an2 = an2_1 + an2_2;

	for (i=0;i<4;i++)
	{
		angle_out[i][0] = 5.0*pi/6.0 - an1;
		angle_out[i][4] = an2 - 5.0*pi/6.0;
	}

	p4p[0] = px;
	p4p[1] = py;
	p4p[2] = pz;

	UnitVector(p4p,p4p_unit);

	cross(x_unit,p4p_unit,k);

	k_value = sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);

	if(k_value > Minimum_Value)
	{
		UnitVector(k,k_unit);

		cross(k_unit,x_unit,x_vertical);
		UnitVector(x_vertical,x_vertical_unit);

		for(i=0;i<3;i++)
			tanAlphaY = tanAlphaY + p4p_unit[i]*x_vertical_unit[i];

		for(i=0;i<3;i++)
			tanAlphaX = tanAlphaX + p4p_unit[i]*x_unit[i];

		alpha = atan2(tanAlphaY,tanAlphaX);

		z1_origin_unit[0] = k_unit[0]*k_unit[1]*(1-cos(alpha))-k_unit[2]*sin(alpha);
		z1_origin_unit[1] = k_unit[1]*k_unit[1]*(1-cos(alpha))+cos(alpha);
		z1_origin_unit[2] = k_unit[1]*k_unit[2]*(1-cos(alpha))-k_unit[0]*sin(alpha);

		cross(z1_origin_unit,p4p_unit,z1_origin_vertical);
		UnitVector(z1_origin_vertical,z1_origin_vertical_unit);
	}
	else
	{
		if(p4p[0] > 0)
		{
			z1_origin_unit[0] = 0.0;
			z1_origin_unit[1] = 1.0;
			z1_origin_unit[2] = 0.0;

			z1_origin_vertical_unit[0] = 0.0;
			z1_origin_vertical_unit[1] = 0.0;
			z1_origin_vertical_unit[2] = -1.0;
		}
		if(p4p[0] < 0)
		{
			return 1;
		}
	}

	for(i=0;i<3;i++)
		z1[i] = z1_origin_unit[i]*cos(beta) + z1_origin_vertical_unit[i]*sin(beta);
	UnitVector(z1,z1_unit);

	an3_1 = acos((L1*L1 + L3*L3 - L2*L2)/(2*L1*L3));
	an3_2 = acos((L*L + L3*L3 - L1*L1)/(2*L*L3));
	an3 = an3_1 + an3_2;

	cross(z1_unit,p4p_unit,p4p_vertical_unit);

	for(i=0;i<3;i++)
		p1_unit[i] = p4p_unit[i]*cos(an3) + p4p_vertical_unit[i]*sin(an3);

	cross(p1_unit,z1_unit,x1_unit);
	cross(z1_unit,x1_unit,y1_unit);

	for(i=0;i<3;i++)
		R41[i][0] = x1_unit[i];

	for(i=0;i<3;i++)
		R41[i][1] = y1_unit[i];

	for(i=0;i<3;i++)
		R41[i][2] = z1_unit[i];

	angle_out[0][2] = acos(-R41[2][1]);
	angle_out[1][2] = acos(-R41[2][1]);
	angle_out[2][2] = -acos(-R41[2][1]);
	angle_out[3][2] = -acos(-R41[2][1]);

	if (abs(sin(angle_out[0][2])) < 1e-5)
	{
		for(i=0;i<4;i++)
		{
			angle_out[i][1] = atan2(R41[0][2],R41[0][0]);
			angle_out[i][3] = anglein[3];
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			angle_out[i][1] = atan2(R41[2][2]/sin(angle_out[i][2]),R41[2][0]/sin(angle_out[i][2]));
			angle_out[i][3] = atan2(-R41[1][1]/sin(angle_out[i][2]),R41[0][1]/sin(angle_out[i][2]));
		}
	}

	//i = 1时
	Inv_angle1[0] = angle_out[0][0];
	Inv_angle1[1] = angle_out[0][1];
	Inv_angle1[2] = angle_out[0][2];
	Inv_angle1[3] = angle_out[0][3];

	KinOneLeg(Inv_angle1,T14);

	for (i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
			T1p[i][j] = 0;
	}
	T1p[0][0] = cos(angle_out[0][4]);T1p[0][1] = -sin(angle_out[0][4]);
	T1p[1][0] = sin(angle_out[0][4]);T1p[1][1] = cos(angle_out[0][4]);
	T1p[2][2] = 1;
	T1p[3][3] = 1;

	inv_matrix(T14,T41);

	matrix_multiply(T41,T11p,Ttemp1);
	matrix_multiply(Ttemp1,T1p,T15);

	inv_matrix(T15,Ttemp2);
	matrix_multiply(Ttemp2,T08,T68_1);

	//i = 3时
	Inv_angle3[0] = angle_out[2][0];
	Inv_angle3[1] = angle_out[2][1];
	Inv_angle3[2] = angle_out[2][2];
	Inv_angle3[3] = angle_out[2][3];

	KinOneLeg(Inv_angle3,T14);
	
	for (i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
			T2p[i][j] = 0;
	}
	T2p[0][0] = cos(angle_out[2][4]);T2p[0][1] = -sin(angle_out[2][4]);
	T2p[1][0] = sin(angle_out[2][4]);T2p[1][1] = cos(angle_out[2][4]);
	T2p[2][2] = 1;
	T2p[3][3] = 1;

	inv_matrix(T14,T41);

	matrix_multiply(T41,T11p,Ttemp1);
	matrix_multiply(Ttemp1,T2p,T15);

	inv_matrix(T15,Ttemp2);
	matrix_multiply(Ttemp2,T08,T68_2);

	angle_out[0][6] = acos(-T68_1[1][2]);
	angle_out[1][6] = -acos(-T68_1[1][2]);
	angle_out[2][6] = acos(-T68_2[1][2]);
	angle_out[3][6] = -acos(-T68_2[1][2]);

	for (i=0;i<2;i++)
	{
		if (abs(sin(angle_out[i][6])) < 1e-5)
		{
			angle_out[i][5] = atan2(T68_1[2][0],T68_1[2][1]);
			angle_out[i][7] = anglein[7];
		}
		else
		{
			angle_out[i][5] = atan2(T68_1[2][2]/sin(angle_out[i][6]),T68_1[0][2]/sin(angle_out[i][6]));
			angle_out[i][7] = atan2(-T68_1[1][1]/sin(angle_out[i][6]),T68_1[1][0]/sin(angle_out[i][6]));
		}
	}

	for (i=2;i<4;i++)
	{
		if (abs(sin(angle_out[i][6])) < 1e-5)
		{
			angle_out[i][5] = atan2(T68_2[2][0],T68_2[2][1]);
			angle_out[i][7] = anglein[7];
		}
		else
		{
			angle_out[i][5] = atan2(T68_2[2][2]/sin(angle_out[i][6]),T68_2[0][2]/sin(angle_out[i][6]));
			angle_out[i][7] = atan2(-T68_2[1][1]/sin(angle_out[i][6]),T68_2[1][0]/sin(angle_out[i][6]));
		}
	}

	return 0;
}

double Five_Interpolation(double x0,double v0,double a0,double x1,double v1,double a1,double tf, double t)
{
	double aa0 = 0.0;
	double aa1 = 0.0;
	double aa2 = 0.0;
	double aa3 = 0.0;
	double aa4 = 0.0;
	double aa5 = 0.0;
	double out = 0.0;
	aa0 = x0;
	aa1 = v0;
	aa2 = a0/2.0;
	aa3 = (20.0*x1 - 20.0*x0 - (8.0*v1+12.0*v0)*tf - (3.0*a0-a1)*tf*tf)/(2.0*tf*tf*tf);
	aa4 = (30.0*x0 - 30.0*x1 + (14.0*v1 + 16.0*v0)*tf + (3.0*a0-2.0*a1)*tf*tf)/(2.0*tf*tf*tf*tf);
	aa5 = (12.0*x1 - 12.0*x0 - (6.0*v1+6.0*v0)*tf - (a0-a1)*tf*tf)/(2.0*tf*tf*tf*tf*tf);

	out = aa0 + aa1*t + aa2*t*t + aa3*t*t*t + aa4*t*t*t*t + aa5*t*t*t*t*t;

	if(t>=tf)	    out = x1;
	return out;
}
