#include "trajectory.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

double L1 = 290.5;
double L2 = 311.357;
double pi = 3.1415926;
double d3 = 310.5;
double d4 = 338;
double dend = 100;
double Tool_Position[3] = {0.0, 0.0, 0.0};

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
	//		printf("out[%d][%d]=%f in1=%f in2= %f\n", i,j,out[i][j], in1[i][j], in2[i][j]);
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

int KinTwoLegL(double angle8[8],double T18[4][4])
{
	double T11p[4][4] = {{-0.5,sqrt3c2,0,-L2/2},{-sqrt3c2,-0.5,0,sqrt3c2*L2},{0,0,1,0},{0,0,0,1}};

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

double betaCal_Left(double x[8])
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

	KinTwoLegL(x,beta_T08);

	beta_p4p[0] = beta_T08[0][3];
	beta_p4p[1] = beta_T08[1][3];
	beta_p4p[2] = beta_T08[2][3];

	UnitVector(beta_p4p,beta_p4p_unit);

	cross(beta_p4p_unit,beta_x_unit,beta_k);
	UnitVector(beta_k,beta_k_unit);

	cross(beta_x_unit,beta_k_unit,beta_x_vertical);
	UnitVector(beta_x_vertical,beta_x_vertical_unit);

	tanAlphaX = 0;
	tanAlphaY = 0;

	for(i=0;i<3;i++)
		tanAlphaY = tanAlphaY + beta_p4p_unit[i]*beta_x_vertical_unit[i];

	for(i=0;i<3;i++)
		tanAlphaX = tanAlphaX + beta_p4p_unit[i]*beta_x_unit[i];

	beta_alpha = atan2(tanAlphaY,-tanAlphaX);

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

int InvCham(double T08[4][4],double anglein[8],double beta,double angle_out2[8])
{
	int i;
	int j;
	double T11p[4][4] = {{-0.5,-sqrt3c2,0,L2/2},{sqrt3c2,-0.5,0,sqrt3c2*L2},{0,0,1,0},{0,0,0,1}};
	double angle_out[4][8];

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
			z1_origin_unit[0] = 0.0;
			z1_origin_unit[1] = -1.0;
			z1_origin_unit[2] = 0.0;

			z1_origin_vertical_unit[0] = 0.0;
			z1_origin_vertical_unit[1] = 0.0;
			z1_origin_vertical_unit[2] = -1.0;
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

	if (fabs(sin(angle_out[0][2])) < 1e-5)
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
	T2p[0][0] = cos(angle_out[2][4]);
	T2p[0][1] = -sin(angle_out[2][4]);
	T2p[1][0] = sin(angle_out[2][4]);
	T2p[1][1] = cos(angle_out[2][4]);
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
		if (fabs(sin(angle_out[i][6])) < 1e-5)
		{
			angle_out[i][5] = atan2(T68_1[2][0],T68_1[2][1]);
			angle_out[i][7] = anglein[7];
		}
		else
		{
			angle_out[i][5] = atan2(T68_1[2][2]/sin(angle_out[i][6]),T68_1[0][2]/sin(angle_out[i][6]));
			angle_out[i][7] = atan2(-T68_1[1][1]/sin(angle_out[i][6]),T68_1[1][0]/sin(angle_out[i][6]));
		}
//		printf("-T68_1[1][1] = %f  T68_1[1][0] = %f   angle_out[i][6]=%f\n", -T68_1[1][1], T68_1[1][0], angle_out[i][6]);
	//	printf("fabs(sin(angle_out[i][6])=%f\n", fabs(sin(angle_out[i][6])));
	}

	for (i=2;i<4;i++)
	{
		if (fabs(sin(angle_out[i][6])) < 1e-5)
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

	printf("\n");
	for(i=0; i<4; i++)
	{
		for(j=0; j<8; j++)
		{
			printf("%f   ", angle_out[i][j]);
		}
		printf("\n");
	}


	select_inv(anglein, angle_out, angle_out2);

	return 0;
}

int InvCham_Left(double T08[4][4],double anglein[8],double beta,double angle_out2[8])
{
	int i;
	int j;
	double T11p[4][4] = {{-0.5,sqrt3c2,0,-L2/2},{-sqrt3c2,-0.5,0,sqrt3c2*L2},{0,0,1,0},{0,0,0,1}};

    double angle_out[4][8];

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
		angle_out[i][0] = an1 - 5.0*pi/6.0;
		angle_out[i][4] = 5.0*pi/6.0 - an2;
	}

	p4p[0] = px;
	p4p[1] = py;
	p4p[2] = pz;

	UnitVector(p4p,p4p_unit);

	cross(p4p_unit,x_unit,k);

	k_value = sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);

	if(k_value > Minimum_Value)
	{
		UnitVector(k,k_unit);

	cross(x_unit,k_unit,x_vertical);
	UnitVector(x_vertical,x_vertical_unit);

	for(i=0;i<3;i++)
		tanAlphaY = tanAlphaY + p4p_unit[i]*x_vertical_unit[i];

	for(i=0;i<3;i++)
		tanAlphaX = tanAlphaX + p4p_unit[i]*x_unit[i];

	alpha = atan2(tanAlphaY,-tanAlphaX);

	z1_origin_unit[0] = k_unit[0]*k_unit[1]*(1-cos(alpha))-k_unit[2]*sin(alpha);
	z1_origin_unit[1] = k_unit[1]*k_unit[1]*(1-cos(alpha))+cos(alpha);
	z1_origin_unit[2] = k_unit[1]*k_unit[2]*(1-cos(alpha))-k_unit[0]*sin(alpha);

		cross(z1_origin_unit,p4p_unit,z1_origin_vertical);
		UnitVector(z1_origin_vertical,z1_origin_vertical_unit);
	}
	else
	{
		if(p4p[0] < 0)
		{
			z1_origin_unit[0] = 0.0;
			z1_origin_unit[1] = 1.0;
			z1_origin_unit[2] = 0.0;

			z1_origin_vertical_unit[0] = 0.0;
			z1_origin_vertical_unit[1] = 0.0;
			z1_origin_vertical_unit[2] = 1.0;
		}
		if(p4p[0] > 0)
		{
			z1_origin_unit[0] = 0.0;
			z1_origin_unit[1] = -1.0;
			z1_origin_unit[2] = 0.0;

			z1_origin_vertical_unit[0] = 0.0;
			z1_origin_vertical_unit[1] = 0.0;
			z1_origin_vertical_unit[2] = 1.0;
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
		p1_unit[i] = p4p_unit[i]*cos(an3) + (-p4p_vertical_unit[i])*sin(an3);

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

	if (fabs(sin(angle_out[0][2])) < 1e-5)
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

	//i = 1Ê±
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

	//i = 3Ê±
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
	T2p[0][0] = cos(angle_out[2][4]);
	T2p[0][1] = -sin(angle_out[2][4]);
	T2p[1][0] = sin(angle_out[2][4]);
	T2p[1][1] = cos(angle_out[2][4]);
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
		if (fabs(sin(angle_out[i][6])) < 1e-5)
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
		if (fabs(sin(angle_out[i][6])) < 1e-5)
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

  select_inv(anglein, angle_out, angle_out2);
	return 0;
}


void select_inv(double angle_last[8], double angle_in[4][8], double angle_out[8])
{
	int i, j;
	double min_error = 1000000.0;
	double error;
	int inv_nmb = 0;

	for(i=0; i<4; i++)
	{
		error = 0.0;

		for(j=0; j<8; j++)
		{
			error = error + pow((angle_last[j] - angle_in[i][j]), 2);
		}

		if (error < min_error)
		{
			min_error = error;
			inv_nmb = i;
		}
	}

	for(i=0; i<8; i++)
	{
		angle_out[i] = angle_in[inv_nmb][i];
	}
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

double parabolic_interpolation(double x0, double xf, double vb, double acc, double t)
{
	double out, t_total, tb, pb, pb2,  tb2, t1, delta_x, vb2;

	if(xf < x0)
	{
		acc = -acc;
		vb = -vb;
	}

	delta_x = fabs(vb*vb / acc);

	if(fabs(xf - x0) >= delta_x)
	{
		t1 = (xf - x0 - vb*vb/acc)/vb;
		tb = vb/acc;
		pb = 0.5*acc*tb*tb;
		pb2 = pb + vb*t1;
		t_total = 2.0*tb + t1;
		tb2 = t_total - tb;
		vb2= vb;
	}
	else
	{
		t1 = 0.0;
		tb = sqrt((xf -x0)/acc);
		pb = 0.5*acc*tb*tb;
		pb2 = pb;
		t_total = 2.0*tb + t1;
		tb2 = t_total - tb;
		vb2 = acc*tb;
	}

//	printf("tb=%f,  t1=%f,  pb=%f, pb2=%f, t_total= %f, tb2=%f\n", tb,  t1,  pb, pb2, t_total, tb2);





//	vb = acc*tb;


	if(t < tb)
	{
		out = 0.5*acc*t*t + x0;
	}
	else if(t < tb2)
	{
		out = pb + vb2*(t - tb) + x0;
	}
	else
	{
		out = pb2 + vb2*(t-tb2) - 0.5*acc*(t - tb2)*(t - tb2) + x0;
	}
	return out;
}

double parabolic_interpolation_time(double x0, double xf, double vb, double acc)
{
	double out, t_total, tb, pb, pb2,  tb2, t1, delta_x;

	if(xf < x0)
	{
		acc = -acc;
		vb = -vb;
	}

	delta_x = fabs(vb*vb / acc);

	if(fabs(xf - x0) >= delta_x)
	{
		t1 = (xf - x0 - vb*vb/acc)/vb;
		tb = vb/acc;
	}
	else
	{
		t1 = 0.0;
		tb = sqrt((xf -x0)/acc);
	}


	t_total = 2.0*tb + t1;

	return t_total;
}


struct Cubic_Struct cubic[14];
int cubicAddPoint(int index, double point)
{
	if (cubic[index].needNextPoint == 0)
	{
		return -1;
	}


	if (cubic[index].filled == 0)
	{
		cubic[index].x0 = point;
		cubic[index].x1 = point;
		cubic[index].x2 = point;
		cubic[index].x3 = point;
		cubic[index].filled = 1;
	}
	else
	{
		cubic[index].x0 = cubic[index].x1;
		cubic[index].x1 = cubic[index].x2;
		cubic[index].x2 = cubic[index].x3;
		cubic[index].x3 = point;
	}

	cubic[index].wp0 = (cubic[index].x0 + 4*cubic[index].x1 + cubic[index].x2)/6.0;
	cubic[index].wp1 = (cubic[index].x1 + 4*cubic[index].x2 + cubic[index].x3)/6.0;
	cubic[index].vel0 = (cubic[index].x2 - cubic[index].x0)/(2.0*cubic[index].segmentTime);
	cubic[index].vel1 = (cubic[index].x3 - cubic[index].x1)/(2.0*cubic[index].segmentTime);
	cubic[index].coeffd = cubic[index].wp0;
	cubic[index].coeffc = cubic[index].vel0;
	cubic[index].coeffb = 3*(cubic[index].wp1 - cubic[index].wp0)/cubic[index].segmentTime/cubic[index].segmentTime - (2*cubic[index].vel0 + cubic[index].vel1)/cubic[index].segmentTime;
	cubic[index].coeffa = (cubic[index].vel1 - cubic[index].vel0)/(3*cubic[index].segmentTime*cubic[index].segmentTime) - (2*cubic[index].coeffb)/(3*cubic[index].segmentTime);

	cubic[index].interpolationTime = 0;
	cubic[index].needNextPoint = 0;

	return 1;
}

double cubicInterpolate(int index)
{
	if (cubic[index].needNextPoint == 1)
	{
		cubicAddPoint(index, cubic[index].x3);
	}

	cubic[index].interpolationTime = cubic[index].interpolationTime + cubic[index].interpolationIncrement;

	if (fabs(cubic[index].segmentTime - cubic[index].interpolationTime)<0.5*cubic[index].interpolationIncrement)
	{
		cubic[index].needNextPoint = 1;
	}

	double out = 0.0;
	out = cubic[index].coeffa*cubic[index].interpolationTime*cubic[index].interpolationTime*cubic[index].interpolationTime + cubic[index].coeffb*cubic[index].interpolationTime*cubic[index].interpolationTime + cubic[index].coeffc*cubic[index].interpolationTime + cubic[index].coeffd;
//	if(index == 0)
//	printf("cubictime = %f  out = %f\n", cubic[index].interpolationTime, out);
//	printf("cubictime = %f  x1 = %f  x2 = %f  x3 = %f  x4 = %f\n", cubic[index].interpolationTime, cubic[index].x0, cubic[index].x1, cubic[index].x2,cubic[index].x3);
	return out;
}


//3维叉乘
void VecCross(double va[3],double vb[3],double vre[3])
{
	vre[0] = va[1]*vb[2] - va[2]*vb[1];
	vre[1] = va[2]*vb[0] - va[0]*vb[2];
	vre[2] = va[0]*vb[1] - va[1]*vb[0];
}
//点乘的绝对值
double VecMulti(double va[3],double vb[3])
{
	double ans;

	ans = fabs(va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2]);

	return ans;
}

//*****************************************************************************
// void Matrix_Multiply(int iRow1, int iCol1, int iCol2, double* MA,
//						  double* MB, double* MC)
// 功能：矩阵相乘(MC=MA*MB)。
// 输入参数：
//     iRow1, iCol1, iCol2 矩阵MA的行数、列数，矩阵MB的列数
//     MA, MB            输入源矩阵MA,MB
// 输出参数：
//     MC                 输出矩阵MC=MA*MB　
// 返回值：无。
//*****************************************************************************
void Matrix_Multiply(int iRow1, int iCol1, int iCol2, double* MA, double* MB, double* MC)
{
	int i, j, n, u;

	for (i = 0; i < iRow1; i++)
	{
		for (j = 0; j < iCol2; j++)
		{
			u = i * iCol2 + j;
			MC[u] = 0.0;
			for (n = 0; n < iCol1; n++)
				MC[u] = MC[u] + MA[i*iCol1+n] * MB[n*iCol2+j];
		}
	}
	return;
}

// 碰撞检测相关
struct OBB_struct OBB_ArmL[7];
struct OBB_struct OBB_ArmR[7];
struct OBB_struct OBB_Base[3];

void OBB_Update(float AngleL[7],float AngleR[7],float AngleW[2])
{

	int i,j;

/*	for (i=0;i<7;i++)
	{
		AngleL[i] = AngleL[i]*M_PI/180;
		AngleR[i] = AngleR[i]*M_PI/180;
	}
	AngleW[0] = AngleW[0]*M_PI/180;
	AngleW[1] = -AngleW[1]*M_PI/180;
*/
	//左臂
	double T0L[4][4] = {{1.0, 0, 0, 0}, {0, -1.0, 0, 0}, {0, 0, -1.0, -250.5}, {0, 0, 0, 1.0}};
	double T1L[4][4] = {{cos(AngleL[0]), -sin(AngleL[0]), 0, 0}, {sin(AngleL[0]), cos(AngleL[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2L[4][4] = {{cos(AngleL[1]), -sin(AngleL[1]), 0, 0}, {0, 0, -1.0, 0}, {sin(AngleL[1]), cos(AngleL[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3L[4][4] = {{cos(AngleL[2]), -sin(AngleL[2]), 0, 0}, {0, 0, 1.0, d3}, {-sin(AngleL[2]), -cos(AngleL[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4L[4][4] = {{cos(AngleL[3]), -sin(AngleL[3]), 0, 0}, {0, 0, -1.0, 0}, {sin(AngleL[3]), cos(AngleL[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5L[4][4] = {{cos(AngleL[4]), -sin(AngleL[4]), 0, 0}, {0, 0, 1.0, d4}, {-sin(AngleL[4]), -cos(AngleL[4]), 0, 0}, {0, 0, 0, 1.0}};
	double T6L[4][4] = {{cos(AngleL[5]), -sin(AngleL[5]), 0, 0}, {0, 0, -1.0, 0}, {sin(AngleL[5]), cos(AngleL[5]), 0, 0},{0, 0, 0, 1.0}};
	double T7L[4][4] = {{cos(AngleL[6]), -sin(AngleL[6]), 0, 0}, {0, 0, 1.0, 0}, {-sin(AngleL[6]), -cos(AngleL[6]), 0, 0},{0, 0, 0, 1.0}};
	double T01L[4][4],T02L[4][4],T03L[4][4],T04L[4][4],T05L[4][4],T06L[4][4],T07L[4][4];

	matrix_multiply(T0L,T1L,T01L);
	matrix_multiply(T01L,T2L,T02L);
	matrix_multiply(T02L,T3L,T03L);
	matrix_multiply(T03L,T4L,T04L);
	matrix_multiply(T04L,T5L,T05L);
	matrix_multiply(T05L,T6L,T06L);
	matrix_multiply(T06L,T7L,T07L);

	//计算包围盒的3条轴
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
		  OBB_ArmL[0].Rt[i][j] = T01L[i][j];
		  OBB_ArmL[1].Rt[i][j] = T02L[i][j];
		  OBB_ArmL[2].Rt[i][j] = T03L[i][j];
		  OBB_ArmL[3].Rt[i][j] = T04L[i][j];
		  OBB_ArmL[4].Rt[i][j] = T05L[i][j];
		  OBB_ArmL[5].Rt[i][j] = T06L[i][j];
		  OBB_ArmL[6].Rt[i][j] = T07L[i][j];
		}
	}

	//计算包围盒中心位置
	double ini_posL[7][4] = {
							  {0,0,-82,1},
							  {0,0,0,1},
							  {0,0,-155,1},
							  {0,0,0,1},
							  {0,0,-168.5,1},
							  {0,0,0,1},
							  {0,0,232.5,1}
	                         };
	double now_posL[7][4] = {0};

	Matrix_Multiply(4,4,1,*T01L,*ini_posL,*now_posL);
	Matrix_Multiply(4,4,1,*T02L,*(ini_posL+1),*(now_posL+1));
	Matrix_Multiply(4,4,1,*T03L,*(ini_posL+2),*(now_posL+2));
	Matrix_Multiply(4,4,1,*T04L,*(ini_posL+3),*(now_posL+3));
	Matrix_Multiply(4,4,1,*T05L,*(ini_posL+4),*(now_posL+4));
	Matrix_Multiply(4,4,1,*T06L,*(ini_posL+5),*(now_posL+5));
	Matrix_Multiply(4,4,1,*T07L,*(ini_posL+6),*(now_posL+6));

	for (i=0;i<7;i++)
	{
		for (j=0;j<3;j++)
		 OBB_ArmL[i].pos[j] = now_posL[i][j];
	}

	//给定包围盒的尺寸
	OBB_ArmL[0].lwh[0] = 114;OBB_ArmL[0].lwh[1] = 114;OBB_ArmL[0].lwh[2] = 53;
	OBB_ArmL[1].lwh[0] = 110;OBB_ArmL[1].lwh[1] = 110;OBB_ArmL[1].lwh[2] = 166;
	OBB_ArmL[2].lwh[0] = 136;OBB_ArmL[2].lwh[1] = 136;OBB_ArmL[2].lwh[2] = 200;
	OBB_ArmL[3].lwh[0] = 110;OBB_ArmL[3].lwh[1] = 110;OBB_ArmL[3].lwh[2] = 166;
	OBB_ArmL[4].lwh[0] = 136;OBB_ArmL[4].lwh[1] = 136;OBB_ArmL[4].lwh[2] = 227;
	OBB_ArmL[5].lwh[0] = 110;OBB_ArmL[5].lwh[1] = 110;OBB_ArmL[5].lwh[2] = 166;
	OBB_ArmL[6].lwh[0] = 150;OBB_ArmL[6].lwh[1] = 150;OBB_ArmL[6].lwh[2] = 354;

	//右臂
	double T0R[4][4] = {{1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 250.5}, {0, 0, 0 , 1.0}};
	double T1R[4][4] = {{cos(AngleR[0]), -sin(AngleR[0]), 0, 0}, {sin(AngleR[0]), cos(AngleR[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2R[4][4] = {{cos(AngleR[1]), -sin(AngleR[1]), 0, 0}, {0, 0, 1.0, 0}, {-sin(AngleR[1]), -cos(AngleR[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3R[4][4] = {{cos(AngleR[2]), -sin(AngleR[2]), 0, 0}, {0, 0, -1.0, -d3}, {sin(AngleR[2]), cos(AngleR[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4R[4][4] = {{cos(AngleR[3]), -sin(AngleR[3]), 0, 0}, {0, 0, 1.0, 0}, {-sin(AngleR[3]), -cos(AngleR[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5R[4][4] = {{cos(AngleR[4]), -sin(AngleR[4]), 0, 0}, {0, 0, -1.0, -d4}, {sin(AngleR[4]), cos(AngleR[4]), 0, 0}, {0, 0, 0, 1.0}};
	double T6R[4][4] = {{cos(AngleR[5]), -sin(AngleR[5]), 0, 0}, {0, 0, 1.0, 0}, {-sin(AngleR[5]), -cos(AngleR[5]), 0, 0},{0, 0, 0, 1.0}};
	double T7R[4][4] = {{cos(AngleR[6]), -sin(AngleR[6]), 0, 0}, {0, 0, -1.0, 0}, {sin(AngleR[6]), cos(AngleR[6]), 0, 0},{0, 0, 0, 1.0}};
	double T01R[4][4],T02R[4][4],T03R[4][4],T04R[4][4],T05R[4][4],T06R[4][4],T07R[4][4];

	matrix_multiply(T0R,T1R,T01R);
	matrix_multiply(T01R,T2R,T02R);
	matrix_multiply(T02R,T3R,T03R);
	matrix_multiply(T03R,T4R,T04R);
	matrix_multiply(T04R,T5R,T05R);
	matrix_multiply(T05R,T6R,T06R);
	matrix_multiply(T06R,T7R,T07R);

	//计算包围盒的3条轴
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			OBB_ArmR[0].Rt[i][j] = T01R[i][j];
			OBB_ArmR[1].Rt[i][j] = T02R[i][j];
			OBB_ArmR[2].Rt[i][j] = T03R[i][j];
			OBB_ArmR[3].Rt[i][j] = T04R[i][j];
			OBB_ArmR[4].Rt[i][j] = T05R[i][j];
			OBB_ArmR[5].Rt[i][j] = T06R[i][j];
			OBB_ArmR[6].Rt[i][j] = T07R[i][j];
		}
	}

	//计算包围盒中心位置
	double ini_posR[7][4] = {
								{0,0,-82,1},
								{0,0,0,1},
								{0,0,-155.5,1},
								{0,0,0,1},
								{0,0,-168.5,1},
								{0,0,0,1},
								{0,0,232.5,1}
	                         };

	double now_posR[7][4] = {0};

	Matrix_Multiply(4,4,1,*T01R,*ini_posR,*now_posR);
	Matrix_Multiply(4,4,1,*T02R,*(ini_posR+1),*(now_posR+1));
	Matrix_Multiply(4,4,1,*T03R,*(ini_posR+2),*(now_posR+2));
	Matrix_Multiply(4,4,1,*T04R,*(ini_posR+3),*(now_posR+3));
	Matrix_Multiply(4,4,1,*T05R,*(ini_posR+4),*(now_posR+4));
	Matrix_Multiply(4,4,1,*T06R,*(ini_posR+5),*(now_posR+5));
	Matrix_Multiply(4,4,1,*T07R,*(ini_posR+6),*(now_posR+6));

	for (i=0;i<7;i++)
	{
		for (j=0;j<3;j++)
			OBB_ArmR[i].pos[j] = now_posR[i][j];
	}

	//给定包围盒的尺寸
	OBB_ArmR[0].lwh[0] = 114;OBB_ArmR[0].lwh[1] = 114;OBB_ArmR[0].lwh[2] = 53;
	OBB_ArmR[1].lwh[0] = 110;OBB_ArmR[1].lwh[1] = 110;OBB_ArmR[1].lwh[2] = 166;
	OBB_ArmR[2].lwh[0] = 136;OBB_ArmR[2].lwh[1] = 136;OBB_ArmR[2].lwh[2] = 200;
	OBB_ArmR[3].lwh[0] = 110;OBB_ArmR[3].lwh[1] = 110;OBB_ArmR[3].lwh[2] = 166;
	OBB_ArmR[4].lwh[0] = 136;OBB_ArmR[4].lwh[1] = 136;OBB_ArmR[4].lwh[2] = 227;
	OBB_ArmR[5].lwh[0] = 110;OBB_ArmR[5].lwh[1] = 110;OBB_ArmR[5].lwh[2] = 166;
	OBB_ArmR[6].lwh[0] = 150;OBB_ArmR[6].lwh[1] = 150;OBB_ArmR[6].lwh[2] = 354;

	//OBB_Base依次为从上至下3个包围盒

	//头、胸腔、腰OBB_Base[0]
	OBB_Base[0].Rt[0][0] = 1.0;OBB_Base[0].Rt[0][1] = 0;OBB_Base[0].Rt[0][2] = 0;OBB_Base[0].Rt[0][3] = 0;
	OBB_Base[0].Rt[1][0] = 0;OBB_Base[0].Rt[1][1] = 1.0;OBB_Base[0].Rt[1][2] = 0;OBB_Base[0].Rt[1][3] = 0;
	OBB_Base[0].Rt[2][0] = 0;OBB_Base[0].Rt[2][1] = 0;OBB_Base[0].Rt[2][2] = 1.0;OBB_Base[0].Rt[2][3] = 0;
	OBB_Base[0].Rt[3][0] = 0;OBB_Base[0].Rt[3][1] = 0;OBB_Base[0].Rt[3][2] = 0;OBB_Base[0].Rt[3][3] = 1.0;

	OBB_Base[0].pos[0] = -31;OBB_Base[0].pos[1] = 0;OBB_Base[0].pos[2] = 0;

	OBB_Base[0].lwh[0] = 840;OBB_Base[0].lwh[1] = 300;OBB_Base[0].lwh[2] = 350;

	//底座 AngleW中第一个角度为旋转 第二个为俯仰
	//包围盒的坐标轴 OBB_Base[1] OBB_Base[2]
	double T0w[4][4] = {{1.0, 0, 0, -116.81}, {0, 1.0, 0, 0}, {0, 0, 1.0, 307}, {0, 0, 0 , 1.0}};
	double T1w[4][4] = {{cos(AngleW[0]), -sin(AngleW[0]), 0, 0}, {sin(AngleW[0]), cos(AngleW[0]), 0, 0}, {0, 0, 1.0, 208}, {0, 0, 0, 1.0}};
	double T2w[4][4] = {{cos(AngleW[1]), -sin(AngleW[1]), 0, 0}, {0, 0, 1.0, 0},{-sin(AngleW[1]), -cos(AngleW[1]), 0, 0},  {0, 0, 0, 1.0}};
	double Tb2[4][4] = {{0, 1.0, 0, 0}, {-1.0, 0, 0, -350}, {0, 0, 1.0, 0}, {0, 0, 0 , 1.0}};

	double Ttemp1[4][4],Ttemp2[4][4],Ttemp3[4][4],Ttemp4[4][4],Ttemp5[4][4];

	matrix_multiply(T1w,T2w,Ttemp1);
	matrix_multiply(Ttemp1,Tb2,Ttemp2);
	matrix_multiply(T0w,Ttemp2,Ttemp3);

    inv_matrix(Ttemp2, Ttemp4);//Ttemp4为OBB_Base[1]相对于两臂之间基坐标系
	inv_matrix(Ttemp3, Ttemp5);//Ttemp5为OBB_Base[2]相对于两臂之间基坐标系

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			OBB_Base[1].Rt[i][j] = Ttemp4[i][j];
			OBB_Base[2].Rt[i][j] = Ttemp5[i][j];
		}
	}

	//包围盒的中心位置
	double ini_posB[2][4] = {
		{0,0,0,1},
		{0,0,0,1}
	};

	double now_posB[2][4] = {0};

	Matrix_Multiply(4,4,1,*Ttemp4,*ini_posB,*now_posB);
	Matrix_Multiply(4,4,1,*Ttemp5,*(ini_posB+1),*(now_posB+1));

	for (i=1;i<3;i++)
	{
		for (j=0;j<3;j++)
			OBB_Base[i].pos[j] = now_posB[i-1][j];
	}

	//包围盒的尺寸
	OBB_Base[1].lwh[0] = 300;OBB_Base[1].lwh[1] = 360;OBB_Base[1].lwh[2] = 214;
	OBB_Base[2].lwh[0] = 1050;OBB_Base[2].lwh[1] = 800;OBB_Base[2].lwh[2] = 400;
}

int OBB_Collision(struct OBB_struct OStr1,struct OBB_struct OStr2)
{
	int OBB_Re = 0;
	int i,j;

	//计算15个分离轴(均为单位向量)
	double separate_axis[15][3] = {0};

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			separate_axis[i][j] = OStr1.Rt[j][i];
			separate_axis[i+3][j] = OStr2.Rt[j][i];
		}
	}

	int k = 6;
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			VecCross(separate_axis[i],separate_axis[j+3],separate_axis[k]);
			k++;
		}
	}

	for (i=0;i<15;i++)
	{
		double axis_length = 0.0;
		axis_length = separate_axis[i][0]*separate_axis[i][0] + separate_axis[i][1]*separate_axis[i][1] + separate_axis[i][2]*separate_axis[i][2];

		for (j=0;j<3;j++)
		{
			if(axis_length != 0)
			   separate_axis[i][j] = separate_axis[i][j]/axis_length;
		}
	}


	//检测
	double CCvec[3];

	for (j=0;j<3;j++)
		CCvec[j] = OStr1.pos[j] - OStr2.pos[j];

	double CClength,RAlength,RAlength1,RAlength2;

	for (i=0;i<15;i++)
	{
		RAlength1 = 0.0;
		RAlength2 = 0.0;

	        //中心点连线在分离轴上的投影长度
			CClength = VecMulti(CCvec,separate_axis[i]);

			//半径投影长度之和
			for (j=0;j<3;j++)
			{
				RAlength1 = RAlength1 + VecMulti(separate_axis[j],separate_axis[i])*OStr1.lwh[j];
				RAlength2 = RAlength2 + VecMulti(separate_axis[j+3],separate_axis[i])*OStr2.lwh[j];
			}
			RAlength = (RAlength1 + RAlength2)/2.0;

			if (CClength > RAlength)
				{
					OBB_Re = 1;
					break;;          //只要存在一条分离轴即表示未发生碰撞
			    }

	}

	//结果为0表示发生碰撞，结果为1表示未发生碰撞
	return OBB_Re;
}

int CollisionDetection(float jL[7],float jR[7],float jW[2])
{
	int TestResult;
	TestResult = 1;
	double ReLR[7][7] = {1};//左右臂之间的碰撞
	double ReLB[5][3] = {1};//左臂（3关节开始）和基座之间的碰撞
	double ReRB[5][3] = {1};//右臂（3关节开始）和基座之间的碰撞

	int i,j;
	//更新包围盒
	OBB_Update(jL,jR,jW);

	//检测两臂之间
	for (i=0;i<7;i++)
	{
		for (j=0;j<7;j++)
		{
			ReLR[i][j] = OBB_Collision(OBB_ArmL[i],OBB_ArmR[j]);
			if (ReLR[i][j] == 0)
			    TestResult = 0;
		}
	}

	for (i=0;i<5;i++)
	{
		for (j=0;j<3;j++)
		{
			ReLB[i][j] = OBB_Collision(OBB_ArmL[i+2],OBB_Base[j]);
			ReRB[i][j] = OBB_Collision(OBB_ArmR[i+2],OBB_Base[j]);

			ReLB[0][0] = 1;
			ReRB[0][0] = 1;
			if ((ReLB[i][j] == 0)||(ReRB[i][j] == 0))
				TestResult = 0;
		}
	}
	//结果为0表示发生碰撞，结果为1表示未发生碰撞
	return TestResult;
}



void JacobianL(double anglein[7], double JT[6][7])
{
	double x1 = anglein[0]/57.3;
	double x2 = anglein[1]/57.3;
	double x3 = anglein[2]/57.3;
	double x4 = anglein[3]/57.3;
	double x5 = anglein[4]/57.3;
	double x6 = anglein[5]/57.3;
	double x7 = anglein[6]/57.3;
	double sqrt3c2 = 0.8660254;

	//求Jv
	double Jv0[3][7];
	double Jv7[3][7];

	Jv0[0][0] = d4*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) + dend*(cos(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) + sin(x6)*(cos(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)))) + d3*sin(x1)*sin(x2);
	Jv0[0][1] = dend*(sin(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x4) + cos(x1)*cos(x3)*cos(x4)*sin(x2)) - cos(x1)*sin(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x1)*cos(x2)*cos(x4) - cos(x1)*cos(x3)*sin(x2)*sin(x4))) - d4*(cos(x1)*cos(x2)*cos(x4) - cos(x1)*cos(x3)*sin(x2)*sin(x4)) - d3*cos(x1)*cos(x2);
	Jv0[0][2] = d4*sin(x4)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3)) - dend*(sin(x6)*(sin(x5)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x4)*cos(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))) - cos(x6)*sin(x4)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3)));
	Jv0[0][3] = dend*(cos(x6)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) - cos(x5)*sin(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2))) + d4*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4));
	Jv0[0][4] = -dend*sin(x6)*(sin(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) - cos(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3)));
	Jv0[0][5] = -dend*(sin(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) - cos(x6)*(cos(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))));
	Jv0[0][6] = 0;

	Jv0[1][0] = dend*(cos(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) + sin(x6)*(cos(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3)))) + d4*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) - d3*cos(x1)*sin(x2);
	Jv0[1][1] = dend*(sin(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x4) + cos(x3)*cos(x4)*sin(x1)*sin(x2)) - sin(x1)*sin(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x2)*cos(x4)*sin(x1) - cos(x3)*sin(x1)*sin(x2)*sin(x4))) - d4*(cos(x2)*cos(x4)*sin(x1) - cos(x3)*sin(x1)*sin(x2)*sin(x4)) - d3*cos(x2)*sin(x1);
	Jv0[1][2] = dend*(sin(x6)*(sin(x5)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - cos(x4)*cos(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3))) - cos(x6)*sin(x4)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3))) - d4*sin(x4)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3));
	Jv0[1][3] = - d4*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) - dend*(cos(x6)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) - cos(x5)*sin(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)));
	Jv0[1][4] = dend*sin(x6)*(sin(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) - cos(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)));
	Jv0[1][5] = dend*(sin(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) - cos(x6)*(cos(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3))));
	Jv0[1][6] = 0;

	Jv0[2][0] = 0;
	Jv0[2][1] = dend*(sin(x6)*(cos(x5)*(sin(x2)*sin(x4) - cos(x2)*cos(x3)*cos(x4)) + cos(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x4)*sin(x2) + cos(x2)*cos(x3)*sin(x4))) - d4*(cos(x4)*sin(x2) + cos(x2)*cos(x3)*sin(x4)) - d3*sin(x2);
	Jv0[2][2] = dend*(sin(x6)*(cos(x3)*sin(x2)*sin(x5) + cos(x4)*cos(x5)*sin(x2)*sin(x3)) + cos(x6)*sin(x2)*sin(x3)*sin(x4)) + d4*sin(x2)*sin(x3)*sin(x4);
	Jv0[2][3] = - d4*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) - dend*(cos(x6)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) + cos(x5)*sin(x6)*(cos(x2)*cos(x4) - cos(x3)*sin(x2)*sin(x4)));
	Jv0[2][4] = dend*sin(x6)*(sin(x5)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) + cos(x5)*sin(x2)*sin(x3));
	Jv0[2][5] = -dend*(cos(x6)*(cos(x5)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) - sin(x2)*sin(x3)*sin(x5)) + sin(x6)*(cos(x2)*cos(x4) - cos(x3)*sin(x2)*sin(x4)));
	Jv0[2][6] = 0;

	double T07[4][4];
	double T70[4][4];
	double R70[3][3];
	double xin[7];
	int i,j;

	for(i=0;i<7;i++)
		xin[i] = anglein[i]/57.3;

	KinL(xin,T07);
	inv_matrix(T07,T70);

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
			R70[i][j] = T70[i][j];
	}

	Matrix_Multiply(3,3,7,*R70,*Jv0,*Jv7);

	//求Jw
	double R1[3][3] = {{cos(xin[0]), -sin(xin[0]), 0}, {sin(xin[0]), cos(xin[0]), 0}, {0, 0, 1.0}};
	double R2[3][3] = {{cos(xin[1]), -sin(xin[1]), 0}, {0, 0, -1.0}, {sin(xin[1]), cos(xin[1]),0}};
	double R3[3][3] = {{cos(xin[2]), -sin(xin[2]), 0}, {0, 0, 1.0}, {-sin(xin[2]), -cos(xin[2]),0}};
	double R4[3][3] = {{cos(xin[3]), -sin(xin[3]), 0}, {0, 0, -1.0}, {sin(xin[3]), cos(xin[3]),0}};
	double R5[3][3] = {{cos(xin[4]), -sin(xin[4]), 0}, {0, 0, 1.0}, {-sin(xin[4]), -cos(xin[4]),0}};
	double R6[3][3] = {{cos(xin[5]), -sin(xin[5]), 0}, {0, 0, -1.0}, {sin(xin[5]), cos(xin[5]),0}};
	double R7[3][3] = {{cos(xin[6]), -sin(xin[6]), 0}, {0, 0, 1.0}, {-sin(xin[6]), -cos(xin[6]),0}};
	double Z[3] = {0,0,1};

	double J1w[3][1];
	double J2w[3][2];
	double J3w[3][3];
	double J4w[3][4];
	double J5w[3][5];
	double J6w[3][6];
	double J7w[3][7];
	double R2t[3][3];
	double R3t[3][3];
	double R4t[3][3];
	double R5t[3][3];
	double R6t[3][3];
	double R7t[3][3];

	Matrix_Trans3(R2,R2t);
	Matrix_Trans3(R3,R3t);
	Matrix_Trans3(R4,R4t);
	Matrix_Trans3(R5,R5t);
	Matrix_Trans3(R6,R6t);
	Matrix_Trans3(R7,R7t);

	Matrix_Multiply(3,3,1,*R2t,Z,*J1w);

	for (i = 0;i<3;i++)
	{
		J2w[i][0] = J1w[i][0];
		J2w[i][1] = Z[i];
	}

	double J3temp[3][2];
	Matrix_Multiply(3,3,2,*R3t,*J2w,*J3temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<2;j++)
			J3w[i][j] = J3temp[i][j];

		J3w[i][2] = Z[i];
	}

	double J4temp[3][3];
	Matrix_Multiply(3,3,3,*R4t,*J3w,*J4temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<3;j++)
			J4w[i][j] = J4temp[i][j];

		J4w[i][3] = Z[i];
	}

	double J5temp[3][4];
	Matrix_Multiply(3,3,4,*R5t,*J4w,*J5temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<4;j++)
			J5w[i][j] = J5temp[i][j];

		J5w[i][4] = Z[i];
	}

	double J6temp[3][5];
	Matrix_Multiply(3,3,5,*R6t,*J5w,*J6temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<5;j++)
			J6w[i][j] = J6temp[i][j];

		J6w[i][5] = Z[i];
	}

	double J7temp[3][6];
	Matrix_Multiply(3,3,6,*R7t,*J6w,*J7temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<6;j++)
			J7w[i][j] = J7temp[i][j];

		J7w[i][6] = Z[i];
	}


	//J7
	for (i = 0;i<3;i++)
	{
		for(j=0;j<7;j++)
			JT[i][j] = Jv7[i][j];

	}
	for (i = 0;i<3;i++)
	{
		for(j=0;j<7;j++)
			JT[i+3][j] = J7w[i][j];

	}
}

void JacobianR(double anglein[7], double JT[6][7])
{
	double x1 = anglein[0]/57.3;
	double x2 = anglein[1]/57.3;
	double x3 = anglein[2]/57.3;
	double x4 = anglein[3]/57.3;
	double x5 = anglein[4]/57.3;
	double x6 = anglein[5]/57.3;
	double x7 = anglein[6]/57.3;
	double sqrt3c2 = 0.8660254;

	//求Jv
	double Jv0[3][7];
	double Jv7[3][7];

	Jv0[0][0] = - d4*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) - dend*(cos(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) + sin(x6)*(cos(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)))) - d3*sin(x1)*sin(x2);
	Jv0[0][1] = d4*(cos(x1)*cos(x2)*cos(x4) - cos(x1)*cos(x3)*sin(x2)*sin(x4)) - dend*(sin(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x4) + cos(x1)*cos(x3)*cos(x4)*sin(x2)) - cos(x1)*sin(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x1)*cos(x2)*cos(x4) - cos(x1)*cos(x3)*sin(x2)*sin(x4))) + d3*cos(x1)*cos(x2);
	Jv0[0][2] = dend*(sin(x6)*(sin(x5)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x4)*cos(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))) - cos(x6)*sin(x4)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))) - d4*sin(x4)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3));
	Jv0[0][3] = - dend*(cos(x6)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) - cos(x5)*sin(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2))) - d4*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4));
	Jv0[0][4] = dend*sin(x6)*(sin(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) - cos(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3)));
	Jv0[0][5] = dend*(sin(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) - cos(x6)*(cos(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))));
	Jv0[0][6] = 0;

	Jv0[1][0] = d3*cos(x1)*sin(x2) - d4*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) - dend*(cos(x6)*(sin(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x1)*cos(x4)*sin(x2)) + sin(x6)*(cos(x5)*(cos(x4)*(sin(x1)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) + cos(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x3)*sin(x1) + cos(x1)*cos(x2)*sin(x3))));
	Jv0[1][1] = d4*(cos(x2)*cos(x4)*sin(x1) - cos(x3)*sin(x1)*sin(x2)*sin(x4)) - dend*(sin(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x4) + cos(x3)*cos(x4)*sin(x1)*sin(x2)) - sin(x1)*sin(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x2)*cos(x4)*sin(x1) - cos(x3)*sin(x1)*sin(x2)*sin(x4))) + d3*cos(x2)*sin(x1);
	Jv0[1][2] = d4*sin(x4)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)) - dend*(sin(x6)*(sin(x5)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - cos(x4)*cos(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3))) - cos(x6)*sin(x4)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)));
	Jv0[1][3] = d4*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) + dend*(cos(x6)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) - cos(x5)*sin(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)));
	Jv0[1][4] = -dend*sin(x6)*(sin(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) - cos(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3)));
	Jv0[1][5] = -dend*(sin(x6)*(sin(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) + cos(x4)*sin(x1)*sin(x2)) - cos(x6)*(cos(x5)*(cos(x4)*(cos(x1)*sin(x3) + cos(x2)*cos(x3)*sin(x1)) - sin(x1)*sin(x2)*sin(x4)) + sin(x5)*(cos(x1)*cos(x3) - cos(x2)*sin(x1)*sin(x3))));
	Jv0[1][6] = 0;

	Jv0[2][0] = 0;
	Jv0[2][1] = dend*(sin(x6)*(cos(x5)*(sin(x2)*sin(x4) - cos(x2)*cos(x3)*cos(x4)) + cos(x2)*sin(x3)*sin(x5)) - cos(x6)*(cos(x4)*sin(x2) + cos(x2)*cos(x3)*sin(x4))) - d4*(cos(x4)*sin(x2) + cos(x2)*cos(x3)*sin(x4)) - d3*sin(x2);
	Jv0[2][2] = dend*(sin(x6)*(cos(x3)*sin(x2)*sin(x5) + cos(x4)*cos(x5)*sin(x2)*sin(x3)) + cos(x6)*sin(x2)*sin(x3)*sin(x4)) + d4*sin(x2)*sin(x3)*sin(x4);
	Jv0[2][3] = - d4*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) - dend*(cos(x6)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) + cos(x5)*sin(x6)*(cos(x2)*cos(x4) - cos(x3)*sin(x2)*sin(x4)));
	Jv0[2][4] = dend*sin(x6)*(sin(x5)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) + cos(x5)*sin(x2)*sin(x3));
	Jv0[2][5] = -dend*(cos(x6)*(cos(x5)*(cos(x2)*sin(x4) + cos(x3)*cos(x4)*sin(x2)) - sin(x2)*sin(x3)*sin(x5)) + sin(x6)*(cos(x2)*cos(x4) - cos(x3)*sin(x2)*sin(x4)));
	Jv0[2][6] = 0;

	double T07[4][4];
	double T70[4][4];
	double R70[3][3];
	double xin[7];

	int i,j;

	for(i=0;i<7;i++)
		xin[i] = anglein[i]/57.3;

	KinR(xin,T07);
	inv_matrix(T07,T70);

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
			R70[i][j] = T70[i][j];
	}

	Matrix_Multiply(3,3,7,*R70,*Jv0,*Jv7);

	//求Jw
	double R1[3][3] = {{cos(xin[0]), -sin(xin[0]), 0}, {sin(xin[0]), cos(xin[0]), 0}, {0, 0, 1.0}};
	double R2[3][3] = {{cos(xin[1]), -sin(xin[1]), 0}, {0, 0, 1.0}, {-sin(xin[1]), -cos(xin[1]),0}};
	double R3[3][3] = {{cos(xin[2]), -sin(xin[2]), 0}, {0, 0, -1.0}, {sin(xin[2]), cos(xin[2]),0}};
	double R4[3][3] = {{cos(xin[3]), -sin(xin[3]), 0}, {0, 0, 1.0}, {-sin(xin[3]), -cos(xin[3]),0}};
	double R5[3][3] = {{cos(xin[4]), -sin(xin[4]), 0}, {0, 0, -1.0}, {sin(xin[4]), cos(xin[4]),0}};
	double R6[3][3] = {{cos(xin[5]), -sin(xin[5]), 0}, {0, 0, 1.0}, {-sin(xin[5]), -cos(xin[5]),0}};
	double R7[3][3] = {{cos(xin[6]), -sin(xin[6]), 0}, {0, 0, -1.0}, {sin(xin[6]), cos(xin[6]),0}};
	double Z[3] = {0,0,1};

	double J1w[3][1];
	double J2w[3][2];
	double J3w[3][3];
	double J4w[3][4];
	double J5w[3][5];
	double J6w[3][6];
	double J7w[3][7];
	double R2t[3][3];
	double R3t[3][3];
	double R4t[3][3];
	double R5t[3][3];
	double R6t[3][3];
	double R7t[3][3];

	Matrix_Trans3(R2,R2t);
	Matrix_Trans3(R3,R3t);
	Matrix_Trans3(R4,R4t);
	Matrix_Trans3(R5,R5t);
	Matrix_Trans3(R6,R6t);
	Matrix_Trans3(R7,R7t);

	Matrix_Multiply(3,3,1,*R2t,Z,*J1w);

	for (i = 0;i<3;i++)
	{
		J2w[i][0] = J1w[i][0];
		J2w[i][1] = Z[i];
	}

	double J3temp[3][2];
	Matrix_Multiply(3,3,2,*R3t,*J2w,*J3temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<2;j++)
			J3w[i][j] = J3temp[i][j];

		J3w[i][2] = Z[i];
	}

	double J4temp[3][3];
	Matrix_Multiply(3,3,3,*R4t,*J3w,*J4temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<3;j++)
			J4w[i][j] = J4temp[i][j];

		J4w[i][3] = Z[i];
	}

	double J5temp[3][4];
	Matrix_Multiply(3,3,4,*R5t,*J4w,*J5temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<4;j++)
			J5w[i][j] = J5temp[i][j];

		J5w[i][4] = Z[i];
	}

	double J6temp[3][5];
	Matrix_Multiply(3,3,5,*R6t,*J5w,*J6temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<5;j++)
			J6w[i][j] = J6temp[i][j];

		J6w[i][5] = Z[i];
	}

	double J7temp[3][6];
	Matrix_Multiply(3,3,6,*R7t,*J6w,*J7temp);

	for (i = 0;i<3;i++)
	{
		for(j=0;j<6;j++)
			J7w[i][j] = J7temp[i][j];

		J7w[i][6] = Z[i];
	}


	//J7
	for (i = 0;i<3;i++)
	{
		for(j=0;j<7;j++)
			JT[i][j] = Jv7[i][j];

	}
	for (i = 0;i<3;i++)
	{
		for(j=0;j<7;j++)
			JT[i+3][j] = J7w[i][j];

	}
}

void InvJacobianL(double anglein[7], double InvJ[7][6])	// input: deg
{
	double J[6][7];
	double Jt[7][6];
	JacobianL(anglein,J);
	int i,j;

	for (i=0;i<7;i++)
	{
		for (j=0;j<6;j++)
		{
			Jt[i][j] = J[j][i];
		}
	}

	double Temp[7][7];
	double InvTemp[7][7];
	for (i=0;i<7;i++)
	{
		for (j=0;j<7;j++)
		{
			if (i == j)
				Temp[i][j] = 1;
			else
				Temp[i][j] = 0;
		}
	}
	Temp[0][0] = 30;
	Temp[3][3] = 66;

	Matrix_Reverse(7,*Temp,*InvTemp);

	double Ttemp1[7][6],Ttemp2[6][7],Ttemp3[6][6],Ttemp4[6][6];

	Matrix_Multiply(7,7,6,*InvTemp,*Jt,*Ttemp1);
	Matrix_Multiply(6,7,7,*J,*InvTemp,*Ttemp2);
	Matrix_Multiply(6,7,6,*Ttemp2,*Jt,*Ttemp3);
	Matrix_Reverse(6,*Ttemp3,*Ttemp4);
	Matrix_Multiply(7,6,6,*Ttemp1,*Ttemp4,*InvJ);
}

void InvJacobianR(double anglein[7], double InvJ[7][6])
{
	double J[6][7];
	double Jt[7][6];
	JacobianR(anglein,J);
	int i,j;

	for (i=0;i<7;i++)
	{
		for (j=0;j<6;j++)
		{
			Jt[i][j] = J[j][i];
		}
	}

	double Temp[7][7];
	double InvTemp[7][7];
	for (i=0;i<7;i++)
	{
		for (j=0;j<7;j++)
		{
			if (i == j)
				Temp[i][j] = 1;
			else
				Temp[i][j] = 0;
		}
	}
	Temp[0][0] = 30;
	Temp[3][3] = 66;

	Matrix_Reverse(7,*Temp,*InvTemp);

	double Ttemp1[7][6],Ttemp2[6][7],Ttemp3[6][6],Ttemp4[6][6];

	Matrix_Multiply(7,7,6,*InvTemp,*Jt,*Ttemp1);
	Matrix_Multiply(6,7,7,*J,*InvTemp,*Ttemp2);
	Matrix_Multiply(6,7,6,*Ttemp2,*Jt,*Ttemp3);
	Matrix_Reverse(6,*Ttemp3,*Ttemp4);
	Matrix_Multiply(7,6,6,*Ttemp1,*Ttemp4,*InvJ);
}


void KinL(double Angle[7], double TransMatrix[4][4])
{
	double T1[4][4] = {{cos(Angle[0]), -sin(Angle[0]), 0, 0}, {sin(Angle[0]), cos(Angle[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2[4][4] = {{cos(Angle[1]), -sin(Angle[1]), 0, 0}, {0, 0, -1.0, 0}, {sin(Angle[1]), cos(Angle[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3[4][4] = {{cos(Angle[2]), -sin(Angle[2]), 0, 0}, {0, 0, 1.0, d3}, {-sin(Angle[2]), -cos(Angle[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4[4][4] = {{cos(Angle[3]), -sin(Angle[3]), 0, 0}, {0, 0, -1.0, 0}, {sin(Angle[3]), cos(Angle[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5[4][4] = {{cos(Angle[4]), -sin(Angle[4]), 0, 0}, {0, 0, 1.0, d4}, {-sin(Angle[4]), -cos(Angle[4]), 0, 0}, {0, 0, 0, 1.0}};
	double T6[4][4] = {{cos(Angle[5]), -sin(Angle[5]), 0, 0}, {0, 0, -1.0, 0}, {sin(Angle[5]), cos(Angle[5]), 0, 0},{0, 0, 0, 1.0}};
	double T7[4][4] = {{cos(Angle[6]), -sin(Angle[6]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[6]), -cos(Angle[6]), 0, 0},{0, 0, 0, 1.0}};
	double T_Hand[4][4] = {{1.0, 0.0, 0, Tool_Position[0]}, {0, 1.0, 0.0, Tool_Position[1]}, {0.0, 0.0, 1.0, Tool_Position[2]},{0, 0, 0, 1.0}};


	double T02[4][4], T03[4][4], T04[4][4], T05[4][4], T06[4][4], T07[4][4];

	matrix_multiply(T1, T2, T02);
	matrix_multiply(T02, T3, T03);
	matrix_multiply(T03, T4, T04);
	matrix_multiply(T04, T5, T05);
	matrix_multiply(T05, T6, T06);
	matrix_multiply(T06, T7, T07);
	matrix_multiply(T07, T_Hand, TransMatrix);

	return ;
}
void KinR(double Angle[7], double TransMatrix[4][4])
{
	double T1[4][4] = {{cos(Angle[0]), -sin(Angle[0]), 0, 0}, {sin(Angle[0]), cos(Angle[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2[4][4] = {{cos(Angle[1]), -sin(Angle[1]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[1]), -cos(Angle[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3[4][4] = {{cos(Angle[2]), -sin(Angle[2]), 0, 0}, {0, 0, -1.0, -d3}, {sin(Angle[2]), cos(Angle[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4[4][4] = {{cos(Angle[3]), -sin(Angle[3]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[3]), -cos(Angle[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5[4][4] = {{cos(Angle[4]), -sin(Angle[4]), 0, 0}, {0, 0, -1.0, -d4}, {sin(Angle[4]), cos(Angle[4]), 0, 0}, {0, 0, 0, 1.0}};
	double T6[4][4] = {{cos(Angle[5]), -sin(Angle[5]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[5]), -cos(Angle[5]), 0, 0},{0, 0, 0, 1.0}};
	double T7[4][4] = {{cos(Angle[6]), -sin(Angle[6]), 0, 0}, {0, 0, -1.0, 0}, {sin(Angle[6]), cos(Angle[6]), 0, 0},{0, 0, 0, 1.0}};
	double T_Hand[4][4] = {{1.0, 0.0, 0, Tool_Position[0]}, {0, 1.0, 0.0, Tool_Position[1]}, {0.0, 0.0, 1.0, Tool_Position[2]},{0, 0, 0, 1.0}};


	double T02[4][4], T03[4][4], T04[4][4], T05[4][4], T06[4][4], T07[4][4];

	matrix_multiply(T1, T2, T02);
	matrix_multiply(T02, T3, T03);
	matrix_multiply(T03, T4, T04);
	matrix_multiply(T04, T5, T05);
	matrix_multiply(T05, T6, T06);
	matrix_multiply(T06, T7, T07);
	matrix_multiply(T07, T_Hand, TransMatrix);

	return ;
}


// 逆运动学
int invKinL(double Angle_now[7], double TransMatrix[4][4], double Beta, double Angle_cal[7])
{
	double px, py, pz;
	double alfa = 0.0;
	double ang1[4], ang2[4], ang3[4], ang4[4], ang5[4], ang6[4], ang7[4];
	double p[3], p_unit[3];
	double x0[3] = {1.0, 0.0, 0.0};
	double a[3], a_unit[3], a_unit_vertical[3], b_unit[3], z3[3], y3_1[3], y3[3], x3[3];
	double T1[4][4], T2[4][4], T3[4][4], T4[4][4], T02[4][4], T03[4][4], T04[4][4], T07[4][4], T57_2_1[4][4], T57_2_2[4][4], INV_T04[4][4];

	double T_Hand_inv[4][4] = {{1.0, 0.0, 0, -Tool_Position[0]}, {0, 1.0, 0.0, -Tool_Position[1]}, {0.0, 0.0, 1.0, -Tool_Position[2]},{0, 0, 0, 1.0}};

	int i,j;

	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			T1[i][j] = 0.0;
			T2[i][j] = 0.0;
			T3[i][j] = 0.0;
			T4[i][j] = 0.0;
		}
	}

	matrix_multiply(TransMatrix, T_Hand_inv, T07);

	px = T07[0][3];
	py = T07[1][3];
	pz = T07[2][3];

	alfa = acos(((px*px + py*py+ pz*pz) + d3*d3 - d4*d4)/(2.0*d3*sqrt(px*px + py*py+ pz*pz)));

	ang4[0] = acos(((px*px + py*py+ pz*pz) - d3*d3 - d4*d4)/(2.0*d3*d4));
	ang4[1] = ang4[0];
	ang4[2] = ang4[0];
	ang4[3] = ang4[0];

	p[0] = px;
	p[1] = py;
	p[2] = pz;

	for (i=0; i<3; i++)
	{
		p_unit[i] = p[i]/sqrt(px*px + py*py + pz*pz);
	}

	cross(x0, p_unit, a);
	for (i=0; i<3; i++)
	{
		a_unit[i] = a[i]/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}

	cross(p_unit, a_unit,a_unit_vertical);

	for (i=0; i<3; i++)
	{
		b_unit[i] = a_unit[i]*cos(Beta) + a_unit_vertical[i]*sin(Beta);
		z3[i] = p_unit[i]*cos(alfa) + b_unit[i]*sin(alfa);
	}


	if(fabs(ang4[0]) <1e-8)
	{
		ang2[0] = acos(z3[2]);
		ang2[1] = ang2[0];
		ang2[2] = -ang2[0];
		ang2[3] = -ang2[0];

		for(i=0; i<4; i++)
		{
			ang3[i] = Angle_now[2];
			if(fabs(ang2[i]) < 1e-6)
			{
				ang1[i] = Angle_now[0];
			}
			else
			{
				ang1[i] = atan2(-z3[1]/sin(ang2[i]), -z3[0]/sin(ang2[i]));
			}
		}
	}
	else
	{
		cross(p_unit,z3, y3_1);
		for (i=0; i<3; i++)
		{
			y3[i] = y3_1[i]/sqrt(y3_1[0]*y3_1[0] + y3_1[1]*y3_1[1] + y3_1[2]*y3_1[2]);
		}

		cross(y3, z3, x3);

		//		T32 = [x3 y3 z3];

		ang2[0] = acos(z3[2]);
		ang2[1] = ang2[0];
		ang2[2] = -ang2[0];
		ang2[3] = -ang2[0];

		for(i =0; i<4; i++)
		{
			if(fabs(ang2[i]) < 1e-6)
			{
				ang1[i] = Angle_now[0];
				ang3[i] = atan2(x3[1], x3[0]) - ang1[i];
			}
			else
			{
				ang1[i] = atan2(-z3[1]/sin(ang2[i]), -z3[0]/sin(ang2[i]));
				ang3[i] = atan2(-y3[2]/sin(ang2[i]), x3[2]/sin(ang2[i]));
			}
		}
	}


	T1[0][0] = cos(ang1[0]);
	T1[0][1] = -sin(ang1[0]);
	T1[1][0] = sin(ang1[0]);
	T1[1][1] = cos(ang1[0]);
	T1[2][2] = 1.0;
	T1[3][3] = 1.0;

	T2[0][0] = cos(ang2[0]);
	T2[0][1] = -sin(ang2[0]);
	T2[1][2] = -1.0;
	T2[2][0] = sin(ang2[0]);
	T2[2][1] = cos(ang2[0]);
	T2[3][3] = 1.0;

	T3[0][0] = cos(ang3[0]);
	T3[0][1] = -sin(ang3[0]);
	T3[1][2] = 1.0;
	T3[2][0] = -sin(ang3[0]);
	T3[2][1] = -cos(ang3[0]);
	T3[3][3] = 1.0;

	T4[0][0] = cos(ang4[0]);
	T4[0][1] = -sin(ang4[0]);
	T4[1][2] = -1.0;
	T4[2][0] = sin(ang4[0]);
	T4[2][1] = cos(ang4[0]);
	T4[3][3] = 1.0;


	matrix_multiply(T1,T2, T02);
	matrix_multiply(T02,T3, T03);
	matrix_multiply(T03,T4, T04);

	inv_matrix(T04, INV_T04);
	matrix_multiply(INV_T04, TransMatrix, T57_2_1);


	T1[0][0] = cos(ang1[2]);
	T1[0][1] = -sin(ang1[2]);
	T1[1][0] = sin(ang1[2]);
	T1[1][1] = cos(ang1[2]);
	T1[2][2] = 1.0;
	T1[3][3] = 1.0;

	T2[0][0] = cos(ang2[2]);
	T2[0][1] = -sin(ang2[2]);
	T2[1][2] = -1.0;
	T2[2][0] = sin(ang2[2]);
	T2[2][1] = cos(ang2[2]);
	T2[3][3] = 1.0;

	T3[0][0] = cos(ang3[2]);
	T3[0][1] = -sin(ang3[2]);
	T3[1][2] = 1.0;
	T3[2][0] = -sin(ang3[2]);
	T3[2][1] = -cos(ang3[2]);
	T3[3][3] = 1.0;

	T4[0][0] = cos(ang4[2]);
	T4[0][1] = -sin(ang4[2]);
	T4[1][2] = -1.0;
	T4[2][0] = sin(ang4[2]);
	T4[2][1] = cos(ang4[2]);
	T4[3][3] = 1.0;


	matrix_multiply(T1,T2, T02);
	matrix_multiply(T02,T3, T03);
	matrix_multiply(T03,T4, T04);

	inv_matrix(T04, INV_T04);
	matrix_multiply(INV_T04, TransMatrix, T57_2_2);

	ang6[0] = acos(T57_2_1[1][2]);
	ang6[1] = -acos(T57_2_1[1][2]);
	ang6[2] = acos(T57_2_2[1][2]);
	ang6[3] = -acos(T57_2_2[1][2]);

	for (i = 0; i<2; i++)
	{
		if(fabs(ang6[i]) < 1e-6)
		{
			ang5[i] = Angle_now[4];
			ang7[i] = atan2(-T57_2_1[2][0], T57_2_1[0][0]) - ang5[i];
		}
		else
		{
			ang5[i] = atan2(T57_2_1[2][2]/sin(ang6[i]), -T57_2_1[0][2]/sin(ang6[i]));
			ang7[i] = atan2(-T57_2_1[1][1]/sin(ang6[i]), T57_2_1[1][0]/sin(ang6[i]));
		}
	}

	for(i=2; i<4; i++)
	{
		if(fabs(ang6[i]) < 1e-6)
		{
			ang5[i] = Angle_now[4];
			ang7[i] = atan2(-T57_2_2[2][0], T57_2_2[0][0]) - ang5[i];
		}
		else
		{
			ang5[i] = atan2(T57_2_2[2][2]/sin(ang6[i]), -T57_2_2[0][2]/sin(ang6[i]));
			ang7[i] = atan2(-T57_2_2[1][1]/sin(ang6[i]), T57_2_2[1][0]/sin(ang6[i]));
		}
	}


	double Solve[4][7];
	for(i=0; i<4; i++)
	{
		Solve[i][0] = ang1[i];
		Solve[i][1] = ang2[i];
		Solve[i][2] = ang3[i];
		Solve[i][3] = ang4[i];
		Solve[i][4] = ang5[i];
		Solve[i][5] = ang6[i];
		Solve[i][6] = ang7[i];
	}

	int index = ChooseSolve(Angle_now,Solve);
	for (i=0;i<7;i++)
	{
		Angle_cal[i] = Solve[index][i]*180/3.1415926;
	}
	return 0;
}
int invKinR(double Angle_now[7], double TransMatrix[4][4], double Beta, double Angle_cal[7])
{
	double px, py, pz;
	double alfa = 0.0;
	double ang1[4], ang2[4], ang3[4], ang4[4], ang5[4], ang6[4], ang7[4];
	double p[3], p_unit[3];
	double x0[3] = {1.0, 0.0, 0.0};
	double a[3], a_unit[3], a_unit_vertical[3], b_unit[3], z3[3], y3_1[3], y3[3], x3[3];
	double T1[4][4], T2[4][4], T3[4][4], T4[4][4], T02[4][4], T03[4][4], T04[4][4], T07[4][4], T57_2_1[4][4], T57_2_2[4][4], INV_T04[4][4];

	double T_Hand_inv[4][4] = {{1.0, 0.0, 0, -Tool_Position[0]}, {0, 1.0, 0.0, -Tool_Position[1]}, {0.0, 0.0, 1.0, -Tool_Position[2]},{0, 0, 0, 1.0}};

	int i,j;

	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			T1[i][j] = 0.0;
			T2[i][j] = 0.0;
			T3[i][j] = 0.0;
			T4[i][j] = 0.0;
		}
	}

	matrix_multiply(TransMatrix, T_Hand_inv, T07);

	px = T07[0][3];
	py = T07[1][3];
	pz = T07[2][3];

	alfa = acos(((px*px + py*py+ pz*pz) + d3*d3 - d4*d4)/(2.0*d3*sqrt(px*px + py*py+ pz*pz)));

	ang4[0] = -acos(((px*px + py*py+ pz*pz) - d3*d3 - d4*d4)/(2.0*d3*d4));
	ang4[1] = ang4[0];
	ang4[2] = ang4[0];
	ang4[3] = ang4[0];

	p[0] = px;
	p[1] = py;
	p[2] = pz;

	for (i=0; i<3; i++)
	{
		p_unit[i] = p[i]/sqrt(px*px + py*py + pz*pz);
	}

	cross(x0, p_unit, a);
	for (i=0; i<3; i++)
	{
		a_unit[i] = a[i]/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}

	//cross( a_unit,p_unit,a_unit_vertical);
	cross(p_unit, a_unit, a_unit_vertical);

	for (i=0; i<3; i++)
	{
		b_unit[i] = a_unit[i]*cos(Beta) + a_unit_vertical[i]*sin(Beta);
		z3[i] = p_unit[i]*cos(alfa) + b_unit[i]*sin(alfa);
	}


	if(fabs(ang4[0]) <1e-8)
	{
		ang2[0] = acos(z3[2]);
		ang2[1] = ang2[0];
		ang2[2] = -ang2[0];
		ang2[3] = -ang2[0];

		for(i=0; i<4; i++)
		{
			ang3[i] = Angle_now[2];
			if(fabs(ang2[i]) < 1e-6)
			{
				ang1[i] = Angle_now[0];
			}
			else
			{
				ang1[i] = atan2(z3[1]/sin(ang2[i]), z3[0]/sin(ang2[i]));
			}
		}
	}
	else
	{
		cross(p_unit,z3, y3_1);
		for (i=0; i<3; i++)
		{
			y3[i] = y3_1[i]/sqrt(y3_1[0]*y3_1[0] + y3_1[1]*y3_1[1] + y3_1[2]*y3_1[2]);
		}

		cross(y3, z3, x3);

		//		T32 = [x3 y3 z3];

		ang2[0] = acos(z3[2]);
		ang2[1] = ang2[0];
		ang2[2] = -ang2[0];
		ang2[3] = -ang2[0];

		for(i =0; i<4; i++)
		{
			if(fabs(ang2[i]) < 1e-6)
			{
				ang1[i] = Angle_now[0];
				ang3[i] = atan2(x3[1], x3[0]) - ang1[i];
			}
			else
			{
				ang1[i] = atan2(z3[1]/sin(ang2[i]), z3[0]/sin(ang2[i]));
				ang3[i] = atan2(y3[2]/sin(ang2[i]), -x3[2]/sin(ang2[i]));
			}
		}
	}


	T1[0][0] = cos(ang1[0]);
	T1[0][1] = -sin(ang1[0]);
	T1[1][0] = sin(ang1[0]);
	T1[1][1] = cos(ang1[0]);
	T1[2][2] = 1.0;
	T1[3][3] = 1.0;

	T2[0][0] = cos(ang2[0]);
	T2[0][1] = -sin(ang2[0]);
	T2[1][2] = 1.0;
	T2[2][0] = -sin(ang2[0]);
	T2[2][1] = -cos(ang2[0]);
	T2[3][3] = 1.0;

	T3[0][0] = cos(ang3[0]);
	T3[0][1] = -sin(ang3[0]);
	T3[1][2] = -1.0;
	T3[2][0] = sin(ang3[0]);
	T3[2][1] = cos(ang3[0]);
	T3[3][3] = 1.0;

	T4[0][0] = cos(ang4[0]);
	T4[0][1] = -sin(ang4[0]);
	T4[1][2] = 1.0;
	T4[2][0] = -sin(ang4[0]);
	T4[2][1] = -cos(ang4[0]);
	T4[3][3] = 1.0;


	matrix_multiply(T1,T2, T02);
	matrix_multiply(T02,T3, T03);
	matrix_multiply(T03,T4, T04);

	inv_matrix(T04, INV_T04);
	matrix_multiply(INV_T04, TransMatrix, T57_2_1);


	T1[0][0] = cos(ang1[2]);
	T1[0][1] = -sin(ang1[2]);
	T1[1][0] = sin(ang1[2]);
	T1[1][1] = cos(ang1[2]);
	T1[2][2] = 1.0;
	T1[3][3] = 1.0;

	T2[0][0] = cos(ang2[2]);
	T2[0][1] = -sin(ang2[2]);
	T2[1][2] = 1.0;
	T2[2][0] = -sin(ang2[2]);
	T2[2][1] = -cos(ang2[2]);
	T2[3][3] = 1.0;

	T3[0][0] = cos(ang3[2]);
	T3[0][1] = -sin(ang3[2]);
	T3[1][2] = -1.0;
	T3[2][0] = sin(ang3[2]);
	T3[2][1] = cos(ang3[2]);
	T3[3][3] = 1.0;

	T4[0][0] = cos(ang4[2]);
	T4[0][1] = -sin(ang4[2]);
	T4[1][2] = 1.0;
	T4[2][0] = -sin(ang4[2]);
	T4[2][1] = -cos(ang4[2]);
	T4[3][3] = 1.0;


	matrix_multiply(T1,T2, T02);
	matrix_multiply(T02,T3, T03);
	matrix_multiply(T03,T4, T04);

	inv_matrix(T04, INV_T04);
	matrix_multiply(INV_T04, TransMatrix, T57_2_2);

	ang6[0] = acos(-T57_2_1[1][2]);
	ang6[1] = -acos(-T57_2_1[1][2]);
	ang6[2] = acos(-T57_2_2[1][2]);
	ang6[3] = -acos(-T57_2_2[1][2]);

	for (i = 0; i<2; i++)
	{
		if(fabs(ang6[i]) < 1e-6)
		{
			ang5[i] = Angle_now[4];
			ang7[i] = atan2(T57_2_1[2][0], T57_2_1[0][0]) - ang5[i];
		}
		else
		{
			ang5[i] = atan2(T57_2_1[2][2]/sin(ang6[i]), T57_2_1[0][2]/sin(ang6[i]));
			ang7[i] = atan2(-T57_2_1[1][1]/sin(ang6[i]), T57_2_1[1][0]/sin(ang6[i]));
		}
	}

	for(i=2; i<4; i++)
	{
		if(fabs(ang6[i]) < 1e-6)
		{
			ang5[i] = Angle_now[4];
			ang7[i] = atan2(T57_2_2[2][0], T57_2_2[0][0]) - ang5[i];
		}
		else
		{
			ang5[i] = atan2(T57_2_2[2][2]/sin(ang6[i]), T57_2_2[0][2]/sin(ang6[i]));
			ang7[i] = atan2(-T57_2_2[1][1]/sin(ang6[i]), T57_2_2[1][0]/sin(ang6[i]));
		}
	}
	double atest[7];
	double Tt[4][4];
	for(i=0; i<4; i++)
	{
		atest[0]=ang1[i];atest[1]=ang2[i];atest[2]=ang3[i];atest[3]=ang4[i];atest[4]=ang5[i];atest[5]=ang6[i];atest[6]=ang7[i];
		KinR(atest,Tt);
	}

	double Solve[4][7];
	for(i=0; i<4; i++)
	{
		Solve[i][0] = ang1[i];
		Solve[i][1] = ang2[i];
		Solve[i][2] = ang3[i];
		Solve[i][3] = ang4[i];
		Solve[i][4] = ang5[i];
		Solve[i][5] = ang6[i];
		Solve[i][6] = ang7[i];
	}

	int index = ChooseSolve(Angle_now,Solve);
	for (i=0;i<7;i++)
	{
		Angle_cal[i] = Solve[index][i]*180/3.1415926;
	}
	return 0;
}

// 冗余自由度规划计算
double Beta_CalL(double angle7[7])
{
	int i;
	double T1[4][4] = {{cos(angle7[0]), -sin(angle7[0]), 0, 0}, {sin(angle7[0]), cos(angle7[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2[4][4] = {{cos(angle7[1]), -sin(angle7[1]), 0, 0}, {0, 0, -1.0, 0}, {sin(angle7[1]), cos(angle7[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3[4][4] = {{cos(angle7[2]), -sin(angle7[2]), 0, 0}, {0, 0, 1.0, d3}, {-sin(angle7[2]), -cos(angle7[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4[4][4] = {{cos(angle7[3]), -sin(angle7[3]), 0, 0}, {0, 0, -1.0, 0}, {sin(angle7[3]), cos(angle7[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5[4][4] = {{cos(angle7[4]), -sin(angle7[4]), 0, 0}, {0, 0, 1.0, d4}, {-sin(angle7[4]), -cos(angle7[4]), 0, 0}, {0, 0, 0, 1.0}};
	double px, py, pz, p_unit[3], p3[3];
	double x0[3] = {1.0, 0.0, 0.0};
	double a[3], a_unit[3], a_unit_vertical[3], beta_x, beta_y, b_origin[3], dot_punit_p3;
	double T02[4][4], T03[4][4], T04[4][4], T05[4][4];
	double beta_origin;

	matrix_multiply(T1, T2, T02);
	matrix_multiply(T02, T3, T03);
	matrix_multiply(T03, T4, T04);
	matrix_multiply(T04, T5, T05);

	px = T05[0][3];
	py = T05[1][3];
	pz = T05[2][3];


	p_unit[0] = px/sqrt(px*px + py*py + pz*pz);
	p_unit[1] = py/sqrt(px*px + py*py + pz*pz);
	p_unit[2] = pz/sqrt(px*px + py*py + pz*pz);


	cross(x0, p_unit, a);
	for(i=0; i<3; i++)
	{
		a_unit[i] = a[i]/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}
	cross(p_unit, a_unit, a_unit_vertical);

	for(i=0; i<3; i++)
	{
		p3[i] = T03[i][3];
	}


// 	dot_punit_p3 = p_unit[0]*p3[0] + p_unit[1]*p3[1] + p_unit[2]*p3[2];

	for(i=0; i<3; i++)
	{
		b_origin[i] = p3[i];
	}

	beta_x = b_origin[0]*a_unit[0] + b_origin[1]*a_unit[1] + b_origin[2]*a_unit[2];
	beta_y = b_origin[0]*a_unit_vertical[0] + b_origin[1]*a_unit_vertical[1] + b_origin[2]*a_unit_vertical[2];

	beta_origin = atan2(beta_y, beta_x);
	return(beta_origin);
}
double Beta_CalR(double Angle[7])
{
	int i;
	double T1[4][4] = {{cos(Angle[0]), -sin(Angle[0]), 0, 0}, {sin(Angle[0]), cos(Angle[0]), 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0}};
	double T2[4][4] = {{cos(Angle[1]), -sin(Angle[1]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[1]), -cos(Angle[1]), 0, 0},{0, 0, 0, 1.0}};
	double T3[4][4] = {{cos(Angle[2]), -sin(Angle[2]), 0, 0}, {0, 0, -1.0, -d3}, {sin(Angle[2]), cos(Angle[2]), 0, 0},{0, 0, 0, 1.0}};
	double T4[4][4] = {{cos(Angle[3]), -sin(Angle[3]), 0, 0}, {0, 0, 1.0, 0}, {-sin(Angle[3]), -cos(Angle[3]), 0, 0},{0, 0, 0, 1.0}};
	double T5[4][4] = {{cos(Angle[4]), -sin(Angle[4]), 0, 0}, {0, 0, -1.0, -d4}, {sin(Angle[4]), cos(Angle[4]), 0, 0}, {0, 0, 0, 1.0}};
	double px, py, pz, p_unit[3], p3[3];
	double x0[3] = {1.0, 0.0, 0.0};
	double a[3], a_unit[3], a_unit_vertical[3], beta_x, beta_y, b_origin[3], dot_punit_p3;
	double T02[4][4], T03[4][4], T04[4][4], T05[4][4];
	double beta_origin;

	matrix_multiply(T1, T2, T02);
	matrix_multiply(T02, T3, T03);
	matrix_multiply(T03, T4, T04);
	matrix_multiply(T04, T5, T05);

	px = T05[0][3];
	py = T05[1][3];
	pz = T05[2][3];


	p_unit[0] = px/sqrt(px*px + py*py + pz*pz);
	p_unit[1] = py/sqrt(px*px + py*py + pz*pz);
	p_unit[2] = pz/sqrt(px*px + py*py + pz*pz);


	cross(x0, p_unit, a);
	for(i=0; i<3; i++)
	{
		a_unit[i] = a[i]/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}
	//cross( a_unit,p_unit,a_unit_vertical);
	cross(p_unit, a_unit, a_unit_vertical);
	for(i=0; i<3; i++)
	{
		p3[i] = T03[i][3];
	}


// 	dot_punit_p3 = p_unit[0]*p3[0] + p_unit[1]*p3[1] + p_unit[2]*p3[2];
//
// 	for(i=0; i<3; i++)
// 	{
// 		b_origin[i] = p3[i] - dot_punit_p3*p_unit[i];
// 	}
	for(i=0; i<3; i++)
	{
		b_origin[i] = p3[i];
	}

	beta_x = b_origin[0]*a_unit[0] + b_origin[1]*a_unit[1] + b_origin[2]*a_unit[2];
	beta_y = b_origin[0]*a_unit_vertical[0] + b_origin[1]*a_unit_vertical[1] + b_origin[2]*a_unit_vertical[2];

	beta_origin = atan2(beta_y, beta_x);
	return(beta_origin);
}
// 逆运动学选解
int ChooseSolve(double Angle_now[7], double Solve[4][7])
{
	double AngleSum;
	double MinSum = 10000.0;
	int MinIndex = -1;
	int index,i;
	for (index = 0; index<4; index++)
	{
		AngleSum = 0.0;
		for (i = 0; i<7; i++)			// 计算当前解的关节运动总量
		{
			AngleSum+=fabs(Solve[index][i] - Angle_now[i]);
		}
		if (AngleSum<MinSum)
		{
			MinSum = AngleSum;
			MinIndex = index;
		}
	}
	return MinIndex;
}

void delta2tr(double delta[6], double tr[4][4])
{
	double Eye4[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	int i,j;
	double temp[4][4] = {{0, -delta[5], delta[4],delta[0]},
						 {delta[5],	0, -delta[3],delta[1]},
						 {-delta[4],delta[3],0, delta[2]},
						 {0, 0, 0, 0}};
	for (i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
		{
			tr[i][j] = Eye4[i][j] + temp[i][j];
		}
	}
}

void Matrix_Trans3(double MA[3][3], double MB[3][3])
{
	int i = 0;
	int j = 0;
	for(i=0; i<3; i++)
	{
		for(j=0; j<3;j++)
		{
			MB[i][j] = MA[j][i];
		}
	}
}


int Matrix_Reverse(int iNum, double* pSourceR, double* pDestR)
{
	int i, j, k, i0, j0;
	double dMax, dTem;

	int* iRow = (int*)malloc(sizeof(int) * iNum);
	int* iCol = (int*)malloc(sizeof(int) * iNum);
	double* dTem1 = (double*)malloc(sizeof(double) * iNum);
	double* dTem2 = (double*)malloc(sizeof(double) * iNum);

	// 复制数组, 在缓冲区之间拷贝字符
	memcpy(pDestR, pSourceR, (iNum * iNum) * sizeof(double));

	for (k = 0; k < iNum; k++)
	{
		dMax = 0.0;

		// 首先，全选主元；从第k行k列开始的右下角子阵中选取绝对值最大的元素
		for (i = k; i < iNum; i++)
		{
			for (j = k; j < iNum; j++)
			{
				if (fabs(pDestR[i*iNum+j]) > fabs(dMax))
				{
					dMax = pDestR[i*iNum+j];
					i0 = i;
					j0 = j;
				}
			}
		}

		// 若绝对值最大的元素为零，则为奇异矩阵，求逆失败
		if (fabs(dMax) < 0.000000000001)
		{
			free(iRow);  free(iCol);  free(dTem1);  free(dTem2);
			return 0;
		}

		// 将绝对值最大的元素所在行与k行互换
		if (i0 != k)
		{
			for (j = 0; j < iNum; j++)
			{
				dTem = pDestR[i0*iNum+j];
				pDestR[i0*iNum+j] = pDestR[k*iNum+j];
				pDestR[k*iNum+j] = dTem;
			}
		}

		// 将绝对值最大的元素所在列与k列互换，即可交换到主元素位置上
		if (j0 != k)
		{
			for (i = 0; i < iNum; i++)
			{
				dTem = pDestR[i*iNum+j0];
				pDestR[i*iNum+j0] = pDestR[i*iNum+k];
				pDestR[i*iNum+k] = dTem;
			}
		}

		// 记录该元素所在的行号和列号
		iRow[k] = i0;
		iCol[k] = j0;

		// 然后求解
		for (j = 0; j < iNum; j++)
		{
			if (j == k)
			{
				dTem1[j] = 1 / dMax;
				dTem2[j] = 1.0;
			}
			else
			{
				dTem1[j] =  - pDestR[k*iNum+j] / dMax;
				dTem2[j] = pDestR[j*iNum+k];
			}
			pDestR[k*iNum+j] = 0.0;
			pDestR[j*iNum+k] = 0.0;
		}
		for (i = 0; i < iNum; i++)
			for (j = 0; j < iNum; j++)
				pDestR[i*iNum+j] += dTem2[i] * dTem1[j];
	}

	// 最后，根据在全选主元过程中所记录的行、列交换的信息进行恢复
	for (k = iNum-1; k >= 0; k--)
	{
		i0 = iRow[k];
		j0 = iCol[k];
		if (i0 != k)
		{
			for (i = 0; i < iNum; i++)
			{
				dTem = pDestR[i*iNum+i0];
				pDestR[i*iNum+i0] = pDestR[i*iNum+k];
				pDestR[i*iNum+k] = dTem;
			}
		}
		if (j0 != k)
		{
			for (j = 0; j < iNum; j++)
			{
				dTem = pDestR[j0*iNum+j];
				pDestR[j0*iNum+j] = pDestR[k*iNum+j];
				pDestR[k*iNum+j] = dTem;
			}
		}
	}

	free(iRow);  free(iCol);  free(dTem1);  free(dTem2);
	return 1;
}

// 将旋转矩阵正交单位化
void Schmidt(double Raw[4][4], double Out[4][4])
{
	double U[3][3];
	double Uu[3];
	int i,j;
	for(i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
		{
			Out[i][j] = Raw[i][j];
		}
	}
	for(i=0;i<3;i++)
	{
		U[0][i] = Raw[0][i];
		U[1][i] = Raw[1][i];
		U[2][i] = Raw[2][i];

		if(i == 0)
		{
			Out[0][i] = U[0][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
			Out[1][i] = U[1][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
			Out[2][i] = U[2][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
		}
		else
		{
			Uu[0] = U[0][i];
			Uu[1] = U[1][i];
			Uu[2] = U[2][i];
			for (j=0;j<i;j++)
			{
				Uu[0] = Uu[0] - (Raw[0][i]*U[0][j]+Raw[1][i]*U[1][j]+Raw[2][i]*U[2][j])/(U[0][j]*U[0][j]+U[1][j]*U[1][j]+U[2][j]*U[2][j])*U[0][j];
				Uu[1] = Uu[1] - (Raw[0][i]*U[0][j]+Raw[1][i]*U[1][j]+Raw[2][i]*U[2][j])/(U[0][j]*U[0][j]+U[1][j]*U[1][j]+U[2][j]*U[2][j])*U[1][j];
				Uu[2] = Uu[2] - (Raw[0][i]*U[0][j]+Raw[1][i]*U[1][j]+Raw[2][i]*U[2][j])/(U[0][j]*U[0][j]+U[1][j]*U[1][j]+U[2][j]*U[2][j])*U[2][j];
			}
			U[0][i] = Uu[0];
			U[1][i] = Uu[1];
			U[2][i] = Uu[2];

			if((U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i])==0)
			{
				Out[0][i] = 0;
				Out[1][i] = 0;
				Out[2][i] = 0;
			}
			else
			{
				Out[0][i] = U[0][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
				Out[1][i] = U[1][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
				Out[2][i] = U[2][i]/sqrt(U[0][i]*U[0][i]+U[1][i]*U[1][i]+U[2][i]*U[2][i]);
			}

		}
	}
}
