/*计算逆解 根据机器人坐标计算机器人关节角度
 *关节参数在文件 param_table中
 *机器人坐标在文件 xyzrpy中
 *计算结果在屏幕输出 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#define PI (3.1415926535898)
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
#define RAD2ANG (3.1415926535898/180.0)
#define IS_ZERO(var) if(var < 0.0000000001 && var > -0.0000000001){var = 0;} 
 // #define IS_ZERO(var) ( (var) < 0.0000000001 && (var) > -0.0000000001 )?0 :1 
#define JUDGE_ZERO(var) ( (var) < 0.0000000001 && (var) > -0.0000000001 )

#define MATRIX_N 4

using namespace std;

typedef struct {
	double joint_v;  //joint variable
	double length;
	double d;
	double angle;
}param_t;

param_t param_table[6] = { 0 };
double worldx = 0, worldy = 0, worldz = 0,
worldrr = 0, worldrp = 0, worldry = 0;
double z_offset = 0;

void printMatrix(double matrix[MATRIX_N][MATRIX_N], int fRow, int fCloum);

int fun_j2(double J1, double j3, double *p_j2,double a2, 
	double d1, double d4, double d6,
	double ax, double ay, double az,
	double px, double py, double pz)
{//计算关节2的角度 
	double A, B, var_M, var_K;
	double v1_c, v1_s, v3_c, v3_s;

	v1_c = cos(J1);
	IS_ZERO(v1_c);
	v1_s = sin(J1);
	IS_ZERO(v1_s);
	v3_c = cos(j3);
	IS_ZERO(v3_c);
	v3_s = sin(j3);
	IS_ZERO(v3_s);

	A = -d6 * ax*v1_c + v1_c * px - d6 * ay*v1_s + py * v1_s;
	B = -d6 * az + pz - d1;

	var_M = B * d4 * v3_c - A * (d4 * v3_s + a2);
	var_K = A * d4 * v3_c + B * (d4 * v3_s + a2);

	p_j2[0] = atan2(var_M, var_K);
	
	return 1;
}

int fun_j3(double j1, double *p_j3,double a2, 
	double d1, double d4, double d6,
	double ax, double ay, double az,
	double px, double py, double pz)
{//计算关节3的角度 
	double v1_c, v1_s;
	double var_M, var_K, tmp;
	double var_sqrt[2] = { 0 };

	v1_c = cos(j1);
	IS_ZERO(v1_c);
	v1_s = sin(j1);
	IS_ZERO(v1_s);

	var_M = -d6 * az + pz - d1;
	var_K = -d6 * ax*v1_c + v1_c * px - d6 * ay*v1_s + py * v1_s;
	tmp = (var_M * var_M + var_K * var_K - d4 * d4-a2 * a2)/(2 * d4 * a2);

	p_j3[0] = asin(tmp);
	if (p_j3[0] == 0)
		p_j3[1] = PI;
	else
		p_j3[1] = (PI - fabs(p_j3[0]))*(p_j3[0]/ fabs(p_j3[0]));

	return 1;
}

int fun_j456(double j1, double j2, double j3,
	double *p_j4, double *p_j5, double *p_j6,
	double nx, double ny, double nz,
	double ox, double oy, double oz, 
	double ax, double ay, double az)
{//计算关节456的角度 
	double c1, c2, c3, s1, s2, s3, k, l;
	double A, B, C, D, E, F, G;

	c1 = cos(j1);
	IS_ZERO(c1);
	s1 = sin(j1);
	IS_ZERO(s1);
	c2 = cos(j2);
	IS_ZERO(c2);
	s2 = sin(j2);
	IS_ZERO(s2); 
	c3 = cos(j3);
	IS_ZERO(c3);
	s3 = sin(j3);
	IS_ZERO(s3);

	A = nz * (c2*s3 + c3 * s2) + ny * s1*(c2*c3 - s2 * s3) + c1 * nx*(c2*c3 - s2 * s3);
	B = oz * (c2*s3 + c3 * s2) + oy * s1*(c2*c3 - s2 * s3) + c1 * ox*(c2*c3 - s2 * s3);
	C = az * (c2*s3 + c3 * s2) + ax * c1*(c2*c3 - s2 * s3) + ay * s1*(c2*c3 - s2 * s3);
	D = ax*s1 - ay*c1;
	E = az * (c2*c3 - s2 * s3) - ax * c1*(c2*s3 + c3 * s2) - ay * s1*(c2*s3 + c3 * s2);
	F = nz * (c2*c3 - s2 * s3) - ny * s1*(c2*s3 + c3 * s2) - c1 * nx*(c2*s3 + c3 * s2);
	G = nx * s1 - c1 * ny;

	p_j5[0] = atan2(C, sqrt(A * A + B * B));
	p_j5[1] = atan2(C, -sqrt(A * A + B * B));

	l = cos(p_j5[0]);
	IS_ZERO(l);
	if (l == 0)
	{
		k = atan2(-G, -F);
		p_j4[0] = 0;
		p_j6[0] = k - p_j4[0];
		
		p_j4[1] = p_j6[0];
		p_j6[1] = p_j4[0];
	}else{
		p_j4[0] = atan2(D, E);
		p_j4[1] = atan2(-D, -E);
		p_j6[0] = atan2(-B, A);
		p_j6[1] = atan2(B, -A);
	}
	
	return 1;
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
/* 计算过程 根据运动方程 计算矩阵 列出等式 计算 j1 j2 j3
 * 计算旋转矩阵 根据 j1 j2 j3 计算T3 并转置 与旋转矩阵相乘 3*3
 * 计算zyz 就是 j4 j5 j6 */
int main()
{
	double matrix_R[MATRIX_N][MATRIX_N];

	double j1[2] = { 0 };  //元素值 >=360 度或 < -360 度 表示角度无效
	double j2[4] = { 0 };
	double j3[4] = { 0 };
	double j4[8] = { 0 };
	double j5[8] = { 0 };
	double j6[8] = { 0 };

	int i;

	FILE * fp = NULL;
	fp = fopen("../nijie/image_16.txt", "r");
	if (fp == NULL) {
		perror("open pose file error\n");
		return 0;
	}
	for (int i = 0; i < 4; i++) {
		fscanf(fp, "%lf%lf%lf%lf",
			&matrix_R[i][0],
			&matrix_R[i][1],
			&matrix_R[i][2],
			&matrix_R[i][3]);
	}

	printMatrix(matrix_R, MATRIX_N, MATRIX_N);

	fp = fopen("../nijie/param_table.txt", "r");
	if (fp == NULL) {
		perror("open param_table file error\n");
		return 0;
	}

	for (i = 0; i < 6; i++) {
		fscanf(fp, "%lf%lf%lf",
			&param_table[i].length,
			&param_table[i].d,
			&param_table[i].angle);
	}
	fscanf(fp, "%lf", &z_offset);
	fclose(fp);

	param_table[0].angle *= RAD2ANG;
	param_table[1].angle *= RAD2ANG;
	param_table[2].angle *= RAD2ANG;
	param_table[3].angle *= RAD2ANG;
	param_table[4].angle *= RAD2ANG;
	param_table[5].angle *= RAD2ANG;

	double a1 = param_table[0].length;
	double a2 = param_table[1].length;
	double a3 = param_table[2].length;
	double d1 = param_table[0].d;
	double d4 = param_table[3].d;
	double d6 = param_table[5].d;
	double nx = matrix_R[0][0];
	double ny = matrix_R[1][0];
	double nz = matrix_R[2][0];
	double ox = matrix_R[0][1];
	double oy = matrix_R[1][1];
	double oz = matrix_R[2][1];
    double ax = matrix_R[0][2];
	double ay = matrix_R[1][2];
	double az = matrix_R[2][2];
	double px = matrix_R[0][3];
	double py = matrix_R[1][3];
	double pz = matrix_R[2][3];
	
	//计算 j1
	j1[0] = atan2(matrix_R[1][3] - param_table[5].d * matrix_R[1][2],  matrix_R[0][3] - param_table[5].d * matrix_R[0][2]);
	IS_ZERO(j1[0]);
	j1[1] = j1[0] + PI;
	JUDGE_ZERO(j1[1] - 2 * PI) ? (j1[1] = 0) : 1;

	//计算 j3
	int v_bool;
	v_bool = fun_j3(j1[0], j3, a2, d1, d4, d6, ax, ay, az, px, py, pz);
	if (!v_bool)
	{
		printf("this j3 invalid\n");
		j3[0] = 2 * PI; j3[1] = 2 * PI;
		//      j2[0]>0 ? (j2[0] += 2*PI): (j2[0] -= 2*PI) ;    
		//      j2[1]>0 ? (j2[1] += 2*PI): (j2[1] -= 2*PI) ;    
	}
	v_bool = fun_j3(j1[1], j3 + 2, a2, d1, d4, d6, ax, ay, az, px, py, pz);
	if (!v_bool)
		{
		printf("this j3 invalid\n");
		j3[2] = 2 * PI; j3[3] = 2 * PI;
	}
	
	//计算 j2
	for (i = 0; i < 4; i++) {
		v_bool = fun_j2(j1[i / 2], j3[i], j2 + i, a2, d1, d4, d6, ax, ay, az, px, py, pz);
	}

	//计算j4 j5 j6
	for (i = 0; i < 4; i++) 
	{
		v_bool = fun_j456(j1[i / 2], j2[i], j3[i], j4 + 2*i, j5 + 2 * i, j6 + 2 * i, nx, ny, nz, ox, oy, oz, ax, ay, az);
	};
	printf("\nj1 j2 j3 j4 j5 j6：\n ");
	for (i = 0; i < 8; i++) {
		
		if(-180*RAD2ANG<=j1[i / 4] && j1[i / 4 ]<=180 * RAD2ANG)
			if (-115 * RAD2ANG<= j2[i / 2] && j2[i / 2]<= 115 * RAD2ANG)
				if (-140 * RAD2ANG<= j3[i / 4] && j3[i / 4]<= 220 * RAD2ANG)
					if (-180 * RAD2ANG<= j4[i] && j4[i]<= 180 * RAD2ANG)
						if (-2250 * RAD2ANG<= j5[i] && j5[i]<= 75 * RAD2ANG)
							if (-180 * RAD2ANG<= j6[i] && j6[i]<= 180 * RAD2ANG)
							{
								printf("\n %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n",
									ANG2RAD(j1[i / 4]), ANG2RAD(j2[i / 2]), ANG2RAD(j3[i / 2])-90,
									ANG2RAD(j4[i]), -ANG2RAD(j5[i])-90, 180-ANG2RAD(j6[i]));
							}
	};
}

void printMatrix(double matrix[MATRIX_N][MATRIX_N], int fRow, int fCloum)
{	
	for (int i = 0; i < fRow; i++) {
		for (int j = 0; j < fCloum; j++) {
			printf(" %lf ", matrix[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}
