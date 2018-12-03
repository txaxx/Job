#include <iostream>
#include<math.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
#include <cmath>
#include <random>

using namespace std;

#define N 6    //测试矩阵维数定义
#define RAD2ANG (3.1415926535898/180.0)
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
double t1, t2, t3, t4, t5, t6;
double J1, J2, J3, J4, J5, J6;

//按第一行展开计算|A|
double getA(double arcs[N][N], int n)
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	double ans = 0;
	double temp[N][N] = { 0.0 };
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		double t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
void  getAStart(double arcs[N][N], int n, double ans[N][N])
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	double temp[N][N];
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}


			ans[j][i] = getA(temp, n - 1);  //此处顺便进行了转置
			if ((i + j) % 2 == 1)
			{
				ans[j][i] = -ans[j][i];
			}
		}
	}
}

//得到给定矩阵src的逆矩阵保存到des中。
bool GetMatrixInverse(double src[N][N], int n, double des[N][N])
{
	double flag = getA(src, n);
	double t[N][N];
	if (0 == flag)
	{
		cout << "原矩阵行列式为0，无法求逆。请重新运行" << endl;
		return false;//如果算出矩阵的行列式为0，则不往下进行
	}
	else
	{
		getAStart(src, n, t);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
			{
				des[i][j] = t[i][j] / flag;
			}

		}
	}

	return true;
}

int main()
{
	bool flag;//标志位，如果行列式为0，则结束程序
	int row = N;
	int col = N;
	int k = 0;
	double e = 0;
	double matrix_before[N][N]{};//{1,2,3,4,5,6,7,8,9};
	double *D = new double[6];//迭代向量
	double x1, x2, x3, x4, x5, x6;
	double J_T[6][12] = {};
	double K[6][6] = {};
	double H[6][12] = {};
	double tx[6][1] = {};
	double zuobiao[4][4];

	default_random_engine e1,e2,e3,e4,e5,e6;
	uniform_real_distribution<double> u1(-180*RAD2ANG, 180* RAD2ANG), u2(-150* RAD2ANG,150 * RAD2ANG),u3(-40 * RAD2ANG, 220 * RAD2ANG),u4(-180 * RAD2ANG, 180 * RAD2ANG),u5(-40 * RAD2ANG, 220 * RAD2ANG),u6(-180 * RAD2ANG, 180 * RAD2ANG);

	x1 = u1(e1);
	x2 = u2(e3);
	x3 = u3(e3);
	x4 = u4(e4);
	x5 = u5(e5);
	x6 = u6(e6);
	printf("初始值为：\n");
	printf("T1 = %f\n", x1);
	printf("T2 = %f\n", x2);
	printf("T3 = %f\n", x3);
	printf("T4 = %f\n", x4);
	printf("T5 = %f\n", x5);
	printf("T6 = %f\n", x6);
	printf("\n");

	FILE * fp = NULL;
	fp = fopen("../nijie/image_16.txt", "r");//打开坐标文件
	if (fp == NULL) {
		perror("open xyzrpy file error\n");
		return 0;
	}
	for (int i = 0; i<4; i++) {
		fscanf_s(fp, "%lf%lf%lf%lf",
			&zuobiao[i][0],
			&zuobiao[i][1],
			&zuobiao[i][2],
			&zuobiao[i][3]);
	}
	fclose(fp);
	printf("坐标矩阵：\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n",
		zuobiao[0][0],
		zuobiao[0][1],
		zuobiao[0][2],
		zuobiao[0][3],
		zuobiao[1][0],
		zuobiao[1][1],
		zuobiao[1][2],
		zuobiao[1][3],
		zuobiao[2][0],
		zuobiao[2][1],
		zuobiao[2][2],
		zuobiao[2][3],
		zuobiao[3][0],
		zuobiao[3][1],
		zuobiao[3][2],
		zuobiao[3][3]
	);
	printf("\n");

		while (true) {
			if (k > 1000) {

				printf("达到迭代次数，当前值为：\n");
				printf("X1 = %f\n", D[0]);
				printf("X2 = %f\n", D[1]);
				printf("X3 = %f\n", D[2]);
				printf("X4 = %f\n", D[3]);
				printf("X5 = %f\n", D[4]);
				printf("X6 = %f\n", D[5]);
				printf("\n");
				
				x1 = u1(e1);
				x2 = u2(e3);
				x3 = u3(e3);
				x4 = u4(e4);
				x5 = u5(e5);
				x6 = u6(e6);
				printf("重新取值为：\n");
				printf("x1 = %f\n", x1);
				printf("x2 = %f\n", x2);
				printf("x3 = %f\n", x3);
				printf("x4 = %f\n", x4);
				printf("x5 = %f\n", x5);
				printf("x6 = %f\n", x6);
				printf("\n");
				k = 0;
			}

			double J[12][6] = { {-sin(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - cos(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))), sin(x4)*sin(x6)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) + cos(x4)*sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))),   sin(x4)*sin(x6)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - cos(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) + cos(x4)*sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))), sin(x6)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - cos(x6)*sin(x5)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))), -cos(x6)*(cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))),   sin(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) - cos(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)))},
								{-sin(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - cos(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))), -cos(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))) - sin(x4)*sin(x6)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), -cos(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))) - sin(x4)*sin(x6)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), cos(x6)*sin(x5)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x6)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))),  cos(x6)*(cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))),   cos(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
								{0, cos(x6)*(cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + cos(x4)*sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2))) - sin(x4)*sin(x6)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), cos(x6)*(cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + cos(x4)*sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2))) - sin(x4)*sin(x6)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), cos(x4)*sin(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + cos(x6)*sin(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)), -cos(x6)*(sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) + cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))), cos(x6)*sin(x4)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - sin(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)))},
								{sin(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))) - cos(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))), sin(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) + cos(x4)*sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) + cos(x6)*sin(x4)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)),   sin(x6)*(cos(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) + cos(x4)*sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) + cos(x6)*sin(x4)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)), cos(x6)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + sin(x5)*sin(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))),  sin(x6)*(cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))),   sin(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)))},
								{sin(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) - cos(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))), sin(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))) - cos(x6)*sin(x4)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)),   sin(x6)*(cos(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))) - cos(x6)*sin(x4)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), -cos(x6)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x5)*sin(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))), -sin(x6)*(cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3))), -sin(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - cos(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
								{0, -sin(x6)*(cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + cos(x4)*sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2))) - cos(x6)*sin(x4)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), -sin(x6)*(cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + cos(x4)*sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2))) - cos(x6)*sin(x4)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), cos(x4)*cos(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - sin(x4)*sin(x5)*sin(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)), sin(x6)*(sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) + cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))), -cos(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))) - sin(x4)*sin(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))},
								{cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)),  cos(x4)*cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - sin(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)), cos(x4)*cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - sin(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)), cos(x5)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))),          -sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)), 0},
								{cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)), -sin(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), -sin(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - cos(x4)*cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), -cos(x5)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))), sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), 0},
								{0, sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - cos(x4)*cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - cos(x4)*cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), -cos(x5)*sin(x4)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)), cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)), 0},
								{225 * sin(x1)*sin(x2) - 50 * cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + 50 * sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)) - (11443 * cos(x2)*cos(x3)*sin(x1)) / 50 + (11443 * sin(x1)*sin(x2)*sin(x3)) / 50, 50 * sin(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) - 225 * cos(x1)*cos(x2) - (11443 * cos(x1)*cos(x2)*sin(x3)) / 50 - (11443 * cos(x1)*cos(x3)*sin(x2)) / 50 - 50 * cos(x4)*cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)),   50 * sin(x5)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2)) - (11443 * cos(x1)*cos(x2)*sin(x3)) / 50 - (11443 * cos(x1)*cos(x3)*sin(x2)) / 50 - 50 * cos(x4)*cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)), -50 * cos(x5)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))), 50 * sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + 50 * cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)), 0},
								{50 * sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - 50 * cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - 225 * cos(x1)*sin(x2) - (11443 * cos(x1)*sin(x2)*sin(x3)) / 50 + (11443 * cos(x1)*cos(x2)*cos(x3)) / 50, 50 * sin(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - 225 * cos(x2)*sin(x1) - (11443 * cos(x2)*sin(x1)*sin(x3)) / 50 - (11443 * cos(x3)*sin(x1)*sin(x2)) / 50 + 50 * cos(x4)*cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)),   50 * sin(x5)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2)) - (11443 * cos(x2)*sin(x1)*sin(x3)) / 50 - (11443 * cos(x3)*sin(x1)*sin(x2)) / 50 + 50 * cos(x4)*cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), 50 * cos(x5)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))), -50 * sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - 50 * cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)), 0},
								{0, (11443 * cos(x2)*cos(x3)) / 50 - 225 * sin(x2) - (11443 * sin(x2)*sin(x3)) / 50 - 50 * sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + 50 * cos(x4)*cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), (11443 * cos(x2)*cos(x3)) / 50 - (11443 * sin(x2)*sin(x3)) / 50 - 50 * sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + 50 * cos(x4)*cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), 50 * cos(x5)*sin(x4)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)), 50 * cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - 50 * cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)), 0} };
								
								
			double F[12][1] = { {  -zuobiao[0][0] - sin(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - cos(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) },
								{ sin(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][0] + cos(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
								{ cos(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))) - zuobiao[2][0] + sin(x4)*sin(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) },
								{ sin(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) - cos(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][1]},
								{ cos(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][1] - sin(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
								{ cos(x6)*sin(x4)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - sin(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))) - zuobiao[2][1]},
								{ cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][2] - sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))},
								{ sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)) - cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][2]},
								{ sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - zuobiao[2][2] + cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))},
								{ 50 * sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - 225 * cos(x1)*sin(x2) - 50 * cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][3] - (11443 * cos(x1)*sin(x2)*sin(x3)) / 50 + (11443 * cos(x1)*cos(x2)*cos(x3)) / 50},
								{ 50 * cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - 225 * sin(x1)*sin(x2) - zuobiao[1][3] - 50 * sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)) + (11443 * cos(x2)*cos(x3)*sin(x1)) / 50 - (11443 * sin(x1)*sin(x2)*sin(x3)) / 50},
								{ 225 * cos(x2) - zuobiao[2][3] + (11443 * cos(x2)*sin(x3)) / 50 + (11443 * cos(x3)*sin(x2)) / 50 - 50 * sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - 50 * cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + 242} };

			/*printf("J:\n");
			for (int i = 0; i<12; i++) {
			for (int j = 0; j<6; j++) {
			printf(" %lf ", J[i][j]);
			}
			printf("\n");
			}
			printf("\n");*/

	for (int k = 0; k<12; k++)
		for (int g = 0; g < 6; g++)
			J_T[g][k] = J[k][g];

	/*printf("J_T:\n");
	for (int i = 0; i<6; i++) {
	for (int j = 0; j<12; j++) {
	printf(" %lf ", J_T[i][j]);
	}
	printf("\n");
	}
	printf("\n");*/

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			K[i][j] = J_T[i][0] * J[0][j] + J_T[i][1] * J[1][j] + J_T[i][2] * J[2][j] + J_T[i][3] * J[3][j] + J_T[i][4] * J[4][j] + J_T[i][5] * J[5][j] + J_T[i][6] * J[6][j] + J_T[i][7] * J[7][j] + J_T[i][8] * J[8][j] + J_T[i][9] * J[9][j] + J_T[i][10] * J[10][j] + J_T[i][11] * J[11][j];

	/*	printf("K:\n");

	for (int i = 0; i<6; i++) {
	for (int j = 0; j<6; j++) {
	printf(" %lf ", K[i][j]);
	}
	printf("\n");
	}
	printf("\n");
	*/

	for (int i = 0; i<N; i++)
	{
		for (int j = 0; j<N; j++)
		{
			matrix_before[i][j] = K[i][j];
		}
	}

	/*cout << "原矩阵：" << endl;

	for (int i = 0; i<N; i++)
	{
	for (int j = 0; j<N; j++)
	{
	//cout << matrix_before[i][j] <<" ";
	cout << *(*(matrix_before + i) + j) << " ";
	}
	cout << endl;
	}
	*/

	double matrix_after[N][N]{};
	flag = GetMatrixInverse(matrix_before, N, matrix_after);
	if (false == flag)
		return 0;

/*
	cout << "逆矩阵：" << endl;

	for (int i = 0; i<row; i++)
	{
		for (int j = 0; j<col; j++)
		{
			cout << matrix_after[i][j] << " ";
			//cout << *(*(matrix_after+i)+j)<<" ";
		}
		cout << endl;
	}
	cout << endl;*/
	/*
	GetMatrixInverse(matrix_after, N, matrix_before);

	cout << "反算的原矩阵：" << endl;//为了验证程序的精度

	for (int i = 0; i<N; i++)
	{
	for (int j = 0; j<N; j++)
	{
	//cout << matrix_before[i][j] <<" ";
	cout << *(*(matrix_before + i) + j) << " ";
	}
	cout << endl;
	}
	*/
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 12; j++)
			H[i][j] = matrix_after[i][0] * J_T[0][j] + matrix_after[i][1] * J_T[1][j] + matrix_after[i][2] * J_T[2][j] + matrix_after[i][3] * J_T[3][j] + matrix_after[i][4] * J_T[4][j] + matrix_after[i][5] * J_T[5][j];

	/*printf("H:\n");
	for (int i = 0; i<6; i++) {
	for (int j = 0; j<12; j++) {
	printf(" %lf ", H[i][j]);
	}
	printf("\n");
	}
	printf("\n");*/

	for (int i = 0; i < 6; i++)
		tx[i][0] = H[i][0] * F[0][0] + H[i][1] * F[1][0] + H[i][2] * F[2][0] + H[i][3] * F[3][0] + H[i][4] * F[4][0] + H[i][5] * F[5][0] + H[i][6] * F[6][0] + H[i][7] * F[7][0] + H[i][8] * F[8][0] + H[i][9] * F[9][0] + H[i][10] * F[10][0] + H[i][11] * F[11][0];

/*		printf("F:\n");

	for (int i = 0; i<12; i++) {
	printf(" %lf ", F[i][0]);
	}
	printf("\n");

printf("tx:\n");

	for (int i = 0; i<6; i++) {
		printf(" %lf ", tx[i][0]);
	}
	printf("\n");
*/


			D[0] = x1 - tx[0][0];
			D[1] = x2 - tx[1][0];
			D[2] = x3 - tx[2][0];
			D[3] = x4 - tx[3][0];
			D[4] = x5 - tx[4][0];
			D[5] = x6 - tx[5][0];


			x1 = fmod(D[0], 6.2831853072);
			x2 = fmod(D[1], 6.2831853072);
			x3 = fmod(D[2], 6.2831853072);
			x4 = fmod(D[3], 6.2831853072);
			x5 = fmod(D[4], 6.2831853072);
			x6 = fmod(D[5], 6.2831853072);

			double f[12][1] = { {  -zuobiao[0][0] - sin(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - cos(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) },
			{ sin(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][0] + cos(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
			{ cos(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))) - zuobiao[2][0] + sin(x4)*sin(x6)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) },
			{ sin(x6)*(sin(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) + cos(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))) - cos(x6)*(cos(x4)*sin(x1) + sin(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][1]},
			{ cos(x6)*(cos(x1)*cos(x4) - sin(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][1] - sin(x6)*(sin(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) + cos(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)))},
			{ cos(x6)*sin(x4)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) - sin(x6)*(cos(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - cos(x4)*sin(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))) - zuobiao[2][1]},
			{ cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][2] - sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3))},
			{ sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)) - cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - zuobiao[1][2]},
			{ sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - zuobiao[2][2] + cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3))},
			{ 50 * sin(x5)*(cos(x1)*sin(x2)*sin(x3) - cos(x1)*cos(x2)*cos(x3)) - 225 * cos(x1)*sin(x2) - 50 * cos(x5)*(sin(x1)*sin(x4) - cos(x4)*(cos(x1)*cos(x2)*sin(x3) + cos(x1)*cos(x3)*sin(x2))) - zuobiao[0][3] - (11443 * cos(x1)*sin(x2)*sin(x3)) / 50 + (11443 * cos(x1)*cos(x2)*cos(x3)) / 50},
			{ 50 * cos(x5)*(cos(x1)*sin(x4) + cos(x4)*(cos(x2)*sin(x1)*sin(x3) + cos(x3)*sin(x1)*sin(x2))) - 225 * sin(x1)*sin(x2) - zuobiao[1][3] - 50 * sin(x5)*(cos(x2)*cos(x3)*sin(x1) - sin(x1)*sin(x2)*sin(x3)) + (11443 * cos(x2)*cos(x3)*sin(x1)) / 50 - (11443 * sin(x1)*sin(x2)*sin(x3)) / 50},
			{ 225 * cos(x2) - zuobiao[2][3] + (11443 * cos(x2)*sin(x3)) / 50 + (11443 * cos(x3)*sin(x2)) / 50 - 50 * sin(x5)*(cos(x2)*sin(x3) + cos(x3)*sin(x2)) - 50 * cos(x4)*cos(x5)*(cos(x2)*cos(x3) - sin(x2)*sin(x3)) + 242} };


			e = f[0][0] * f[0][0]*10000 + f[1][0] * f[1][0] * 10000 + f[2][0] * f[2][0] * 10000 + f[3][0] * f[3][0]*10000 + f[4][0] * f[4][0] * 10000 + f[5][0] * f[5][0] * 10000 + f[6][0] * f[6][0] * 10000 + f[7][0] * f[7][0] * 10000 + f[8][0] * f[8][0] * 10000 + f[9][0] * f[9][0] + f[10][0] * f[10][0] + f[11][0] * f[11][0];
				
			if (e< 0.0001) {

				t1 = ANG2RAD(x1);
				t2 = ANG2RAD(x2);
				t3 = ANG2RAD(x3);
				t4 = ANG2RAD(x4);
				t5 = ANG2RAD(x5);
				t6 = ANG2RAD(x6);
				
				if(-180<=t1 && t1 <=180)
					if(-150<t2 && t2<150)
						if (-40<t3 && t3<220)
							if (-180<t4 && t4<180)
								if (-40<t5 && t5<220)
									if (-180<t6 && t6<180)
				{ 
				cout << "模型迭代法的结果为：" << endl;
				printf("X1 = %f\n", t1);
				printf("X2 = %f\n", t2);
				printf("X3 = %f\n", t3);
				printf("X4 = %f\n", t4);
				printf("X5 = %f\n", t5);
				printf("X6 = %f\n", t6);
				printf("\n");

				J1 = t1;
				J2 = t2;
				J3 = t3 - 90;
				J4 = t4;
				J5 = t5 - 90;
				J6 = t6;
				cout << "机械臂迭代法的结果为：" << endl;
				printf("J1 = %f\n", J1);
				printf("J2 = %f\n", J2);
				printf("J3 = %f\n", J3);
				printf("J4 = %f\n", J4);
				printf("J5 = %f\n", J5);
				printf("J6 = %f\n", J6);

			/*	printf("f:\n");
				for (int i = 0; i<12; i++) {
					printf(" %lf ", f[i][0]);
				}
				printf("\n");

				printf("e: %lf ", e);*/
				printf("\n");
				break;}
			}


			k++;
		}

		//printf("e: %lf ", e);
	system("Pause");
	return 0;

}

