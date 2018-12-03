/* 4阶矩阵计算机器人正解 
 * 关节角度在文件 J1_J6中
 * 机器人参数在文件 param_table中
 * 坐标结果在屏幕输出 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//#define XYZ_F_J "E:/Halcon/Images/12320/J1_J6_23.txt"
#define DESIGN_DT "param_table.txt"
#define XYZ_F_TOOL "./tool_xyz"
//#define XYZ_F_WORLD "E:/Halcon/Images/12320/image_23.txt"


#define RAD2ANG (3.1415926535898/180.0)
#define IS_ZERO(var) if(var < 0.0000000001 && var > -0.0000000001){var = 0;} 

#define MATRIX_1 1
#define MATRIX_N 4

#define DEF_TOOLXYZ 0  
    /* 0 没有工具坐标系 非零 有工具坐标系 */

/*角度偏移*/
#define ANGLE_OFFSET 90
//#define ANGLE_OFFSET_J4 -90

typedef struct {
    double joint_v;  //joint variable关节角度值
    double length;   //连杆长度a
    double d;        //关节距离d
    double angle;    //连杆扭角
}param_t;


    double matrix_A1[MATRIX_N][MATRIX_N];
    double matrix_A2[MATRIX_N][MATRIX_N];
    double matrix_A3[MATRIX_N][MATRIX_N];
    double matrix_A4[MATRIX_N][MATRIX_N];
    double matrix_A5[MATRIX_N][MATRIX_N];
    double matrix_A6[MATRIX_N][MATRIX_N];

double matrix_worldxyz[MATRIX_N][MATRIX_N];
double matrix_toolxyz[MATRIX_N][MATRIX_N];


void initmatrix_A(param_t *p_table);
void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], param_t *p_param);

int matrix_mul(double matrix_a[MATRIX_N][MATRIX_N],
            double matrix_b[MATRIX_N][MATRIX_N],
            double matrix_result[MATRIX_N][MATRIX_N]);

int matrix_add(double matrix_a[MATRIX_N][MATRIX_N], 
            double matrix_b[MATRIX_N][MATRIX_N], 
            double matrix_sum[MATRIX_N][MATRIX_N], int m, int n);

void matrix_copy(double matrix_a[MATRIX_N][MATRIX_N], 
            double matrix_b[MATRIX_N][MATRIX_N], int m, int n);

void initmatrix_tool(double toolx, double tooly, double toolz);

void printmatrix(double matrix[MATRIX_N][MATRIX_N], int m, int n)
{
    int i, j;

    for(i=0; i<m; i++){
        for(j=0; j<n; j++){
            printf(" %lf ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");

}

void printmatrix_1(double matrix[MATRIX_N][1], int m, int n)
{
    int i, j;

    for(i=0; i<m; i++){
        for(j=0; j<n; j++){
            printf(" %lf ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");

}


int main()
{

    double matrix_T1[MATRIX_N][MATRIX_N];
    double matrix_T2[MATRIX_N][MATRIX_N];
    double matrix_T3[MATRIX_N][MATRIX_N];
    double matrix_T4[MATRIX_N][MATRIX_N];
    double matrix_T5[MATRIX_N][MATRIX_N];
    double matrix_T6[MATRIX_N][MATRIX_N];

    //double j1=0, j2=0, j3=0, j4=0, j5=0, j6=0;
    //double l1=0, l2=0, l3=0, d4=0, z_offset=0;
    double toolx=0, tooly=0, toolz=0, toolrx=0, toolry=0, toolrz=0;
    double worldx=0, worldy=0, worldz=0, worldrx=0, worldry=0, worldrz=0;
    double z_offset=0;

    param_t param_table[6] ={0};
    memset(param_table, 0, sizeof(param_table) );

	int p;
	FILE * fp=NULL;
	char namein[100],namein1[100];
	for (p=1;p<22;p++){

	sprintf(namein,"E:/Halcon/Images/1130/robot/J1_J6_%02d.txt",p);
	sprintf(namein1,"E:/Halcon/Images/1130/robot/image_%02d.txt",p);
    fp=fopen(namein, "r");
    if(fp== NULL){
        perror("open J1_J6 file error\n");
        return 0;
    }
    fscanf(fp, "%lf%lf%lf%lf%lf%lf", 
                &param_table[0].joint_v, 
                &param_table[1].joint_v, 
                &param_table[2].joint_v, 
                &param_table[3].joint_v, 
                &param_table[4].joint_v, 
                &param_table[5].joint_v 
                );
	//机械臂的坐标变成模拟的坐标
	param_table[2].joint_v += ANGLE_OFFSET;
	param_table[4].joint_v = -param_table[4].joint_v - ANGLE_OFFSET;
	param_table[5].joint_v = 180-param_table[5].joint_v;

    printf("j1...j6\n%lf %lf %lf %lf %lf %lf\n",
                param_table[0].joint_v, 
                param_table[1].joint_v, 
                param_table[2].joint_v, 
                param_table[3].joint_v, 
                param_table[4].joint_v, 
                param_table[5].joint_v 
                );
    //加初始角度偏移 j2 j5 
	//param_table[0].joint_v = - param_table[0].joint_v;
	//param_table[3].joint_v = - param_table[3].joint_v;
	//param_table[5].joint_v = param_table[5].joint_v - 180;
    param_table[1].joint_v += ANGLE_OFFSET;
    param_table[4].joint_v += ANGLE_OFFSET;

    //将机器人关节角度转换成弧度
    param_table[0].joint_v *= RAD2ANG;
    param_table[1].joint_v *= RAD2ANG;
    param_table[2].joint_v *= RAD2ANG;
    param_table[3].joint_v *= RAD2ANG;
    param_table[4].joint_v *= RAD2ANG;
    param_table[5].joint_v *= RAD2ANG;

    printf("\nj1...j6 RAD2ANG\n%lf %lf %lf %lf %lf %lf\n", 
                param_table[0].joint_v, 
                param_table[1].joint_v, 
                param_table[2].joint_v, 
                param_table[3].joint_v, 
                param_table[4].joint_v, 
                param_table[5].joint_v 
          );

    fclose(fp);

    fp=fopen(DESIGN_DT, "r");
    if( fp== NULL){
        perror("open param_table file error\n");
        return 0;
    }

//读入关节参数
    int i;
    for(i=0; i<6; i++){
        fscanf(fp, "%lf%lf%lf", 
                    &param_table[i].length, 
                    &param_table[i].d, 
                    &param_table[i].angle );
    }
    fscanf(fp, "%lf", &z_offset );
	printf("\nDH参数表\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n", 
                param_table[0].joint_v, 
				param_table[0].length,
                param_table[0].d, 
                param_table[0].angle,
				param_table[1].joint_v, 
                param_table[1].length, 
                param_table[1].d, 
                param_table[1].angle,
				param_table[2].joint_v, 
				param_table[2].length, 
                param_table[2].d, 
                param_table[2].angle,
				param_table[3].joint_v, 
				param_table[3].length, 
                param_table[3].d, 
                param_table[3].angle,
				param_table[4].joint_v, 
				param_table[4].length, 
                param_table[4].d, 
                param_table[4].angle,
				param_table[5].joint_v, 
				param_table[5].length, 
                param_table[5].d, 
                param_table[5].angle
          );
    fclose(fp);

    param_table[0].angle *= RAD2ANG;
    param_table[1].angle *= RAD2ANG;
    param_table[2].angle *= RAD2ANG;
    param_table[3].angle *= RAD2ANG;
    param_table[4].angle *= RAD2ANG;
    param_table[5].angle *= RAD2ANG;
	
    initmatrix_A(param_table);
/*
    //fscanf(fp, "%lf %lf %lf", &toolx, &tooly, &toolz);
    //printf("tool x y z\n%lf %lf %lf\n", toolx, tooly, toolz);

    fp=fopen(XYZ_F_TOOL, "r");
    if(fp== NULL || DEF_TOOLXYZ == 0){
        printf("no toolxyz \n");    
    }else{
        fscanf(fp, "%lf %lf %lf %lf %lf %lf", 
                    &toolx, &tooly, &toolz, &toolrx, &toolry, &toolrz);
        printf("\ntoolxyz\n%lf %lf %lf %lf %lf %lf\n", 
                    toolx, tooly, toolz, toolrx, toolry, toolrz);
        fclose(fp);
    }
    initmatrix_tool(toolx, tooly, toolz);
	*/
	
 //计算变换矩阵 matrix T1---T6
    matrix_copy(matrix_A1, matrix_T1, MATRIX_N, MATRIX_N);
    printf("matrix_T1 =  \n");
    printmatrix(matrix_T1, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T1, matrix_A2, matrix_T2);
    printf("matrix_T2 =  \n");
    printmatrix(matrix_T2, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T2, matrix_A3, matrix_T3);
    printf("matrix_T3 =  \n");
    printmatrix(matrix_T3, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T3, matrix_A4, matrix_T4);
    printf("matrix_T4 =  \n");
    printmatrix(matrix_T4, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T4, matrix_A5, matrix_T5);
    printf("matrix_T5 =  \n");
    printmatrix(matrix_T5, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T5, matrix_A6, matrix_T6);
    printf("matrix_T6 =  \n");
    printmatrix(matrix_T6, MATRIX_N, MATRIX_N);

    //add();
    //matrix_worldxyz[2][0] +=z_offset;

    printf("\n----curent x, y, z-----\n%lf \n %lf\n %lf\n ", 
                matrix_T6[0][3], matrix_T6[1][3], 
                matrix_T6[2][3]+z_offset);


fp=fopen(namein1,"w");
fprintf(fp,"%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf",
	matrix_T6[0][0],matrix_T6[0][1],matrix_T6[0][2],matrix_T6[0][3],
	matrix_T6[1][0],matrix_T6[1][1],matrix_T6[1][2],matrix_T6[1][3],
	matrix_T6[2][0],matrix_T6[2][1],matrix_T6[2][2],matrix_T6[2][3],
	matrix_T6[3][0],matrix_T6[3][1],matrix_T6[3][2],matrix_T6[3][3]);
fclose(fp);
}
	system("Pause");
}

void initmatrix_A(param_t *p_table)
{//计算并输出各个关节坐标变换矩阵 matrix A1--A6
    calculate_matrix_A(matrix_A1, p_table+0);
    printf("matrix_A1 =  \n");
    printmatrix(matrix_A1, MATRIX_N, MATRIX_N);

    calculate_matrix_A(matrix_A2, p_table+1);
    printf("matrix_A2 =  \n");
    printmatrix(matrix_A2, MATRIX_N, MATRIX_N);

    calculate_matrix_A(matrix_A3, p_table+2);
    printf("matrix_A3 =  \n");
    printmatrix(matrix_A3, MATRIX_N, MATRIX_N);

    calculate_matrix_A(matrix_A4, p_table+3);
    printf("matrix_A4 =  \n");
    printmatrix(matrix_A4, MATRIX_N, MATRIX_N);

    calculate_matrix_A(matrix_A5, p_table+4);
    printf("matrix_A5 =  \n");
    printmatrix(matrix_A5, MATRIX_N, MATRIX_N);

    calculate_matrix_A(matrix_A6, p_table+5);
    printf("matrix_A6 =  \n");
    printmatrix(matrix_A6, MATRIX_N, MATRIX_N);
}

void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], param_t *p_param)
{//根据各个关节参数计算矩阵 
    double *pmatrix=(double *)matrix;
    double value, var_c, var_s, angle_c, angle_s;

    var_c = cos(p_param->joint_v);
    IS_ZERO(var_c);
    var_s = sin(p_param->joint_v);
    IS_ZERO(var_s);
    angle_c = cos(p_param->angle);
    IS_ZERO(angle_c);
    angle_s = sin(p_param->angle);
    IS_ZERO(angle_s);

    *pmatrix++ = var_c;
    //value = -var_s * angle_c;
    //IS_ZERO(value);
    *pmatrix++ = -var_s * angle_c;
    //value = var_s * angle_s;
    //IS_ZERO(value);
    *pmatrix++ = var_s * angle_s;
    //value = p_param->length * var_c;
    //IS_ZERO(value);
    *pmatrix++ = p_param->length * var_c;

    *pmatrix++ = var_s;
    //value = var_c * angle_c;
    //IS_ZERO(value);
    *pmatrix++ = var_c * angle_c;
    //value = -var_c *angle_s;
    //IS_ZERO(value);
    *pmatrix++ = -var_c *angle_s;
    //value = p_param->length * var_s;
    //IS_ZERO(value);
    *pmatrix++ = p_param->length * var_s;

    *pmatrix++ =0;
    *pmatrix++ = angle_s;
    *pmatrix++ = angle_c;
    *pmatrix++ = p_param->d;

    *pmatrix++ =0;
    *pmatrix++ =0;
    *pmatrix++ =0;
    *pmatrix =1;

}

void initmatrix_tool(double toolx, double tooly, double toolz)
{
    if(DEF_TOOLXYZ == 0){
        matrix_toolxyz[2][0] =1;
    }else{
        matrix_toolxyz[0][0] =toolx;
        matrix_toolxyz[1][0] =tooly;
        matrix_toolxyz[2][0] =toolz;

        {/* 初始化 toolx, tooly, toolz */}
    }

}

int matrix_mul(double matrix_a[MATRIX_N][MATRIX_N], 
            double matrix_b[MATRIX_N][MATRIX_N], 
            double matrix_result[MATRIX_N][MATRIX_N])
{//根据各个关节参数计算矩阵，求解关节间的变换关系
    int i,j,k;
    double sum;
    double matrix_tmp[MATRIX_N][MATRIX_N]={0};

    /*嵌套循环计算结果矩阵（m*p）的每个元素*/
    for(i=0;i<MATRIX_N;i++)
      for(j=0;j<MATRIX_N;j++)
      {   
          /*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
          sum=0;
          for(k=0;k<MATRIX_N;k++)
          sum += matrix_a[i][k] * matrix_b[k][j];
          matrix_tmp[i][j] = sum;
      }   
    matrix_copy(matrix_tmp, matrix_result, MATRIX_N, MATRIX_N);
    return 0;
}

int matrix_add(double matrix_a[MATRIX_N][MATRIX_N], 
            double matrix_b[MATRIX_N][MATRIX_N], 
            double matrix_sum[MATRIX_N][MATRIX_N], int m, int n)
{
    int i,j;
    double matrix_tmp[MATRIX_N][MATRIX_N]={0};

    for(i=0; i<m; i++){
        for(j=0; j<n; j++){
            matrix_tmp[i][j] = matrix_a[i][j] + matrix_b[i][j];
        }
    }
    matrix_copy(matrix_tmp, matrix_sum, MATRIX_N, MATRIX_N);

    return 0;
}


void matrix_copy(double matrix_src[MATRIX_N][MATRIX_N], 
            double matrix_des[MATRIX_N][MATRIX_N], int m, int n)
{
    int i,j;
    for(i=0; i<m; i++){
        for(j=0; j<n; j++){
            matrix_des[i][j] = matrix_src[i][j];
        }
    }
}

