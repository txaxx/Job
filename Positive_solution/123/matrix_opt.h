/*matrix_opt.hÎÄ¼þ*/
#ifndef MATRIX_OPT
#define MATRIX_OPT

#define MATRIX_1 1
#define MATRIX_M 4
#define MATRIX_N 4
#define MATRIX_J 6
#define VECTOR_N 3

#define ANGLE_OFFSET_J2 90
#define ANGLE_OFFSET_J3 90

#define PI (3.1415926535898)
#define ANG2RAD_EQU(N) (N *= (180.0/3.1415926535898) )
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
#define RAD2ANG (3.1415926535898/180.0)
#define IS_ZERO(var) if(var < 0.0000000001 && var > -0.0000000001){var = 0;} 


#ifndef DH_PARAM
#define DH_PARAM
typedef struct {
    double joint_v;  //joint variable
    double length;
    double d;
    double alpha;
}param_t;
#endif

typedef union
{
    //struct{   
        double matrix_A[6][MATRIX_N][MATRIX_N];
    //};  
    struct{   
        double matrix_a1[MATRIX_N][MATRIX_N];
        double matrix_a2[MATRIX_N][MATRIX_N];
        double matrix_a3[MATRIX_N][MATRIX_N];
        double matrix_a4[MATRIX_N][MATRIX_N];
        double matrix_a5[MATRIX_N][MATRIX_N];
        double matrix_a6[MATRIX_N][MATRIX_N];
    };  
}matrix_union_t;

matrix_union_t union_A;

void printmatrix(double matrix[MATRIX_N][MATRIX_N], int m, int n);

void matrix_translate(double matrix[MATRIX_M][MATRIX_N], int m, int n);

int matrix_mul(double matrix_a[MATRIX_N][MATRIX_N],
            double matrix_b[MATRIX_N][MATRIX_N],
            double matrix_result[MATRIX_N][MATRIX_N], int m, int n);

void matrix_copy(double matrix_a[MATRIX_N][MATRIX_N],
            double matrix_b[MATRIX_N][MATRIX_N], int m, int n);

void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], 
            param_t *p_param);

void calculate_matrix_R(double angle_r, double angle_p, double angle_y,
            double (*matrix_R)[MATRIX_N]);

void matrix_copy2(double (*matrix_s)[], double (*matrix_d)[], int m, int n);

void print_mtx(double (*p_matrix)[], int m, int n);

void initmatrix_A(param_t *p_table);
#endif