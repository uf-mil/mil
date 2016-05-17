/* Produced by CVXGEN, 2016-05-02 06:54:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double Q_final[6];
  double A_0[36];
  double x_0[6];
  double B_0[18];
  double A_1[36];
  double B_1[18];
  double A_2[36];
  double B_2[18];
  double A_3[36];
  double B_3[18];
  double A_4[36];
  double B_4[18];
  double A_5[36];
  double B_5[18];
  double A_6[36];
  double B_6[18];
  double A_7[36];
  double B_7[18];
  double A_8[36];
  double B_8[18];
  double A_9[36];
  double B_9[18];
  double A_10[36];
  double B_10[18];
  double A_11[36];
  double B_11[18];
  double A_12[36];
  double B_12[18];
  double A_13[36];
  double B_13[18];
  double A_14[36];
  double B_14[18];
  double A_15[36];
  double B_15[18];
  double A_16[36];
  double B_16[18];
  double A_17[36];
  double B_17[18];
  double xdot_max[1];
  double u_max[1];
  double *A[18];
  double *x[1];
  double *B[18];
} Params;
typedef struct Vars_t {
  double *x_18; /* 6 rows. */
  double *t_01; /* 1 rows. */
  double *x_1; /* 6 rows. */
  double *t_02; /* 1 rows. */
  double *x_2; /* 6 rows. */
  double *t_03; /* 1 rows. */
  double *x_3; /* 6 rows. */
  double *t_04; /* 1 rows. */
  double *x_4; /* 6 rows. */
  double *t_05; /* 1 rows. */
  double *x_5; /* 6 rows. */
  double *t_06; /* 1 rows. */
  double *x_6; /* 6 rows. */
  double *t_07; /* 1 rows. */
  double *x_7; /* 6 rows. */
  double *t_08; /* 1 rows. */
  double *x_8; /* 6 rows. */
  double *t_09; /* 1 rows. */
  double *x_9; /* 6 rows. */
  double *t_10; /* 1 rows. */
  double *x_10; /* 6 rows. */
  double *t_11; /* 1 rows. */
  double *x_11; /* 6 rows. */
  double *t_12; /* 1 rows. */
  double *x_12; /* 6 rows. */
  double *t_13; /* 1 rows. */
  double *x_13; /* 6 rows. */
  double *t_14; /* 1 rows. */
  double *x_14; /* 6 rows. */
  double *t_15; /* 1 rows. */
  double *x_15; /* 6 rows. */
  double *t_16; /* 1 rows. */
  double *x_16; /* 6 rows. */
  double *t_17; /* 1 rows. */
  double *x_17; /* 6 rows. */
  double *t_18; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *t_22; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *t_26; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *t_30; /* 1 rows. */
  double *t_31; /* 1 rows. */
  double *t_32; /* 1 rows. */
  double *t_33; /* 1 rows. */
  double *t_34; /* 1 rows. */
  double *t_35; /* 1 rows. */
  double *t_36; /* 1 rows. */
  double *t_37; /* 1 rows. */
  double *t_38; /* 1 rows. */
  double *t_39; /* 1 rows. */
  double *t_40; /* 1 rows. */
  double *t_41; /* 1 rows. */
  double *t_42; /* 1 rows. */
  double *t_43; /* 1 rows. */
  double *t_44; /* 1 rows. */
  double *t_45; /* 1 rows. */
  double *t_46; /* 1 rows. */
  double *t_47; /* 1 rows. */
  double *t_48; /* 1 rows. */
  double *t_49; /* 1 rows. */
  double *t_50; /* 1 rows. */
  double *t_51; /* 1 rows. */
  double *t_52; /* 1 rows. */
  double *t_53; /* 1 rows. */
  double *t_54; /* 1 rows. */
  double *t_55; /* 3 rows. */
  double *u_0; /* 3 rows. */
  double *t_56; /* 3 rows. */
  double *u_1; /* 3 rows. */
  double *t_57; /* 3 rows. */
  double *u_2; /* 3 rows. */
  double *t_58; /* 3 rows. */
  double *u_3; /* 3 rows. */
  double *t_59; /* 3 rows. */
  double *u_4; /* 3 rows. */
  double *t_60; /* 3 rows. */
  double *u_5; /* 3 rows. */
  double *t_61; /* 3 rows. */
  double *u_6; /* 3 rows. */
  double *t_62; /* 3 rows. */
  double *u_7; /* 3 rows. */
  double *t_63; /* 3 rows. */
  double *u_8; /* 3 rows. */
  double *t_64; /* 3 rows. */
  double *u_9; /* 3 rows. */
  double *t_65; /* 3 rows. */
  double *u_10; /* 3 rows. */
  double *t_66; /* 3 rows. */
  double *u_11; /* 3 rows. */
  double *t_67; /* 3 rows. */
  double *u_12; /* 3 rows. */
  double *t_68; /* 3 rows. */
  double *u_13; /* 3 rows. */
  double *t_69; /* 3 rows. */
  double *u_14; /* 3 rows. */
  double *t_70; /* 3 rows. */
  double *u_15; /* 3 rows. */
  double *t_71; /* 3 rows. */
  double *u_16; /* 3 rows. */
  double *t_72; /* 3 rows. */
  double *u_17; /* 3 rows. */
  double *x[19];
  double *u[18];
} Vars;
typedef struct Workspace_t {
  double h[324];
  double s_inv[324];
  double s_inv_z[324];
  double b[108];
  double q[270];
  double rhs[1026];
  double x[1026];
  double *s;
  double *z;
  double *y;
  double lhs_aff[1026];
  double lhs_cc[1026];
  double buffer[1026];
  double buffer2[1026];
  double KKT[2562];
  double L[2883];
  double d[1026];
  double v[1026];
  double d_inv[1026];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
// int main(int argc, char **argv);
// void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
