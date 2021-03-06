/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) jit_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[23] = {4, 4, 0, 4, 8, 12, 16, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

/* T_fk:(i0[3])->(o0[4x4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.;
  if (res[0]!=0) res[0][0]=a0;
  a1=0.;
  if (res[0]!=0) res[0][1]=a1;
  if (res[0]!=0) res[0][2]=a1;
  if (res[0]!=0) res[0][3]=a1;
  if (res[0]!=0) res[0][4]=a1;
  a2=arg[0]? arg[0][0] : 0;
  a3=cos(a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=cos(a4);
  a6=(a3*a5);
  a2=sin(a2);
  a4=sin(a4);
  a7=(a2*a4);
  a6=(a6-a7);
  a7=arg[0]? arg[0][2] : 0;
  a8=cos(a7);
  a9=(a6*a8);
  a10=(a3*a4);
  a11=(a2*a5);
  a10=(a10+a11);
  a7=sin(a7);
  a11=(a10*a7);
  a9=(a9-a11);
  if (res[0]!=0) res[0][5]=a9;
  a9=(a2*a5);
  a11=(a3*a4);
  a9=(a9+a11);
  a11=(a9*a8);
  a5=(a3*a5);
  a4=(a2*a4);
  a5=(a5-a4);
  a4=(a5*a7);
  a11=(a11+a4);
  if (res[0]!=0) res[0][6]=a11;
  if (res[0]!=0) res[0][7]=a1;
  if (res[0]!=0) res[0][8]=a1;
  a6=(a6*a7);
  a11=(a10*a8);
  a6=(a6+a11);
  a11=(-a6);
  if (res[0]!=0) res[0][9]=a11;
  a8=(a5*a8);
  a9=(a9*a7);
  a8=(a8-a9);
  if (res[0]!=0) res[0][10]=a8;
  if (res[0]!=0) res[0][11]=a1;
  if (res[0]!=0) res[0][12]=a1;
  a1=8.0000000000000002e-02;
  a6=(a1*a6);
  a9=1.2000000000000000e-01;
  a10=(a9*a10);
  a7=1.7999999999999999e-01;
  a2=(a7*a2);
  a10=(a10+a2);
  a6=(a6+a10);
  a6=(-a6);
  if (res[0]!=0) res[0][13]=a6;
  a1=(a1*a8);
  a9=(a9*a5);
  a7=(a7*a3);
  a3=7.4999999999999997e-02;
  a7=(a7+a3);
  a9=(a9+a7);
  a1=(a1+a9);
  if (res[0]!=0) res[0][14]=a1;
  if (res[0]!=0) res[0][15]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int T_fk(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int T_fk_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int T_fk_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void T_fk_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int T_fk_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void T_fk_release(int mem) {
}

CASADI_SYMBOL_EXPORT void T_fk_incref(void) {
}

CASADI_SYMBOL_EXPORT void T_fk_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int T_fk_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int T_fk_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real T_fk_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* T_fk_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* T_fk_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* T_fk_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* T_fk_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int T_fk_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
