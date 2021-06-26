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
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* M:(i0[3])->(o0[3x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=9.1000000000000004e-03;
  a1=3.5199999999999997e-03;
  a2=1.3970000000000000e-02;
  a3=-9.3000000000000013e-02;
  a4=1.2000000000000000e-01;
  a5=arg[0]? arg[0][2] : 0;
  a6=cos(a5);
  a7=(a4*a6);
  a8=(a3*a7);
  a8=(a2-a8);
  a9=9.4999999999999996e-01;
  a10=(a9*a7);
  a10=(a3-a10);
  a11=(a7*a10);
  a8=(a8-a11);
  a5=sin(a5);
  a4=(a4*a5);
  a11=(a9*a4);
  a12=(a4*a11);
  a8=(a8+a12);
  a1=(a1+a8);
  a8=-4.1999999999999996e-02;
  a12=(a3*a6);
  a13=(a9*a6);
  a14=(a7*a13);
  a12=(a12-a14);
  a14=(a9*a5);
  a15=(a4*a14);
  a12=(a12-a15);
  a12=(a8+a12);
  a15=1.7999999999999999e-01;
  a16=arg[0]? arg[0][1] : 0;
  a17=cos(a16);
  a17=(a15*a17);
  a12=(a12*a17);
  a12=(a1-a12);
  a18=(a3*a5);
  a19=(a9*a5);
  a20=(a7*a19);
  a18=(a18-a20);
  a9=(a9*a6);
  a4=(a4*a9);
  a18=(a18+a4);
  a16=sin(a16);
  a15=(a15*a16);
  a18=(a18*a15);
  a12=(a12+a18);
  a18=(a6*a10);
  a16=(a5*a11);
  a18=(a18-a16);
  a8=(a8+a18);
  a18=6.9999999999999996e-01;
  a16=(a6*a13);
  a4=(a5*a14);
  a16=(a16+a4);
  a16=(a18+a16);
  a16=(a16*a17);
  a16=(a8-a16);
  a4=(a6*a19);
  a20=(a5*a9);
  a4=(a4-a20);
  a4=(a4*a15);
  a16=(a16+a4);
  a16=(a17*a16);
  a12=(a12-a16);
  a10=(a5*a10);
  a11=(a6*a11);
  a10=(a10+a11);
  a13=(a5*a13);
  a14=(a6*a14);
  a13=(a13-a14);
  a13=(a13*a17);
  a13=(a10-a13);
  a19=(a5*a19);
  a9=(a6*a9);
  a19=(a19+a9);
  a18=(a18+a19);
  a18=(a18*a15);
  a13=(a13+a18);
  a13=(a15*a13);
  a12=(a12+a13);
  a0=(a0+a12);
  if (res[0]!=0) res[0][0]=a0;
  a8=(a17*a8);
  a8=(a1-a8);
  a10=(a15*a10);
  a8=(a8+a10);
  if (res[0]!=0) res[0][1]=a8;
  a7=(a3*a7);
  a7=(a2-a7);
  a6=(a3*a6);
  a17=(a17*a6);
  a17=(a7-a17);
  a3=(a3*a5);
  a15=(a15*a3);
  a17=(a17+a15);
  if (res[0]!=0) res[0][2]=a17;
  if (res[0]!=0) res[0][3]=a8;
  if (res[0]!=0) res[0][4]=a1;
  if (res[0]!=0) res[0][5]=a7;
  if (res[0]!=0) res[0][6]=a17;
  if (res[0]!=0) res[0][7]=a7;
  if (res[0]!=0) res[0][8]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int M(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int M_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int M_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void M_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int M_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void M_release(int mem) {
}

CASADI_SYMBOL_EXPORT void M_incref(void) {
}

CASADI_SYMBOL_EXPORT void M_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int M_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int M_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real M_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* M_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* M_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* M_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* M_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int M_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
