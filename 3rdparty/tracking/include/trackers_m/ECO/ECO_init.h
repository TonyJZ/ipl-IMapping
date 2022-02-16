//
// MATLAB Compiler: 6.4 (R2017a)
// Date: Tue Sep 19 21:52:40 2017
// Arguments:
// "-B""macro_default""-W""cpplib:ECO_init""-T""link:lib""ECO_init.m""-C""-d""D:
// \code_ipl\bin\vs2013\debug"
//

#ifndef __ECO_init_h
#define __ECO_init_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_ECO_init
#define PUBLIC_ECO_init_C_API __global
#else
#define PUBLIC_ECO_init_C_API /* No import statement needed. */
#endif

#define LIB_ECO_init_C_API PUBLIC_ECO_init_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_ECO_init
#define PUBLIC_ECO_init_C_API __declspec(dllexport)
#else
#define PUBLIC_ECO_init_C_API __declspec(dllimport)
#endif

#define LIB_ECO_init_C_API PUBLIC_ECO_init_C_API


#else

#define LIB_ECO_init_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_ECO_init_C_API 
#define LIB_ECO_init_C_API /* No special import/export declaration */
#endif

extern LIB_ECO_init_C_API 
bool MW_CALL_CONV ECO_initInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_ECO_init_C_API 
bool MW_CALL_CONV ECO_initInitialize(void);

extern LIB_ECO_init_C_API 
void MW_CALL_CONV ECO_initTerminate(void);



extern LIB_ECO_init_C_API 
void MW_CALL_CONV ECO_initPrintStackTrace(void);

extern LIB_ECO_init_C_API 
bool MW_CALL_CONV mlxECO_init(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_ECO_init
#define PUBLIC_ECO_init_CPP_API __declspec(dllexport)
#else
#define PUBLIC_ECO_init_CPP_API __declspec(dllimport)
#endif

#define LIB_ECO_init_CPP_API PUBLIC_ECO_init_CPP_API

#else

#if !defined(LIB_ECO_init_CPP_API)
#if defined(LIB_ECO_init_C_API)
#define LIB_ECO_init_CPP_API LIB_ECO_init_C_API
#else
#define LIB_ECO_init_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_ECO_init_CPP_API void MW_CALL_CONV ECO_init(int nargout, mwArray& location, mwArray& params, const mwArray& im, const mwArray& region);

#endif
#endif
