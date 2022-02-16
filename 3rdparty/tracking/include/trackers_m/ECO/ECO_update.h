//
// MATLAB Compiler: 6.4 (R2017a)
// Date: Tue Sep 19 21:53:17 2017
// Arguments:
// "-B""macro_default""-W""cpplib:ECO_update""-T""link:lib""ECO_update.m""-C""-d
// ""D:\code_ipl\bin\vs2013\debug"
//

#ifndef __ECO_update_h
#define __ECO_update_h 1

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

#ifdef EXPORTING_ECO_update
#define PUBLIC_ECO_update_C_API __global
#else
#define PUBLIC_ECO_update_C_API /* No import statement needed. */
#endif

#define LIB_ECO_update_C_API PUBLIC_ECO_update_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_ECO_update
#define PUBLIC_ECO_update_C_API __declspec(dllexport)
#else
#define PUBLIC_ECO_update_C_API __declspec(dllimport)
#endif

#define LIB_ECO_update_C_API PUBLIC_ECO_update_C_API


#else

#define LIB_ECO_update_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_ECO_update_C_API 
#define LIB_ECO_update_C_API /* No special import/export declaration */
#endif

extern LIB_ECO_update_C_API 
bool MW_CALL_CONV ECO_updateInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_ECO_update_C_API 
bool MW_CALL_CONV ECO_updateInitialize(void);

extern LIB_ECO_update_C_API 
void MW_CALL_CONV ECO_updateTerminate(void);



extern LIB_ECO_update_C_API 
void MW_CALL_CONV ECO_updatePrintStackTrace(void);

extern LIB_ECO_update_C_API 
bool MW_CALL_CONV mlxECO_update(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_ECO_update
#define PUBLIC_ECO_update_CPP_API __declspec(dllexport)
#else
#define PUBLIC_ECO_update_CPP_API __declspec(dllimport)
#endif

#define LIB_ECO_update_CPP_API PUBLIC_ECO_update_CPP_API

#else

#if !defined(LIB_ECO_update_CPP_API)
#if defined(LIB_ECO_update_C_API)
#define LIB_ECO_update_CPP_API LIB_ECO_update_C_API
#else
#define LIB_ECO_update_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_ECO_update_CPP_API void MW_CALL_CONV ECO_update(int nargout, mwArray& params, mwArray& region, const mwArray& params_in, const mwArray& im);

#endif
#endif
