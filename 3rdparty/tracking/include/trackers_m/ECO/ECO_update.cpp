//
// MATLAB Compiler: 6.4 (R2017a)
// Date: Tue Sep 19 21:53:17 2017
// Arguments:
// "-B""macro_default""-W""cpplib:ECO_update""-T""link:lib""ECO_update.m""-C""-d
// ""D:\code_ipl\bin\vs2013\debug"
//

#include <stdio.h>
#define EXPORTING_ECO_update 1
#include "ECO_update.h"

static HMCRINSTANCE _mcr_inst = NULL;


#if defined( _MSC_VER) || defined(__BORLANDC__) || defined(__WATCOMC__) || defined(__LCC__) || defined(__MINGW64__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#include <windows.h>

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultPrintHandler(const char *s)
{
  return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultErrorHandler(const char *s)
{
  int written = 0;
  size_t len = 0;
  len = strlen(s);
  written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
  if (len > 0 && s[ len-1 ] != '\n')
    written += mclWrite(2 /* stderr */, "\n", sizeof(char));
  return written;
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_ECO_update_C_API
#define LIB_ECO_update_C_API /* No special import/export declaration */
#endif

LIB_ECO_update_C_API 
bool MW_CALL_CONV ECO_updateInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
  if (_mcr_inst != NULL)
    return true;
  if (!mclmcrInitialize())
    return false;
  if (!GetModuleFileName(GetModuleHandle("ECO_update"), path_to_dll, _MAX_PATH))
    return false;
    bResult = mclInitializeComponentInstanceNonEmbeddedStandalone(  &_mcr_inst,
                                                                    path_to_dll,
                                                                    "ECO_update",
                                                                    LibTarget,
                                                                    error_handler, 
                                                                    print_handler);
    if (!bResult)
    return false;
  return true;
}

LIB_ECO_update_C_API 
bool MW_CALL_CONV ECO_updateInitialize(void)
{
  return ECO_updateInitializeWithHandlers(mclDefaultErrorHandler, mclDefaultPrintHandler);
}

LIB_ECO_update_C_API 
void MW_CALL_CONV ECO_updateTerminate(void)
{
  if (_mcr_inst != NULL)
    mclTerminateInstance(&_mcr_inst);
}

LIB_ECO_update_C_API 
void MW_CALL_CONV ECO_updatePrintStackTrace(void) 
{
  char** stackTrace;
  int stackDepth = mclGetStackTrace(&stackTrace);
  int i;
  for(i=0; i<stackDepth; i++)
  {
    mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
    mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
  }
  mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_ECO_update_C_API 
bool MW_CALL_CONV mlxECO_update(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
  return mclFeval(_mcr_inst, "ECO_update", nlhs, plhs, nrhs, prhs);
}

LIB_ECO_update_CPP_API 
void MW_CALL_CONV ECO_update(int nargout, mwArray& params, mwArray& region, const 
                             mwArray& params_in, const mwArray& im)
{
  mclcppMlfFeval(_mcr_inst, "ECO_update", nargout, 2, 2, &params, &region, &params_in, &im);
}

