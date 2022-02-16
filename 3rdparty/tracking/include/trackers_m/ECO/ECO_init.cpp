//
// MATLAB Compiler: 6.4 (R2017a)
// Date: Tue Sep 19 21:52:40 2017
// Arguments:
// "-B""macro_default""-W""cpplib:ECO_init""-T""link:lib""ECO_init.m""-C""-d""D:
// \code_ipl\bin\vs2013\debug"
//

#include <stdio.h>
#define EXPORTING_ECO_init 1
#include "ECO_init.h"

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
#ifndef LIB_ECO_init_C_API
#define LIB_ECO_init_C_API /* No special import/export declaration */
#endif

LIB_ECO_init_C_API 
bool MW_CALL_CONV ECO_initInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
  if (_mcr_inst != NULL)
    return true;
  if (!mclmcrInitialize())
    return false;
  if (!GetModuleFileName(GetModuleHandle("ECO_init"), path_to_dll, _MAX_PATH))
    return false;
    bResult = mclInitializeComponentInstanceNonEmbeddedStandalone(  &_mcr_inst,
                                                                    path_to_dll,
                                                                    "ECO_init",
                                                                    LibTarget,
                                                                    error_handler, 
                                                                    print_handler);
    if (!bResult)
    return false;
  return true;
}

LIB_ECO_init_C_API 
bool MW_CALL_CONV ECO_initInitialize(void)
{
  return ECO_initInitializeWithHandlers(mclDefaultErrorHandler, mclDefaultPrintHandler);
}

LIB_ECO_init_C_API 
void MW_CALL_CONV ECO_initTerminate(void)
{
  if (_mcr_inst != NULL)
    mclTerminateInstance(&_mcr_inst);
}

LIB_ECO_init_C_API 
void MW_CALL_CONV ECO_initPrintStackTrace(void) 
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


LIB_ECO_init_C_API 
bool MW_CALL_CONV mlxECO_init(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
  return mclFeval(_mcr_inst, "ECO_init", nlhs, plhs, nrhs, prhs);
}

LIB_ECO_init_CPP_API 
void MW_CALL_CONV ECO_init(int nargout, mwArray& location, mwArray& params, const 
                           mwArray& im, const mwArray& region)
{
  mclcppMlfFeval(_mcr_inst, "ECO_init", nargout, 2, 2, &location, &params, &im, &region);
}

