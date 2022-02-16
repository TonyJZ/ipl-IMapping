// trax_server_ECO.cpp : Defines the entry point for the console application.
//

#include <cmath>

#include "trax.h"
#include "trax/opencv.hpp"
#include "matlab/helpers.h"

#include "ECO/ECO_init.h"
#include "ECO/ECO_update.h"

#include "engine.h"


bool Numeric_mxTomw(mxArray* A, mwArray &B)
{
	if (!mxIsNumeric(A))
	{
		return false;
	}

	void *ptr = mxGetPr(A);

	if (mxIsDouble(A))
	{
		double *data = (double*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsSingle(A))
	{
		float *data = (float*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsInt64(A))
	{
		INT64_T *data = (INT64_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}
		
	if (mxIsUint64(A))
	{
		UINT64_T *data = (UINT64_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsInt32(A))
	{
		INT32_T *data = (INT32_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsUint32(A))
	{
		UINT32_T *data = (UINT32_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsInt16(A))
	{
		INT16_T *data = (INT16_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsUint16(A))
	{
		UINT16_T *data = (UINT16_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}
	
	if (mxIsInt8(A))
	{
		INT8_T *data = (INT8_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}

	if (mxIsUint8(A))
	{
		UINT8_T *data = (UINT8_T*)ptr;
		B.SetData(data, sizeof(A));
		return true;
	}
	
// 	if (mxIsChar(A))
// 	{
// 		CHAR16_T *data = (CHAR16_T*)ptr;
// 		B.SetData(data, sizeof(A));
// 		return true;
// 	}

	return false;
}

int main(int argc, char** argv) 
{
	mclInitializeApplication(NULL, 0);
	ECO_initInitialize();
	ECO_updateInitialize();

// 	Engine *ep; //定义Matlab引擎指针。
// 	if (!(ep = engOpen(NULL))) //测试是否启动Matlab引擎成功。
// 	{
// 		cout << "Can't start Matlab engine!" << endl;
// 		exit(1);
// 	}
// 
// 	engEvalString(ep, "figure");        //开一个新的显示窗口

	trax::Server handle(trax::Configuration(TRAX_REGION_RECTANGLE, TRAX_IMAGE_ANY, "ECO", "ECO", "MATLAB"), trax_no_log);

	while (true) {

		trax::Image image;
		trax::Region region;
		trax::Properties properties;

		int nf = 0;

		mwArray location;
		mwArray params;
		mwArray im;
		mwArray tr_region;
		mwArray nframe;

		int tr = handle.wait(image, region, properties);
		if (tr == TRAX_INITIALIZE) 
		{
			nf = 1;
			//kcfms_tracker.init(trax::region_to_rect(region), trax::image_to_mat(image));
			//tracker.init(trax::image_to_mat(image), trax::region_to_rect(region));

			mxArray* mxim = image_to_array(image);
			mxArray* mxreg = region_to_array(region);

			/*
			mwArray *A;
			mxArray *B;
			A->GetData(B, sizeof(A)); */

			Numeric_mxTomw(mxim, im);
			Numeric_mxTomw(mxreg, tr_region);

			nframe.SetData(&(nf), sizeof(nf));
//			mlxECO_init(2, mxArray *plhs[], 3, mxArray *prhs[]);
			ECO_init(2, location, params, im, tr_region);

			mxDestroyArray(mxim);
			mxDestroyArray(mxreg);

			handle.reply(region, trax::Properties());

		}
		else if (tr == TRAX_FRAME) 
		{
			nf++;

			//cv::Rect result = kcfms_tracker.update(image_to_mat(image));
			//cv::Rect result = tracker.update(image_to_mat(image));
			mxArray* mxim = image_to_array(image);
			Numeric_mxTomw(mxim, im);
			ECO_update(2, params, location, params, im);
			mxDestroyArray(mxim);

			double *ptr = NULL;
			location.GetData(ptr, sizeof(location));
			
			cv::Rect result(std::lround(ptr[0]), std::lround(ptr[1]), std::lround(ptr[2]), std::lround(ptr[3]));
			handle.reply(trax::rect_to_region(result), trax::Properties());

		}
		else break;
	}

//	engClose(ep); //关闭Matlab引擎。

	ECO_initTerminate();
	ECO_updateTerminate();
	mclTerminateApplication();
	return 1;
}

