// test_simple_frame.cpp : Defines the entry point for the console application.
//
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
//#include "samples_utility.hpp"

#include <cmath>

#include "trax.h"
#include "trax/opencv.hpp"
#include "matlab/helpers.h"

#include "ECO/ECO_init.h"
#include "ECO/ECO_update.h"

#include "engine.h"

using namespace std;
using namespace cv;
using namespace trax;

bool Numeric_mxTomw(mxArray* A, mwArray &B)
{
	mxClassID id = mxGetClassID(A);
	bool bret = false;
	void *ptr = mxGetPr(A);

 	int m = mxGetM(A);
 	int n = mxGetN(A);
	
	int dim = mxGetNumberOfDimensions(A);//返回数组维数
	const size_t *pdim = mxGetDimensions(A);//返回各维的元素个数

	B = mwArray(dim, pdim, id);

	size_t len = 1;
	for (int i = 0; i < dim; i++)
	{
		len = len*pdim[i];
	}

	switch (id)
	{
	case mxDOUBLE_CLASS:
	{
		double *data = (double*)ptr;
		B.SetData(data, len);
		bret = true; 
	}
		break;

	case mxSINGLE_CLASS:
	{
		float *data = (float*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxINT8_CLASS:
	{
		INT8_T *data = (INT8_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxUINT8_CLASS:
	{
		UINT8_T *data = (UINT8_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxINT16_CLASS:
	{
		INT16_T *data = (INT16_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxUINT16_CLASS:
	{
		UINT16_T *data = (UINT16_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxINT32_CLASS:
	{
		INT32_T *data = (INT32_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxUINT32_CLASS:
	{
		UINT32_T *data = (UINT32_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxINT64_CLASS:
	{
		INT64_T *data = (INT64_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	case mxUINT64_CLASS:
	{
		UINT64_T *data = (UINT64_T*)ptr;
		B.SetData(data, len);
		bret = true;
	}
		break;

	default:
		break;
	}
	return bret;
}

int main(int argc, char** argv) {
	// show help
	if (argc < 2) {
		cout <<
			" Usage: tracker <video_name>\n"
			" examples:\n"
			" example_tracking_kcf Bolt/img/%04d.jpg\n"
			" example_tracking_kcf faceocc2.webm\n"
			<< endl;
		return 0;
	}

	// declares all required variables
	Rect2d roi;
	Mat frame;

	mclInitializeApplication(NULL, 0);
	ECO_initInitialize();
	ECO_updateInitialize();

	Engine *ep = NULL;
	if (!(ep = engOpen(NULL))) //打开Matlab引擎

	{

		printf("Can't start MATLAB engine\n");

		return (-1);

	}

	// set input video
	std::string video = argv[1];
	VideoCapture cap(video);

	// get bounding box
	cap >> frame;
	roi = selectROI("tracker", frame);

	//quit if ROI was not selected
	if (roi.width == 0 || roi.height == 0)
		return 0;

	trax::Image image = mat_to_image(frame);
	trax::Region region = Region::create_rectangle(roi.x, roi.y, roi.width, roi.height);
	trax::Properties properties;

	int nf = 0;

	mwArray location;
	mwArray params, params_update;
	mwArray im;
	mwArray tr_region;
//	mwArray nframe;

	nf = 1;
	//kcfms_tracker.init(trax::region_to_rect(region), trax::image_to_mat(image));
	//tracker.init(trax::image_to_mat(image), trax::region_to_rect(region));

	mxArray* mxim = image_to_array(image);
	mxArray* mxreg = region_to_array(region);


// 	Image im1 = array_to_image(mxim);
// 	cv::Mat mat1 = image_to_mat(im1);
// 
// 	rectangle(mat1, roi, Scalar(255, 0, 0), 2, 1);
// 
// 	// show image with the tracked object
// 	imshow("tracker", mat1);
// 	if (waitKey(1) == 27)
// 		;

	/*
	mwArray *A;
	mxArray *B;
	A->GetData(B, sizeof(A)); */

	Numeric_mxTomw(mxim, im);
	Numeric_mxTomw(mxreg, tr_region);

// 	engEvalString(ep, "imshow(mxim);");
// 	engEvalString(ep, "imshow(im);");

	//			mlxECO_init(2, mxArray *plhs[], 3, mxArray *prhs[]);
	ECO_init(2, location, params, im, tr_region);

	mxDestroyArray(mxim);
	mxDestroyArray(mxreg);
	// initialize the tracker

	// perform the tracking process
	printf("Start the tracking process, press ESC to quit.\n");
	for (;;) {
		// get frame from the video
		cap >> frame;
		
		// stop the program if no more images
		if (frame.rows == 0 || frame.cols == 0)
			break;

		image = mat_to_image(frame);
		nf++;

		//cv::Rect result = kcfms_tracker.update(image_to_mat(image));
		//cv::Rect result = tracker.update(image_to_mat(image));
		mxArray* mxim = image_to_array(image);
		Numeric_mxTomw(mxim, im);
		ECO_update(2, params_update, location, params, im);
		mxDestroyArray(mxim);

		double *ptr = NULL;
		location.GetData(ptr, sizeof(location));

		cv::Rect result(std::lround(ptr[0]), std::lround(ptr[1]), std::lround(ptr[2]), std::lround(ptr[3]));

		// draw the tracked object
		rectangle(frame, result, Scalar(255, 0, 0), 2, 1);

		// show image with the tracked object
		imshow("tracker", frame);

		//quit on ESC button
		if (waitKey(1) == 27)break;
	}

	ECO_initTerminate();
	ECO_updateTerminate();
	mclTerminateApplication();
	return 0;
}
