// testTrackerCSK.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CSK/csk.h"
#include "CSK/benchmark_info.h"

using namespace cv;
using namespace std;

//int tracker(string video_path, string video_name, double &precision, double &fps);
//vector<double>PrecisionCalculate(vector<Rect>groundtruth_rect, vector<Rect>result_rect);

void usage(bool wait = false)
{
	printf("test tracker CSK: Appropolis Inc.\n");
	printf("usage:\n");
	printf("video \n");
	printf("---------------------------------------------\n");
	if (wait)
	{
		printf("<press ENTER>\n");
		getc(stdin);
	}
	exit(1);
}

static void byebye(bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	/*	exit(1);*/
}

int main(int argc, char * argv[])
{
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pInputVideo = argv[i]; i++;
	//	char *pOutFileName = argv[i]; i++;
	int trackerID = 0;

	bool bCheck = false;
	for (; i < argc; i++)
	{
		// 		if (strcmp(argv[i], "--tracker") == 0)
		// 		{
		// 			i++;
		// 			trackerID = atoi(argv[i]);
		// 		}

	}

	// declares all required variables
	Rect2d roi;
	Mat im;

	// set input video
	std::string video = argv[1];
	VideoCapture cap(video);
	// get bounding box
	cap >> im;
	roi = selectROI("tracker", im);
	//quit if ROI was not selected
	if (roi.width == 0 || roi.height == 0)
		return 0;
	// initialize the tracker
	//tracker.init(frame, roi);

	double padding = 2;
	double output_sigma_factor = 1. / 16;
	double sigma = 0.2;
	double lambda = 1e-2;
	double interp_factor = 0.075;

// 	groundtruth_rect[0].x -= 1;     //cpp is zero based
// 	groundtruth_rect[0].y -= 1;
	Point pos = centerRect(roi);
	Size target_sz(roi.width, roi.height);
	bool resize_image = false;
	if (std::sqrt(target_sz.area()) >= 1000) 
	{
		pos.x = cvFloor(double(pos.x) / 2);
		pos.y = cvFloor(double(pos.y) / 2);
		target_sz.width = cvFloor(double(target_sz.width) / 2);
		target_sz.height = cvFloor(double(target_sz.height) / 2);
		resize_image = true;
	}
	Size sz = scale_size(target_sz, (1.0 + padding));

	double output_sigma = sqrt(double(target_sz.area())) * output_sigma_factor;
	Mat y = CreateGaussian2(sz, output_sigma, CV_64F);
	Mat yf;
	dft(y, yf, DFT_COMPLEX_OUTPUT);

	Mat cos_window(sz, CV_64FC1);
	CalculateHann(cos_window, sz);

//	Mat im;
	Mat im_gray;
	Mat z, new_z;
	Mat alphaf, new_alphaf;
	Mat x;
	Mat k, kf;
	Mat response;
	double time = 0;
	int64 tic, toc;

	// perform the tracking process
	printf("Start the tracking process, press ESC to quit.\n");
	//im = imread(img_files[frame], IMREAD_COLOR);
	

	for (int iframe=0;;iframe++ ) {
		// get frame from the video
		cv::cvtColor(im, im_gray, cv::COLOR_BGR2GRAY);
		//im_gray = imread(img_files[frame], IMREAD_GRAYSCALE);

		if (resize_image)
		{
			resize(im, im, im.size() / 2, 0, 0, INTER_CUBIC);
			resize(im_gray, im_gray, im.size() / 2, 0, 0, INTER_CUBIC);
		}
		
		// update the tracking result
		//tracker.update(frame, roi);
		tic = getTickCount();
		GetSubWindow(im_gray, x, pos, sz, cos_window);

		if (iframe > 0)
		{
			DenseGaussKernel(sigma, x, z, k);
			dft(k, kf, DFT_COMPLEX_OUTPUT);
			cv::idft(ComplexMul(alphaf, kf), response, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT); // Applying IDFT
			Point maxLoc;
			minMaxLoc(response, NULL, NULL, NULL, &maxLoc);
			pos.x = pos.x - cvFloor(float(sz.width) / 2.0) + maxLoc.x + 1;
			pos.y = pos.y - cvFloor(float(sz.height) / 2.0) + maxLoc.y + 1;
		}

		//get subwindow at current estimated target position, to train classifer
		GetSubWindow(im_gray, x, pos, sz, cos_window);

		DenseGaussKernel(sigma, x, x, k);
		dft(k, kf, DFT_COMPLEX_OUTPUT);
		new_alphaf = ComplexDiv(yf, kf + Scalar(lambda, 0));
		new_z = x;

		if (iframe == 0)
		{
			alphaf = new_alphaf;
			z = x;
		}
		else
		{
			alphaf = (1.0 - interp_factor) * alphaf + interp_factor * new_alphaf;
			z = (1.0 - interp_factor) * z + interp_factor * new_z;
		}
		toc = getTickCount() - tic;
		time += toc;
		Rect rect_position(pos.x - target_sz.width / 2, pos.y - target_sz.height / 2, target_sz.width, target_sz.height);
		
		// draw the tracked object
		rectangle(im, rect_position, Scalar(255, 0, 0), 2, 1);
		// show image with the tracked object
		imshow("tracker", im);
		//quit on ESC button
		if (waitKey(1) == 27)break;

		cap >> im;
		// stop the program if no more images
		if (im.rows == 0 || im.cols == 0)
			break;
	}
	return 0;
}

/*
int tracker(string video_path, string video_name, double &precision, double &fps) {

	vector<Rect> groundtruth_rect;
	vector<String>img_files;
	if (load_video_info(video_path, video_name, groundtruth_rect, img_files) != 1)
		return -1;

	double padding = 1;
	double output_sigma_factor = 1. / 16;
	double sigma = 0.2;
	double lambda = 1e-2;
	double interp_factor = 0.075;

	Point pos = centerRect(groundtruth_rect[0]);
	Size target_sz(groundtruth_rect[0].width, groundtruth_rect[0].height);
	bool resize_image = false;
	if (std::sqrt(target_sz.area()) >= 1000) {
		pos.x = cvFloor(double(pos.x) / 2);
		pos.y = cvFloor(double(pos.y) / 2);
		target_sz.width = cvFloor(double(target_sz.width) / 2);
		target_sz.height = cvFloor(double(target_sz.height) / 2);
		resize_image = true;
	}
	Size sz = scale_size(target_sz, (1.0 + padding));
	vector<Rect>result_rect;

	double output_sigma = sqrt(double(target_sz.area())) * output_sigma_factor;
	Mat y = CreateGaussian2(sz, output_sigma, CV_64F);
	Mat yf;
	dft(y, yf, DFT_COMPLEX_OUTPUT);

	Mat cos_window(sz, CV_64FC1);
	CalculateHann(cos_window, sz);

	Mat im;
	Mat im_gray;
	Mat z, new_z;
	Mat alphaf, new_alphaf;
	Mat x;
	Mat k, kf;
	Mat response;
	double time = 0;
	int64 tic, toc;
	for (int frame = 0; frame < img_files.size(); ++frame)
	{

		im = imread(img_files[frame], IMREAD_COLOR);
		im_gray = imread(img_files[frame], IMREAD_GRAYSCALE);
		if (resize_image) {
			resize(im, im, im.size() / 2, 0, 0, INTER_CUBIC);
			resize(im_gray, im_gray, im.size() / 2, 0, 0, INTER_CUBIC);
		}

		tic = getTickCount();

		if (frame > 0)
		{
			GetSubWindow(im_gray, x, pos, sz, cos_window);
			DenseGaussKernel(sigma, x, z, k);
			cv::dft(k, kf, DFT_COMPLEX_OUTPUT);
			cv::idft(ComplexMul(alphaf, kf), response, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT); // Applying IDFT
			Point maxLoc;
			minMaxLoc(response, NULL, NULL, NULL, &maxLoc);
			pos.x = pos.x - cvFloor(float(sz.width) / 2.0) + maxLoc.x + 1;
			pos.y = pos.y - cvFloor(float(sz.height) / 2.0) + maxLoc.y + 1;
		}


		//get subwindow at current estimated target position, to train classifer
		GetSubWindow(im_gray, x, pos, sz, cos_window);

		DenseGaussKernel(sigma, x, x, k);
		dft(k, kf, DFT_COMPLEX_OUTPUT);
		new_alphaf = ComplexDiv(yf, kf + Scalar(lambda, 0));
		new_z = x;

		if (frame == 0)
		{
			alphaf = new_alphaf;
			z = x;
		}
		else
		{
			alphaf = (1.0 - interp_factor) * alphaf + interp_factor * new_alphaf;
			z = (1.0 - interp_factor) * z + interp_factor * new_z;
		}
		toc = getTickCount() - tic;
		time += toc;
		Rect rect_position(pos.x - target_sz.width / 2, pos.y - target_sz.height / 2, target_sz.width, target_sz.height);
		if (resize_image)
			result_rect.push_back(Rect(rect_position.x * 2, rect_position.y * 2, rect_position.width * 2, rect_position.height * 2));
		else
			result_rect.push_back(rect_position);

		rectangle(im, rect_position, Scalar(0, 255, 0), 2);
		putText(im, to_string(frame), Point(20, 40), 6, 1, Scalar(0, 255, 255), 2);
		imshow(video_name, im);
		char key = waitKey(1);
		if (key == 27)
			break;

	}
	time = time / getTickFrequency();
	vector<double>precisions = PrecisionCalculate(groundtruth_rect, result_rect);
	printf("%12s - Precision (20px):%1.3f, FPS:%4.2f\n", video_name, precisions[20], double(img_files.size()) / time);
	destroyAllWindows();
	precision = precisions[20];
	fps = double(img_files.size()) / time;
	return 0;
}

vector<double>PrecisionCalculate(vector<Rect>groundtruth_rect, vector<Rect>result_rect) {
	int max_threshold = 50;
	vector<double>precisions(max_threshold + 1, 0);
	if (groundtruth_rect.size() != result_rect.size()) {
		int n = min(groundtruth_rect.size(), result_rect.size());
		groundtruth_rect.erase(groundtruth_rect.begin() + n, groundtruth_rect.end());
		result_rect.erase(groundtruth_rect.begin() + n, groundtruth_rect.end());
	}
	vector<double>distances;
	for (int i = 0; i < result_rect.size(); i++)
	{
		double distemp = sqrt(double(pow(result_rect[i].x + result_rect[i].width / 2 - groundtruth_rect[i].x - groundtruth_rect[i].width / 2, 2) +
			pow(result_rect[i].y + result_rect[i].height / 2 - groundtruth_rect[i].y - groundtruth_rect[i].height / 2, 2)));
		distances.push_back(distemp);
	}
	for (int i = 0; i <= max_threshold; i++)
	{
		for (int j = 0; j < distances.size(); j++)
		{
			if (distances[j] < double(i))
				precisions[i]++;

		}
		precisions[i] = precisions[i] / distances.size();
	}
	return precisions;
}
*/
