// testTrackerKCF.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "KCF/KCF_MS/kcftracker.hpp"
#include "KCF/KCF_QW/kcf.hpp"

using namespace cv;
using namespace std;

void usage(bool wait = false)
{
	printf("test tracker KCF: Appropolis Inc.\n");
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

	Rect2d roi;
	Mat frame;

	// create a tracker object
	KCFTracker kcfms_tracker;

	std::string kernel_type = "gaussian";//gaussian polynomial linear
	std::string feature_type = "hog";//hog gray

	KCF kcfqw_tracker(kernel_type, feature_type);

	//Ptr<Tracker> tracker = TrackerTLD::create();
	// set input video
	std::string video = argv[1];
	VideoCapture cap(video);
	// get bounding box
	cap >> frame;
	roi = selectROI("tracker", frame);
	//quit if ROI was not selected
	if (roi.width == 0 || roi.height == 0)
		return 0;
	// initialize the tracker
	kcfms_tracker.init(roi/*Rect(roi.x, roi.y, roi.width, roi.height)*/, frame);

	kcfqw_tracker.Init(frame, roi);
	//g_tracker->init(*(cv::Mat *)param, g_topLeft.x, g_topLeft.y, g_botRight.x, g_botRight.y);

	// perform the tracking process
	printf("Start the tracking process, press ESC to quit.\n");
	for (;; ) {
		// get frame from the video
		cap >> frame;
		// stop the program if no more images
		if (frame.rows == 0 || frame.cols == 0)
			break;
		// update the tracking result
		Rect result1 = kcfms_tracker.update(frame);

		Rect result2 = kcfqw_tracker.Update(frame);

		// draw the tracked object
		rectangle(frame, result1, Scalar(255, 0, 0), 2, 1);
		rectangle(frame, result2, Scalar(0, 0, 255), 2, 1);
		// show image with the tracked object
		imshow("tracker", frame);
		//quit on ESC button
		if (waitKey(1) == 27)break;
	}

	return 0;
}

