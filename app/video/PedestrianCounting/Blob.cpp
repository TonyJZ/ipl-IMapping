// Blob.cpp

#include "Blob.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
Blob::Blob(std::vector<cv::Point> _contour) {

    currentContour = _contour;

    currentBoundingRect = cv::boundingRect(currentContour);

    cv::Point currentCenter;

    currentCenter.x = (currentBoundingRect.x + currentBoundingRect.x + currentBoundingRect.width) / 2;
    currentCenter.y = (currentBoundingRect.y + currentBoundingRect.y + currentBoundingRect.height) / 2;

	miny = maxy = currentCenter.y;

	offset = currentCenter;

    centerPositions.push_back(currentCenter);

    dblCurrentDiagonalSize = sqrt(pow(currentBoundingRect.width, 2) + pow(currentBoundingRect.height, 2));

    dblCurrentAspectRatio = (float)currentBoundingRect.width / (float)currentBoundingRect.height;

    blnStillBeingTracked = true;
    blnCurrentMatchFoundOrNewBlob = true;
	bAbnormal = false;

    intNumOfConsecutiveFramesWithoutAMatch = 0;

	state = Unknown;


	kalmanFilter.init(4, 2);                             // instantiate Kalman Filter

	float fltTransitionMatrixValues[4][4] = { { 1, 0, 1, 0 },           // declare an array of floats to feed into Kalman Filter Transition Matrix, also known as State Transition Model
											{ 0, 1, 0, 1 },
											{ 0, 0, 1, 0 },
											{ 0, 0, 0, 1 } };         // A

	float* p;
	p = (float*)kalmanFilter.transitionMatrix.data;
	memcpy(p, fltTransitionMatrixValues, sizeof(float) * 16);

//		= cv::Mat(4, 4, CV_32F, fltTransitionMatrixValues);       // set Transition Matrix

	float fltMeasurementMatrixValues[2][4] = { { 1, 0, 0, 0 },          // declare an array of floats to feed into Kalman Filter Measurement Matrix, also known as Measurement Model
												{ 0, 1, 0, 0 } };        // H

	p = (float*)kalmanFilter.measurementMatrix.data;
	memcpy(p, fltMeasurementMatrixValues, sizeof(float) * 8);
//	 = cv::Mat(2, 4, CV_32F, fltMeasurementMatrixValues);     // set Measurement Matrix

	cv::setIdentity(kalmanFilter.processNoiseCov, cv::Scalar::all(0.0001));           // default is 1, for smoothing try 0.0001
	cv::setIdentity(kalmanFilter.measurementNoiseCov, cv::Scalar::all(10));         // default is 1, for smoothing try 10
	cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar::all(0.1));               // default is 0, for smoothing try 0.1
}

Blob::~Blob()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////
void Blob::predictNextPosition(void)
{

	cv::Mat matPredicted = kalmanFilter.predict();

	predictedNextPosition.x = matPredicted.at<float>(0) + offset.x;
	predictedNextPosition.y = matPredicted.at<float>(1) + offset.y;

// 	if (centerPositions.size() > 1)
// 	{
// 		predictedNextPosition.x = matPredicted.at<float>(0);
// 		predictedNextPosition.y = matPredicted.at<float>(1);
// 	}
// 	else
// 	{
// 		predictedNextPosition.x = centerPositions.back().x;
// 		predictedNextPosition.y = centerPositions.back().y;
// 	}
	

//	cv::Point ptPredicted((int)matPredicted.at<float>(0), (int)matPredicted.at<float>(1));

	cv::Mat matActualPosition(2, 1, CV_32F, cv::Scalar::all(0));

	matActualPosition.at<float>(0, 0) = (float)centerPositions.back().x - (float)offset.x;
	matActualPosition.at<float>(1, 0) = (float)centerPositions.back().y - (float)offset.y;

	cv::Mat matCorrected = kalmanFilter.correct(matActualPosition);        // function correct() updates the predicted state from the measurement

	cv::Point ptCorrected((int)matCorrected.at<float>(0), (int)matCorrected.at<float>(1));

	ptCorrected.x += offset.x;
	ptCorrected.y += offset.y;

	predictedcenterPositions.push_back(predictedNextPosition);
	//actualMousePositions.push_back(ptActualMousePosition);
	correctedcenterPositions.push_back(ptCorrected);

	predictedNextPosition = ptCorrected;

	// predicted, actual, and corrected are all now calculated, time to draw stuff

// 	drawCross(imgBlank, ptPredicted, SCALAR_BLUE);                      // draw a cross at the most recent predicted, actual, and corrected positions
// 	drawCross(imgBlank, ptActualMousePosition, SCALAR_WHITE);
// 	drawCross(imgBlank, ptCorrected, SCALAR_GREEN);

}

/*
//old version
void Blob::predictNextPosition(void) {

    int numPositions = (int)centerPositions.size();

    if (numPositions == 1) {

        predictedNextPosition.x = centerPositions.back().x;
        predictedNextPosition.y = centerPositions.back().y;

    }
    else if (numPositions == 2) {

        int deltaX = centerPositions[1].x - centerPositions[0].x;
        int deltaY = centerPositions[1].y - centerPositions[0].y;

        predictedNextPosition.x = centerPositions.back().x + deltaX;
        predictedNextPosition.y = centerPositions.back().y + deltaY;

    }
    else if (numPositions == 3) {

        int sumOfXChanges = ((centerPositions[2].x - centerPositions[1].x) * 2) +
            ((centerPositions[1].x - centerPositions[0].x) * 1);

        int deltaX = (int)std::round((float)sumOfXChanges / 3.0);

        int sumOfYChanges = ((centerPositions[2].y - centerPositions[1].y) * 2) +
            ((centerPositions[1].y - centerPositions[0].y) * 1);

        int deltaY = (int)std::round((float)sumOfYChanges / 3.0);

        predictedNextPosition.x = centerPositions.back().x + deltaX;
        predictedNextPosition.y = centerPositions.back().y + deltaY;

    }
    else if (numPositions == 4) {

        int sumOfXChanges = ((centerPositions[3].x - centerPositions[2].x) * 3) +
            ((centerPositions[2].x - centerPositions[1].x) * 2) +
            ((centerPositions[1].x - centerPositions[0].x) * 1);

        int deltaX = (int)std::round((float)sumOfXChanges / 6.0);

        int sumOfYChanges = ((centerPositions[3].y - centerPositions[2].y) * 3) +
            ((centerPositions[2].y - centerPositions[1].y) * 2) +
            ((centerPositions[1].y - centerPositions[0].y) * 1);

        int deltaY = (int)std::round((float)sumOfYChanges / 6.0);

        predictedNextPosition.x = centerPositions.back().x + deltaX;
        predictedNextPosition.y = centerPositions.back().y + deltaY;

    }
    else if (numPositions >= 5) {

        int sumOfXChanges = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) * 4) +
            ((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) * 3) +
            ((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) * 2) +
            ((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) * 1);

        int deltaX = (int)std::round((float)sumOfXChanges / 10.0);

        int sumOfYChanges = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) * 4) +
            ((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) * 3) +
            ((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) * 2) +
            ((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) * 1);

        int deltaY = (int)std::round((float)sumOfYChanges / 10.0);

        predictedNextPosition.x = centerPositions.back().x + deltaX;
        predictedNextPosition.y = centerPositions.back().y + deltaY;

    }
    else {
        // should never get here
    }

}*/




