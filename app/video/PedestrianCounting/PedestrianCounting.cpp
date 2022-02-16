// PedestrianCounting.cpp : Defines the entry point for the console application.
//
//boost
#include<boost/date_time/posix_time/posix_time.hpp>

//opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

//std
#include<iostream>

//ipl
#include "core/iplcore.h"
#include "core/iplfiles.h"

#include "Blob.h"

#define SAVE_VIDEO

#define SAVE_Snapshot            

// global variables ///////////////////////////////////////////////////////////////////////////////
const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);
const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);

// function prototypes ////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs);
void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex);
void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs);
double distanceBetweenPoints(cv::Point point1, cv::Point point2);
void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName);

void drawContours(cv::Mat &imgFrame2Copy, std::vector<Blob> blobs, std::string strImageName);
void drawAContour(cv::Mat &imgFrame2Copy, std::vector<Blob> blobs, int idx, std::string Name);

bool checkBlobsCrossedTheLine(std::vector<Blob> &blobs, int &InLinePosition, int &OutLinePosition, int &nIn, int &nOut);
//bool checkBlobsOut(std::vector<Blob> &blobs, int &intHorizontalLinePosition, int &Count);

void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy);

bool drawCountOnImage(std::vector<Blob> &blobs, int &InLinePosition, int &OutLinePosition, int &Count, cv::Mat &imgFrame2Copy);

int backgroundSubtractor(cv::Mat &img1, cv::Mat &img2, std::vector<std::vector<cv::Point> > &contours);


int main(int argc, char* argv[])
{
	//char *pInputVideo = argv[1];
//	char *pInputVideo = "rtsp://admin:14863@10.10.10.171:554/Streaming/Channels/501";
	char *pInputVideo = "rtsp://admin:123456@10.10.10.225:554/live1.sdp";

//   	char *pInputVideo = "D:/data_video/test5_raw.flv";   
//   	#undef SAVE_VIDEO

	//IplImage* frame = 0;

	cv::VideoCapture capVideo;
	cv::VideoWriter  writer_raw, writer_marked;

	std::vector<Blob> blobs;

	cv::Mat refBackground;   //如何建立基准背景图

	cv::Mat imgFrame1;
	cv::Mat imgFrame2;
	cv::Mat frame;

	int totalCount = 0;

	capVideo.open(pInputVideo);

	if (!capVideo.isOpened()) {                                                 // if unable to open video file
		std::cout << "error reading video file" << std::endl << std::endl;      // show error message
		return(0);                                                              // and exit program
	}

	cv::Point inLine[2], outLine[2];

	int width = capVideo.get(cv::CAP_PROP_FRAME_WIDTH);
	int height = capVideo.get(cv::CAP_PROP_FRAME_HEIGHT);
	cv::Size frameSize(width, height);
	int fps = capVideo.get(cv::CAP_PROP_FPS);

//	int num = capVideo.get(cv::CAP_PROP_FRAME_COUNT);

	char *pOutputDir = argv[2];
	ipl::createFolder(pOutputDir);

	std::string pure_name, suffix_name;
	ipl::extractPureFileName(pInputVideo, pure_name, suffix_name);

	std::string pLocalBackupVideo_raw, pLocalBackupVideo_marked;
	pLocalBackupVideo_raw = pOutputDir;
	pLocalBackupVideo_raw += "/";
	pLocalBackupVideo_raw += pure_name;
	pLocalBackupVideo_raw += "_raw.flv";

	pLocalBackupVideo_marked = pOutputDir;
	pLocalBackupVideo_marked += "/";
	pLocalBackupVideo_marked += pure_name;
	pLocalBackupVideo_marked += "_marked.flv";


#ifdef SAVE_VIDEO
	/*CV_FOURCC('P', 'I', 'M', '1') = MPEG - 1 codec
		CV_FOURCC('M', 'J', 'P', 'G') = motion - jpeg codec
		CV_FOURCC('M', 'P', '4', '2') = MPEG - 4.2 codec
		CV_FOURCC('D', 'I', 'V', '3') = MPEG - 4.3 codec
		CV_FOURCC('D', 'I', 'V', 'X') = MPEG - 4 codec
		CV_FOURCC('U', '2', '6', '3') = H263 codec
		CV_FOURCC('I', '2', '6', '3') = H263I codec
		CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec*/
	if (!writer_raw.open(pLocalBackupVideo_raw,
		CV_FOURCC('F', 'L', 'V', '1'),
		fps,
		frameSize))
	{
		std::cout << "error! can't save video file!" << std::endl << std::endl;      // show error message
		return(0);
	}

	if (!writer_marked.open(pLocalBackupVideo_marked,
		CV_FOURCC('F', 'L', 'V', '1'),
		fps,
		frameSize))
	{
		std::cout << "error! can't save video file!" << std::endl << std::endl;      // show error message
		return(0);
	}
#endif

#ifdef SAVE_Snapshot
	std::string snapDir;
	snapDir = pOutputDir;
	snapDir += "/snapshot";

	ipl::createFolder(snapDir);
#endif
	
	cv::namedWindow("Frame");

	float ratio = 0.30;
	int intHorizontalLinePosition = (int)std::round(height * ratio);

	inLine[0].x = 0;
	inLine[0].y = intHorizontalLinePosition;
	inLine[1].x = width - 1;
	inLine[1].y = intHorizontalLinePosition;

	intHorizontalLinePosition = (int)std::round(height * (1-ratio));
	outLine[0].x = 0;
	outLine[0].y = intHorizontalLinePosition;
	outLine[1].x = width - 1;
	outLine[1].y = intHorizontalLinePosition;

	char chCheckForEscKey = 0;
	bool blnFirstFrame = true;
	int nframe = 0;
	while (capVideo.isOpened() && chCheckForEscKey != 27)
	{
// 		capVideo >> frame;
// 		if (frame.rows == 0 || frame.cols == 0)
// 			break;

		bool bAbnormalOccured = false;

		std::vector<Blob> currentFrameBlobs;

		cv::Mat imgFrame1Copy;
		cv::Mat imgFrame2Copy;

		if (nframe == 0)
		{
			if (!capVideo.read(imgFrame1))
			{
				std::cout << "end of video\n";
				break;
			}
			//imgFrame2Copy = frame.clone();

			refBackground = imgFrame1.clone();

#ifdef SAVE_VIDEO
			writer_raw << imgFrame1;
			writer_marked << imgFrame1;
#endif
			nframe++;
			continue;
		}

		if (nframe == 1)
		{
			if (!capVideo.read(imgFrame2))
			{
				std::cout << "end of video\n";
				break;
			}

			//imgFrame2Copy = frame.clone();
			//goto Display_Flag;
		}

		std::vector<std::vector<cv::Point> > contours;
		
		backgroundSubtractor(imgFrame1, imgFrame2, contours);

		if (contours.size() > 0)
		{//检测到变换区域
		 //drawAndShowContours(imgThresh.size(), contours, "imgContours");

			std::vector<std::vector<cv::Point> > convexHulls;
			convexHulls.resize((contours.size()));
			for (unsigned int i = 0; i < contours.size(); i++) {
				cv::convexHull(contours[i], convexHulls[i]);
			}

			//drawAndShowContours(imgThresh.size(), convexHulls, "imgConvexHulls");

			for (int i=0; i<convexHulls.size(); i++) //auto &convexHull : convexHulls)
			{
				Blob possibleBlob(convexHulls[i]);
				//std::vector<cv::Point> currentContour = possibleBlob.currentContour;
				double barea = possibleBlob.currentBoundingRect.area();
				double carea = cv::contourArea(possibleBlob.currentContour);

				if (barea > 400 &&
					possibleBlob.dblCurrentAspectRatio > 0.2 &&
					possibleBlob.dblCurrentAspectRatio < 4.0 &&
					possibleBlob.currentBoundingRect.width > 15 &&
					possibleBlob.currentBoundingRect.height > 15 &&
					possibleBlob.dblCurrentDiagonalSize > 60.0 &&
					carea / barea > 0.50)
				{
					currentFrameBlobs.push_back(possibleBlob);
				}
			}

			//drawAndShowContours(imgThresh.size(), currentFrameBlobs, "imgCurrentFrameBlobs");

			if (blnFirstFrame == true)
			{
// 				for (auto &currentFrameBlob : currentFrameBlobs)
// 				{
// 					blobs.push_back(currentFrameBlob);
// 				}

				blobs.insert(blobs.end(), currentFrameBlobs.begin(), currentFrameBlobs.end());
				blnFirstFrame = false;
			}
			else
			{
				matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
			}
			//drawAndShowContours(imgThresh.size(), blobs, "imgBlobs");
		}
		
		imgFrame2Copy = imgFrame2.clone(); // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above

		//drawBlobInfoOnImage(blobs, imgFrame2Copy);
// 		int nIn, nOut;
// 
// 		bool blnAtLeastOneBlobCrossedTheLine = checkBlobsCrossedTheLine(blobs, inLine[0].y, outLine[0].y, nIn, nOut);
// 
// 		totalCount += nIn;
// 		totalCount -= nOut;

		cv::line(imgFrame2Copy, inLine[0], inLine[1], SCALAR_GREEN, 2);
		cv::line(imgFrame2Copy, outLine[0], outLine[1], SCALAR_RED, 2);


		//drawCountOnImage(totalCount, imgFrame2Copy);
		bool blnAtLeastOneBlobCrossedTheLine = drawCountOnImage(blobs, inLine[0].y, outLine[0].y, totalCount, imgFrame2Copy);

		if(contours.size() == 0)
		{//无变化  contours.size() == 0
		 //更新跟踪集合
			if (blobs.size() > 0)
			{
				backgroundSubtractor(refBackground, imgFrame2, contours);
				std::vector<std::vector<cv::Point> > convexHulls(contours.size());
				for (unsigned int i = 0; i < contours.size(); i++)
				{
					cv::convexHull(contours[i], convexHulls[i]);
				}

				std::vector<Blob> diffToRef;
				for (int i = 0; i < convexHulls.size(); i++)
				{
					Blob possibleBlob(convexHulls[i]);
					//std::vector<cv::Point> currentContour = possibleBlob.currentContour;
					double barea = possibleBlob.currentBoundingRect.area();
					double carea = cv::contourArea(possibleBlob.currentContour);

					if (barea > 400 &&
						possibleBlob.dblCurrentAspectRatio > 0.2 &&
						possibleBlob.dblCurrentAspectRatio < 4.0 &&
						possibleBlob.currentBoundingRect.width > 30 &&
						possibleBlob.currentBoundingRect.height > 30 &&
						possibleBlob.dblCurrentDiagonalSize > 60.0 &&
						carea / barea > 0.50)
					{
						diffToRef.push_back(possibleBlob);
					}
				}

				std::vector<Blob>::iterator iter;
				for (iter = blobs.begin(); iter != blobs.end(); ++iter)
				{
					if (iter->blnStillBeingTracked == false)
					{
						int intIndexOfLeastDistance = 0;
						double dblLeastDistance = 100000.0;
						for (int i = 0; i < diffToRef.size(); i++)
						{
							double dblDistance = distanceBetweenPoints(iter->predictedNextPosition,
								diffToRef[i].centerPositions.back());

							if (dblDistance < dblLeastDistance) {
								dblLeastDistance = dblDistance;
								intIndexOfLeastDistance = i;
							}
						}

						bool bStop = false;
						if (dblLeastDistance < iter->dblCurrentDiagonalSize * 0.5)
						{
							bStop = true;//进入监控视场后的静止物体
						}

						if (bStop)
						{
							std::string coutName;
							if (iter->state == Blob::Unknown)
							{//徘徊者
								coutName = "abnormal stop";
							}
							else if (iter->state == Blob::LockedIn)
							{
								coutName = "abnormal In&stop";
							}
							else
							{//区分是消失还是停留
								coutName = "abnormal Out&stop";
							}

							drawAContour(imgFrame2Copy, diffToRef, intIndexOfLeastDistance, coutName);
						}
						else
						{
							iter = blobs.erase(iter);
							if(iter == blobs.end())
								break;
						}
							
					}
				}
			}
		}

		if (blobs.size() == 0)
			blnFirstFrame = true;

		drawContours(imgFrame2Copy, blobs, "imgConvexHulls");

#ifdef SAVE_VIDEO
		writer_raw << imgFrame2;
		writer_marked << imgFrame2Copy;
#endif

#ifdef SAVE_Snapshot
		if (blnAtLeastOneBlobCrossedTheLine)
		{
			const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
			// Get the time offset in current day
			const boost::posix_time::time_duration td = now.time_of_day();
			boost::gregorian::date today = boost::gregorian::day_clock::local_day();
			
			int yyyy = today.year();
			int mm = today.month();
			int dd = today.day();

			int h = td.hours();
			int m = td.minutes();
			int s = td.seconds();
			int ms = td.total_milliseconds() - ((h * 3600 + m * 60 + s) * 1000);

			std::string name;
			name = snapDir;
			
			char buf[256];
			sprintf(buf, "/snapshot_%4d_%02d_%02d_%02d_%02d_%02d.%03d.bmp", yyyy, mm, dd, h,m,s,ms);
			
			name += buf;
			cv::imwrite(name, imgFrame2Copy);
		}
		
#endif
		
		cv::imshow("Frame", imgFrame2Copy);

// 		for (int i = 0; i < contours.size(); i++)
// 		{
// 			contours[i].clear();
// 		}
//		contours.clear();

		currentFrameBlobs.clear();
//		convexHulls.clear();

		imgFrame1 = imgFrame2.clone();           // move frame 1 up to where frame 2 is

// 		if ((capVideo.get(CV_CAP_PROP_POS_FRAMES) + 1) < capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
// 			capVideo.read(imgFrame2);
// 		}
// 		else {
// 			std::cout << "end of video\n";
// 			break;
// 		}
		if (!capVideo.read(imgFrame2))
		{
			std::cout << "end of video\n";
			break;
		}
				
		nframe++;
		chCheckForEscKey = cv::waitKey(1);
	}

	if (chCheckForEscKey != 27) {               // if the user did not press esc (i.e. we reached the end of the video)
		cv::waitKey(0);                         // hold the windows open to allow the "end of video" message to show
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs) {

	//for (auto &existingBlob : existingBlobs) 
	for(int i=0; i<existingBlobs.size(); i++)
	{
// 		existingBlob.blnCurrentMatchFoundOrNewBlob = false;
// 		existingBlob.predictNextPosition();

		existingBlobs[i].blnCurrentMatchFoundOrNewBlob = false;
		existingBlobs[i].predictNextPosition();
	}

	//for (auto &currentFrameBlob : currentFrameBlobs)
	std::vector<Blob>::iterator currentFrameBlob;
	for(currentFrameBlob = currentFrameBlobs.begin(); currentFrameBlob != currentFrameBlobs.end(); ++currentFrameBlob)
	{
		int intIndexOfLeastDistance = 0;
		double dblLeastDistance = 100000.0;

		for (unsigned int i = 0; i < existingBlobs.size(); i++) {

			if (existingBlobs[i].blnStillBeingTracked == true) {

				double dblDistance = distanceBetweenPoints(currentFrameBlob->centerPositions.back(), existingBlobs[i].predictedNextPosition);

				if (dblDistance < dblLeastDistance) {
					dblLeastDistance = dblDistance;
					intIndexOfLeastDistance = i;
				}
			}
		}

		if (dblLeastDistance < currentFrameBlob->dblCurrentDiagonalSize * 0.5) {
			addBlobToExistingBlobs(*currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
		}
		else {
			addNewBlob(*currentFrameBlob, existingBlobs);
		}

	}

	for (auto &existingBlob : existingBlobs) {

		if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
			existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
		}

		if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
			existingBlob.blnStillBeingTracked = false;
		}

	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {

	existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
	existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

	existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

	existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
	existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

	existingBlobs[intIndex].blnStillBeingTracked = true;
	existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;

	if (existingBlobs[intIndex].miny > currentFrameBlob.centerPositions.back().y)
		existingBlobs[intIndex].miny = currentFrameBlob.centerPositions.back().y;
	if (existingBlobs[intIndex].maxy < currentFrameBlob.centerPositions.back().y)
		existingBlobs[intIndex].maxy = currentFrameBlob.centerPositions.back().y;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {

	currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

	existingBlobs.push_back(currentFrameBlob);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double distanceBetweenPoints(cv::Point point1, cv::Point point2) {

	int intX = abs(point1.x - point2.x);
	int intY = abs(point1.y - point2.y);

	return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName) {
	cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

	cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);

	cv::imshow(strImageName, image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawContours(cv::Mat &imgFrame2Copy, std::vector<Blob> blobs, std::string strImageName) 
{

//	cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

	std::vector<std::vector<cv::Point> > contours;

	for (auto &blob : blobs) {
		if (blob.blnStillBeingTracked == true) {
			contours.push_back(blob.currentContour);
		}
	}

	cv::drawContours(imgFrame2Copy, contours, -1, SCALAR_YELLOW, /*-1*/1);

//	cv::imshow(strImageName, image);
}

void drawAContour(cv::Mat &imgFrame2Copy, std::vector<Blob> blobs, int idx, std::string Name)
{

	//	cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

	std::vector<std::vector<cv::Point> > contours;

	for (auto &blob : blobs) {
		if (blob.blnStillBeingTracked == true) {
			contours.push_back(blob.currentContour);
		}
	}

	cv::drawContours(imgFrame2Copy, contours, idx, SCALAR_YELLOW, /*-1*/1);

	int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
	double dblFontScale = blobs[idx].dblCurrentDiagonalSize / 60.0;
	int intFontThickness = (int)std::round(dblFontScale * 1.0);

	cv::putText(imgFrame2Copy, Name, blobs[idx].centerPositions.back(), intFontFace, dblFontScale, SCALAR_RED, intFontThickness);

	//	cv::imshow(strImageName, image);
}
//
///////////////////////////////////////////////////////////////////////////////////////////////////
bool checkBlobsCrossedTheLine(std::vector<Blob> &blobs, int &InLinePosition, int &OutLinePosition, int &nIn, int &nOut)
{
	bool blnAtLeastOneBlobCrossedTheLine = false;
	nIn = 0;
	nOut = 0;

	for (auto blob : blobs) {
		if (blob.blnStillBeingTracked == true && blob.centerPositions.size() >= 2) 
		{
			int prevFrameIndex = (int)blob.centerPositions.size() - 2;
			int currFrameIndex = (int)blob.centerPositions.size() - 1;

			if (blob.centerPositions[prevFrameIndex].y > InLinePosition && blob.centerPositions[currFrameIndex].y <= InLinePosition) {
				nIn++;
				blnAtLeastOneBlobCrossedTheLine = true;
			}

			if (blob.centerPositions[prevFrameIndex].y < OutLinePosition && blob.centerPositions[currFrameIndex].y >= OutLinePosition) {
				nOut++;
				blnAtLeastOneBlobCrossedTheLine = true;
			}
		}

	}

	return blnAtLeastOneBlobCrossedTheLine;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {

	for (unsigned int i = 0; i < blobs.size(); i++) {

		if (blobs[i].blnStillBeingTracked == true) {
			cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);

			int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
			double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
			int intFontThickness = (int)std::round(dblFontScale * 1.0);

			cv::putText(imgFrame2Copy, std::to_string(i), blobs[i].centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool drawCountOnImage(std::vector<Blob> &blobs, int &InLinePosition, int &OutLinePosition, int &Count, cv::Mat &imgFrame2Copy)
{
	bool blnAtLeastOneBlobCrossedTheLine = false;

	int nIn = 0;
	int nOut = 0;

	int midLine = static_cast<int>(0.5*(InLinePosition + OutLinePosition));

	for (auto &blob : blobs) {
		if (blob.blnStillBeingTracked == true && blob.centerPositions.size() >= 2)
		{
			int prevFrameIndex = (int)blob.centerPositions.size() - 2;
			int currFrameIndex = (int)blob.centerPositions.size() - 1;

			
			if (blob.centerPositions[prevFrameIndex].y /*blob.maxy*/ > /*midLine*/InLinePosition && blob.centerPositions[currFrameIndex].y <= InLinePosition) {
								
				blnAtLeastOneBlobCrossedTheLine = true;

				int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
				double dblFontScale = blob.dblCurrentDiagonalSize / 60.0;
				int intFontThickness = (int)std::round(dblFontScale * 1.0);
				
				if (blob.state != Blob::LockedIn)
				{
					nIn++;
					blob.state = Blob::LockedIn;
					cv::rectangle(imgFrame2Copy, blob.currentBoundingRect, SCALAR_GREEN, 2);
					cv::putText(imgFrame2Copy, std::to_string(nIn), blob.centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
				}
				else
				{
					cv::rectangle(imgFrame2Copy, blob.currentBoundingRect, SCALAR_BLUE, 2);
				}
			}

			if (blob.centerPositions[prevFrameIndex].y/*blob.miny*/ < /*midLine*/OutLinePosition && blob.centerPositions[currFrameIndex].y >= OutLinePosition) {
				
				blnAtLeastOneBlobCrossedTheLine = true;

				int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
				double dblFontScale = blob.dblCurrentDiagonalSize / 60.0;
				int intFontThickness = (int)std::round(dblFontScale * 1.0);

				if (blob.state != Blob::LockedOut)
				{
					nOut++;
					blob.state = Blob::LockedOut;
					cv::rectangle(imgFrame2Copy, blob.currentBoundingRect, SCALAR_RED, 2);
					cv::putText(imgFrame2Copy, std::to_string(nOut), blob.centerPositions.back(), intFontFace, dblFontScale, SCALAR_RED, intFontThickness);
				}
				else
				{
					cv::rectangle(imgFrame2Copy, blob.currentBoundingRect, SCALAR_BLUE, 2);
				}
				
			}
		}
	}

	Count += nIn;
	Count -= nOut;

	int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
	double dblFontScale = (imgFrame2Copy.rows * imgFrame2Copy.cols) / 300000.0;
	int intFontThickness = (int)std::round(dblFontScale * 1.5);

	cv::Size textSize = cv::getTextSize(std::to_string(Count), intFontFace, dblFontScale, intFontThickness, 0);

	cv::Point ptTextBottomLeftPosition;

	ptTextBottomLeftPosition.x = imgFrame2Copy.cols - 1 - (int)((double)textSize.width * 1.25);
	ptTextBottomLeftPosition.y = (int)((double)textSize.height * 1.25);

	cv::putText(imgFrame2Copy, std::to_string(Count), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_BLUE, intFontThickness);

	return blnAtLeastOneBlobCrossedTheLine;
}

int backgroundSubtractor(cv::Mat &img1, cv::Mat &img2, std::vector<std::vector<cv::Point> > &contours)
{
	cv::Mat imgFrame1Copy, imgFrame2Copy;
	cv::Mat imgDifference;
	cv::Mat imgThresh;

	imgFrame1Copy = img1.clone();
	imgFrame2Copy = img2.clone();

	cv::cvtColor(imgFrame1Copy, imgFrame1Copy, CV_BGR2GRAY);
	cv::cvtColor(imgFrame2Copy, imgFrame2Copy, CV_BGR2GRAY);

	cv::GaussianBlur(imgFrame1Copy, imgFrame1Copy, cv::Size(5, 5), 0);
	cv::GaussianBlur(imgFrame2Copy, imgFrame2Copy, cv::Size(5, 5), 0);

	cv::absdiff(imgFrame1Copy, imgFrame2Copy, imgDifference);

	cv::threshold(imgDifference, imgThresh, 30, 255.0, CV_THRESH_BINARY);

	//cv::imshow("imgThresh", imgThresh);

	cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::Mat structuringElement15x15 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));

	for (unsigned int i = 0; i < 2; i++) {
		cv::dilate(imgThresh, imgThresh, structuringElement5x5);
		cv::dilate(imgThresh, imgThresh, structuringElement5x5);
		cv::erode(imgThresh, imgThresh, structuringElement5x5);
	}

	cv::Mat imgThreshCopy = imgThresh.clone();

	contours.clear();
	cv::findContours(imgThreshCopy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	return contours.size();
}
