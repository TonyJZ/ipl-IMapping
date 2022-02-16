// Blob.h

#ifndef MY_BLOB
#define MY_BLOB

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>


///////////////////////////////////////////////////////////////////////////////////////////////////
class Blob {
public:
	enum BlobState
	{
		Unknown = 0,
		LockedIn,
		LockedOut
	};
    // member variables ///////////////////////////////////////////////////////////////////////////
    std::vector<cv::Point> currentContour;

    cv::Rect currentBoundingRect;

    std::vector<cv::Point> centerPositions;
	std::vector<cv::Point> predictedcenterPositions;
	std::vector<cv::Point> correctedcenterPositions;

    double dblCurrentDiagonalSize;
    double dblCurrentAspectRatio;

	double miny, maxy;

    bool blnCurrentMatchFoundOrNewBlob;

    bool blnStillBeingTracked;

	bool bAbnormal;

    int intNumOfConsecutiveFramesWithoutAMatch;

	BlobState state;

	cv::KalmanFilter kalmanFilter;

    cv::Point predictedNextPosition;

	cv::Point offset;

    // function prototypes ////////////////////////////////////////////////////////////////////////
    Blob(std::vector<cv::Point> _contour);
	~Blob();
    void predictNextPosition(void);

};

#endif    // MY_BLOB


