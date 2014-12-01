#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main2()
{
	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	CvCapture* capture = cvCaptureFromCAM(0);
	cout << "open capture" << endl;
	IplImage *img;
	Size boardSize(5, 4);
	vector<Point2f> corners;
	
	while (1) {
		cout << "start while" << endl;
		img = cvQueryFrame(capture);
		cv::Mat imgmat(img, false);
		if (findChessboardCorners(imgmat, Size(5, 4), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE))
		{
			drawChessboardCorners(imgmat, boardSize, Mat(corners), true);
			//calibrateCamera()
		}
		cout << "query frame" << endl;
		if (!img) break;
		//imshow("Example2", frame);
		imshow("Example2", imgmat);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&capture);
	cvWaitKey(0);
	return 0;
}
