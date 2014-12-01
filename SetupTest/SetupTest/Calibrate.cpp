#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	IplImage *img;
	Mat imgMat;
	Mat rvecsMat;
	Mat tvecsMat;
	Size boardSize(5, 4);
	vector<Point2f> corners;
	int photoNum = 0;
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	vector<Point3f> objPoints;
	vector<vector<Point2f>> imgPoints;
	vector<Point3f> cameraMatrix;
	Mat distCoeffs;
	vector<Mat> rvecArr;
	vector<Mat> tvecArr;
	double rms;
	vector<double>rmsArr;

	cameraMatrix.push_back(Point3f(0, 0, 0));
	cameraMatrix.push_back(Point3f(0, 0, 0));
	cameraMatrix.push_back(Point3f(0, 0, 0));

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objPoints.push_back(Point3f(float(j*1), float(i*1), 0));
	

	while (1) {
		cout << "start while" << endl;
		string imageName = "./images/" + to_string(photoNum) + ".jpg";
		imgMat = imread(imageName, 1);
		if (!imgMat.data) break;
		if (findChessboardCorners(imgMat, Size(5, 4), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE))
		{
			imgPoints.push_back(corners);
			drawChessboardCorners(imgMat, boardSize, Mat(corners), true);
			//calibrateCamera()
			rms = calibrateCamera(objPoints, imgPoints, Size(640, 480), cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));
			//Mat copyctn;
			rvecArr.push_back(rvecsMat);
			tvecArr.push_back(tvecsMat);
			rmsArr.push_back(rms);
			cout << "error for " << photoNum << " is: " << rms << endl;
		}
		cout << "done through frame" << endl;
		imshow("Example2", imgMat);
		char c = cvWaitKey(33);
		if (c == 27) break;
		photoNum++;
	}
	cvWaitKey(0);

	return 0;
}