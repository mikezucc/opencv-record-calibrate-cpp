#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	CvCapture* capture = cvCaptureFromCAM(0);
	cout << "open capture" << endl;
	IplImage *img;
	Size boardSize(5, 4);
	vector<Point2f> corners;
	vector<vector<Point2f>> imgPoints;
	Mat testImage;
	Mat outputImage;
	testImage = imread("test.jpg", 1);
	outputImage = Mat(testImage.size(), CV_8UC3);

	vector<Point2f> imageFrame;
	imageFrame.push_back(Point2f(400, 0));
	imageFrame.push_back(Point2f(400, 400));
	imageFrame.push_back(Point2f(0, 0));
	imageFrame.push_back(Point2f(0, 400));

	vector<Point3f> initialFrame;
	initialFrame.push_back(Point3f(400, 0, 0));
	initialFrame.push_back(Point3f(400, 0, 400));
	initialFrame.push_back(Point3f(0, 0, 0));
	initialFrame.push_back(Point3f(0, 0, 400));

	vector<Point2f> transformedFrame;

	initialFrame.resize(4, initialFrame[0]);

	vector<Point3f> objPoints;
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objPoints.push_back(Point3f(float(j * 1), float(i * 1), 0));

	Mat transfMat;

	objPoints.resize(imgPoints.size(), objPoints[0]);
	FileStorage fs;
	fs.open("cameraParams.xml", FileStorage::READ);
	Mat cameraMatrix, distCoeffs;
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;

	cout << "Distortion_Coefficients matrix: " << distCoeffs << endl;

	Mat rvec, tvec;
	
	while (1) {
		cout << "start while" << endl;
		img = cvQueryFrame(capture);
		cv::Mat imgmat(img, false);
		if (findChessboardCorners(imgmat, Size(5, 4), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE))
		{
			drawChessboardCorners(imgmat, boardSize, Mat(corners), true);
			//calibrateCamera()
			bool solved = solvePnP(objPoints, corners, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
			//projectPoints(initialFrame, rvec, tvec, cameraMatrix, distCoeffs, transformedFrame, noArray(), 0);
			//transfMat = getPerspectiveTransform(imageFrame, transformedFrame);
			//warpPerspective(testImage, outputImage, transfMat, outputImage.size(), INTER_LINEAR, BORDER_CONSTANT, 0);
		}
		cout << "query frame" << endl;
		if (!img) break;
		//imshow("Example2", frame);
		imshow("chessboards", imgmat);
		imshow("warped", outputImage);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&capture);
	cvWaitKey(0);
	return 0;
}
