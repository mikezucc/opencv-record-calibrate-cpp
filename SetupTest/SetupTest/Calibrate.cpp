#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void saveCameraParams(Mat& cameraMatrix, Mat& distCoeffs)
{
	FileStorage fs("cameraParams.xml", FileStorage::WRITE);
	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;
}

int main4()
{
	double widthInp, heightInp;

	cout << "Please Enter your symmetric board size width: ";
	cin >> widthInp;
	cout << "Please Enter your symmetric baord size height: ";
	cin >> heightInp;

	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	IplImage *img;
	Mat imgMat;
	Mat rvecsMat;
	Mat tvecsMat;
	Size boardSize(widthInp, heightInp);
	vector<Point2f> corners;
	int photoNum = 0;
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	vector<vector<Point2f>> imgPoints;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	//Mat distCoeffs;
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	vector<Mat> rvecArr;
	vector<Mat> tvecArr;
	double rms;
	vector<double>rmsArr;

	while (1) {
		if (photoNum == (widthInp * heightInp)) break;
		cout << "start while" << endl;
		string imageName = "./images/" + to_string(photoNum) + ".jpg";
		imgMat = imread(imageName, 1);
		if (!imgMat.data) break;
		cout << "image properly read with " << imgMat.cols << " cols" << endl;
		if (findChessboardCorners(imgMat, Size(5, 4), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE))
		{
			cout << "found chessboard corners" << endl;
			imgPoints.push_back(corners);
			cout << "imgPoints dims are " << imgPoints.size() << endl;
			drawChessboardCorners(imgMat, boardSize, Mat(corners), true);
			//calibrateCamera()
		}
		cout << "done through frame " << photoNum << endl;
		imshow("Example2", imgMat);
		char c = cvWaitKey(33);
		if (c == 27) break;
		photoNum++;
	}
	vector<vector<Point3f>> objPoints(1);
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objPoints[0].push_back(Point3f(float(j * 1), float(i * 1), 0));

	objPoints.resize(imgPoints.size(), objPoints[0]);

	cout << "objPoints dims are " << objPoints.size() << endl;
	rms = calibrateCamera(objPoints, imgPoints, Size(640, 480), cameraMatrix, distCoeffs, rvecs, tvecs, 0, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));
	//Mat copyctn;
	rmsArr.push_back(rms);
	cout << "error for " << photoNum << " is: " << rms << endl;

	double totalAvgErr = computeReprojectionErrors(objPoints, imgPoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	cout << "total average error: " << totalAvgErr << endl;
	cout << "Saving parameters..." << endl;
	saveCameraParams(cameraMatrix, distCoeffs);
	cout << "Done saving to \"cameraParams.xml\"" << endl;

	cvWaitKey(0);

	return 0;
}