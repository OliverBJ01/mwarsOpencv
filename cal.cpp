//
// Created by bernard on 4/06/17.
//
// The camera calibration routines built around chessboard produce two files which get
// saved in the one .xml file (out_camera_data.xml):
// a)'camera matrix' of size 3x3.  Intrinsic matrix: distortions
// b) 'distortion coefficients' of size 5x1.  k1 to k6
//  (Ref: https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html)
// Then src image is undistorted to dst:
//      undistortPoints( src, dst, cameraMatrix, distCoeffs );
// and the point extracted:
//		xx = (uint) (dst.at<Vec2d>(0,0) [0] * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2));
//      yy = (uint) (dst.at<Vec2d>(0,0) [1] * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2));
// NOTE: THE METHOD UNDISTORTS USING A CAMERA/LENS MODEL BASED ON TWO SMALL MATRICES: 3X3 AND 5X1

// My approach will use homography to map 10x10 points (in scanner units) displayed by the scanner on the wall
// to the corresponding camera sensor points (e.g. 640x480). Then scan co-ords = H * camera co-ords to engage.

//const int n = 100;
//Point2f object_points[n],scene_points[n];
//Mat H = findHomography(object_points, scene_points, CV_RANSAC );
//[s*x1, s*y1, s] = H * [x2, y2, 1], and a division by s would give you the points [x1, y1, 1]
//
//https://www.learnopencv.com/homography-examples-using-opencv-python-c/
//		the H Matrix is (matlab-like syntax):
//				[a b c; d e f; g h 1]
////

#include "../mwars.hh"

extern bool gshowCalRegion;	// set by button callback onButton() to display cal region
//extern bool gdisableEngagement;
										//co-ords send to scanner


// Calibration rectangle enabled from button bar function call "howCalRectangle"

//-----------------------------------------------------------------------------------------------
//  void showCalRectangle(int state, void*)
//
//   button bar callback that sets flag to display calibration rectangle
// -----------------------------------------------------------------------------------------------
void showCalRectangle(int state, void*) {
	if (state) {
		gshowCalRegion = true;}
	else {
		gshowCalRegion = false; }
	cout << "click " << state  << " gshowCalRegion " << gshowCalRegion << endl;
}





// -------------------------------------------------------------------------------------------------------------------
//  void displayCalPoint (Image, point)

//  draws co-ords and cross at the point
//-------------------------------------------------------------------------------------------------------------------
void displayCalPoint(IplImage &IplImage, Point point) {
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;    //0.5f
	int thickness = 1;
	char str[12];

	// draw cross at point
	line(cvarrToMat(&IplImage), Point(point.x-4, point.y), Point(point.x+4, point.y), Scalar(0, 0, 255));
	line(cvarrToMat(&IplImage), Point(point.x, point.y-4), Point(point.x, point.y+4), Scalar(0, 0, 255));

	// write co-ords at point
	sprintf(str, "%d,%d", point.x, point.y);
    //y offset to be seen
	putText(cvarrToMat(&IplImage), str, Point(point.x, point.y+6), fontFace, 0.5F, CV_RGB(255,0,0), thickness, 8);
}


// -------------------------------------------------------------------------------------------------------------------
//  void displayCalibrationLines(IplImage &IplImage, int frameWidth, int frameHeight)
//-------------------------------------------------------------------------------------------------------------------
void displayCalibrationLines(IplImage &IplImage, int frameWidth, int frameHeight) {

	Point point_a, point_b;

	// draw cross at boresight
	//   draw_cross1 (cvarrToMat(& IplImage), CvPoint( (frameWidth)/2, (frameHeight)/2), CV_RGB(255,0,0), 20);			// red cross

	// vert and horz lines through centrepoint
	line(cvarrToMat(&IplImage), Point(0, frameHeight / 2), Point(frameWidth, frameHeight / 2), Scalar(0, 0, 255));
	line(cvarrToMat(&IplImage), Point(frameWidth / 2, 0), Point(frameWidth / 2, frameHeight), Scalar(0, 0, 255));

	//draw frame at frame extent
	rectangle(cvarrToMat(&IplImage), Point(0, 0), Point(frameWidth - 1, frameHeight - 1), Scalar(0, 0, 255));

	// draw frame at extents/2
	point_a = Point(frameWidth / 4, frameHeight / 4);
	point_b = Point(frameWidth * 3 / 4 - 1, frameHeight * 3 / 4 - 1);
	//rectangle(cvarrToMat(&IplImage), Point(frameWidth / 4, frameHeight / 4),
	//          Point(frameWidth * 3 / 4 - 1, frameHeight * 3 / 4 - 1), Scalar(0, 0, 255));
	rectangle(cvarrToMat(&IplImage), point_a, point_b, Scalar(0, 0, 255));
	displayCalPoint(IplImage, point_a);
	displayCalPoint(IplImage, point_b);
	//cvInitFont(&font, FONT_HERSHEY_PLAIN, fontScale, fontScale, 0, thickness );

	for (int i = 0; i <= 640; i +=60) {
		displayCalPoint(IplImage, Point(i, 240));
	}
	for (int i = 0; i <= 480; i +=60) {
		displayCalPoint(IplImage, Point(320, i));
	}

	displayCalPoint(IplImage, Point(0, 0));
	displayCalPoint(IplImage, Point(100, 200));
}


// -------------------------------------------------------------------------------------------------------------------
//  int getCameraCalibrationData(string camCalFile, Mat &cameraMatrix, Mat &distCoeffs))
//
// undistort points process:
// Previously have run the camera calibration routine which saves camera instrinics into .xml file
// 1. read camera intrinsics .xml file saved by camera calibration app
// 1.1 cameraMatrix: (double) 3 x 3 with focalX/Y and principalX/Y are in the (3x3) camera matrix in
//			fX 0 pX
//			0 fY pY
//   		0 0 1
// 1.2 distCoeffs:  (double) 5 x 1 array
// 2.  undistortPoints( src, dst, cameraMatrix, distCoeffs )
// 		The undistorted points are relative to the center of the camera lens relative to the sensor
// 		(the principal point) and normalized relative to the focal distance in x and y:
//			 i.e. undistortPoints() moves the points so that the centre of the image is 0,0 and scales everything so that the width and height are both 1 unit.
//			 if my image is 640*480 the coordinate locations are : (320+640*x_value,240+480*y_value);
//			(refer: https://forum.openframeworks.cc/t/undistortpoints-opencv/4192/3)
//	 3.  convert back to 0,0 at top left corner with:
//					undistorted.x = (x* focalX + principalX);  and undistorted.y = (y* focalY + principalY);
//					i.e.
//			   		uint xx = (dst.at<Vec2d>(0,0) [0] * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2));
//			   		uint yy = (dst.at<Vec2d>(0,0) [1] * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2));
//-------------------------------------------------------------------------------------------------------------------
int getCameraCalibrationData(Mat &cameraMatrix, Mat &distCoeffs) {

	FileStorage fs (CAMCALFILE, FileStorage::READ);
	if (fs.isOpened()) {
		fs ["camera_matrix"] >> cameraMatrix;
		fs ["distortion_coefficients"] >> distCoeffs;
		fs.release();
		cout << "Read camera calibration file: " << CAMCALFILE << endl;
		//cout << "camera matrix: " << cameraMatrix << endl;
		//cout << "distortion coefficients: " << distCoeffs << endl;
		return 0;
	}
	else  {
		cerr << "unable to open camera calibration file: " << CAMCALFILE << " " << strerror(errno) << endl;
		return 1;
	}
}




