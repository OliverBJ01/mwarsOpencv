
/*
 *  mwars_03    11/01/2016
 *
 *  Hover over window and CTRL P to present calibrate button to present X at boresight and extent frames for laser alignment
 *
 *  Camera calibration uses camera intrinics file saved by camera calibration routine
 *  at samples/cpp/tutorial_code/calib3d/camera_calibration/
 *
 *
//  1. initLaserEnableTimer() raises a signal @ 3Hz rate which sets flag gLaserEnable
//  2. when an engageable track is detected, flag gEngage is set
//  3. when gLaserEnable&&gEngage true, laserThread() is forked
//  4. fireLaserThread() turns off gEngage, turns on laser, waits, turns off laser
//  5. starts over at step 1.
 *
 * using andrewssobral/bgslibrary
 * refer example bgslibrary/Demo.cpp
 *
 * // cam setting frame rate to 15fps, in bright light the whole thing runs at 30fps

 *
 * video is captured in thread getFrameThread() while main processing loop executes
 *
 * controlling framerate
 * ref: https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python
 * v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute=10
 *It overwrites your camera shutter time to manual settings and changes the shutter time (in ms?) with the last parameter
 * to (in this example) 10. The lower this value, the darker the image
 *
 *
 *
 *
 * */

#include "../mwars.hh"
#include "../comms.hh"
#include "../package_bgs/bgslibrary.h"

// Globals
bool gLaserEnable = false;			// set locally from ggLaserEnable
bool gEngage = false;				// set when target is to be engaged, results in laser turn on command
bool gshowCalRegion = false;		// set by button callback onButton() to display cal region
bool gdisableEngagement;			// set by button bar to disable any engagement
float xx, yy;						// co-ords send to scanner

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

// associated with void getFrameThread()
VideoCapture cap;			    // opencv video capture class
volatile int produceCount = 0, consumeCount = 0;
int frameBufferSize = 1;

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//  int mwars()
// -----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

int mwars() {

	pthread_t lt, ft;
	int s;
	int frameCounter = 0;
	int frameWidth, frameHeight;    // camera frame size read during videocapture

	//VideoCapture cap;
	Mat frame;						// the live image  (3 channels)
	Mat bgMask;   					// foreground image  ( 3 channels)
	Mat fgMask;   					// foreground mask  ( 1 channel)
	Mat backGnd; 					//background image  (3 channels)
	Mat targetROI;                   // patch of frame displayed

	IplImage  iplFrame;			    // iplImage copy of (Mat) frame for cvBlobs use
	IplImage * iplbgMask;		    // ptr to iplImage copy of (Mat)  for cvBlobs use
	IplImage * imgLabel;		    // cvBlob's cvLabel's output image matrix
	CvTracks tracks;				//cvBlob's map of tracks

	ofstream logfile;				// logfile handle

	int roi_len = 60;	            // size of the focus frame

	// for camera intrinsics
	Mat cameraMatrix, distCoeffs;   // camera calibration matrices
	// for homography file:
	Mat H;                          // homography matrix
	vector<Point2f> dstVec;         // homog vectors
	vector<Point2f> srcVec;
	vector<Point2i> srcCrnrs;       // vector of 2 points: TLHC and BRHC used to specify limits that scanner is to slew to

#ifdef USECUDA
    cuda::GpuMat cudaframe, cudabgMask;
    cuda::setDevice(0);             // init CUDA
#endif

    // ---------  timer for controlling and measuring frame rate ---------------------
    constexpr float frameRate  {28};  				// required frame rate
    //constexpr float framePeriod {1000/frameRate};	// required frame period in mS
    //double tickCount;							    // tick count per loop
    //double fps, sleepTime, execTime;
    //double tickFreq = getTickFrequency();		    // gives ticks per second  (10^9)
    unsigned long nFrames = 0;						// frame counter, No. frames


    // -----------------------------------  Tuning -------------------------------------
	//----------------------------------------------------------------------------------

    // -- cvBlob cvFilterbyArea() params
    int minBlobArea = 50;		    // init Control Panelslider value
    int maxBlobArea = 3000;			// init Control Panel slider value

    // -- image threshold parameter
    int threshold_value = 120;	    // init Control Panel slider value

    // -- cvUpdateTracks()    tracking params
    double distance = 200.;			// Max distance to determine when a track and a blob match.
    unsigned int inactive = 5;		// Max number of frames a track can be inactive.
    unsigned int active = 2;	    // If a track becomes inactive but it has been active  // default = 0
    							    // less than Active frames, the track will be deleted.
	// -- Control Panel
	// CNTRL P to display, while mouse on windoe
	// a) Thresholding & blob size filters
	// b) button: show calibration square
	// c) button: disable engagement
	namedWindow ("TrackBarWnd", WINDOW_NORMAL);   // seems to need this even though not used

	// image thresholding trackbar
	createTrackbar( "blob Thold", nullptr, &threshold_value, 255, nullptr, nullptr); // nullptr to display on Control Panel

	// min/max blob area trackbar cvFilterByArea()
	createTrackbar( "blob min", nullptr,  &minBlobArea, 300,  nullptr, nullptr);
	createTrackbar( "blob max", nullptr,  &maxBlobArea, 2000, nullptr, nullptr);

#ifdef USEMOG2
	// Trackbar to set MOG2 Variance Threshold
	int varThreshold = 10;   // TH on the squared Mahalanobis distance between the pixel and the model
	createTrackbar( "Mog2 varThold", nullptr, &varThreshold, 20, nullptr, nullptr);
#endif

	// Hover over window and Control P presents the control panel
	createButton("Show Cal Rectangle", showCalRectangle,  nullptr, QT_CHECKBOX, false);
	createButton("Disable Engagement", disableEngagement, nullptr, QT_CHECKBOX, false);

	cout << "CTL P to show Control Panel " << endl;


	// ********************** STARTS HERE ***********************************
	if (openLogFile (logfile, LOGFILE))
		return EXIT_FAILURE;

	// load camera intrinsics files
	if (getCameraCalibrationData(cameraMatrix, distCoeffs) )
		return EXIT_FAILURE;

	// connect camera; IP or USB
	string camType = "USB";
	int camNo = 0;
	string ipCamAddr = IPCAM_STREAMADDRESS;
	if (cameraCapture (cap, camType, camNo, ipCamAddr, frameWidth, frameHeight ))         // establish videocapture with USB or IP camera
			return EXIT_FAILURE;
	cap >> frame;           // needs initialisation before using cameraThreadingTest()


//	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));


	// create getFrameThread(); updates Mat frame for this thread
	s = pthread_create(&ft, nullptr, getFrameThread, &frame);
	if (s) error("pthread_getFrameThread()_create error");

	// load homography matrix
	if (readHomography(H, HOMOGFILE, frameWidth, frameHeight, srcCrnrs))
		return EXIT_FAILURE;

	// ----- Options; Connect to scanner server OR scanner console
#ifdef USESCANNERSERVER
	// make TCP connection to mcu scanner server, set up network and return global socket descriptor
	if (connectToScanServer()) {
		error("Network connection not established");
		return EXIT_FAILURE;
	}
#else
	// establish UDP socket to stream UDP packets to QT5 Scanner Console
	if (createSocketScannerServer()) {
		error("Network connection not established");
		return EXIT_FAILURE;
	}
#endif

	// init scanner, and point down boreline
#ifdef USESCANNERSERVER
	initScanner();
#endif

	// MUST PULSE THE LASER ELSE system tracks the laser bloom
	// this timer raises signal SIGALM at 3Hz. initLaserEnableTimer() is the handler and it sets gLaserEnable = true;
	// on engagement, LaserThread() is run which turns laser on for 500mS, then off and exits
	//NEED TO LOOK AT THIS
	if (initLaserEnableTimer())
		return EXIT_FAILURE;  // failed

	imgLabel = cvCreateImage(frame.size(), IPL_DEPTH_LABEL, 1);	// for cvLabel later

	// targetROI is a 60x60 pixel patch from frame that is displayed in the Target Window
	targetROI =  frame (Rect (0, 0, roi_len, roi_len));  //looks at 60x60 patch TLH corner by default

#ifdef USELITIVSUBSENSE
	// uses opencv 2 paradigm
	Ptr<BackgroundSubtractorSuBSENSE> bg_model = new BackgroundSubtractorSuBSENSE();
	bg_model->initialize( frame, bgMask);   //! (re)initiaization method; needs to be called before starting background subtraction
#endif

#ifdef USEBGSLIBSUBSENSE
	//IBGS *bgs;                      // this wraps the functions in a class
	//bgs = new SuBSENSE;
	Ptr<BackgroundSubtractorSuBSENSE> bg_model;
	bg_model = new	BackgroundSubtractorSuBSENSE(BGSSUBSENSE_DEFAULT_LBSP_REL_SIMILARITY_THRESHOLD,
			BGSSUBSENSE_DEFAULT_DESC_DIST_THRESHOLD_OFFSET,
			BGSSUBSENSE_DEFAULT_MIN_COLOR_DIST_THRESHOLD,
			BGSSUBSENSE_DEFAULT_NB_BG_SAMPLES,
			BGSSUBSENSE_DEFAULT_REQUIRED_NB_BG_SAMPLES,
			BGSSUBSENSE_DEFAULT_N_SAMPLES_FOR_MV_AVGS);
	bg_model->initialize( frame, bgMask);   //! (re)initiaization method; needs to be called before starting background subtraction
#endif

#ifdef USEMOG2
	#ifdef USECUDA
	Ptr< cuda::BackgroundSubtractorMOG2> bg_model = cuda::createBackgroundSubtractorMOG2();
	#else
        Ptr<BackgroundSubtractorMOG2> bg_model = createBackgroundSubtractorMOG2();
	#endif
	// default initialisation params (history = 500, varThreshold = 16, detectShadows = true)
	bg_model->setVarThreshold((double)varThreshold);          // for both CUDA & non-CUDA
#endif




	//********************************************************************************
	// ********************** MAIN PROCESSING LOOP ***********************************
	auto start = chrono::high_resolution_clock::now();
	while (1)      {

		// frames are updated by thread getFrameThread(); yield if not updated yet
		while (produceCount - consumeCount == 0) {
			// following allows buffer > 1..
			//consumeToken(&sharedBuffer[consumeCount % BUFFER_SIZE]);
			sched_yield();      // sharedBuffer is empty
		}
		++consumeCount;

		if( frame.empty() )   break;


        // ***********************  Background Subtraction  ***************************
#ifdef USELITIVSUBSENSE
		bg_model->apply(frame, bgMask);
#endif

#ifdef USEBGSLIBSUBSENSE
		// classed version
		//bgs->process(frame, bgMask, backGnd); // by default, it shows automatically the foreground mask image
		bg_model->apply(frame, bgMask);
#endif

#ifdef USEMOG2
        //update the MOG2 model
        // background Mask = 1 channel grey scale
		bg_model->setVarThreshold((double)varThreshold); // both CUDA & non-CUDA
#ifdef USECUDA
		cudaframe.upload(frame);                        //convert MAT to GPUMat
        bg_model->apply(cudaframe, cudabgMask);		    // background Mask = 1 channel grey scale
#else
        bg_model->apply(frame, bgMask);
#endif
#endif


        // ************************  Image Thresholding  **************************************
#ifdef USECUDA
        // threshold the greys in bgMask, this removes them but slider works strange?
        cuda::threshold(cudabgMask, cudabgMask, (double)threshold_value, 255, CV_THRESH_BINARY); // 0: CV_THRESH_BINARY
#else
        threshold(bgMask, bgMask, (double)threshold_value, 255, CV_THRESH_BINARY);              // 0: CV_THRESH_BINARY
#endif

        // *************************** Morphology  **************************************
        //change morphological close value (25) to another value to get a solid blob
        Mat element(5, 5, CV_8U, Scalar(1));
#ifdef USECUDA
        Ptr<cuda::Filter> openFilter = cuda::createMorphologyFilter(MORPH_OPEN, cudabgMask.type(), element);
        Ptr<cuda::Filter> closeFilter = cuda::createMorphologyFilter(MORPH_CLOSE, cudabgMask.type(), element);
        openFilter->apply(cudabgMask, cudabgMask);
        closeFilter->apply(cudabgMask, cudabgMask);
        cudabgMask.download(bgMask);
#else
        morphologyEx(bgMask, bgMask, MORPH_OPEN, element);	 // dilation then erosion
        morphologyEx(bgMask, bgMask, MORPH_CLOSE, element);  // erosion then dilation
#endif

		namedWindow("bgMask Window", WINDOW_NORMAL);
		imshow     ("bgMask Window", bgMask);			//morphed foreground mask


		// ********************** cvBlobs: Connected Components Section  **********************
		CvBlobs blobs;							// map of blobs, needs to stay in the loop

		iplbgMask = new IplImage(bgMask);		// ptr to iplImage copy of bgMask for cvBlobs use

		// labels the connected parts (blobs).
		// Returns: a) imgLabel image of labels, b) an stl map of blobs
		cvLabel(iplbgMask, imgLabel, blobs);
		//namedWindow("imgLabel", WINDOW_NORMAL);
		//imshow("imgLabel", cvarrToMat(imgLabel));

		// filter blobs by size
		cvFilterByArea(blobs, (unsigned int)minBlobArea, (unsigned int)maxBlobArea);
		//void cvFilterBlobsByColour(CvBlobs &blobs, IplImage const *imgLabel, Mat &frame);
		// filter blobs by colour, reject red
		//cvFilterBlobsByColour(blobs, imgLabel, frame);

		iplFrame = frame;						// iplImage shallow copy of (Mat) frame for cvBlobs use
		cvRenderBlobs(imgLabel, blobs, &iplFrame, &iplFrame, CV_BLOB_RENDER_BOUNDING_BOX);

		//unused cvBlob functions to consider: : cvsetregionof interest(), cvrenderblob() ,colour()
		//namedWindow("Blobs_Connected", CV_GUI_EXPANDED);
		//imshow("Blobs_Connected", cvarrToMat(&iplFrame));


        // ************************* cvBlobs: Tracking Section  ******************************
        // cvUpdateTracks()
        //	brief Updates list of tracks based on current blobs.
        // 		param: List of blobs - input
        // 		param: List of tracks - updated
        // 		param: Max distance to determine when a track and a blob match.
        // 		param: Max number of frames a track can be inactive.
        // 		param: If a track becomes inactive but it has been active less than thActive frames, the track will be deleted.
		cvUpdateTracks(blobs, tracks, distance, inactive, active);

		// namedWindow ( "Blobs Tracking Wnd",  CV_GUI_EXPANDED);
		//  imshow("Blobs Tracking Wnd", cvarrToMat(&iplFrame));

		// line( cvarrToMat(&iplFrame),  CvPoint(10, 20), CvPoint(40, 50), CV_RGB(255,0,0), 10, 2 );

		// if gshowCalRegion flag set, set calibration lines on image
		// these are used by scanner server to align camera and scanner, and scale camera pixels to
		// scanner units
		if (gshowCalRegion)
			displayCalibrationLines( iplFrame, frameWidth, frameHeight);

		// --------------- DON'T ENGAGE: when there are between 1 and 3 tracks ----------------------------
		if (tracks.empty() || tracks.size() > 3)  {
			gEngage = false;
			gLaserEnable = false;
			//printf(" Path 1: %d %d %d \n", gEngage, gLaserEnable, laserOnCount);
		}
		else {
			//------------- POSSIBLY ENGAGE: when ther are 1 to 3 tracks ----------------------------------
			//cvRenderTracks()											    //CvTracks is pair: {CvID, CvTrack}
			//	brief Prints tracks information.
			// 		param: List of tracks input
			// 		param: Input image (depth=IPL_DEPTH_8U and num. channels=3).
			// 		param: Output image (depth=IPL_DEPTH_8U and num. channels=3).
			// 		param: Render mode. By default is CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX.
			// 		param: OpenCV font for print on the image.
			// 				|CV_TRACK_RENDER_TO_STD to print track stats
			cvRenderTracks(tracks, &iplFrame, &iplFrame, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

			//  Select the track with longest lifetime
			// searches each key/value pair in tracks (STL map) and returns iterator pointing to longest lifetime track
			CvTracks::const_iterator iter, iterMax;
			unsigned int maxLifetime = 0;
			for (iter = tracks.begin();
			     iter != tracks.end(); ++iter) {    // scan each track and select track with longest lifetime
				if ((iter->second->lifetime) >
				    maxLifetime) {                // Note iter = tracks.end() at end of this loop, and if used to
					maxLifetime = iter->second->lifetime;     // access an element causes error-less crash
					iterMax = iter;                           // iterMax points to longest duration track
				}
			}
			// obtain centroid of longest lifetime track, we already have its lifetime
			CvPoint2D64f trackCentroid = iterMax->second->centroid;
			//cout << "Lifetime: " << maxLifetime  <<" Centroid:  "  << (int)trackCentroid.x << ", " << (int)trackCentroid.y  <<endl;

			// display track as white cross
			draw_cross1(cvarrToMat(&iplFrame), cvPoint((int) trackCentroid.x, (int) trackCentroid.y),
			            CV_RGB(255, 255, 255), 10);


			// ******************************************************************************************
			// ********************************* engagement  ********************************************
			// ******************************************************************************************
			// using the longest lifetime track
#define MIN_TRACK_LIFETIME_TO_ENGAGE 30

			// centres targetROI, the the 60x60 patch of frame to be displayed in Target Window, over the target centroid
			// keeping inside frame's bounds
			int roi_x, roi_y;
			if ((roi_x = (int) (trackCentroid.x - roi_len / 2)) < 0) roi_x = 0;;
			if ((roi_x + roi_len) > CAMERA_X_MAX) roi_x = CAMERA_X_MAX - roi_len;
			if ((roi_y = (int) (trackCentroid.y - roi_len / 2)) < 0) roi_y = 0;;
			if ((roi_y + roi_len) > CAMERA_Y_MAX) roi_y = CAMERA_Y_MAX - roi_len;

			targetROI = frame(Rect(roi_x, roi_y, roi_len, roi_len));

			if (maxLifetime < MIN_TRACK_LIFETIME_TO_ENGAGE) {
				draw_cross1(cvarrToMat(&iplFrame), cvPoint((int) trackCentroid.x, (int) trackCentroid.y),
				            CV_RGB(0, 255, 0), 10);
				//	draw_cross(cvPoint( (int)measurementCoord.x, (int)measurementCoord.y), CV_RGB(0,255,0), 7);	// green cross to rawImage
				//cvCircle( &iplFrame,cvPoint( (int) trackCentroid.x, (int) trackCentroid.y), 6, CV_RGB(0, 255,0), 4, CV_AA, 0);
				gEngage = false;                                                                    // laser off
				//cout << "laser "  << gEngage  << endl;
				cvCircle(&iplFrame, cvPoint(20, 20), 6, CV_RGB(0, 255, 0), 4, CV_AA, 0);
			} else {
				// engage
				draw_cross1(cvarrToMat(&iplFrame), cvPoint((int) trackCentroid.x, (int) trackCentroid.y),
				            CV_RGB(255, 0, 0), 10);;    // red cross
				//draw_cross(cvPoint( (int)measurementCoord.x, (int)measurementCoord.y), CV_RGB(255,0,0), 7);	// red cross
				gEngage = true;                                                                        // laser on
				//cout << "laser "  << gEngage  << endl;
				cvCircle(&iplFrame, cvPoint(20, 20), 6, CV_RGB(255, 0, 0), 4, CV_AA, 0);
			}

			// ---------------------------------- Optics corrections -----------------------------------
			// define USECAMERAMODEL to use checkerboard camera calibration intrinsics file
			// Note: if homography was done using intrinsics file, must do instrinsics correction now
#ifdef USECAMERAMODEL
			// a) Correct Camera Distortion -------------------------------------
			//    uses camera intrinics file save by camera calibration routine
			//      from openCV undistortPoints docs;
			//      (u,v) is the input point, (u', v') is the output point
			//      camera_matrix=[fx 0 cx; 0 fy cy; 0 0 1]
			//      (x',y') = undistort(x",y",dist_coeffs)
			//      u' = x*fx' + cx'
			//      v' = y*fy' + cy'

			Mat dst;

			// convert Point trackCentroid into Mat for undistortPoints()
			Mat_<Vec2d> src(1, 1, (Vec2d(trackCentroid.x, trackCentroid.y)));
			//Mat_<Point2d> src (1, 1, (Point2f (111.1, 222.2)));
			//cout << "mwars " << mwars.at<Vec2d>(0,0) [0] << endl;  cout << "mwars " << mwars.at<Vec2d>(0,0) [1] << endl;

			undistortPoints(src, dst, cameraMatrix, distCoeffs);
			//cout << "dst " << dst.at<Vec2d>(0,0) [0] << endl;  cout << "dst " << dst.at<Vec2d>(0,0) [1] << endl;

			// translate undistorted points back to Mat co-ordinates
			xx = (float) (dst.at<Vec2d>(0, 0)[0] * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2));
			yy = (float) (dst.at<Vec2d>(0, 0)[1] * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2));
			//cout << "xx " << xx << endl;  cout << "yy " << yy << endl;
#else
			// centroid points for homography
			xx =  trackCentroid.x;
			yy =  trackCentroid.y;
#endif  //USECAMERAMODEL

			// b) apply the homography matrix to xx, yy -----------------------------------------
			//      uses homography matrix save by homography app
			dstVec = {Point2f(xx, yy)};
			perspectiveTransform(dstVec, srcVec, H);
			//cout << "   dst " << int(dstVec[0].x) << int(dstVec[0].y) << endl;
			xx =  srcVec[0].x;
			yy =  srcVec[0].y;
			//cout << "xx yy " << xx << " " << yy << endl;

			// limit scanner slewing
			// srcCrnrs is vector of 2 points; TLHC & BRHC
			xx = (xx < srcCrnrs[0].x) ? srcCrnrs[0].x : xx;     // tlhc
			xx = (xx > srcCrnrs[1].x) ? srcCrnrs[1].x : xx;     // brhc
			yy = (yy < srcCrnrs[0].y) ? srcCrnrs[0].y : yy;
			yy = (yy > srcCrnrs[1].y) ? srcCrnrs[1].y : yy;
			// ---------------------   end of optics ------------------------------------------------

			// slew scanner
			sndMssgToScanServer((int)xx, (int)yy, CMD_SLEW);

			// if engagement params met then fire laser for brief period
			if (gLaserEnable && gEngage && !gdisableEngagement) {        // ENGAGED & ENABLED; laser ON
				//printf("Path 2:  %d %d  \n", (int)xx,  (int)yy);

				s = pthread_create(&lt, nullptr, fireLaserThread, (void *) "fire!");        // turn laser on for period
				if (s) {
					cerr << "Cannnot create laser pthread" << strerror(errno) << endl;
					return 1;
				}
			}
			else  // engagement params not

			// different message formats for scanner server OR QT5 scanner console
#ifdef USESCANNERSERVER
			{
			sndMssgToScanServer((int)xx, (int)yy, CMD_LASER_OFF);
			//cout << "xx (int)yy: " << xx << " " << (int)yy << endl;
			}
#else
			sndMssgToScanConsole((int)xx, (int)yy, false);	 						// track target only
#endif
		}  // END OF POSSIBLY ENGAGE ***************
       	   // thread holding laser on continues until it times out



		namedWindow ("Main Window", WINDOW_NORMAL);
		imshow      ("Main Window", cvarrToMat(&iplFrame));

		// displays the patch of frame aound the target
		namedWindow ("Target Window", WINDOW_NORMAL);
		imshow      ("Target Window", targetROI);

	    //logFile << "Track#: " << longestTrackID << " Lifetime: " << maxLifetime << " Target: " << target.at<float>(0) << " " << target.at<float>(1) <<  endl;
	    //cout << "Track#: " << longestTrackID << " Lifetime: " << maxLifetime << " Target: " << target.at<float>(0) << " " << target.at<float>(1);
	    //cout << " fps: " << fps;
	    //cout  << " blob min: " << minBlobArea << " blob max: " << maxBlobArea;
	    //cout << endl;

        // --------------------- User input ---------------------
        auto k = (char)waitKey(1);     // param is mS delay wait for key event
        if( k == 27 ) break;
        if( k == ' ' )  {   printf("Space bar pressed\n");  break;  }

		// measure the frame rate
	    ++frameCounter;
		int cycles = 50;                               // measure this No cycles
		if (frameCounter >= cycles) {                  // count 100 frames
			auto finish = chrono::high_resolution_clock::now();
			chrono::duration<double> elapsed = finish - start;
			cout << "frame rate: "<< cycles/elapsed.count() << " Hz\n";        //elapsed.count() in seconds
			frameCounter = 0;
			start = chrono::high_resolution_clock::now();
		}

	}	//  while (1) ****************************   MAIN PROCESSING LOOP  ***********************************

	sndMssgToScanServer(30000, 30000, CMD_LASER_OFF);
	cout << "destroying windows, closing logfile...." <<endl;
	logfile << "closed" << endl;
	logfile.close();
	destroyAllWindows();

    return EXIT_SUCCESS;
}






// intended to detect sndMssgToScanServer() failures, resolved with select()

//	// setup signal handler for comms
//	memset(&sa, 0, sizeof(sa));
//	sa.sa_sigaction = sighandler;
//	sa.sa_flags = SA_SIGINFO;
//	sigaction(SIGPIPE, &sa, nullptr);          // pipe error

//void sighandler(int signum, siginfo_t *info, void *ptr) {
//	printf("Received signal %d\n", signum);
//	printf("sighandler- Signal originates from process %lu\n",
//	(unsigned long)info->si_pid);
//}



/*
//-------------------------------------------------------------------------
// 	cvBlobMeanColor() returns cvScalar with count of red  green blue pixels
// 	red: colour.val[0] green: colour.val[1]  blue: colour.val[2]

// ------------------------------------------------------------------------

void cvFilterBlobsByColour(CvBlobs &blobs, IplImage const *imgLabel, Mat &frame)  {

	CvBlobs::iterator it=blobs.begin();
	while(it!=blobs.end())
	{
		CvBlob *blob=(*it).second;			// blob is a pointer
		//CvBlob *blob = it->second;

		//CvScalar cvBlobMeanColor(CvBlob const *blob, IplImage const *imgLabel, IplImage const *img);

	    IplImage iplFrame = frame;	   // iplImage shallow copy of Mat frame
	    CvScalar colour = cvBlobMeanColor( blob, imgLabel, &iplFrame);

		  /// \param blob Blob.
		  /// \param imgLabel Image of labels.
		  /// \param img Original image.
		  /// \return Average color:

	    double area = it->second->area;
	    double r = colour.val[0]/(colour.val[0]+colour.val[1]+colour.val[2]);
	    double g = colour.val[1]/(colour.val[0]+colour.val[1]+colour.val[2]);
	    double b = colour.val[2]/(colour.val[0]+colour.val[1]+colour.val[2]);

		cout << "rgb: " << r << g << b << " area " << area <<endl;

		// if red pixels predominateit->second
	    //  cout << "b " << (unsigned int)colour.val[0] <<" " << (unsigned int)colour.val[1]
		//			<<" " <<  (unsigned int)colour.val[2] << endl;

		if (colour.val[0] > 2 * (colour.val[1]+colour.val[2]))
		{
			cvReleaseBlob(blob);			// deletes the blob pointed to by blob

		    cout  << "released blob *************"  << endl;

			CvBlobs::iterator tmp=it;		// save iterator
			++it;
			blobs.erase(tmp);				// erases the map element
		}
			else
			++it;
	  }
	// i inc it in both arms, could do it here
}
*/



/*-------------------------------------------
  prints out changes to the trackbar setting as they occur

-------------------------------------------*/
/*
void tbarFunction(int posn) {
    printf("trackbar function - pos %d \n", posn);
}
void buttonFunction(int state, void * a) {
    printf("button function - state %d \n", state);
}*/



/*
void printKalmanArrays (cv::Mat & measurementNoiseCov, cv::Mat & errorCovPost, cv::Mat & errorCovPre, cv::Mat & processNoiseCov)  {
//	printf("\nmeasurement Noise Covariance\n%f %f \n%f %f \n",  measurementNoiseCov.at<float>(0,0), measurementNoiseCov.at<float>(0,1),
//			measurementNoiseCov.at<float>(1,0), measurementNoiseCov.at<float>(1,1));
    printf("\nerror Cov Pre:\n");
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++)
            printf("%f ", errorCovPre.at<float>(i, j));
        printf("\n");
    }
    printf("error Cov Post:\n");
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++)
            printf("%f ", errorCovPost.at<float>(i, j));
        printf("\n");
    }
//	printf("\nprocess Noise Covariance:\n");
//	for (int i=0; i<6; i++) {
//		for (int j=0; j<6; j++)
////			printf("%f ", processNoiseCov.at<float>(i, j));
//		printf("\n");
//	}

}*/

