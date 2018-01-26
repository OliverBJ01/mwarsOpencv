/*
 * helpers.cpp
 *
 *  Created on: 30 Dec 2015
 *      Author: bernard
 */


#include "../mwars.hh"
#include "../comms.hh"

//extern bool gshowCalRegion;	// set by button callback onButton() to display cal region
extern bool gLaserEnable;
extern bool gdisableEngagement;
extern uint xx, yy;			    //co-ords to send to scanner

// associated with void getFrameThread()
extern VideoCapture cap;			    // opencv video capture class
extern volatile int produceCount, consumeCount;
extern int frameBufferSize;


//--------------------------------------------------------------------------
//         void getFrameThread()
//
// consumer/producer code for 1xprodocer & 1xconsumer
// refer https://en.wikipedia.org/wiki/Producer%E2%80%93consumer_problem
//--------------------------------------------------------------------------
void *getFrameThread(void *arg) {

	Mat frame = *((Mat *) arg);

	while (1) {
		while (produceCount - consumeCount == frameBufferSize) {
			sched_yield();
		}
		// following allows use of buffer > 1
		// sharedBuffer[produceCount % BUFFER_SIZE] = produceToken();
		cap >> frame;
		++produceCount;
	}
}




//-----------------------------------------------------------------------------------------------
//  void disableEngagement(int state, void*)
//
//   button bar callback that sets flag to disable engagement
// -----------------------------------------------------------------------------------------------
void disableEngagement(int state, void*) {
	if (state)
		gdisableEngagement = true;
	else
		gdisableEngagement = false;
	cout << "click " << state  << " disableEngagement " << gdisableEngagement << endl;
}


//-----------------------------------------------------------------------------------------------
//  void fireLaserThread(void *arg)
//
//  1. initLaserEnableTimer() raises a signal @ 3Hz rate which sets flag gLaserEnable
//  2. when an engageable track is detected, flag gEngage is set
//  3. when gLaserEnable&&gEngage true, fireLaserThread() is forked
//  4. fireLaserThread() turns off gEngage, turns on laser, waits, turns off laser
//  5. starts over at step 1.
// -----------------------------------------------------------------------------------------------
void *fireLaserThread(void *arg)  {

	char *s = (char *)arg;
	cout << s <<endl;
	 //  printf("pthread:  %d %d  \n", xx,  yy);
	 gLaserEnable = false;		// turned off until raised again by initLaserEnableTimer()	 	 	 	 	 	 	 	 	 	 	 	//

#ifdef USESCANNERSERVER
	sndMssgToScanServer((int)xx, (int)yy, CMD_LASER_ON);
	//usleep (500000);
	usleep (100000);   // changed to see if affetcs frame rate
	sndMssgToScanServer((int)xx, (int)yy, CMD_LASER_OFF);
#else
	sndMssgToScanConsole((int)xx, (int)yy, true);
	usleep (500000);			// in uS    i.e. 333mS
	sndMssgToScanConsole((int)xx, (int)yy, false);	// because xx, yy are global, this  turns
#endif								// off laser at current scanner position
	 pthread_exit (nullptr);
}



//-----------------------------------------------------------------------------------------------
//  static void signalrmHandler(int sig)
//
//   invoked by signal SIGALRM raised by timer in initLaserEnableTimer()
// -----------------------------------------------------------------------------------------------
static void signalrmHandler(int sig) {
	// cout << "signal received "  << endl;
	gLaserEnable = true;
}

// -------------------------------------------------------------------------------------------------------------------
//  int initLaserEnableTimer(void)
//
//  runs at low frequency; raises signal SIGALRM which invokes signal handler signalrmHandler()
//  which sets flag gLaserEnable
//-------------------------------------------------------------------------------------------------------------------
int initLaserEnableTimer()  {

	struct itimerval itv;			// interval timer structure
	struct sigaction sa;			// signal structure

	sigemptyset (&sa.sa_mask);
	sa.sa_flags = 0;
	sa.sa_handler = signalrmHandler;    //handler is: signalrmHandler(int sig)

	// SIGALARM generated on expiry of real-time timer
	if (sigaction (SIGALRM, &sa, nullptr) == -1) {
			cerr << "unable to create signal handler: " << strerror(errno) << endl;
			return EXIT_FAILURE;
	}

	itv.it_interval.tv_sec = 0;					// interval period
	//	itv.it_interval.tv_usec = 33000;	// 33mS   30Hz
	//	itv.it_interval.tv_usec = 66000;	// 66mS   15Hz
    //	itv.it_interval.tv_usec = 20000;	// 20mS   50Hz
	//itv.it_interval.tv_usec = 50000;		// 50mS   20Hz
	//itv.it_interval.tv_usec = 500000;	// 500mS   2Hz
	itv.it_interval.tv_usec = 333000;		// 333 mS   3Hz
	//itv.it_interval.tv_usec = 200000;		// 200mS  5Hz

	itv.it_value.tv_sec = 1;						// initial timeout period
	itv.it_value.tv_usec = 0;

	if (setitimer (ITIMER_REAL, &itv, nullptr) == -1) {
		cerr << "unable to create timer: " << strerror(errno) << endl;
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}



//=============================================
// int cameraCapture( VideoCapture &cap, string camType, int usbDeviceNo, string ipCamStreamAddress)
//
//  establishes camera capture from USB or IP cameras
// 	camType: "USB" or "IP"
// 	for USB cam: usbDeviceNo: generally 0, or 1 with 2 cameras
// 	for IP cam: 	ipCamStreamAddress: stream address
//		e.g.    "http://root:Isabel00@10.1.1.69/mjpg/video.mjpg"	//Axis IP Camera @ Port 80
//										root  PW: usual
//				"http://admin:Isabel00@10.1.1.5/video.cgi?.mjpg"  	// D-Link IP camera @ Port 80
//=============================================
int cameraCapture( VideoCapture &cap, string &camType, int &usbDeviceNo, string &ipCamStreamAddress,
			int &frameWidth, int &frameHeight)   {

	int frameRate;

	// IP camera capture
	if (camType == "IP") {
		cap.open(ipCamStreamAddress);
		if (!cap.isOpened())       {
			cerr << "Cannnot initialise IP camera:" << strerror(errno) << endl;
			return -1;
		}
	}
	// USB camera capture
    else  if (camType == "USB")  {
		cap.open(usbDeviceNo);
		if(!cap.isOpened()) {
			cerr << "Cannnot initialise USB camera:" << usbDeviceNo << " " << strerror(errno) << endl;
			return 1;
		}
    }
	// bad camera type
    else  {
        cerr << "bad camera parameter; " << camType << "use USB or IP"  << endl;
        return 1;
    }

	// falls through if OK
	cout << "camera initialised" << endl;

	frameWidth =  (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
	frameHeight = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	frameRate =   (int)cap.get(CV_CAP_PROP_FPS);
	cout << "frame width: " << frameWidth << " height: " << frameHeight << "  rate: " << frameRate <<endl;

	return 0;
    }


//-----------------------------------------------------------------------------------------------
//  void initScanner()
//
// init scanner, and point down boreline
// -----------------------------------------------------------------------------------------------
void initScanner() {
	Point2i boreLine = Point(DAC_X_MAX/2, DAC_Y_MAX/2 );
	sndMssgToScanServer(boreLine.x, boreLine.y, CMD_LASER_OFF);        // the mcuServe ignores x,y except for slew, but
	sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SCANXPWR_ON);      // seeing 65,000 from 0,0in messages was causing confusion
	sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SCANYPWR_ON);
	sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SLEW);

	// disbaled as they drive the scanner to do crap things
	//sndMssgToScanServer(BLX, BLY, CMD_SCANXPWR_OFF);
	//sndMssgToScanServer(BLX, BLY, CMD_SCANYPWR_OFF);
	return;
}



//=============================================
//
// int openLogFile(ofstream &logfile, string logFileName)
//
//  log file for whatever
//=============================================
int openLogFile(ofstream &logfile, const char * logFileName)  {

	logfile.open (logFileName);
	if (logfile.is_open()) {
		  cout << "Opened logfile: " << logFileName << endl;
		  return 0;
	  }
	  else {
		  cerr << "Cannnot open log file: " << logFileName  << strerror(errno) << endl;
		  return 1;
	  }
}

/*
// -----------------------------------------------
// -----------------------------------------------
// read homography_old matrix, scanner limits to file
//------------------------------------------------
// -----------------------------------------------
int readHomography(Mat &H, string filename, int &frameWidth, int &frameHeight) {

	int fw, fh;
	string calibrationDate;

	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "Could not open homography file: " << filename << endl;
		return 1;
	}

	fs["frameWidth"] >> fw;
	fs["frameHeight"] >> fh;
	if ((frameWidth != fw ) || (frameHeight != fh)) {
		cout << "framewidth or frameheight different to when file recorded" << endl;
		return 1;
	}

	fs["calibrationDate"] >> calibrationDate;
	fs["homography_H"] >> H;
	fs.release();

	cout <<"Using homography matrix read from " << filename \
	     << " recorded: " << calibrationDate << "H" << endl << H << endl;

	return 0;
}
*/

// ----------------------------------------------------------------------------
// void error(char *msg)
// ----------------------------------------------------------------------------
void error(const char *msg) {
	perror(msg);
	exit(1);
}




// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
//void helpKalman(void) {
//    printf("\n");
//}



//// --------------------------------------------------
//// IP Camera constructor
//Camera::Camera(const string &ipStreamAddress) : ipAddress(ipStreamAddress) {
//	cap.open(ipAddress);
//	if (!cap.isOpened())       {
//		cerr << "Cannnot initialise IP Camera:" << strerror(errno) << endl;
//		fault = true;
//	}
//	else
//		setFrameSize();
//}
//
//// USB cam costructor
//Camera::Camera(int usbDeviceNo) : usbDevNo(usbDeviceNo) {
//	cap.open(usbDeviceNo);
//	if (!cap.isOpened()) {
//		cerr << "Cannnot initialise USB Camera: " << usbDeviceNo << strerror(errno) << endl;
//		fault = true;
//	}
//	else
//		setFrameSize();
//}
//
//void Camera::setFrameSize() {
//	frameWidth =  (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
//	frameHeight = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
//	cout << "frame width: " << frameWidth << " frame height: " << frameHeight <<endl;
//}
//
//VideoCapture Camera::getCamInfo(int &cols, int &rows)  {
//	cols = frameWidth;
//	rows = frameHeight;
//	if (fault == false)
//		return cap;
//	else
//		return -1;
//}

