/*
 *
 * This application has two external comms methods:
 * a) UDP stream to the QT5 Scanner Console, which forwards the stream to the Scanner Server
 *      10.1.1.31 on Port 9988.    NOTE THIS IS (CURRENTLY) THE ADDRESS OF THIS PC
 * b) TCP stream directly to the Scanner Server:
 *      10.1.1.99 on port 1000
 *
 */

#include "../mwars.hh"
#include "../comms.hh"

#include<arpa/inet.h>
#include <netdb.h>      // getaddrinfo
//#include <fcntl.h>


// --- TCP
int mcuScannersockfd;
fd_set sset;                 // for select()
struct timeval timeout;      // for select()
struct clientMssg clientMssg;
struct serverMssg serverMssg;

// --- UDP
int sockfd;
struct sockaddr_in servaddr;
socklen_t slen = sizeof(servaddr);
struct opencvMssg mssg;     // message structure

// -------------------------------------------------------------------------------
// -------------- TCP COMMS WITH SCANNER SERVER (SS) -----------------------------
//
// TCP comms with the mcu in the laser Scanner Server
// -------------------------------------------------------------------------------

//=======================================================
// void sndMssgToScanServer(int x, int y, unt info)
//
// sends TCP message to the mcuScanner server
//
// blocking version; waits until response received before sending
// avoids need for delays in sending
//
//  errno = 32 is EPIPE or "broken Pipe"
// EPIPE: This socket was connected but the connection is now broken.
// In this case, send generates a SIGPIPE signal first; if that signal is
//ignored or blocked, or if its handler returns, then send fails with EPIPE.
//
// detect signal SIGPIPE and re-connect
//

// We expect write failures to occur but we want to handle them where
// the error occurs rather than in a SIGPIPE handler.
// signal(SIGPIPE, SIG_IGN);

// ======================================================
void sndMssgToScanServer(int x, int y, int command) {

	ssize_t numBytesSent, numBytesRecv;

	//printf("Sent: %d %d %d \n ", x, y, command);

	// reverse direction; scanner direction is backwards.
	y = SCANNER_DAC_LIMIT - y;
	x = SCANNER_DAC_LIMIT - x;

	clientMssg.scanX = htons((uint16_t)x);
	clientMssg.scanY = htons((uint16_t)y);
	clientMssg.info =  htons((uint16_t)command);

	if((numBytesSent = send(mcuScannersockfd, &clientMssg, sizeof clientMssg, 0)) != sizeof(clientMssg)) 	{
		perror("error sending mssg to mcuScanner");
		cout << "errno: " << errno << endl;
		//cout << "numbyte: " << numBytesSent << endl;
		if (numBytesSent == -1 || errno == EPIPE) {     // broken pipe, try and reconnect
			close(mcuScannersockfd);
			connectToScanServer();
			return;
		}
	}

	//cout << "waiting to read...." << endl;
	//blocks waiting for mcuServer response, when not blocking, ie the next lines missing
	// it all just stopped and had to use delays
	//numBytesRecv = recv(mcuScannersockfd, &serverMssg, sizeof serverMssg, 0);



//	cout << "Rcvd: " << SCANNER_DAC_LIMIT-ntohs(serverMssg.scanX)
//	     << " " << SCANNER_DAC_LIMIT-ntohs(serverMssg.scanY)
//	     << " " << ntohs(serverMssg.info) <<endl;

//	if (numBytesRecv == -1)   // == -1, then nothing received
//		perror("No response msuScanner");


	// select checks pipe before we do a read. It returns:
	//      0 if timeout, 1 if input available, -1 if error
	int numBytesAvail;
	numBytesAvail = select (FD_SETSIZE, &sset, nullptr, nullptr, &timeout);

	//  no bytes avail, pipe broken, reconnect
	if (numBytesAvail == 0) {
		cout << "nBytes = 0" << endl;
		cout << "Pipe broke, restarting..." << endl;
		close(mcuScannersockfd);
		connectToScanServer();

		// init scanner, and point down boreline
		Point2i boreLine = Point(DAC_X_MAX/2, DAC_Y_MAX/2 );
		sndMssgToScanServer(boreLine.x, boreLine.y, CMD_LASER_OFF);        // the mcuServer ignores x,y except for slew, but
		sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SCANXPWR_ON);      // seeing 65,000 from 0,0in messages was causing confusion
		sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SCANYPWR_ON);
		sndMssgToScanServer(boreLine.x, boreLine.y, CMD_SLEW);
		return;
	} else if (numBytesAvail == -1) {
		cout << "nBytes = 0 i.e. read error " << endl;
	    }
	// falls through if nBytes = 1, i.e. input available

	//cout << "nBytes = 1" << endl;

	numBytesRecv = read(mcuScannersockfd, &serverMssg, sizeof serverMssg);
	//cout << "numBytesRecv: " << numBytesRecv << endl;

	return;
}


//=======================================================================
//  int connectToScanServer()
//
//  creates TCP socket to the mcu scanner server for mcuScannersockfd
//
//  Note:  sets pipe non-blocking to allow pipe broken recovery in sndMssgToScanServer
//
//========================================================================
int connectToScanServer() {
	int status;
	struct addrinfo hints, *servinfo;       // address info structure
	char ip4[INET_ADDRSTRLEN];              // space to hold the IPv4 string

	// set hints in structure addrinfo hints
	memset(&hints, 0, sizeof hints);        // clear structure
	hints.ai_family = AF_INET;              // force IPv4
	hints.ai_socktype = SOCK_STREAM;        // TCP stream socket

	//  Note: neither getaddrinfo() or socket() care if the server is connected
	// set up structure addrinfo servinfo
	if ((status = getaddrinfo(SCANNERADDRESS, SCANNERPORT, &hints, &servinfo)) != 0) {
		printf("getaddrinfo: %s\n", gai_strerror(status));
		return EXIT_FAILURE;
	}

	while (true) {    // if connection times out, keeps trying
		printf("client: connecting to %s, if hung check firewall & server.....\n", SCANNERADDRESS);
		if ((mcuScannersockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
			perror("client: socket error");
			continue;
		}

		// hangs here if server not available, times out with "Connection timed out"
		if (connect(mcuScannersockfd, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
			close(mcuScannersockfd);
			perror("client: connect error");
			continue;
		}
		break;          // breaks on connection
	}

	// in sndMssgToScanServer(), we are going to use select()to confirm when data can be read,
	// with a timeout to detect broken pipe calling this app to re-connect
	// requires read() to be non-blocking to allow pipe broken recovery in sndMssgToScanServer
	fcntl(mcuScannersockfd, F_SETFL, O_NONBLOCK);

	// Initialize the file descriptor set.
	FD_ZERO (&sset);
	FD_SET (mcuScannersockfd, &sset);

	// Initialize select()s timeout data structure.
	timeout.tv_sec = 0;
	timeout.tv_usec = 200000;

	// get and report the internet of the other end
	auto *ipv4 = (struct sockaddr_in *)servinfo->ai_addr;
	inet_ntop(AF_INET, &(ipv4->sin_addr), ip4, INET_ADDRSTRLEN);
	printf("client: connected to scanner server @ %s\n", ip4);

	freeaddrinfo(servinfo);             // done with the linked list
	return EXIT_SUCCESS;                           // OK
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------





// -----------------------------------------------------------------------------
// -------  UDP COMMS WITH QT5 SCANNER CONSOLE (SC) ----------------------------
//
//  UDP to the QT5 Scanner Console which does processing and passes it on
// -----------------------------------------------------------------------------

//=================================================
//	int createSocketScannerConsole(void )
//
//  creates UDP socket to the QT5 Scanner Console (SC)
//=================================================
int createSocketScannerConsole()   {

	// create socket
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		cerr << "socket error: " << strerror(errno) << endl;
		return EXIT_FAILURE ;
	}

    memset((char *) &servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(SERV_PORT);

    //if (inet_aton(SERV_ADDR , &servaddr.sin_addr) == 0)      {
    if ((inet_pton(AF_INET, SERV_ADDR , &servaddr.sin_addr) == 0))      {
			cerr << "inet_aton() failed: " << strerror(errno) << endl;
			return EXIT_FAILURE;
    }
    cout << "UDP socket to scanner server created" << endl;
    return EXIT_SUCCESS;
}

//=============================================
//
// void sndMssgToScanConsole(int x, int y, bool laserOn)
//
//  sends UDP message to QT5 Scanner Console (SC)
//=============================================
void sndMssgToScanConsole(int x, int y, bool laserOn)  {

	//printf("sndMssgToScanConsole() %d %d l:%d \n", x, y, laserOn);

	mssg.XAngle = htons( (uint16_t)x);
	mssg.YAngle = htons( (uint16_t)y);
	mssg.laserOn = laserOn;

	//printf(" l:%d \n", mssg.laserOn);
	//printf("sndMssgToScanConsole()  %d %d l:%d \n", mssg.XAngle, mssg.YAngle, mssg.laserOn);
	// use sendto() for UDP because unconnected and need specify destination each transmission
	if (sendto (sockfd,  & mssg, sizeof(struct opencvMssg) , 0 , (struct sockaddr *) &servaddr, slen)==-1)      {
		cerr << "sndMssgToScanConsole failed: " << strerror(errno) << endl;
		return ;
	}
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------



