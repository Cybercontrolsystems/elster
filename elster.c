/* ELSTER Interface program */

#include <stdio.h>	// for FILE
#include <stdlib.h>	// for timeval
#include <string.h>	// for strlen etc
#include <time.h>	// for ctime
#include <sys/types.h>	// for fd_set
#include <netdb.h>	// for sockaddr_in 
#include <fcntl.h>	// for O_RDWR
#include <termios.h>	// for termios
#include <unistd.h>		// for getopt
#ifdef linux
#include <errno.h>		// for Linux
#endif
#include "common.h"		// Common Serial Framework

#define PROGNAME "Elster"
const char progname[] = "elster";
#define LOGON "meter"
#define REVISION "$Revision: 2.2 $"
/* 1.0 19/11/2007 Initial version copied from Steca 1.3
   1.3 25/11/2007 Uses -i interval flag, and 'read' command. Sends 'meter 1 49.123'
   1.4 19/03/2008 Hard code message length to prevent possible buffer overrun
   1.5 05/06/2008 Fixed problems with 'short reads' by intriducting 01.s delays
   1.6 09/10/2008 Fixed problem with RTS on TS-SER4 by removing CRTSCTS
   1.7 16/11/2008 Changed logon from elster to meter.
   1.8 14/01/2010 Changed data line from meter 1 %.3f to meter 2 %.3f %.3f for import/export (fallback using -1 as parameter)
   2.0 06/07/2010 Use Common components. Improved synchronisation at startup.
   2.1 13/08/2010 More persistent. After due-to-send, will persist until valid packet found.  And -m to
		suppress Checksum errors. (-m 100 - report as INFO every 100 errors)
   2.2 2011/03/28 Add -o / -p offset for a number to be added to reading.  Can be negative.
*/


/* WARNING - if you start this with a non-existent device like /dev/ttyUSB0 it does a core
 dump is reopenserial a progname is not valid - but saem code works for Resol
 */
static char* id="@(#)$Id: elster.c,v 2.2 2011/03/28 18:41:32 martin Exp $";

#define PORTNO 10010
#define LOGFILE "/tmp/%s%d.log"
#define SERIALNAME "/dev/ttyAM1"	/* although it MUST be supplied on command line */
#define BAUD B2400

// Severity levels.  ERROR and FATAL terminate program
#define INFO	0
#define	WARN	1
#define	ERROR	2
#define	FATAL	3
// Socket retry params
#define NUMRETRIES 3
int numretries = NUMRETRIES;
#define RETRYDELAY	1000000	/* microseconds */
int retrydelay = RETRYDELAY;
// Elster values expected
// #define NUMPARAMS 15
// If defined, send fixed data instead of timeout message
// #define DEBUGCOMMS

/* SOCKET CLIENT */

#ifndef linux
extern
#endif
int errno;  

// Procedures in this file
int processSocket(void);			// process server message
void usage(void);					// standard usage message
char * getversion(void);
int getbuf(int fd, int max);	// get a buffer full of message
void printbuf(unsigned char * buf);			// decode a bufferfull
char * bcd5(unsigned char * buf);				// 5 bytes to BCD
char * bcd3(unsigned char * buf);				// 3 bytes to BCD
time_t timeMod(time_t t, int offset);
int writedata();			// send meter data
void waitforgap(int fd, int mSec);			// Wait for a pause in the data
void dumpbuf(unsigned char * buf);		// Raw dump for singleshot mode.

/* GLOBALS */
FILE * logfp = NULL;
int sockfd[1] = {0};
int debug = 0;
int noserver = 0;		// prevents socket connection when set to 1
float offset1 = 0.0;		// Value SUBTRACTED from meter reading
float offset2 = 0.0;		// Value SUBTRACTED from export reading

int controllernum = -1;	//	only used in logon message

#define debugfp stderr

// Common Serial Framework
#define BUFSIZE 128	/* should be longer than max possible line of text */
struct data {	// The serial buffer
	int count;
	unsigned char buf[BUFSIZE];
	int status;
} data;

char buffer[256];	// For messages
char * serialName = SERIALNAME;
int totalpackets = 0, badpackets = 0, badchecksum = 0, shortpacket = 0, valid = 0, starttime;
int suppressChecksum = 0;

/********/
/* MAIN */
/********/
int main(int argc, char *argv[])
// arg1: serial device file
// arg2: optional timeout in seconds, default 60
// arg3: optional 'nolog' to carry on when filesystem full
{
    int commfd;
	int nolog = 0;

	int interval = 300;
	time_t next = 0;

	int run = 1;		// set to 0 to stop main loop
	fd_set readfd; 
	int numfds;
	struct timeval timeout;
	int tmout = 90;
	int logerror = 0;
	int online = 1;		// used to prevent messages every minute in the event of disconnection
	int option; 
	int retries = 0;
	int singleshot = 0;
	// Command line arguments
	
	// optind = -1;
	opterr = 0;
	while ((option = getopt(argc, argv, "dt:slV1i:Zm:o:p:S")) != -1) {
		switch (option) {
			case 's': noserver = 1; break;
			case 'l': nolog = 1; break;
			case '?': usage(); exit(1);
			case 't': tmout = atoi(optarg); break;
			case 'd': debug++; break;
			case 'o': offset1 = strtod(optarg, NULL); break;
			case 'p': offset2 = strtod(optarg, NULL); break;
				//		case '1': single = 1; count = 1; current = 1; break;
			case 'i': interval = atoi(optarg); break;
			case 'm': suppressChecksum = atoi(optarg); break;
			case 'S': singleshot = 1;  break;
			case 'V': printf("Version %s %s\n", getversion(), id); exit(0);
			case 'Z': decode("(b+#Gjv~z`mcx-@ndd`rxbwcl9Vox=,/\x10\x17\x0e\x11\x14\x15\x11\x0b\x1a" 
							 "\x19\x1a\x13\x0cx@NEEZ\\F\\ER\\\x19YTLDWQ'a-1d()#!/#(-9' >q\"!;=?51-??r"); exit(0);
				
		}
	}
	
	DEBUG printf("Debug %d. optind %d argc %d\n", debug, optind, argc);
	
	if (optind < argc) serialName = argv[optind];		// get serial/device name: parameter 1
	optind++;
	if (optind < argc) controllernum = atoi(argv[optind]);	// get optional controller number: parameter 2
	
	sprintf(buffer, LOGFILE, progname, controllernum);
	
	if (!nolog) if ((logfp = fopen(buffer, "a")) == NULL) logerror = errno;	
	
	// There is no point in logging the failure to open the logfile
	// to the logfile, and the socket is not yet open.

	sprintf(buffer, "STARTED %s on %s as %d timeout %d %s", argv[0], serialName, controllernum, tmout, nolog ? "nolog" : "");
	logmsg(INFO, buffer);
	
	openSockets(1, 1, LOGON,  REVISION, "");

	// Open serial port
	if ((commfd = openSerial(serialName, BAUD, 0, CS8, 1)) < 0) {
		sprintf(buffer, "ERROR " PROGNAME " %d Failed to open %s: %s", controllernum, serialName, strerror(errno));
#ifdef DEBUGCOMMS
		logmsg(INFO, buffer);			// FIXME AFTER TEST
		printf("Using stdio\n");
		commfd = 0;		// use stdin
#else
		logmsg(FATAL, buffer);
#endif
	}

	// If we failed to open the logfile and were NOT called with nolog, warn server
	// Obviously don't use logmsg!
	if (logfp == NULL && nolog == 0) {
		sprintf(buffer, "event WARN " PROGNAME " %d could not open logfile %s: %s", controllernum, LOGFILE, strerror(logerror));
		sockSend(sockfd[0], buffer);
	}
		
	numfds = (sockfd[0] > commfd ? sockfd[0] : commfd) + 1;		// nfds parameter to select. One more than highest descriptor

	// Main Loop
	FD_ZERO(&readfd); 
	starttime = time(NULL);
	while(run) {
		waitforgap(commfd, 100);	// Wait for a gap of 100mSec
		timeout.tv_sec = tmout;
		timeout.tv_usec = 0;
		FD_SET(sockfd[0], &readfd);
		FD_SET(commfd, &readfd);
		if (select(numfds, &readfd, NULL, NULL, &timeout) == 0) {	// select timed out. Bad news 
#ifdef DEBUGCOMMS
			processLine("63;59;59;1976;53000;30;100;31;32;33;34;1;371;73;12345;");
#else
			if (online == 1) {
				logmsg(WARN, "WARN " PROGNAME " No data for last period");
				online = 0;	// Don't send a message every minute from now on
			}
#endif
			continue;
		}
		if (FD_ISSET(commfd, &readfd)) { 
			int num;
			data.count = 0;
			num = getbuf(commfd, 4);
			online = 1;	// Back online
			DEBUG2 fprintf(debugfp, "Got packet %d long 0x%x 0x%x 0x%x 0x%x\n", num,
						   data.buf[0], data.buf[1], data.buf[2], data.buf[3]);
			totalpackets++;
			if (data.buf[2] != 0x68) {
				DEBUG fprintf(stderr, "Count is %d (0x%x) not 104\n", data.buf[2], data.buf[2]);
				continue;
			}
			num = getbuf(commfd, 106);
			DEBUG fprintf(stderr,"Got packet %d ", data.count);
			if (num < 110) {
				DEBUG fprintf(stderr, "Discarding short packet %d\n", num);
				shortpacket++;
				continue;
				
			}
			
			if (singleshot) {
				dumpbuf(data.buf);
				run = 0;
			}
			if (num < 0) {
				DEBUG fprintf(stderr, "Error reading %s : %s\n", serialName, strerror(errno));
				run = 0;
				break;
			} else {
				if (time(NULL) > next) {
					DEBUG2 fprintf(stderr, "Calling writedata. Next = %zu\n", next);
					if (writedata()) {
						DEBUG if (retries) fprintf(stderr, "%d retries\n", retries);
						retries = 0;
						next = timeMod(interval, 0);
					} else {
						retries++;
						DEBUG2 fprintf(stderr,"Rejected packet ");
					}
					DEBUG2 fprintf(stderr, "Back from writedata\n");
				}
			}
		}
		if ((noserver == 0) && FD_ISSET(sockfd[0], &readfd))
			run = processSocket();	// the server may request a shutdown by setting run to 0
			if (run == 2) {
				printbuf(data.buf);	// this doesn't stop the loop
				run = 1;
			}
	}
	logmsg(INFO,"INFO " PROGNAME " Shutdown requested");
	close(sockfd[0]);
	closeSerial(commfd);

	return 0;
}

/*********/
/* USAGE */
/*********/
void usage(void) {
	printf("Usage: elster [-t timeout] [-l] [-s] [-d] [-V] [-o importoffset] [-p exportoffset] /dev/ttyname controllernum\n");
	printf("-l: no log  -s: no server  -d: debug on\n -V: version -1: single shot -i: interval in seconds\n");
	return;
}

/*****************/
/* PROCESSSOCKET */
/*****************/
int processSocket(void){
// Deal with commands from MCP.  Return to 0 to do a shutdown
	short int msglen, numread;
	char buffer2[192];	// about 128 is good but rather excessive since longest message is 'truncate'
	char * cp = &buffer[0];
	int retries = NUMRETRIES;
		
	if (read(sockfd[0], &msglen, 2) != 2) {
		logmsg(WARN, "WARN " PROGNAME " Failed to read length from socket");
		return 1;
	}
	msglen =  ntohs(msglen);
	while ((numread = read(sockfd[0], cp, msglen)) < msglen) {
		cp += numread;
		msglen -= numread;
		if (--retries == 0) {
			logmsg(WARN, "WARN " PROGNAME " Timed out reading from server");
			return 1;
		}
		usleep(RETRYDELAY);
	}
	cp[numread] = '\0';	// terminate the buffer 
	
	if (strcmp(buffer, "exit") == 0)
		return 0;	// Terminate program
	if (strcmp(buffer, "Ok") == 0)
		return 1;	// Just acknowledgement
	if (strcmp(buffer, "truncate") == 0) {
		if (logfp) {
		// ftruncate(logfp, 0L);
		// lseek(logfp, 0L, SEEK_SET);
			freopen(NULL, "w", logfp);
			logmsg(INFO, "INFO " PROGNAME " Truncated log file");
		} else
			logmsg(INFO, "INFO " PROGNAME " Log file not truncated as it is not open");
		return 1;
	}
	if (strcmp(buffer, "debug 0") == 0) {	// turn off debug
		debug = 0;
		return 1;
	}
	if (strcmp(buffer, "debug 1") == 0) {	// enable debugging
		debug = 1;
		return 1;
	}
	if (strcmp(buffer, "help") == 0) {
		strcpy(buffer2, "INFO " PROGNAME" Commands are: debug 0|1, exit, truncate, read, stats, reset");
		logmsg(INFO, buffer2);
		return 1;
	}
	if (strcmp(buffer, "read") == 0) {
		return 2;	// to signal s full read
	}
	if (strcmp(buffer, "stats") == 0) {
		sprintf(buffer, "INFO " PROGNAME " Stats Total: %d Valid %d Short: %d Bad Checksum %d Seconds: %ld", 
				totalpackets, valid, shortpacket, badchecksum, time(NULL) - starttime);
		logmsg(INFO, buffer);
		return 1;
	}
	if (strcmp(buffer, "reset") == 0) {
		totalpackets = shortpacket = valid = badchecksum = 0;
		starttime = time(NULL);
		return 1;
	}
	strcpy(buffer2, "INFO " PROGNAME " Unknown message from server: ");
	strcat(buffer2, buffer);
	logmsg(INFO, buffer2);	// Risk of loop: sending unknown message straight back to server
	
	return 1;	
};

/**************/
/* GETVERSION */
/**************/
char *getversion(void) {
// return pointer to version part of REVISION macro
	static char version[10] = "";	// Room for xxxx.yyyy
	if (!strlen(version)) {
		strcpy(version, REVISION+11);
		version[strlen(version)-2] = '\0';
	}
return version;
}

/**************/
/* WAITFORGAP */
/**************/
void waitforgap(int fd, int mSec) {
	// Wait until there is at least 100mSec without data
	struct timeval timeout;
	fd_set readfd;
	char ch;
	FD_ZERO(&readfd);
	int num = 0;
	while(1) {
		FD_SET(fd, &readfd);
		timeout.tv_sec = mSec / 1000;
		timeout.tv_usec = (mSec * 1000) % 1000000;
		if (select(fd + 1, &readfd, NULL, NULL, &timeout) == 0) {
			DEBUG2 fprintf(stderr, "Waitforgap discarded %d chars\n", num);
			return;
		}
		num += read(fd, &ch, 1);
	}
}		
	
#define bcd2dec(x) (((x >> 4) * 10) + (x & 0x0f))

/*************/
/* BCD2FLOAT */
/*************/
float bcd2float(unsigned char * data, int n) {
// convert consecutive 5 bytes of BCD into decimal with 3 decimal places
	int i, sum = 0;
	for (i = 0; i < n; i++)  {
		sum = sum * 100 + bcd2dec(data[i]);
	}
	return sum / 1000.0;
}

/*************/
/* WRITEDATA */
/*************/
int writedata() {
// write a data record of two values
// Return 1 for a good packet, 0 for invalid
	char msg[60];
	int i, sum, m1, m2, m3;
/* Bytes 46 - 48: Meter definition
   49 - 53: Reg 1
	54 - 58: Reg 2
	59 - 63: Reg 3
*/
	m1 = data.buf[46];
	m2 = data.buf[47];
	m3 = data.buf[48];
	if (data.count != 110) {
		sprintf(buffer, "WARN " PROGNAME " Datacount %d != 110", data.count);
		logmsg(WARN, buffer);
		return 0;
	}
	DEBUG fprintf(stderr, "Meter definitions bytes %d %d %d ",m1, m2, m3);
	sum = 0;
	for (i = 0; i < 109; i++) sum += data.buf[i];
	if ((sum & 0xff) != data.buf[109]) { 
		DEBUG fprintf(stderr, "Checksum failure ");
		badchecksum++;
		if (suppressChecksum && (badchecksum % suppressChecksum) == 0) {
			sprintf(buffer, "INFO " PROGNAME " %d Checksum failures", badchecksum);
			logmsg(INFO, buffer);
		}
		else
			if (!suppressChecksum) logmsg(WARN, "WARN " PROGNAME " Checksum failure");
		return 0;
	}
	if (m2 == 2)
		sprintf(msg, "meter 2 %.3f %.3f", bcd2float(data.buf + 49, 5) + offset1, 
				bcd2float(data.buf + 54, 5) - offset2);
	else
		sprintf(msg, "meter 2 %.3f %.3f", bcd2float(data.buf + 49, 5) + offset1, 
				bcd2float(data.buf + 59, 5) - offset2);
	
	DEBUG fprintf(stderr, "Writedata sending '%s'\n", msg);
	sockSend(sockfd[0], msg);
	valid++;
	return 1;	// Valid packet
}

/************/
/* PRINTBUF */
/************/	
void printbuf(unsigned char * buf) {
// Write it out as three consecutive info messages with unique headers
// INFO Elster config
// INFO Elster registers
// INFO Elster status

	unsigned char msg[60];
	int i, sum;
	strncpy(msg, buf+4, 12);  msg[12] = 0;
 	sprintf(buffer, "INFO " PROGNAME " config %s ", msg);
	strncpy(msg, buf+16, 9);	msg[9] = 0;
	strcat(buffer, msg);
	sprintf(msg, " MFG:%d Cfg:%d ", (buf[25] << 16) + (buf[26] << 8) + buf[27], (buf[28] << 8) + buf[29]);
	strcat(buffer, msg);
	strncpy(msg, buf+30, 16); msg[16] = 0;
	strcat(buffer, "S: ");
	strcat(buffer, msg);
	sprintf(msg, " T: %02x%02x%02x", buf[46], buf[47], buf[48]);
	strcat(buffer, msg);
	logmsg(INFO, buffer);

	sprintf(buffer, "INFO " PROGNAME " registers Rate 1 ");
	strcat(buffer, bcd5(buf+49)); // Reg 1 - forward energy
	strcat(buffer, bcd5(buf+54));	// Reg 2 - reverse energy
	strcat(buffer, bcd5(buf+59));
	strcat(buffer, "Rate 2 ");
	strcat(buffer, bcd5(buf+64));
	strcat(buffer, bcd5(buf+69));
	strcat(buffer, bcd5(buf+74));
	logmsg(INFO, buffer);
	
	sprintf(buffer, "INFO " PROGNAME " status PF: %d St: 0x%02x Err: 0x%02x", buf[79], buf[80], buf[81]);
	strcat(buffer, " Hours: Anticreep ");
	strcat(buffer, bcd3(buf+82));
	strcat(buffer, "Powerup ");	strcat(buffer, bcd3(buf+85));
	strcat(buffer, "R1 ");	strcat(buffer, bcd3(buf+88));
	strcat(buffer, "R2 ");		strcat(buffer, bcd3(buf+91));
	sprintf(msg, "Counts: Powerfail %d ", (buf[95] << 8) + buf[94]);	strcat(buffer, msg);
	sprintf(msg, "Watchdog %d ", buf[96]);							strcat(buffer, msg);
	sprintf(msg, "Reverse %d", buf[97]);							strcat(buffer, msg);
	logmsg(INFO, buffer);
	sum = 0;
	for (i = 0; i < 109; i++) sum += buf[i];
	if ((sum & 0xff) != buf[109]) logmsg(WARN, "WARN " PROGNAME " Checksum failure");
}

char * bcd5(unsigned char * buf) {
	static char data1[12];
	sprintf(data1, "%02x%02x%02x%01x.%01x%02x ", buf[0], buf[1], buf[2], buf[3] >> 4, buf[3] & 0xF, buf[4]);
	return data1;
}

char * bcd3(unsigned char * buf) {
	static char data2 [8];
	sprintf(data2, "%02x%02x%02x ", buf[0], buf[1], buf[2]);
	return data2;
}

time_t timeMod(time_t t, int offset) {
// Return a time in the future at modulus t;
// ie, if t = 3600 (1 hour) the time returned
// will be the next time on the hour.
// The offset may be positive or negative.
//	time_t now = time(NULL);
	if (t == 0) t = 600;
	DEBUG2 fprintf(stderr,"TimeMod now = %zu delta = %zu result = %zu\n", time(NULL), t, (time(NULL) / t) * t +t);
	return (time(NULL) / t) * t + t + offset;
}

/**********/
/* GETBUF */
/**********/
int getbuf(int fd, int max) {
	// Read up to max chars into supplied buf. Return number
	// of chars read or negative error code if applicable
	
	int ready, numtoread, now;
	fd_set readfd; 
	struct timeval timeout;
	FD_ZERO(&readfd);
	// numread = 0;
	numtoread = max;
	DEBUG2 fprintf(stderr, "Getbuf %d count=%d ", max ,data.count);
	
	while(1) {
		FD_SET(fd, &readfd);
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;	 // 0.5sec
		ready = select(fd + 1, &readfd, NULL, NULL, &timeout);
		DEBUG4 {
			gettimeofday(&timeout, NULL);
			fprintf(stderr, "%03ld.%03d ", timeout.tv_sec%100, timeout.tv_usec / 1000);
		}
		if (ready == 0) 
			return data.count;		// timed out - return what we've got
		DEBUG4 fprintf(stderr, "Getbuf: before read1 ");
		now = read(fd, data.buf + data.count, 1);
		DEBUG4 fprintf(stderr, "After read1\n");
		DEBUG3 fprintf(stderr, "0x%02x ", data.buf[data.count]);
		if (now < 0)
			return now;
		if (now == 0) {
			fprintf(stderr, "ERROR fd was ready but got no data\n");
			fd = reopenSerial(fd, serialName, BAUD, 0, CS8, 1);
			//			usleep(1000000); // 1 Sec
			continue;
		}
		
		data.count += now;
		numtoread -= now;
		if (numtoread == 0) return data.count;
		if (numtoread < 0) {	// CANT HAPPEN
			fprintf(stderr, "ERROR buffer overflow - increase max from %d (numtoread = %d numread = %d)\n", 
					max, numtoread, data.count);
			return data.count;
			
		}
	}
}

void dumpbuf(unsigned char * buf) {
	// Just dump it out:
	int i;
	fprintf(stderr,"Buffer dump: ");
	for (i = 0; i < 110; i++) {
		fprintf(stderr, "%02x ", buf[i]);
	}
	fprintf(stderr, "\n");
}

