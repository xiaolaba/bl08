/*
File: bl08.c

Version: 1.0.0.1

Copyright (c) 2004,2008	Kustaa Nyholm
Copyright (c) 2008 	Robert Larice (SWI return to MON, QY2 chip)
Copyright (c) 2010 	Tormod Volden
Copyright (c) 2013 	Jaromir Sukuba (added A and B ROM routines entry instances, 
			dozen new supported chips, reset polarity)
			http://jaromir.xf.cz/

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public
License as published by the Free Software Foundation; either
version 2.0 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/
// This code compiles cleanly with i686-apple-darwin8-gcc-4.0.1 with command:
// gcc -Wall -std=c99 -pedantic -o bl08 bl08.c
// and uses only POSIX standard headers so this should be pretty 
// easy to port anywhere: Mac OS X,  Linux , MinGW, Cygwin
//
// Having said that I'm pretty sure there is implicit assumptions that
// int is 32 bits, char is signed 8 bits
// I also expect that extending this code to handle S-records in the +2G range
// will uncover more implicit assumptions.
//
// cheers Kusti

/*
 * 2018-NOV-04, xiaolaba, fork
 * add baud rate support 11520, 6MHZ RC or XTAL, and baud rate 9600 x 1.2, 20% offset
 * set baud rate default to 11520
*/

#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>


#if defined(__linux__)

# include <sys/wait.h>
# include <sys/ioctl.h> 

#else

// Terrible hack here because POSIX says 'ioctl()' is in <stropts.h> but
// for example Mac OS X Tiger does not have this header, OTOH I think
// that <sys/ioctl.h> is not POSIX either so how do you write
// actual POSIX compliant code that compiles cleanly on POSIX...
extern int ioctl (int filedes, int command, ...);
// End of hack

#endif

	

 
#define PIDFILENAME "bl08PIDfile.temp"

//char* COM = "/dev/tty.usbserial-FTOXM3NX";
char* COM = "/dev/ttyS0";

typedef struct {
char* str;
int val;
} pair;

pair portPins[]= {
{"LE",TIOCM_LE},
{"DTR",TIOCM_DTR},
{"RTS",TIOCM_RTS},
{"ST",TIOCM_ST},
{"SR",TIOCM_SR},
{"CTS",TIOCM_CTS},
{"CAR",TIOCM_CAR},
{"CD",TIOCM_CD},
{"RNG",TIOCM_RNG},
{"RI",TIOCM_RI},
{"DSR",TIOCM_DSR}
};

int forcePins[sizeof(portPins)/sizeof(portPins[0])] = {0};

pair baudrates[]= {
// Some common, but non POSIX baudrates here
#ifdef B14400
{"14400",B14400},
#endif
#ifdef B57600
{"57600",B57600},
#endif
#ifdef B115200
{"115200",B115200},
#endif
#ifdef B230400
{"230400",B230400},
#endif

{"0",B0},
{"50",B50},
{"75",B75},
{"110",B110},
{"134",B134},
{"150",B150},
{"200",B200},
{"300",B300},
{"600",B600},
{"1200",B1200},
{"1800",B1800},
{"2400",B2400},
{"4800",B4800},
{"9600",B9600},
{"11520",B11520},
{"19200",B19200},
{"38400",B38400},
};
	
//int baudRate=B9600;
int baudRate=B11520;

int com;

char version[] = "1.0.0.1";

unsigned char image[ 0x10000 ]; // HC908 memory image
unsigned char scode[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// EARRNG, CTRLBYT parameter
#define ERARRNG_PAGE_ERASE 0x00
#define ERARRNG_MASS_ERASE 0x40

// These will be initialized by setCPUtype()
int CPUSPEED;	// 2 x Fbus freq, e.g.  ext osc 16 MHz -> Fbus == 4 Mh => CPUSPEED==2
int RAM;
int FLASH;	// Flash start address
int PUTBYTE;	// Receive byte routine address
int GETBYTE;	// Receive byte routine address
int RDVRRNG;	// Read/verify flash routine address
int ERARRNG;	// Erase flash routine address
int PRGRNGE;	// Flash programming routine address
int FLBPR;	// Flash block proctection register address
int MONRTN;	// Monitor mode return jump address
int EADDR;	// For FLBPR in Flash the mass erase must use FLBPR as the erase address
int MONTYPE;	// 0 - type A, 1 - type B instance, difference in ROM routines calling, see AN2874
// These will be calculated
int MONDATA;	// Flashing routines parameter block address (==RAM+8)
int CTRLBYT;	// Address of flashing routine control variable (==MONDATA+0)
int CPUSPD;	// Address of flashing routine cpu speed variable (==MONDATA+1)
int LADDR;	// Address of flashing routine last address variable (==MONDATA+2)
int DATABUF;	// Flashing routines data buffer address (==MONDATA+4)
int PAGESIZE;	// Databuffer size
int WORKRAM;	// Work storage needed for calling flashing routines
int WORKTOP;	// Topmost work storage address

// HC908GZ16 Memory usage (note for some HC908 variants the ROM routines use memory starting from 0x80):
// 0x40 - 0x47 reserved for future ROM routine expansion needs
// 0x48 - 0x4B ROM routine parameters
// 0x4C - 0x6C ROM routine data buffer (64 bytes as used in this code)
// 0xAC - 0xFF Working storage for calling the ROM routines (about 17 bytes used)

int tickP1=15;
int tickP2=1023;
int verbose = 1;
int size = sizeof(image);
int useStdin=0;
int dumpStart=0;
int dumpSize=0;
char* dumpFormat="hex";
int eraseFlash=0;
int verify=0;
char* executeCode=NULL;
int pageErase=0;
int uploadOnly=0;
int terminalMode=0;
int connected=0;
int useFastProg=0;
int resetPulse=0;
int resetPolarity=0;
int killPrevious=0;
int loadOnly=0;

void comErr(char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(stderr,"%s", buf);
	perror(COM);
	va_end(va);
	abort(); 
	}

void flsprintf(FILE* f, char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(f,"%s", buf);
	fflush(f);
	va_end(va);
	}
	
void ioctlErrCheck(int e) {
	if (e) {
		flsprintf(stdout,"ioctl returned %d\n",e);
		abort();
		}
	}
	
void setHandshake() {
	int i,s;
	ioctlErrCheck(ioctl(com, TIOCMGET, &s)); 	
	for (i=0; i<sizeof(forcePins)/sizeof(forcePins[0]); i++) {
		int v=forcePins[i]-1;
		if (v==1) 
			s &= ~portPins[i].val;
		
		if (v==0) 
			s |= portPins[i].val;
		}			
	ioctlErrCheck(ioctl(com, TIOCMSET, &s)); 
	}
	
	
void initSerialPort() {
	com =  open(COM, O_RDWR | O_NOCTTY | O_NDELAY);
	if (com <0) 
		comErr("Failed to open serial port\n");
		
	fcntl(com, F_SETFL, 0);
		
	struct termios opts;
	
	tcgetattr(com, &opts);

	opts.c_lflag  &=  ~(ICANON | ECHO | ECHOE | ISIG);

	opts.c_cflag |=  (CLOCAL | CREAD);
	opts.c_cflag &=  ~PARENB;
	opts.c_cflag |=  CSTOPB; // two stop bits
	opts.c_cflag &=  ~CSIZE;
	opts.c_cflag |=  CS8;
	
	opts.c_oflag &=  ~OPOST;
	
	opts.c_iflag &=  ~INPCK;
	opts.c_iflag &=  ~(IXON | IXOFF | IXANY);	
	opts.c_cc[ VMIN ] = 0;
	opts.c_cc[ VTIME ] = 10;//0.1 sec
	

	cfsetispeed(&opts, baudRate);   
	cfsetospeed(&opts, baudRate);   
	
	setHandshake();
	
	if (tcsetattr(com, TCSANOW, &opts) != 0) {
		perror(COM); 
		abort(); 
		}
		
	tcflush(com,TCIOFLUSH); // just in case some crap is the buffers
	/*		
	if (!terminalMode) {
		char buf = -2;
		while (read(com, &buf, 1)>0) {
			if (verbose)
				flsprintf(stderr,"Unexpected data from serial port: %02X\n",buf & 0xFF);
			}
		}
*/
	}
	

	
void putByte(int byte) {
	char buf = byte;
	if (verbose>3)
		flsprintf(stdout,"TX: 0x%02X\n", byte);
	int n = write(com, &buf, 1);
	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
	}
	
int getByte() {
	char buf;
	int n = read(com, &buf, 1);
	if (verbose>3)
		flsprintf(stdout,n<1?"RX: fail\n":"RX:  0x%02X\n", buf & 0xFF);
	if (n == 1)
		return buf & 0xFF;
	
	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
	}

// This reads away break 'character' from the serial line
void flushBreak() { 
	int i;
	for (i=0; i<2; ++i) {
		char buf;
		int n = read(com, &buf, 1);
		if (verbose>3)
			flsprintf(stdout,n<1?"FL: nothing\n":"FL:  0x%02X\n", buf & 0xFF);
		}
	}
		
void sendByte(int byte) {
	byte &=  0xFF;
	putByte(byte);
	char buf;
	if (read(com, &buf, 1)!=1)
		comErr("Loopback failed, nothing was received\n");
	int rx=buf &0xFF;
	if (byte !=  rx)
		comErr("Loopback failed, sent 0x%02X, got 0x%02X\n", byte,  rx);
	rx = getByte();
	if (byte !=  rx)
		comErr("Target echo failed, sent 0x%02X, got 0x%02X\n", byte, rx);
	}
	
	
void readMemory(int addr, int n,int tick) {
	if (verbose>2) 
		flsprintf(stdout,"Read memory address %04X size %04X\n",addr,n);
	unsigned char* p = &image[ addr ];
	sendByte(0x4A); // Monitor mode READ command
	sendByte(addr >> 8);
	sendByte(addr & 0xFF);
	*(p++) = getByte();
	n--;
	int tc=0;
	while (n>0) {
		sendByte(0x1A); // Monitor mode IREAD command
		int b1 = getByte();
		int b2 = getByte();
		*(p++) = b1;
		n--;
		if (n > 0)
			*(p++) = b2;
		n--;
		if (tick) {
			tc++;
			if ((tc & tickP1)==0)
				flsprintf(stdout,".");
			//if ((tc & tickP2)==0)
			//	flsprintf(stdout,"\n");
			}
		}
	}

	/*
void writeMemory(int addr, int n, int tick) {
	if (verbose>2) 
		flsprintf(stdout,"Write memory address %04X size %04X\n",addr,n);
	unsigned char* p = &image[ addr ];
	char buf;
	int i;
	putByte(0x49); // Monitor mode WRITE command
	putByte(addr >> 8);
	putByte(addr & 0xFF);
	putByte(*(p++));
	for (i=0;i<4;i++)
		{
		read(com, &buf, 1);
		flsprintf(stdout,"%02X ",buf);
		}
	int tc=1;
	while (n>1) {
		putByte(0x19); // Monitor mode IWRITE command
		putByte(*(p++));
		n -= 1;
		if (tick) {
			tc++;
			if ((tc & tickP1)==0)
				flsprintf(stdout,".");
			//if ((tc & tickP2)==0)
			//	flsprintf(stdout,"\n");
			}
		}
	for (i=0;i<(n-1);i++)
		{
		read(com, &buf, 1);
		flsprintf(stdout,"%02X ",buf);
		}
	flsprintf(stdout,"--\n");
	}
	*/
	
void writeMemory(int addr, int n, int tick) {
	if (verbose>2) 
		flsprintf(stdout,"Write memory address %04X size %04X\n",addr,n);
	unsigned char* p = &image[ addr ];
	sendByte(0x49); // Monitor mode WRITE command
	sendByte(addr >> 8);
	sendByte(addr & 0xFF);
	sendByte(*(p++));
	int tc=1;
	while (n>1) {
		sendByte(0x19); // Monitor mode IWRITE command
		sendByte(*(p++));
		n -= 1;
		if (tick) {
			tc++;
			if ((tc & tickP1)==0)
				flsprintf(stdout,".");
			//if ((tc & tickP2)==0)
			//	flsprintf(stdout,"\n");
			}
		}
	}
	
void connectTarget() {
	int j;
	if (connected)
		return;

	flsprintf(stdout, "Security code: ");
	for (j = 0; j<8; ++j) {
		sendByte(scode[j]);
	        flsprintf(stdout, ".");
		}
	flsprintf(stdout, " ");

	flushBreak();
	readMemory(RAM, 1 , 0);
	connected=1;
	if ((image[ RAM ] & 0x40) == 0)
		flsprintf(stdout,"failed\n");
	else
		flsprintf(stdout,"passed\n");
	
	// in case FLBPR is RAM based we clear it first by just writing it
	image[FLBPR]=0xFF;
	writeMemory(FLBPR,1,0);
	}
	

void dumpMemory(int addr, int n) {
	unsigned char* p = &image[ addr ];
	int i;
	for (i = 0; i<n; ++i) {
		if ((i&0xF) == 0)
			flsprintf(stdout,"%04X  ", addr+i);
		flsprintf(stdout,"%02X ", *(p++) & 0xFF);
		if ((i&0xF) == 7)
			flsprintf(stdout," ");
		if ((i&0xF) == 15)
			flsprintf(stdout,"\n");
		}
	if ((i&0xF) != 0)
		flsprintf(stdout,"\n");
	}
	
	
void dumpMemorySrec(int addr, int size) {
	unsigned char* p = &image[ addr ];
	while (size>0) {
		int n = size>16 ? 16 : size;
		int bc=2+n+1;
		flsprintf(stdout,"S1%02X%04X",bc,addr);
		int s=(addr >> 8) + (addr & 0xFF) + bc;	
		int i;
		for (i=0; i<n; ++i) {
			int bty=*p & 0xFF;
			s += bty;
			flsprintf(stdout,"%02X",bty);
			++p;
			++addr;
			}
		size -= n;
		flsprintf(stdout,"%02X\n",~s & 0xFF);
		}
	}

int readSP() {
	if (verbose>2) 
		flsprintf(stdout,"Read Stack Pointer\n");
	sendByte(0x0C); // Monitor mode READSP command
	return  (((getByte() << 8) | (getByte() & 0xFF)) - 1) & 0xFFFF;
	}	

int runFrom(int PC, int A, int CC, int HX) {
	int SP=readSP();
	if (verbose>2) 
		flsprintf(stdout,"Execute code PC=%04X A=%02X CC=%02X H:X=%04X SP=%04X\n",PC,A,CC,HX,SP);
	image[ SP + 1 ] = HX >> 8;
	image[ SP + 2 ] = CC;
	image[ SP + 3 ] = A;
	image[ SP + 4 ] = HX & 0xFF;
	image[ SP + 5 ] = PC >> 8;
	image[ SP + 6 ] = PC & 0xFF;
	writeMemory(SP + 1 , 6 , 0);
	sendByte(0x28); // Monitor mode RUN command
	return SP;
	}	
			
int lastMon=-1;

/*
mass erase - callMonitor(ERARRNG , ERARRNG_MASS_ERASE, 0, EADDR, 0);
program - callMonitor(PRGRNGE, 0 , 0, addr , addr+n-1);
read at verify - callMonitor(RDVRRNG, 0 , 1 , addr , addr+n-1);
page erase - callMonitor(ERARRNG, ERARRNG_PAGE_ERASE, 0, a, 0);


*/


int callMonitor_A(int mon, int ctrlbyt, int accu, int faddr, int laddr) {
	int SP;

	image[ CTRLBYT ] = ctrlbyt; // CTRLBYT BIT 6  =  1  = > mass erase
	image[ CPUSPD ] = CPUSPEED; // CPUSPD  =  16 MHz ext clock  = > 4 MHz Fbus speed  = > 8
	image[ LADDR ] = laddr>>8;
	image[ LADDR+1 ] = laddr&0xFF;
	writeMemory(MONDATA, 4, 0);
		
	if (WORKRAM>0xFF) {
		flsprintf(stderr,"Work RAM must be on zero page\n");
		abort();
		}

	if (lastMon!=mon) {
		// construct small HC908 code fragment to call the monitor function and return results
		int i = WORKRAM;
		
		image[ i++ ] = 0xCD; // JSR mon ; calls the monitor routine
		image[ i++ ] = mon>>8;
		image[ i++ ] = mon&0xFF;

		image[ i++ ] = 0x83; // SWI ; return to monitor
		
		if (WORKRAM>=WORKTOP) { // leave some stack space for monitor routines
			flsprintf(stderr,"Not enough WORKRAM on target\n");
			abort();
			}
		
		writeMemory(WORKRAM , i-WORKRAM , 0);
		lastMon=mon;
		}
	
	// now execute the fragment
	runFrom(WORKRAM, accu, 0x00, faddr); 

        // SWI drops into MON, which will send a BREAK
	{ 	char buf;
		if (read(com, &buf, 1) != 1)
			comErr("ERROR: waiting for MON, nothing was received\n");
		if(buf != 0)
			comErr("ERROR: unexpected swi answer read %x\n", buf);
	}

	SP = readSP();
	readMemory(SP + 1 , 6 , 0);
	if (verbose>2) 
		flsprintf(stdout, "SP=%02x SP+1..6: hi(X)=%02x CC=%02x A=%02x lo(X)=%02x hi(PC)=%02x lo(PC)=%02x\n",
			SP,
			image[SP+1],  // hi(X)
			image[SP+2],  // CC
			image[SP+3],  // A
			image[SP+4],  // lo(X)
			image[SP+5],  // hi(PC)
			image[SP+6]); // lo(PC)
	return ((image[SP+2] & 0xff) << 8) | (image[SP+3] & 0xff); // return condition codes and accu
	}	

/*
int cca=callMonitor_B(RDVRRNG, 0 , 1 , addr , n);
callMonitor_B(PRGRNGE, 0 , 0, addr , n);
callMonitor_B(ERARRNG , 0, 0, 0xFFFF, 0);
	MONDATA = RAM+8;
	CTRLBYT  =  MONDATA+0;
	CPUSPD  =  MONDATA+1;
	LADDR  =  MONDATA+2;
	DATABUF  =  MONDATA+4;
	WORKRAM = DATABUF + PAGESIZE;
*/


int callMonitor_B(int mon, int ctrlbyt, int accu, int addr, int len) {
	int SP;

	//CTRLBYT = MONDATA

	image[ MONDATA + 0 ] = CPUSPEED; 
	image[ MONDATA + 1 ] = len; 
	image[ MONDATA + 2 ] = addr>>8;
	image[ MONDATA + 3 ] = addr&0xFF;
	writeMemory(MONDATA, 4, 0);
		
	if (WORKRAM>0xFF) {
		flsprintf(stderr,"Work RAM must be on zero page\n");
		abort();
		}

	if (lastMon!=mon) {
		// construct small HC908 code fragment to call the monitor function and return results
		int i = WORKRAM;
		image[ i++ ] = 0x45;		//LDHX MONDATA
		image[ i++ ] = MONDATA>>8;
		image[ i++ ] = MONDATA;
		image[ i++ ] = 0xCD; // JSR mon ; calls the monitor routine
		image[ i++ ] = mon>>8;
		image[ i++ ] = mon&0xFF;

		image[ i++ ] = 0x83; // SWI ; return to monitor
		
		if (WORKRAM>=WORKTOP) { // leave some stack space for monitor routines
			flsprintf(stderr,"Not enough WORKRAM on target\n");
			abort();
			}
		
		writeMemory(WORKRAM , i-WORKRAM , 0);
		lastMon=mon;
		}
	
	// now execute the fragment
	runFrom(WORKRAM, accu, 0x00, addr); 

        // SWI drops into MON, which will send a BREAK
	{ 	char buf;
		if (read(com, &buf, 1) != 1)
			comErr("ERROR: waiting for MON, nothing was received\n");
		if(buf != 0)
			comErr("ERROR: unexpected swi answer read %x\n", buf);
	}

	SP = readSP();
	readMemory(SP + 1 , 6 , 0);
	if (verbose>2) 
		flsprintf(stdout, "SP=%02x SP+1..6: hi(X)=%02x CC=%02x A=%02x lo(X)=%02x hi(PC)=%02x lo(PC)=%02x\n",
			SP,
			image[SP+1],  // hi(X)
			image[SP+2],  // CC
			image[SP+3],  // A
			image[SP+4],  // lo(X)
			image[SP+5],  // hi(PC)
			image[SP+6]); // lo(PC)
	return ((image[SP+2] & 0xff) << 8) | (image[SP+3] & 0xff); // return condition codes and accu
	}	

int fastProg(int faddr,int n) {
	static int n_addr;
	static int last_n;
	flsprintf(stdout, "-------------------------FP %04X, %04X\n",faddr,n);
	if (WORKRAM>0xFF) {
		flsprintf(stderr,"Work RAM must be on zero page\n");
		abort();
		}


	if (lastMon!=-1) {
		int SP = readSP();
		image[ CTRLBYT ] = 0; // CTRLBYT =0
		image[ CPUSPD ] = CPUSPEED; // CPUSPD  =  16 MHz ext clock  = > 4 MHz Fbus speed  = > 8

		// construct small HC908 code fragment to call the monitor function and return results
		int i = WORKRAM;
		image[ i++ ] = 0x9D; // NOP / reserve space
		image[ i++ ] = 0x9D; // NOP / reserve space
				
		image[ i++ ] = 0x35; // STHX WORKRAM (dir)
		image[ i++ ] = WORKRAM;
		
		image[ i++ ] = 0x45; // LDHX #DATABUF
		image[ i++ ] = DATABUF>>8;
		image[ i++ ] = DATABUF&0xFF;
		
		image[ i++ ] = 0xCD; // JSR GETBYTE ; calls the monitor routine
		image[ i++ ] = GETBYTE>>8;
		image[ i++ ] = GETBYTE&0xFF;
//		image[ i++ ] = 0xFE9C>>8;  ; // 0xFE9C = GET WITH echo
//		image[ i++ ] = 0xFE9C&0xFF;

		image[ i++ ] = 0xF7; // STA ,X
		
		image[ i++ ] = 0x5C; // INCX
		
		image[ i++ ] = 0xA3; // cpx #DATABUF+n
		n_addr=i;
		image[ i++ ] = 0; // place holder

		image[ i++ ] = 0x25; // BLO *-10
		image[ i++ ] = (-9)&0xFF;
		
		image[ i++ ] = 0x55; // LDHX WORKRAM (dir)
		image[ i++ ] = WORKRAM;
		
		image[ i++ ] = 0xCD; // JSR PRGRNGE ; calls the monitor routine
		image[ i++ ] = PRGRNGE>>8;
		image[ i++ ] = PRGRNGE&0xFF;
		
		image[ i++ ] = 0x45; // LDHX #SP+1 ; restore stack pointer
		image[ i++ ] = (SP+1) >> 8;
		image[ i++ ] = (SP+1) & 0xFF;
		
		image[ i++ ] = 0x94; // TXS 
		
		image[ i++ ] = 0xCC; // JMP back to MON (this is the only way)
		image[ i++ ] = MONRTN>>8;
		image[ i++ ] = MONRTN&0xFF;
		
		if (WORKRAM>=WORKTOP) { // leave some stack space for monitor routines
			flsprintf(stderr,"Not enough WORKRAM on target\n");
			abort();
			}
		
		writeMemory(WORKRAM , i-WORKRAM , 0);
		dumpMemorySrec(WORKRAM , i-WORKRAM);
		lastMon=-1;
		}
		
	if (last_n!=n) {
		image[ n_addr ] = DATABUF+n;
		writeMemory(n_addr,1,0);
		last_n=n;
		}
		
	int laddr = faddr+n-1;
	image[ LADDR ] = laddr>>8;
	image[ LADDR+1 ] = laddr&0xFF;
	writeMemory(LADDR, 2, 0); 
	
	// now execute the fragment
	runFrom(WORKRAM+2, 0x00, 0x00, faddr); // NOTE! to override flash security and erase FLBPR at be used as the erase address for mass erase
	flsprintf(stdout, "-------------------------EXE\n");
	int i;
	for (i=0; i<n; ++i) 
		putByte(image[faddr+i]);
	for (i=0; i<n; ++i) 
		getByte();
		
	readSP();
	return 0;
	}	
	
/*
	PAGESIZE = 64;
	MONDATA = RAM+8;
	CTRLBYT  =  MONDATA+0;
	CPUSPD  =  MONDATA+1;
	LADDR  =  MONDATA+2;
	DATABUF  =  MONDATA+4;
	WORKRAM = DATABUF + PAGESIZE;
	WORKTOP = 0xF0; // this leaves 16 bytes for stack, the deepest ROM routines use 11 so there some for myself too
*/
int fastProg2(int faddr,int progn) {
	if (WORKRAM>0xFF) {
		flsprintf(stderr,"Work RAM must be on zero page\n");
		abort();
		}
	int SP = readSP();

	image[ CTRLBYT ] = 0; // CTRLBYT =0
	image[ CPUSPD ] = CPUSPEED; // CPUSPD=  16 MHz ext  => 4 MHz Fbus  = > 8

	// construct small HC908 code fragment to call the monitor function 
	int i = WORKRAM;
	int j;
	
	int FADDR=i;
	image[ i++ ] = faddr>>8; // FADDR initial contents
	image[ i++ ] = faddr&0xFF; 
	
	int PROGN=i;
	image[ i++ ] = progn>>8; // PROGN initial contents
	image[ i++ ] = progn&0xFF;	
		
	int start=i;
	flsprintf(stdout,"helper start %04x\n",i);
	if (lastMon!=-1) {
		image[ i++ ] = 0x55 ; //	LDHX PROGN  get bytes left to program
		image[ i++ ] = PROGN;		
		
		image[ i++ ] = 0x27 ; //	BEQ	DONE branch if all done
		int patchDone=i;
		image[ i++ ] = 0 ;
		
		image[ i++ ] = 0x65 ; //	CPHX #PAGESIZE  only page full at a time
		image[ i++ ] = 0 ; 
		image[ i++ ] = PAGESIZE; 
		
		
		image[ i++ ] = 0x25 ; //	BLO DOIT
		image[ i++ ] = 2 ; 
		
		image[ i++ ] = 0xAE ; //	LDX #PAGESIZE
		image[ i++ ] = PAGESIZE;  
		
		image[ i++ ] = 0x9F ; //	TXA
		
		image[ i++ ] = 0x87 ; //	PSHA
		
		image[ i++ ] = 0xB6 ; //	LDA  PROGN+1
		image[ i++ ] = PROGN+1;	
		
		image[ i++ ] = 0x9E; //		SUB 1,SP	
		image[ i++ ] = 0xE0;	
		image[ i++ ] = 1;	
		
		image[ i++ ] = 0xB7 ; //	STA PROGN+1
		image[ i++ ] = PROGN+1;	
		
		image[ i++ ] = 0xB6 ; //	LDA  PROGN
		image[ i++ ] = PROGN;	
		
		image[ i++ ] = 0xA2 ; //	SBC #0
		image[ i++ ] = 0 ; 
		
		image[ i++ ] = 0xB7 ; //	STA PROGN
		image[ i++ ] = PROGN;	

		image[ i++ ] = 0x9F ; //	TXA
				
		image[ i++ ] = 0x4A ; //	DECA
		
		image[ i++ ] = 0xBB ; //	ADD FADDR+1
		image[ i++ ] = FADDR+1;	
			
		image[ i++ ] = 0xB7 ; //	STA LADDR+1
		image[ i++ ] = LADDR+1;	
			
		image[ i++ ] = 0xB6 ; //	LDA  FADDR
		image[ i++ ] = FADDR;	
		
		image[ i++ ] = 0xA9 ; //	ADC #0
		image[ i++ ] = 0 ; 
		
		image[ i++ ] = 0xB7 ; //	STA  LADDR
		image[ i++ ] = LADDR;	
		
		image[ i++ ] = 0x45; //		LDHX #DATABUF
		image[ i++ ] = DATABUF>>8;
		image[ i++ ] = DATABUF&0xFF;
		
		image[ i++ ] = 0x86 ;//		PULA		

		int getMore=i;
		
		image[ i++ ] = 0x87 ; //	PSHA
				
		image[ i++ ] = 0xCD; //		JSR GETBYTE 
		image[ i++ ] = GETBYTE>>8;
		image[ i++ ] = GETBYTE&0xFF;
		
		image[ i++ ] = 0xF7; //		STA ,X
		
		image[ i++ ] = 0x5C; //		INCX
		
		image[ i++ ] = 0x86 ; //	PULA
		
		image[ i++ ] = 0x4A ; //	DECA
		
		image[ i++ ] = 0x26 ; //	BNE GETMORE
		j=i;
		image[ i++ ] = (getMore-(j+1))&0xFF ; 
		
		image[ i++ ] = 0x55 ; //	LDHX FADDR
		image[ i++ ] = FADDR;		

		image[ i++ ] = 0xCD; //		JSR PRGRNGE 
		image[ i++ ] = PRGRNGE>>8;
		image[ i++ ] = PRGRNGE&0xFF;
		
		image[ i++ ] = 0x8B;		// PSHH Annoyingly JB8 RDVRNG leaves H:X off by one
		image[ i++ ] = 0x89;		// PSHX compared to GZ16 necessiating this pus/pul sequence
		
		
		image[ i++ ] = 0x55 ;//		LDHX FADDR
		image[ i++ ] = FADDR;		

		image[ i++ ] = 0xA6 ; //	LDAA #1
		image[ i++ ] = 1 ; 
		
		image[ i++ ] = 0xCD; //		JSR RDVRRNG 
		image[ i++ ] = RDVRRNG>>8;
		image[ i++ ] = RDVRRNG&0xFF;
		
		image[ i++ ] = 0x88;		// PULX
		image[ i++ ] = 0x8A;		// PULH

		image[ i++ ] = 0x35 ; //	STHX FADDR
		image[ i++ ] = FADDR;	
		
		image[ i++ ] = 0xCD; //		JSR PUTBYTE 
		image[ i++ ] = PUTBYTE>>8;
		image[ i++ ] = PUTBYTE&0xFF;
							
		image[ i++ ] = 0x20 ; //	BRA start
		j=i;
		image[ i++ ] = (start-(j+1))&0xFF;
		
		image[patchDone] = (i-(patchDone+1))&0xFF;
		
		image[ i++ ] = 0x45; // LDHX #SP+1 ; restore stack pointer
		image[ i++ ] = (SP+1) >> 8;
		image[ i++ ] = (SP+1) & 0xFF;
		
		image[ i++ ] = 0x94; // TXS 
		
		image[ i++ ] = 0xCC; // JMP back to MON (this is the only way)
		image[ i++ ] = MONRTN>>8;
		image[ i++ ] = MONRTN&0xFF;
	
		flsprintf(stdout,"helper end %04x\n",i);
/*
		if (i>=WORKTOP) { // leave some stack space for monitor routines
		  
			flsprintf(stderr,"Not enough WORKRAM on target\n");
			abort();
			}
*/		
		writeMemory(WORKRAM , i-WORKRAM , 0);
		lastMon=-1;
		}
	else
		writeMemory(WORKRAM , 4 , 0);
		
	runFrom(start, 0x00, 0x00, 0); 
	
	while (progn) {
		int n = progn < PAGESIZE ? progn : PAGESIZE;
		int sum=0;
		for (i=0; i<n; ++i) {
			putByte(image[faddr+i]);
			}
		for (i=0; i<n; ++i) {
			int b=getByte();
			//flsprintf(stderr,"%d %02X\n",i,b);
			sum = (sum+b)&0xFF;
			if (b != image[faddr+i])
				flsprintf(stderr,"Program data transfer error, at %04X sent %02X got %02X\n",faddr+i,image[faddr+i],b);
			}
		int back=getByte();
			//flsprintf(stderr,"%02X\n",back);
		if (back != sum) {
			flsprintf(stderr,"Program checksum failure, at %04X size %04X checksum calculated %02X received %02X\n",faddr,n,sum,back);
			abort();
			}
		progn -= n;
		faddr += n;
		if (verbose)
			flsprintf(stdout,".");
		}
	return 0;
	}	
		
void massErase() {
// NOTE! to override flash security and erase FLBPR must be used as the erase address for mass erase
	if (verbose) 
		flsprintf(stdout,"Mass erase\n");
	if (MONTYPE == 0)
		callMonitor_A(ERARRNG , ERARRNG_MASS_ERASE, 0, EADDR, 0);
	if (MONTYPE == 1)
		callMonitor_B(ERARRNG , 0, 0, 0xFFFF, 0);
	}
	
void flashProgram(int addr,int size,int verify) {
	if (addr < FLASH) {
		flsprintf(stdout,"Programming address %04X below flash start address %04X\n",addr,FLASH);
		abort();
		}
	
	if (useFastProg) {
		if (verbose) 
			flsprintf(stdout,"Program %04X - %04X ",addr,addr+size-1);
				
		fastProg(addr,size);
		if (verbose)
			flsprintf(stdout,"\n");
		}
	else {
		while (size>0) {
			int	n=size <= PAGESIZE ? size : PAGESIZE;
			if (verbose) 
				flsprintf(stdout,"Program %04X - %04X ",addr,addr+n-1);
				
			memcpy(&image[ DATABUF ] , &image[addr] , n);
			writeMemory(DATABUF , n , verbose);
			if (MONTYPE == 0)
				callMonitor_A(PRGRNGE, 0 , 0, addr , addr+n-1);
			if (MONTYPE == 1)
				callMonitor_B(PRGRNGE, 0 , 0, addr , n);
			if (verify) {
				int cca;
				int sum=0;
				int i;
				if (MONTYPE == 0)
					cca=callMonitor_A(RDVRRNG, 0 , 1 , addr , addr+n-1);
				if (MONTYPE == 1)
					cca=callMonitor_B(RDVRRNG, 0 , 1 , addr , n);
				for (i=0; i<n; ++i) 
					sum = (sum + image[addr+i]) & 0xFF;
				int back= cca & 0xFF; // get Accu from target
				if (sum != back) {
					flsprintf(stderr,"Program checksum failure, at %04X size %04X checksum calculated %02X received %02X\n",addr,n,sum,back);
					abort();
					}
					
				if (!(cca & 0x0100)) { // check Carry bit from target
					flsprintf(stderr,"Verify failed\n");
					abort();
					}
				if (verbose)
					flsprintf(stdout,"OK");
				}
			if (verbose)
				flsprintf(stdout,"\n");
			addr += n;
			size -= n;
			}
		}
	}

int readSrec(int verbose,FILE* sf,unsigned char* image, int size,  int base, int* ranges, int rn) {
	if (verbose)
		flsprintf(stdout,"Reading S-records\n");
	memset(image,0xff,size);
	char line[2+2+255*2+2+1]; // Sx + count + 255 bytes for data address & checksum + CR/LF +nul (in windows)
	int amax=0;
	int rc=0;
	while (fgets(line,sizeof(line),sf)!=NULL) {
	   int o=0;
	   if (line[0]=='S') {
			unsigned int n,a;
			sscanf(line+2,"%2x",&n);
			n--;
			if (line[1]=='1') {
				sscanf(line+4,"%4x",&a);
				n=n-2;
				o=8;
			}
			if (line[1]=='2') {
				sscanf(line+4,"%6x",&a);
				n=n-4;
				o=10;
			}
			if (line[1]=='3') {
				sscanf(line+4,"%8x",&a);
				n=n-6;
				o=12;
			}
			if (o!=0) {
				int i,j;
				if (ranges) {
				    for (i=0; i<rc; i+=2) {
						int rlo=ranges[i];
						int rhi=rlo+ranges[i+1];
						if (!((a+n<=rlo) || (rhi<=a))) {
							flsprintf(stderr,"Overlapping S-record ranges %04X,%04X and %0x4 %04X\n",rlo,rhi-rlo,a,n);
							abort();
							}
						}
					if (rc + 2 >= rn) 
						return -1;
					ranges[rc]=a;
					ranges[rc+1]=n;
					rc += 2;
					
					int cf=0;
					do compact: {
						for (i=0; i<rc; i+=2) {
							for (j=i+2; j<rc; j+=2) {
								cf=1;
								if (ranges[i]+ranges[i+1]==ranges[j])
									ranges[i+1] += ranges[j+1];
								else if (ranges[i]==ranges[j]+ranges[j+1])
									ranges[i]-=ranges[j+1];
								else 
									cf=0;
								if (cf) {
									for (i=j+2; i<rc; i++)	
										ranges[i]=ranges[i+2];
									rc-=2;
									cf=0;
									goto compact;
									}
								}
							}
						} while (cf);
					}
				for (i=0; i<n; ++i) {
					unsigned int d;
					sscanf(line+o+i*2,"%2x",&d);
					if ( (a >= base) && (a < base+size)) {
						image[ a - base ] = d;
						a++;
						amax = a>amax ? a : amax;
						}
					}
				}
			}
		if (verbose>1)
			flsprintf(stdout,">>> %s",line);
		if (verbose && o==0)
			flsprintf(stdout,"Line ignored: %s\n",line);
		}
	if (verbose) {
		if (ranges) {
			int i;
			for (i=0; i<rc; i+=2) 
				flsprintf(stdout,"S-record data address %06X size %06X\n",ranges[i],ranges[i+1]);
			}
		flsprintf(stdout,"\n");
		}
	return rc;
	}
void printHelp() {
		flsprintf(stdout,"bl08 burns MC68HC908 Flash memory from S-record file(s) using Monitor mode\n");
		flsprintf(stdout,"Usage: \n");
		flsprintf(stdout," bl08 [-aBbcdefhiklmnpqrstuvwxy] [filename...]\n");
		flsprintf(stdout,"  -a address     Set dump memory address (needs -s option too)\n");
		flsprintf(stdout,"  -B baudrate    Set baud rate for target communication\n");
		flsprintf(stdout,"  -b speed       Set baud rate using speed_t value as defined in termios.h.\n");
		flsprintf(stdout,"                 The value is passed directly to cfsetXspeed() and can be\n");
		flsprintf(stdout,"                 used to set non-Posix baud rates. On MacOSX the value is\n");
		flsprintf(stdout,"                 equivalent to actual baud rate, whereas on Linux it is\n");
		flsprintf(stdout,"                 very different. If in doubt, use the -B option instead.\n");
		flsprintf(stdout,"  -c device      Set serial com device used to communicate with target\n"); 
		flsprintf(stdout,"                 (default '/dev/ttyS0')\n");
		flsprintf(stdout,"  -d dumpformat  Set dump format, supported formats are: 'srec'\n");
		flsprintf(stdout,"  -e             Erase target using mass erase mode, clearing security bytes\n");
		flsprintf(stdout,"  -f             Use fast programming method\n");		
		flsprintf(stdout,"  -g address     Go execute target code from address or use '-g reset'\n");
		flsprintf(stdout,"  -h             Print out this help text\n");
		flsprintf(stdout,"  -i             Read input (S-records) from standard input\n");
		flsprintf(stdout,"  -k             Kill previous instance of bl08\n");
		flsprintf(stdout,"  -l verbosity   Set verbosity level, valid values are 0-4, default 1\n");
		flsprintf(stdout,"  -m             Terminal emulator mode\n");
		flsprintf(stdout,"  -n             Print bl08 version number\n");
		flsprintf(stdout,"  -o param=value Override target parameter value\n");	
		flsprintf(stdout,"                 param = ROMBASE,FLASH,PUTBYTE,GETBYTE,RDVRRNG,MONRTN\n");
		flsprintf(stdout,"                         ERARRNG,PRGRNGE,FLBPR,MONDATA,PAGESIZE,EADDR,RAM\n");	
		flsprintf(stdout,"  -p             Use page erase when programming flash\n");	
		flsprintf(stdout,"  -q             Run quietly, same as -l 0\n");		
		flsprintf(stdout,"  -s size        Set dump memory size\n");
		flsprintf(stdout,"  -r pulse       Pulse DTR for pulse milliseconds\n");
		flsprintf(stdout,"  -R pulse       Pulse DTR for pulse milliseconds with opposite polarity\n");
		flsprintf(stdout,"                 r is OK for transitor directly on DTR pin\n");
		flsprintf(stdout,"                 R is OK for transitor on MAX232 (inverted) pin, as usual\n");
		flsprintf(stdout,"                 on classic RS232 MON08 boards, \n");
		flsprintf(stdout,"  -t cputype     Set CPU type, valid values are: gz16, jb8, qy2, qy4, jk1,\n"); 
		flsprintf(stdout,"                 jk3, jk8, jl16, gr4, gr8, kx2, kx8\n");
		flsprintf(stdout,"  -u             Upload only (do not program flash)\n");
		flsprintf(stdout,"  -v             Verify when programming \n");
		flsprintf(stdout,"  -w pin=value   Set serial port pin state\n");
		flsprintf(stdout,"                 pin=LE,DTR,RTS,ST,SR,CTS,CAR,CD,RNG,RI,DSR\n");
		flsprintf(stdout,"                 val=1/0, 1=negative pin voltage\n");
		flsprintf(stdout,"  -x cpuspeed    Set CPU speed, typically set for Fbus (in MHz) x 4 \n");
		flsprintf(stdout,"  -y string      Security code, as a string of hex bytes\n");
		flsprintf(stdout,"  -z             Do not program, do no upload, just read in the S-rec file \n");
		flsprintf(stdout," addresses and sizes in decimal, for hex prefix with '0x'\n");
		// Example
	exit(0);
	}



void termEmu()
	{
	int STDIN=0;
	int STDOUT=1;
	// get rid of stuff that has been echoed to us
	tcflush(com,TCIFLUSH ); // .. then get rid of the echoes and what ever...


	struct termios newtio,oldtio;
	tcgetattr(STDIN,&oldtio);  // we do not want to change the console setting forever, so keep the old
	tcgetattr(STDIN,&newtio);  // configure the console to return as soon as it a key is pressed
    newtio.c_lflag = 0;    
    newtio.c_cc[VTIME]    = 0;   
    newtio.c_cc[VMIN]     = 1;   
    tcsetattr(STDIN,TCSANOW,&newtio);

	flsprintf(stdout,"\nTerminal mode, press CTRL-C to exit.\n");

    fd_set readfs;    
    int    maxfd;     
    int res;
	char buf[2];
    maxfd = com+1;  
    
    while (1) {
		// wait for something from console or serial port
		FD_SET(STDIN, &readfs);  
		FD_SET(com, &readfs);  
		struct timeval tout;
		tout.tv_usec = 10; 
		tout.tv_sec  = 0; 
		select(maxfd, &readfs, NULL, NULL, &tout);

		// copy console stuff to the serial port
		if (FD_ISSET(STDIN,&readfs))   {    
			res=read(STDIN,buf,1);
			if (buf[0]==3) break; // CTRL-C terminates terminal mode
			write(com,buf,res);
		}
		// copy serial port stuff to console				
		if (FD_ISSET(com,&readfs)){    
			res=read(com,buf,1);
			write(STDOUT,buf,res);
			fflush(stdout);
			}
	}

	tcsetattr(0,TCSANOW,&oldtio); // restore console settings

///	close(con);
	}

void setCPUtype(char* cpu) {
	if (strcmp("gz16",cpu)==0 || strcmp("mc68hc908gz16",cpu)==0) {
		// These settings depend on the CPU version
		CPUSPEED=8;
		FLASH=0xC000;
		PUTBYTE=0xFEAF; 
		GETBYTE=0x1C00;
		RDVRRNG=0x1C03;
		ERARRNG=0x1C06;
		PRGRNGE=0x1C09;
		MONRTN=0xFE20;
		FLBPR=0xFF7E;
		EADDR=FLBPR;
		RAM = 0x40;
		MONTYPE = 0;
		}
	else if (strcmp("jb8",cpu)==0 || strcmp("mc68hc908jb8",cpu)==0) {
		// These settings depend on the CPU version
		CPUSPEED=12; // typically run at 6 MHz, bus freq 3 Mhz
		FLASH=0xDC00;
		PUTBYTE=0xFED6; 
		GETBYTE=0xFC00;
		RDVRRNG=0xFC03;
		ERARRNG=0xFC06;
		PRGRNGE=0xFC09;
		MONRTN=0xFE55;
		FLBPR=0xFE09;
		EADDR=FLASH;
		RAM = 0x40;
		MONTYPE = 0;
		}
	else if (strcmp("qy2",cpu)==0 || strcmp("mc68hc908qy",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xF800;
		PUTBYTE=0xFEA1;  
		GETBYTE=0x2800;
		RDVRRNG=0x2803;
		ERARRNG=0x2806;
		PRGRNGE=0x2809;
		MONRTN=0xFE26;   
		FLBPR=0xFFBE;
		EADDR=FLASH;     
		RAM = 0x80;
		MONTYPE = 0;
		}
	else if (strcmp("qy4",cpu)==0 || strcmp("mc68hc908q4",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xEE00;
		PUTBYTE=0xFEA1;  
		GETBYTE=0x2800;
		RDVRRNG=0x2803;
		ERARRNG=0x2806;
		PRGRNGE=0x2809;
		MONRTN=0xFE26;   
		FLBPR=0xFFBE;
		EADDR=FLASH;     
		RAM = 0x80;
		MONTYPE = 0;
		}
	else if (strcmp("jk8",cpu)==0 || strcmp("mc68hc908jk8",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xDC00;
		PUTBYTE=0xFF00; 
		GETBYTE=0xFC00;
		RDVRRNG=0xFC03;
		ERARRNG=0xFCBE;
		PRGRNGE=0xFC06;
		MONRTN=0xFE6C;  
		FLBPR=0xFFCF;
		EADDR=FLASH;     
		RAM = 0x60;
		MONTYPE = 1;
		}
	else if (strcmp("jk3",cpu)==0 || strcmp("mc68hc908jk3",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xEC00;
		PUTBYTE=0x0000; 
		GETBYTE=0xFC00;
		RDVRRNG=0xFC03;
		ERARRNG=0xFC06;
		PRGRNGE=0xFC09;
		MONRTN=0x0000;  
		FLBPR=0xFE09;
		EADDR=FLASH;     
		RAM = 0x80;
		MONTYPE = 0;
		}
	else if (strcmp("jk1",cpu)==0 || strcmp("mc68hc908jk1",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xF600;
		PUTBYTE=0x0000; 
		GETBYTE=0xFC00;
		RDVRRNG=0xFC03;
		ERARRNG=0xFC06;
		PRGRNGE=0xFC09;
		MONRTN=0x0000;  
		FLBPR=0xFE09;
		EADDR=FLASH;     
		RAM = 0x80;
		MONTYPE = 0;
		}
	else if (strcmp("jl16",cpu)==0 || strcmp("mc68hc908jl16",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xBC00;
		PUTBYTE=0x0000; 
		GETBYTE=0xFF00;
		RDVRRNG=0xFF30;
		ERARRNG=0xFCBE;
		PRGRNGE=0xFC06;
		MONRTN=0x0000;  
		FLBPR=0xFFCF;
		EADDR=FLASH;     
		RAM = 0x60;
		MONTYPE = 1;
		}
	else if (strcmp("gr8",cpu)==0 || strcmp("mc68hc908gr8",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xE000;
		PUTBYTE=0x0000; 
		GETBYTE=0x1C99;
		RDVRRNG=0x1CAD;
		ERARRNG=0x1DA0;
		PRGRNGE=0x1CEC;
		MONRTN=0x0000;  
		FLBPR=0xFF7E;
		EADDR=FLASH;     
		RAM = 0x40;
		MONTYPE = 0;
		}
	else if (strcmp("gr4",cpu)==0 || strcmp("mc68hc908gr4",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xEE00;
		PUTBYTE=0x0000; 
		GETBYTE=0x1C99;
		RDVRRNG=0x1CAD;
		ERARRNG=0x1DA0;
		PRGRNGE=0x1CEC;
		MONRTN=0x0000;  
		FLBPR=0xFF7E;
		EADDR=FLASH;     
		RAM = 0x40;
		MONTYPE = 0;
		}
	else if (strcmp("kx2",cpu)==0 || strcmp("mc68hc908kx2",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xF600;
		PUTBYTE=0x0000; 
		GETBYTE=0x1000;
		RDVRRNG=0x1003;
		ERARRNG=0x1006;
		PRGRNGE=0x1009;
		MONRTN=0x0000;  
		FLBPR=0xFF7E;
		EADDR=FLASH;     
		RAM = 0x40;
		MONTYPE = 0;
		}
	else if (strcmp("kx8",cpu)==0 || strcmp("mc68hc908kx8",cpu)==0) {
		// These settings depend on the CPU version
		FLASH=0xE000;
		PUTBYTE=0x0000; 
		GETBYTE=0x1000;
		RDVRRNG=0x1003;
		ERARRNG=0x1006;
		PRGRNGE=0x1009;
		MONRTN=0x0000;  
		FLBPR=0xFF7E;
		EADDR=FLASH;     
		RAM = 0x40;
		MONTYPE = 0;
		}

	else {
		flsprintf(stderr,"Unsupported CPU type '%s'\n",cpu);
		flsprintf(stderr,"The supported CPU types are:\n");
		flsprintf(stderr,"\tmc68hc908gz16 (gz16)\n");
		flsprintf(stderr,"\tmc68hc908jb8 (jb8)\n");
		flsprintf(stderr,"\tmc68hlc908qy2 (qy2)\n");
		abort();
		}
	// These are independent of CPU type
	PAGESIZE = 64;
	MONDATA = RAM+8;
	CTRLBYT  =  MONDATA+0;
	CPUSPD  =  MONDATA+1;
	LADDR  =  MONDATA+2;
	DATABUF  =  MONDATA+4;
	WORKRAM = DATABUF + PAGESIZE;
	WORKTOP = 0xF0; // this leaves 16 bytes for stack, the deepest ROM routines use 11 so there some for myself too
	if (DATABUF+PAGESIZE>0xFF) {
		flsprintf(stderr,"bl08 limitation, DATABUF+PAGESIZE>0xFF, DATABUF=%04X, PAGESIZE=%04X\n",DATABUF,PAGESIZE);
		abort();
		}
	}
	
	
		
	
int getIntArg(char* arg) {
	if (strlen(arg)>=2 && memcmp(arg,"0x",2)==0) {
		unsigned int u;
		sscanf(arg+2,"%X",&u);
		return u;
		}
	else {
		int d;
		sscanf(arg,"%d",&d);
		return d;
		}
	}
	
	
void parseOverride(char* str) {	
	char* vp=strstr(str,"=");
	if (!vp) {
		flsprintf(stderr,"Bad override syntax, no '=' found\n");
		abort();
		}
	*vp=0;
	vp++;
	int val=getIntArg(vp);
	if (strcmp("ROMBASE",str)==0) {
		GETBYTE=val+0;
		RDVRRNG=val+3;
		ERARRNG=val+6;
		PRGRNGE=val+9;
		}
	else if (strcmp("FLASH",str)==0) FLASH=val;
	else if (strcmp("PUTBYTE",str)==0) PUTBYTE=val;
	else if (strcmp("GETBYTE",str)==0) GETBYTE=val;
	else if (strcmp("RDVRRNG",str)==0) RDVRRNG=val;
	else if (strcmp("ERARRNG",str)==0) ERARRNG=val;
	else if (strcmp("PRGRNGE",str)==0) PRGRNGE=val;
	else if (strcmp("FLBPR",str)==0) FLBPR=val;
	else if (strcmp("RAM",str)==0) RAM=val;
	else if (strcmp("MONDATA",str)==0) MONDATA=val;
	else if (strcmp("PAGESIZE",str)==0) PAGESIZE=val;
	else if (strcmp("MONRTN",str)==0) MONRTN=val;
	else if (strcmp("EADDR",str)==0) EADDR=val;
	else {
		flsprintf(stderr,"Attempt to override unrecognized variable %s\n",str);
		abort();
		}
	}
	
void parseIOControl(char* str) {	
	char* vp=strstr(str,"=");
	if (!vp) {
		flsprintf(stderr,"Bad pin control syntax, no '=' found\n");
		abort();
		}
	*vp=0;
	vp++;
	int val=getIntArg(vp);
	if (val!=0 && val!=1) {
		flsprintf(stderr,"Bad pin control value %d (not 1/0)\n",val);
		abort();
		}
	
	int i;
	for (i=0; i<sizeof(portPins)/sizeof(portPins[0]); i++) {
		if (strcmp(portPins[i].str,str)==0)  {
			forcePins[i]=val+1;
			return;
			}
	}
	
	flsprintf(stderr,"Attempt to set unrecognized pin %s\n",str);
	abort();	
	}


void parseBaudrate(char* str) {	
	int i;
	for (i=0; i<sizeof(baudrates)/sizeof(baudrates[0]); ++i) {
		if (strcmp(baudrates[i].str,str)==0) {
			baudRate=baudrates[i].val;
			return;
			}
		}
	flsprintf(stderr,"Attempt to set unrecognized baud rate %s\n",str);
	abort();	
	}

void parseTermSpeed(char* str) {	
	if (sscanf(str,"%d",&baudRate)!=1) {
		flsprintf(stderr,"Failed to set terminal speed %s\n",str);
		abort();	
		}
	}

void setSecurityCode(char* str) {
	int i;
	int consumed;
	for (i=0; i<8 && *str; ++i) {
		if (sscanf(str," %2hhx%n",&scode[i],&consumed)!=1) {
			flsprintf(stderr,"Bad security code: %s\n",str);
			abort();
			}
		str += consumed;
		while (*str==' ') str++; /* tolerate spaces */
		if (*str==':') str++;    /* and one colon */
		}
	if (i<8) {
		flsprintf(stderr,"Found only %d bytes in security code\n",i);
		abort();
		}
	}
	
void parseArgs(int argc, char *argv[]) {	
	int c;
	while ((c = getopt (argc, argv, "a:B:b:c:d:efg:hikl:mno:pqr:R:s:t:uvw:x:y:z")) != -1) {
		switch (c) {
			case 'a' :
				dumpStart=getIntArg(optarg);
				break;
			case 'B' : 
				parseBaudrate(optarg);
				break;
			case 'b' : 
				parseTermSpeed(optarg);
				break;
			case 'c' : 
				COM=optarg;
				break;
			case 'd' :
				dumpFormat=optarg;
				break;
			case 'e' :
				eraseFlash = 1;
				break;
			case 'f' :
				useFastProg = 1;
				break;
			case 'g' :
				executeCode=optarg;
				break;
			case 'h' :
				printHelp();
				break;
			case 'i' :
				useStdin = 1;
				break;
			case 'k' :
				killPrevious = 1;
				break;
			case 'l' :
				sscanf(optarg,"%d",&verbose); 
				break;
			case 'm' :
				terminalMode=1;
				break;
			case 'n' :
				flsprintf(stdout,"%s\n",version);
				break;
			case 'o' :
				parseOverride(optarg);
				break;
			case 'p' :
				pageErase=1;
				break;
			case 'q' :
				verbose=0;
				break;
			case 'r' :
				resetPolarity = 0;
				resetPulse=getIntArg(optarg);
				break;
			case 'R' :
				resetPolarity = 1;
				resetPulse=getIntArg(optarg);
				break;
			case 's' :
				dumpSize=getIntArg(optarg);
				break;
			case 't' :
				setCPUtype(optarg);
				break;
			case 'u' :
				uploadOnly=1;
				break;
			case 'v' :
				verify=1;
				break;
			case 'w' :
				parseIOControl(optarg);
				break;
			case 'x' :
				sscanf(optarg,"%d",&CPUSPEED); 
				break;
			case 'y' :
				setSecurityCode(optarg);
				break;
			case 'z' :
				loadOnly=1; 
				break;
			case '?' :
				if (isprint (optopt))
					flsprintf(stderr,"Unknown option `-%c'.\n", optopt);
				else
					flsprintf(stderr,"Unknown option character `\\x%x'.\n",optopt);
			  default:
				flsprintf(stderr,"Bug, unhandled option '%c'\n",c);
				abort ();
			}
		}
	if (argc<=1) 
		printHelp();
	}


void generateReset() {
	struct timespec tspec;
	int s;
	if (verbose)
		flsprintf(stderr,"Applying %dms reset.\n",resetPulse);
	ioctlErrCheck(ioctl(com, TIOCMGET, &s)); 
	if (resetPolarity == 0)
		s |= TIOCM_DTR;
	else
		s &= ~TIOCM_DTR;
	ioctlErrCheck(ioctl(com, TIOCMSET, &s)); 
		
	tspec.tv_sec=resetPulse/1000;
	tspec.tv_nsec=(resetPulse%1000)*1000000; 
	nanosleep(&tspec,0);
	
	if (resetPolarity == 0)
		s &= ~TIOCM_DTR;
	else
		s |= TIOCM_DTR;
	ioctlErrCheck(ioctl(com, TIOCMSET, &s)); 

	tspec.tv_sec=0;
	tspec.tv_nsec=1000000; /* wait at least 1 ms */
	nanosleep(&tspec,0);
	}


void deletePidFile() {
	int stat=remove(PIDFILENAME);
	if (stat)
		flsprintf(stderr,"remove returned %d\n",stat);
	}
	
void killPreviousInstance() {
	atexit(deletePidFile);
	int pid;
	FILE* pidf=fopen(PIDFILENAME,"r");
	if (pidf) {
		fscanf(pidf,"%d",&pid);
		int stat=kill(pid,SIGKILL);
		if (stat!=0)
			flsprintf(stderr,"kill returned %d\n",stat);
			
		fclose(pidf);
		waitpid(pid,&stat,0);
		if (WIFEXITED(stat)==0)
			flsprintf(stderr,"waitpid returned %d\n",WIFEXITED(stat));
		}
	pidf=fopen(PIDFILENAME,"w");
	fprintf(pidf,"%d\n",getpid());
	fclose(pidf);
	}

int main(int argc, char *argv[]) {	
	
	// default values
	setCPUtype("gz16");
	
	parseArgs(argc,argv);
	
	
	if (killPrevious)
		killPreviousInstance();
	

	if (verbose)
		flsprintf(stdout,"bl08 - MC68HC908 Bootloader - version %s\n",version);
	
	memset(image,0xFF,sizeof(image));
	
	int maxrc=256;
	int ranges[maxrc];
	int rc=0;
	
	if (useStdin) 
		rc += readSrec(verbose , stdin , image , sizeof(image) , 0x0000 , ranges+rc , maxrc - rc);
	
	int i;
	for (i=optind; i<argc; i++) {
		char* filename=argv[i];
		FILE* sf = fopen(filename, "r");
		if (sf == NULL) { 
			flsprintf(stderr,"Failed to open '%s'\n", filename);
			abort();
			}
		
		int rn=readSrec(verbose , sf , image , sizeof(image) , 0x0000 , ranges+rc , maxrc - rc);
		if (rn<0) {
			flsprintf(stderr,"Too many discontinuous data ranges in S-record file '%s'\n",filename);
			abort();
			}
		rc += rn;		
		fclose(sf);
		}
				
	if (!loadOnly) {
		initSerialPort();
		if (resetPulse)
			generateReset();
		
		if (eraseFlash) {
			connectTarget();
			massErase();
			}
		
		if (rc>0) {
			connectTarget();
			if (pageErase) {
				for (i=0; i<rc; ) {
					int addr=ranges[ i++ ];
					int size=ranges[ i++ ];
					int a = addr / PAGESIZE * PAGESIZE;
					while (a<addr+size) {
						if (verbose)
							flsprintf(stdout,"Erase %04X - %04X\n",a,PAGESIZE);
						if (MONTYPE == 0)
							callMonitor_A(ERARRNG, ERARRNG_PAGE_ERASE, 0, a, 0);
						a += PAGESIZE;
						}
					}
				}
			else
				massErase();	
		
			int i;
			for (i=0; i<rc;) {
				int addr=ranges[ i++ ];
				int size=ranges[ i++ ];
				if (uploadOnly) {
					if (verbose)
						flsprintf(stdout,"Uploading memory contents\n");
					writeMemory(addr,size,verbose);
					if (verbose)
						flsprintf(stdout,"\n");
					}
				else 
					flashProgram(addr , size , verify);
				if (verbose>1) {
					flsprintf(stdout,"Reading back memory/flash content\n");
					readMemory(addr , size, verbose);
					flsprintf(stdout,"\n");
					dumpMemory(addr , size);
					}
				}
			}


		if (executeCode) {
			connectTarget();
			int addr;
			if (strcmp("reset",executeCode)==0) {
				readMemory(0xFFFE,2,0);
				addr=((image[0xFFFE]&0xFF)<<8) | (image[0xFFFF]&0xFF);
				/* Unlocked flash reads as 0xAD everywhere */
				if (addr == 0xADAD) {
					flsprintf(stdout,"Could not read reset vector, flash security locked?\n");
					abort();
					}
				}
			else  
				addr=getIntArg(executeCode);

			if (verbose)
				flsprintf(stdout,"Execute code from %04X\n",addr);
			runFrom(addr,0,0,0);
			}
		
		if (terminalMode) 
			termEmu();
		}

	if (dumpSize>0) {
		if (!loadOnly) {
			connectTarget();
			if (verbose) 
				flsprintf(stdout,"Reading memory\n");
			readMemory(dumpStart,dumpSize, verbose);
			}
		if (verbose) 
			flsprintf(stdout,"\n");
		
		if (dumpFormat) {
			if (strcmp("srec",dumpFormat)==0)
				dumpMemorySrec(dumpStart,dumpSize);
			else if (strcmp("hex",dumpFormat)==0)
				dumpMemory(dumpStart,dumpSize);
			else
				flsprintf(stderr,"Unknown dump format '%s'\n",dumpFormat);
			}
		}
			
	return 0;
	}





