#include <ros/ros.h>
#include "stdio.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define PORT			49152	/* Port the Ethernet DAQ always uses */
#define SAMPLE_COUNT	10		/* 10 incoming samples */
#define SPEED			1 	/* 1000 / SPEED = Speed in Hz */

#define FILTER			4		/* 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz */
#define BIASING_ON		0xFF    /* Biasing on */
#define BIASING_OFF		0x00    /* Biasing off */

#define COMMAND_START	0x0002  /* Command for start streaming */
#define COMMAND_STOP	0x0000  /* Command for stop streaming */
#define COMMAND_BIAS	0x0042  /* Command for toggle biasing */
#define COMMAND_FILTER	0x0081  /* Command for setting filter */
#define COMMAND_SPEED	0x0082  /* Command for setting speed */

#define		UNIT  1 // 0 - Dimensionless  | 1 - Newton/Newton-meter

#if UNIT == 1
#define		FORCE_DIV	10000.0  // Default divide value
#define		TORQUE_DIV	100000.0 // Default divide value
#else
#define FORCE_DIV	1.0
#define TORQUE_DIV	1.0
#endif

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;
typedef struct ResponseStruct {
  unsigned int sequenceNumber;
  unsigned int sampleCounter;
  unsigned int status;
  int32 fx;
  int32 fy;
  int32 fz;
  int32 tx;
  int32 ty;
  int32 tz;
} Response;

class optoforce
{
public:
  static void MySleep(unsigned long ms)
  {
    usleep(ms * 1000);
  }

  static int Connect(SOCKET_HANDLE * handle, const char * ipAddress, uint16 port)
  {
    struct sockaddr_in addr;
    struct hostent *he;
    int err;

    *handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (*handle == -1) {
       fprintf(stderr, "Socket could not be opened.\n");
      return -2;
    }
    he = gethostbyname(ipAddress);
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0) {
      return -3;
    }
    return 0;
  }


  static void Close(SOCKET_HANDLE * handle)
  {
     close(*handle);
  }

  static void SendCommand(SOCKET_HANDLE *socket, uint16 command, uint32 data)
  {
    byte request[8];
    *(uint16*)&request[0] = htons(0x1234);
    *(uint16*)&request[2] = htons(command);
    *(uint32*)&request[4] = htonl(data);
    send(*socket, (const char *)request, 8, 0);
    MySleep(5);
  }


  static Response Receive(SOCKET_HANDLE *socket)
  {
    byte inBuffer[36];
    Response response;
    unsigned int uItems = 0;
    recv(*socket, (char *)inBuffer, 36, 0 );
    response.sequenceNumber = ntohl(*(uint32*)&inBuffer[0]);
    response.sampleCounter = ntohl(*(uint32*)&inBuffer[4]);
    response.status = ntohl(*(uint32*)&inBuffer[8]);
    response.fx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    response.fy = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    response.fz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    response.tx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    response.ty = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    response.tz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    return response;
  }


  static void ShowResponse(Response r)
  {
    double fx = r.fx / FORCE_DIV;
    double fy = r.fy / FORCE_DIV;
    double fz = r.fz / FORCE_DIV;
    double tx = r.tx / TORQUE_DIV;
    double ty = r.ty / TORQUE_DIV;
    double tz = r.tz / TORQUE_DIV;

  #if UNIT == 1
    fprintf(stdout, "S:%u SN: %u SC: %u Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);
  #else
    fprintf(stdout, "S:%u SN: %u SC: %u Fx: %.2f Fy: %.2f Fz: %.2f Tx: %.2f Ty: %.2f Tz: %.2f\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);
  #endif
    fflush(stdout);
  }

private:

};

#endif
