#include "stdio.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include <ati_ft_ethernet/FTsensor.h>

#define PORT			49152	/* Port the Ethernet DAQ always uses */

// #define PORT1			49153	/* Port the Ethernet DAQ always uses */
#define SAMPLE_COUNT	1		/* 10 incoming samples */
#define COMMAND	2  /* Command for start streaming */

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;
typedef struct ResponseStruct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} Response;

class atiforce
{
public:
  /* Sleep micro seconds */
  void MySleep(unsigned long ms)
  {
    usleep(ms * 1);
  }

  int Connect(SOCKET_HANDLE * handle, const char * ipAddress, uint16 port)
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


  void Close(SOCKET_HANDLE * handle)
  {
     close(*handle);
  }

  void SendCommand(SOCKET_HANDLE *socket)
  {
    byte request[8];
    *(uint16*)&request[0] = htons(0x1234);
    *(uint16*)&request[2] = htons(COMMAND);
    *(uint32*)&request[4] = htonl(SAMPLE_COUNT);
    send(*socket, (const char *)request, 8, 0);
    //MySleep(5); // Wait a little just to make sure that the command has been processed by Ethernet DAQ
  }


  Response Receive(SOCKET_HANDLE *socket)
  {
    Response response;
    unsigned int uItems = 0;
    int b;
    ioctl(*socket, FIONREAD, &b);
    if(b == 36)
    {
      recv(*socket, inBuffer, 36, MSG_DONTWAIT);
      response.rdt_sequence = ntohl(*(uint32*)&inBuffer[0]);
      response.ft_sequence = ntohl(*(uint32*)&inBuffer[4]);
      response.status = ntohl(*(uint32*)&inBuffer[8]);
      for(int i = 0; i < 6; i++ ) {
      response.FTData[i] = ntohl(*(int32*)&inBuffer[12 + i * 4]);
      }
    }
    return response;
  }


  void ShowResponse(Response r)
  {
    double fx = (double)r.FTData[0]/1000000.0;
    double fy = (double)r.FTData[1]/1000000.0;
    double fz = (double)r.FTData[2]/1000000.0;
    double tx = (double)r.FTData[3]/1000000.0;
    double ty = (double)r.FTData[4]/1000000.0;
    double tz = (double)r.FTData[5]/1000000.0;
    //std::cout <<"fz " << std::endl;
  }

  std::vector<double> handFT_calib;
private:
	//int32 FTData_before[6];
  byte inBuffer[36];
};
