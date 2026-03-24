#pragma once

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#elif __MINGW32__
#include <winsock2.h>
#include <windows.h>
//#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <sys/types.h>  
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#ifdef __MINGW32__
#define TCP_MAXRT 5
#include <mingw.thread.h>
#else
#include <thread>
#endif

#include <iostream>
#include "XmlRpc.h"
#include "logger.h"

class FRUdpClient
{
public:
    FRUdpClient();
    ~FRUdpClient();

    int Connect(std::string IP, int port);
    int Close();

    void RobotUDPCmdRecvThread();
    int SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, std::string));
    int SendFrame(std::string sendFrame);

    void (*UdpRecvFrameCallBack)(int, int, int, int, std::string) = nullptr;

#ifdef WIN32
    typedef SOCKET socket_fd;
    sockaddr_in addrUDPServer = {};
#else
    typedef int socket_fd;
    struct sockaddr_in addrUDPServer = {};
#endif

    std::string robotIP = "";
    int robotPort = 0;
    int timeout = 500;   //recv¡¢send³¬Ê±Ê±¼äms
    socket_fd fd;
    bool runFlag = true;

};
