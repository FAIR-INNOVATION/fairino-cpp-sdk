#pragma once

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <WS2tcpip.h>
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
#include "FRUDPClientBase.h"
#include "XmlRpc.h"
#include "logger.h"

#ifdef WINDOWS_OPTION
#define FR_LIB_UDP_EXPORT __declspec(dllexport)
#else
#define FR_LIB_UDP_EXPORT
#endif

class FR_LIB_UDP_EXPORT FRUdpClient : public FRUDPClientBase
{
public:
    FRUdpClient();
    ~FRUdpClient() override;

    // FRUDPClientBase overrides
    int Connect(std::string IP, int port) override;
    int Close() override;
    void RobotUDPCmdRecvThread() override;
    int SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, std::string)) override;
    int SendFrame(std::string sendFrame) override;
    int SetTLSCertPath(std::string path) override;
    bool CheckCertExists() override;
};
