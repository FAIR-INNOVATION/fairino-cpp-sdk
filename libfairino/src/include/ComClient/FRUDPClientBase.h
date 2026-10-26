#pragma once

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <WS2tcpip.h>
#elif __MINGW32__
#include <winsock2.h>
#include <windows.h>
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

#include <string>

#ifdef WINDOWS_OPTION
#define FR_LIB_UDP_BASE_EXPORT __declspec(dllexport)
#else
#define FR_LIB_UDP_BASE_EXPORT
#endif

class FR_LIB_UDP_BASE_EXPORT FRUDPClientBase
{
public:
    virtual ~FRUDPClientBase() = default;

    virtual int Connect(std::string IP, int port) = 0;
    virtual int Close() = 0;
    virtual void RobotUDPCmdRecvThread() = 0;
    virtual int SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, std::string)) = 0;
    virtual int SendFrame(std::string sendFrame) = 0;
    virtual int SetTLSCertPath(std::string path) = 0;
    virtual bool CheckCertExists() = 0;

#ifdef WIN32
    typedef SOCKET socket_fd;
#elif __MINGW32__
    typedef SOCKET socket_fd;
#else
    typedef int socket_fd;
#endif

    sockaddr_in addrUDPServer = {};
    std::string robotIP = "";
    int robotPort = 0;
    int timeout = 500;
    socket_fd fd;
    bool runFlag = true;
    void (*UdpRecvFrameCallBack)(int, int, int, int, std::string) = nullptr;
    std::string certPath = "";
};
