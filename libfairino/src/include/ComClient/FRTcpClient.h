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
#include "FRTCPClientBase.h"
#include "XmlRpc.h"
#include "logger.h"

#ifdef WINDOWS_OPTION
#define FR_LIB_TCP_EXPORT __declspec(dllexport)
#else
#define FR_LIB_TCP_EXPORT
#endif


class FR_LIB_TCP_EXPORT FRTcpClient : public FRTCPClientBase
{
public:
    FRTcpClient(std::string IP, int port);
    ~FRTcpClient() override;

    // FRTCPClientBase overrides
    int Connect() override;
    int ReConnect() override;
    int Send(char* sendBuf, int sendSize) override;
    int Recv(char* recvBuf, int recvSize) override;
    int RecvFrame(char* recvBuf, int recvSize) override;
    int Close() override;
    int SetIpConfig(std::string IP) override;
    int SetPortConfig(int port) override;
    int SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50) override;
    bool GetReConnectEnable() override;
    bool GetReConnState() override;
    int SetTLSCertPath(std::string path) override;
    bool CheckCertExists() override;

    // FRTcpClient-specific functions
    int RecvPkg(char* recvBuf, int recvSize);
    int RecvCNDEPkg(char* recvBuf);

private:
    int SetTimeOut(int timeout);

#ifdef WIN32
    typedef SOCKET socket_fd;
    sockaddr_in servAddr;
#else
    typedef int socket_fd;
    struct sockaddr_in servAddr;
#endif

    socket_fd fd = 0;

};
