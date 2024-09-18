#pragma once


#ifdef WIN32
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

#include <iostream>
#include "XmlRpc.h"
#include "logger.h"


//将跨平台网络调用封装起来;
//这一块可以考虑引入一个第三方库;

class FRTcpClient
{
public:
    FRTcpClient(std::string IP, int port);
    ~FRTcpClient();

    int Connect();
    int ReConnect();

    int Send(char* sendBuf, int sendSize);

    int Recv(char* recvBuf, int recvSize);

    int RecvPkg(char* recvBuf, int recvSize);

    int Close();

    int SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50);
    bool GetReConnectEnable();

    bool GetReConnState();

    int SetIpConfig(std::string IP);


private:
    int SetTimeOut(int timeout);

#ifdef WIN32
    typedef SOCKET socket_fd;
    sockaddr_in servAddr;
#else
    typedef int socket_fd;
    struct sockaddr_in servAddr;
#endif

    std::string robotIP = "";
    int robotPort = 0;
    socket_fd fd;

    int reConnTime = 30000;        //30000 ms
    bool reconnEnable = false;  //重连使能

    int timeOut = 1000;  // 默认1s

    bool reconnFlag = false;
};

