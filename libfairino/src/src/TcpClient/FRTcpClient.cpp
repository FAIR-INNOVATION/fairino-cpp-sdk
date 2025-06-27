#include "FRTcpClient.h"

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <WS2tcpip.h>
#include <Mstcpip.h>
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

using namespace std;

FRTcpClient::FRTcpClient(string IP, int port)
{
	robotIP = IP;
	robotPort = port;

#ifdef WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.S_un.S_addr = inet_addr(robotIP.c_str()); /// 服务器ip;
    servAddr.sin_port = htons(robotPort);                      /// 服务器端口;
#else

    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(robotPort);                 /// 服务器端口;
    servAddr.sin_addr.s_addr = inet_addr(robotIP.c_str()); /// 服务器ip;
#endif

    fd = socket(AF_INET, SOCK_STREAM, 0);
    SetTimeOut(timeOut);
}


FRTcpClient::~FRTcpClient()
{

}

int FRTcpClient::Connect()
{
#ifdef WIN32
    int rtn = connect(fd, (SOCKADDR*)&servAddr, sizeof(SOCKADDR));
    if (rtn < 0)
    {
        int connectError = WSAGetLastError();
        return -1;
    }
#else
    if (connect(fd, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0)
    {
        logger_error("connect fail, %s.", strerror(errno));
        return -1;
    }
#endif
    
    return 0;
}

int FRTcpClient::ReConnect()
{
    reconnFlag = true;
    bool reconnectSuccess = false;

    int maxConnTimes = 0;
#ifdef WIN32
    maxConnTimes = reConnTime / 1000;
#else
    maxConnTimes = reConnTime / timeOut;
#endif // WIN32

    for (int i = 0; i < maxConnTimes; i++)
    {
        int connRtn = Connect();
        if (connRtn != 0)
        {
            logger_error("TCPClient reconnect to Robot fail %d/%d, error code is %d", i, maxConnTimes, connRtn);
//#ifdef WIN32
//            std::this_thread::sleep_for(std::chrono::milliseconds(timeOut));  //设置不同的延时时间，用于凑1s一次连接
//#else
//            std::this_thread::sleep_for(std::chrono::milliseconds(timeOut));  //设置不同的延时时间，用于凑1s一次连接
//#endif
            continue;
        }
        else
        {
            reconnectSuccess = true;
            break;  //连接成功
        }
    }
    reconnFlag = false;
    return reconnectSuccess;
}

int FRTcpClient::SetTimeOut(int timeout)
{
    int syncnt = 1;
    /* 设置连接超时 */
#ifdef WIN32
    setsockopt(fd, IPPROTO_TCP, TCP_MAXRT, (char*)&syncnt, sizeof(syncnt));  //win下连接超时1s，非常准
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeOut, sizeof(int));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeOut, sizeof(int));
    int bKeepAlive = TRUE;
    int nRet = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, (char*)&bKeepAlive, sizeof(bKeepAlive));
    if (nRet == SOCKET_ERROR)
    {
        printf("set SDK socket keepalive failed\n ");
        return FALSE;
    }

    // 设置KeepAlive参数
    tcp_keepalive alive_in = { 0 };
    tcp_keepalive alive_out = { 0 };
    alive_in.keepalivetime = 500; // 开始首次KeepAlive探测前的TCP空闭时间 ms
    alive_in.keepaliveinterval = 500; // 两次KeepAlive探测间的时间间隔 ms
    alive_in.onoff = TRUE;
    unsigned long ulBytesReturn = 0;
    nRet = WSAIoctl(fd, SIO_KEEPALIVE_VALS, &alive_in, sizeof(alive_in),
        &alive_out, sizeof(alive_out), &ulBytesReturn, NULL, NULL);
    if (nRet == SOCKET_ERROR)
    {
        printf("set SDK socket keepalive param failed\n ");
        return FALSE;
    }
#else
    //setsockopt(fd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(syncnt));        //这玩意不会用
    struct timeval tv;
    tv.tv_sec = timeOut / 1000;
    tv.tv_usec = timeOut % 1000 * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

    int keepAlive = 1; // 开启keepalive属性
    int keepIdle = 1; // 如该连接在60秒内没有任何数据往来,则进行探测 
    int keepInterval = 1; // 探测时发包的时间间隔为5 秒
    int keepCount = 1; // 探测尝试的次数.如果第1次探测包就收到响应了,则不再发.

    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepAlive, sizeof(keepAlive));
    setsockopt(fd, SOL_TCP, TCP_KEEPIDLE, (void*)&keepIdle, sizeof(keepIdle));
    setsockopt(fd, SOL_TCP, TCP_KEEPINTVL, (void*)&keepInterval, sizeof(keepInterval));
    setsockopt(fd, SOL_TCP, TCP_KEEPCNT, (void*)&keepCount, sizeof(keepCount));

#endif
    return 0;
}

int FRTcpClient::Send(char* sendBuf, int sendSize)
{
#ifdef WIN32
    int sendLength = send(fd, sendBuf, sendSize, 0);
#else
    int sendLength = send(fd, sendBuf, sendSize, MSG_NOSIGNAL);//必须加MSG_NOSIGNAL，否则网络中断后，发送至fd，socket会发送一个信号终止程序
#endif // WIN32
    
    if (sendLength < 0)
    {
        Close();
        if (reconnEnable == false)  //没有使能重连
        {
            return -1;
        }

#ifdef WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif // WIN32

        fd = socket(AF_INET, SOCK_STREAM, 0);
        SetTimeOut(timeOut);
        bool reconnectSuccess = ReConnect();
        if (reconnectSuccess)
        {
            sendLength = send(fd, sendBuf, sendSize, 0);
        }
        else
        {
            return -1;
        }
    }
    return sendLength;
}

int FRTcpClient::Recv(char* recvBuf, int recvSize)
{
    int recvlength = recv(fd, recvBuf, recvSize, 0);
    return recvlength;
}

int FRTcpClient::RecvPkg(char* recvBuf, int recvSize)
{
    uint8_t allRecvBuf[1024] = {};
    int curRecvTotalSize = 0;
    int tmpRecvSize = 0;
    char tmpRecvBuf[1024] = {};
    while (recvSize - curRecvTotalSize > 0)  //还有数据未接收
    {
        memset(tmpRecvBuf, 0, 1024);
        tmpRecvSize = Recv(tmpRecvBuf, recvSize - curRecvTotalSize);
        if (tmpRecvSize < 0)
        {
            logger_error("robot get realtime pkg failed");
            Close();
            if (reconnEnable == false)  //没有使能重连
            {
                return -1;
            }

#ifdef WIN32
            WSADATA wsaData;
            WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif // WIN32

            fd = socket(AF_INET, SOCK_STREAM, 0);
            SetTimeOut(timeOut);
            bool reconnectSuccess = ReConnect();
            if (reconnectSuccess)
            {
                logger_error("reconnect success");
                curRecvTotalSize = 0;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            memcpy(allRecvBuf + curRecvTotalSize, tmpRecvBuf, tmpRecvSize);
            curRecvTotalSize += tmpRecvSize;
        }

        if (recvSize == curRecvTotalSize)
        {
            if (allRecvBuf[0] == 0x5A && allRecvBuf[1] == 0x5A)
            {
                uint16_t len = 0;
                len = len | (uint16_t)allRecvBuf[4];
                len = len << 8;
                len = len | (uint16_t)allRecvBuf[3];
                if (len + 7 > recvSize)
                {
                    recvSize = len + 7;
                    continue;
                }
                else if(len + 7 == recvSize)
                {
                    int j;
                    uint16_t checksum = 0;
                    uint16_t checkdata = 0;

                    checkdata = checkdata | allRecvBuf[recvSize - 1];
                    checkdata = checkdata << 8;
                    checkdata = checkdata | allRecvBuf[recvSize - 2];

                    for (j = 0; j < recvSize - 2; j++)
                    {
                        checksum += allRecvBuf[j];
                    }

                    if (checksum == checkdata)
                    {
                        memcpy(recvBuf, allRecvBuf, recvSize);
                        return 0;
                    }
                    else
                    {
                        logger_error("error check sum");
                        return -2;//和校验失败
                    }
                }
                else
                {
                    logger_error("error SDK version");
                    return -3;  //SDK 比机器人版本新，得更新机器人版本
                }
                  
            }
        }    
    }
    return 0;
}

int FRTcpClient::Close() 
{
#ifdef WIN32
	closesocket(fd);
	WSACleanup();
#else
    shutdown(fd, 2);
	close(fd);
#endif

    return 0;
}

int FRTcpClient::SetReConnectParam(bool enable, int reconnectTime, int period)
{
    this->reConnTime = reconnectTime;
    this->reconnEnable = enable;
    this->timeOut = period;
    SetTimeOut(period);
    return 0;
}

bool FRTcpClient::GetReConnectEnable()
{
    return this->reconnEnable;
}

//判断当前是否正在重连
bool FRTcpClient::GetReConnState()
{
    return reconnFlag;
}

int FRTcpClient::SetIpConfig(std::string IP)
{
    robotIP = IP;

#ifdef WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.S_un.S_addr = inet_addr(robotIP.c_str()); /// ������ip;
    servAddr.sin_port = htons(robotPort);                      /// �������˿�;
#else

    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(robotPort);                 /// �������˿�;
    servAddr.sin_addr.s_addr = inet_addr(robotIP.c_str()); /// ������ip;
#endif

    fd = socket(AF_INET, SOCK_STREAM, 0);
    SetTimeOut(timeOut);
    return 0;
}