#pragma once

#include <string>

#ifdef WINDOWS_OPTION
#define FR_LIB_CLIENT_EXPORT __declspec(dllexport)
#else
#define FR_LIB_CLIENT_EXPORT
#endif

class FR_LIB_CLIENT_EXPORT FRTCPClientBase
{
public:
    virtual ~FRTCPClientBase() = default;

    virtual int Connect() = 0;
    virtual int ReConnect() = 0;
    virtual int Send(char* sendBuf, int sendSize) = 0;
    virtual int Recv(char* recvBuf, int recvSize) = 0;
    virtual int RecvFrame(char* recvBuf, int recvSize) = 0;
    virtual int Close() = 0;

    virtual int SetIpConfig(std::string IP) = 0;
    virtual int SetPortConfig(int port) = 0;
    virtual int SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50) = 0;
    virtual bool GetReConnectEnable() = 0;
    virtual bool GetReConnState() = 0;
    virtual int SetTLSCertPath(std::string path) = 0;
    virtual bool CheckCertExists() = 0;

protected:
    std::string robotIP = "";
    int robotPort = 0;
    int reConnTime = 30000;
    bool reconnEnable = true;
    int timeOut = 50000;
    bool reconnFlag = false;
    bool connFlag = true;
    std::string certPath = "";
};
