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
#include <cstring>
extern "C"
{
#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/error.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "mbedtls/timing.h"    
}

#include <iostream>
#include "XmlRpc.h"
#include "logger.h"
#include <string>
#include <mutex>

#ifdef WINDOWS_OPTION
#define FR_LIB_MTLS_EXPORT __declspec(dllexport)
#else
#define FR_LIB_MTLS_EXPORT
#endif

class FR_LIB_MTLS_EXPORT MTLSClient
{
public:
    MTLSClient(std::string IP, int port);
    ~MTLSClient();

    int Connect();
    int ReConnect();
    void ResetConnection();
    int Send(char* sendBuf, int sendSize);

    int Recv(char* recvBuf, int recvSize);

    int RecvFrame(char* recvBuf, int recvSize);
    int Close();

    int SetIpConfig(std::string IP);
    int SetPortConfig(int port);
    int SetReConnectParam(bool enable, int reconnectTime = 30000, int period = 50);
    bool GetReConnectEnable();
    bool GetReConnState();
    int SetTLSCertPath(std::string path);

    std::string robotIP = "";
    int robotPort = 0;
    std::string CA_CERT_FILE = "ca.crt";
    std::string CLIENT_CERT_FILE = "client.crt";
    std::string CLIENT_KEY_FILE = "client.key";

    std::string pers = "fairino_mtls";

    mbedtls_net_context      net;
    mbedtls_ssl_context      ssl;
    mbedtls_ssl_config       ssl_cfg;
    mbedtls_entropy_context  entropy;
    mbedtls_ctr_drbg_context ctr_drbg;

    mbedtls_x509_crt ca_crt;
    mbedtls_x509_crt client_crt;
    mbedtls_pk_context client_pk;
    std::string certPath = "";
    int reConnTime = 30000;        //30000 ms
    bool reconnEnable = true;      //重连使能
    int timeOut = 50000;           // 默认1s
    bool reconnFlag = false;
    bool connFlag = true;

    bool inited = false;   
    std::mutex sslMutex;
};