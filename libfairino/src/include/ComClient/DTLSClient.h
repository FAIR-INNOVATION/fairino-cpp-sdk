#pragma once
#include <atomic>
#include <mutex>
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
#include "XmlRpc.h"
#include "logger.h"
#include "FRUDPClientBase.h"

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


#ifdef WINDOWS_OPTION
#define FR_LIB_DTLS_EXPORT __declspec(dllexport)
#else
#define FR_LIB_DTLS_EXPORT
#endif

class FR_LIB_DTLS_EXPORT DTLSClient : public FRUDPClientBase
{
public:
    DTLSClient();
    ~DTLSClient() override;

    // FRUDPClientBase overrides
    int Connect(std::string IP, int port) override;
    int Close() override;
    void RobotUDPCmdRecvThread() override;
    int SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, std::string)) override;
    int SendFrame(std::string sendFrame) override;
    int SetTLSCertPath(std::string path) override;
    bool CheckCertExists() override;

    // DTLSClient-specific functions
    int Connect(std::string IP, int port, bool startRecvThread);
    int ReConnect();
    void ResetConnection();
    int RecvFrame(std::string& recvFrame);

    int reConnTime = 30000;        //30000 ms
    bool reconnEnable = true;      //重连使能
    int timeOut = 50000;           // 默认1s
    std::atomic<bool> reconnFlag{false};
    bool connFlag = true;
    std::mutex sslMutex;

    std::string CA_CERT_FILE = "ca.crt";
    std::string CLIENT_CERT_FILE = "client.crt";
    std::string CLIENT_KEY_FILE = "client.key";
    std::string CERT_CN = "FR-Robot-server";  // 服务端证书 CN
    std::string pers = "fairino_dtls";
    mbedtls_net_context          net;
    mbedtls_ssl_context          ssl;
    mbedtls_ssl_config           ssl_cfg;
    mbedtls_entropy_context      entropy;
    mbedtls_ctr_drbg_context     ctr_drbg;
    mbedtls_timing_delay_context timer;  

    mbedtls_x509_crt ca_crt;
    mbedtls_x509_crt client_crt;
    mbedtls_pk_context client_pk;
    bool inited = false;   // 一次性资源是否已初始化
    std::thread recvThread;
    bool recvThreadRunning = false;
};
