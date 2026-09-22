#include "mTLSClient.h"
#include "logger.h"
#include <iostream>
#include <cstring>


MTLSClient::MTLSClient(std::string IP, int port)
{
	robotIP = IP;
	robotPort = port;

	mbedtls_net_init(&net);
	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&ssl_cfg);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_x509_crt_init(&ca_crt);
	mbedtls_x509_crt_init(&client_crt);
	mbedtls_pk_init(&client_pk);
}

MTLSClient::~MTLSClient()
{
    Close();
}

int MTLSClient::Connect()
{
    ResetConnection();   // 关键：先清掉上一次的连接性资源
    
    int rtn = 0;
    
    // ===== 一次性资源：只初始化一次 =====
    if (!inited)
    {
        rtn = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
            (const unsigned char*)(pers.data()), pers.size());
        if (rtn != 0) { logger_error("mbedtls_ctr_drbg_seed failed, %d", rtn); return -1; }

        rtn = mbedtls_x509_crt_parse_file(&ca_crt, (const char*)(CA_CERT_FILE.data()));
        if (rtn != 0) { logger_error("parse CA root cert file, %d", rtn); return -3; }

        rtn = mbedtls_x509_crt_parse_file(&client_crt, (const char*)(CLIENT_CERT_FILE.data()));
        if (rtn != 0) { logger_error("parse client cert file, %d", rtn); return -4; }

        rtn = mbedtls_pk_parse_keyfile(&client_pk, (const char*)(CLIENT_KEY_FILE.data()),
            nullptr, mbedtls_ctr_drbg_random, &ctr_drbg);
        if (rtn != 0) { logger_error("parse client private key file, %d", rtn); return -5; }

        inited = true;
    }

    // ===== 每次 Connect 都要做的配置 =====
    rtn = mbedtls_ssl_config_defaults(&ssl_cfg,
        MBEDTLS_SSL_IS_CLIENT,
        MBEDTLS_SSL_TRANSPORT_STREAM,
        MBEDTLS_SSL_PRESET_DEFAULT);
    if (rtn != 0) { logger_error("ssl_config_defaults, %d", rtn); return -2; }

    mbedtls_ssl_conf_rng(&ssl_cfg, mbedtls_ctr_drbg_random, &ctr_drbg);
    mbedtls_ssl_conf_ca_chain(&ssl_cfg, &ca_crt, nullptr);
    mbedtls_ssl_conf_authmode(&ssl_cfg, MBEDTLS_SSL_VERIFY_REQUIRED);

    rtn = mbedtls_ssl_conf_own_cert(&ssl_cfg, &client_crt, &client_pk);
    if (rtn != 0) { logger_error("ssl_conf_own_cert, %d", rtn); return -6; }

    // ===== 连接性资源 =====
    rtn = mbedtls_ssl_setup(&ssl, &ssl_cfg);
    if (rtn != 0) { logger_error("ssl_setup, %d", rtn); return -7; }

    const char* cert_cn = "FR-Robot-server";
    mbedtls_ssl_set_hostname(&ssl, cert_cn);

    rtn = mbedtls_net_connect(&net, (const char*)(robotIP.data()),
        std::to_string(robotPort).c_str(),
        MBEDTLS_NET_PROTO_TCP);
    if (rtn != 0) { logger_error("net_connect, %d", rtn); return -8; }

    mbedtls_ssl_set_bio(&ssl, &net, mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout);

    // TLS握手（双向证书校验就在握手内部完成）
    rtn = mbedtls_ssl_handshake(&ssl);
    if (rtn != 0)
    {
        // ★ 关键：握手失败也要看证书校验结果
        uint32_t flags = mbedtls_ssl_get_verify_result(&ssl);
        if (flags != 0)
        {
            char msg[512];
            mbedtls_x509_crt_verify_info(msg, sizeof(msg), "  cert verify: ", flags);
            std::cerr << msg << "\n";
        }
        else
        {
            std::cerr << "  verify result: no flag\n";
        }
        return -9;
    }

    // 主动检查服务端证书校验结果
    uint32_t verify_flags = mbedtls_ssl_get_verify_result(&ssl);
    if (verify_flags != 0)
    {
        char msg[512];
        mbedtls_x509_crt_verify_info(msg, sizeof(msg), "Server cert verify error: ", verify_flags);
        std::cerr << msg << "\n";
        return -10;
    }

    return 0;
}
int MTLSClient::ReConnect()
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
            logger_error("mTLS pord %d reconnect to Robot fail %d/%d, error code is %d", robotPort, i, maxConnTimes, connRtn);
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

int MTLSClient::Send(char* sendBuf, int sendSize)
{
    if (sendBuf == nullptr || sendSize <= 0)
    {
        logger_error("Send invalid args, buf=%p, size=%d", sendBuf, sendSize);
        return -1;
    }

    //std::lock_guard<std::mutex> lock(sslMutex);

    int total = sendSize;
    int sent = 0;

    while (sent < total)
    {
        int rtn = mbedtls_ssl_write(&ssl,
            (const unsigned char*)(sendBuf)+sent,
            total - sent);

        // 非错误：重试
        if (rtn == MBEDTLS_ERR_SSL_WANT_READ ||
            rtn == MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            continue;
        }

        // 真错误
        if (rtn <= 0)
        {
            logger_error("ssl_write failed, %d, sent=%d/%d", rtn, sent, total);
            return rtn;
        }

        // 部分写：累加继续
        sent += rtn;
    }

    return sent;   // == sendSize
}

int MTLSClient::Recv(char* recvBuf, int recvSize)
{
    if (recvBuf == nullptr || recvSize <= 0)
    {
        logger_error("Recv invalid args, buf=%p, size=%d", recvBuf, recvSize);
        return -1;
    }
    mbedtls_ssl_conf_read_timeout(&ssl_cfg, 500);
    //std::lock_guard<std::mutex> lock(sslMutex);
    int rtn = mbedtls_ssl_read(&ssl, (unsigned char*)recvBuf, recvSize);
    //std::cout << "mtls recv length " << rtn << std::endl;
    if (rtn > 0)
    {
        return rtn;
    }

    // 暂时没数据 / 需要先写：不算错误，返回 0 让上层重试
    if (rtn == MBEDTLS_ERR_SSL_WANT_READ ||
        rtn == MBEDTLS_ERR_SSL_WANT_WRITE ||
        rtn == MBEDTLS_ERR_SSL_TIMEOUT)
    {
        return 0;
    }

    // 对端发 close_notify：TLS 层正常关闭
    if (rtn == 0 || rtn == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
    {
        logger_error("ssl_read peer close, %d", rtn);
        return -1;   // 断线，交给上层重连
    }

    // 其他负值：真错误
    logger_error("ssl_read failed, %d", rtn);
    return -1;
}

int MTLSClient::RecvFrame(char* recvBuf, int recvSize)
{
    int tmpRecvSize = Recv(recvBuf, recvSize);
    if (tmpRecvSize == 0)
    {
        return 0;
    }
    else if(tmpRecvSize < 0)
    {
        if (!connFlag)
        {
            return 0;
        }

        logger_error("robot get cmd mtls rpy frame failed");

        if (reconnEnable == false)  //没有使能重连
        {
            return -1;
        }
        logger_error("robot mtls start reconnect");
        bool reconnectSuccess = ReConnect();
        if (reconnectSuccess)
        {
            logger_error("port mtls reconnect success");
            tmpRecvSize = RecvFrame(recvBuf, recvSize);
            return tmpRecvSize;
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return tmpRecvSize;
    }
}


int MTLSClient::Close()
{
    connFlag = false;

    // 只清连接性资源
    ResetConnection();

    // 一次性资源：整个对象生命周期只释放一次
    mbedtls_x509_crt_free(&ca_crt);
    mbedtls_x509_crt_free(&client_crt);
    mbedtls_pk_free(&client_pk);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    inited = false;
    return 0;
}


int MTLSClient::SetIpConfig(std::string IP)
{
    robotIP = IP;
    return 0;
}

int MTLSClient::SetPortConfig(int port)
{
    robotPort = port;
    return 0;
}

int MTLSClient::SetReConnectParam(bool enable, int reconnectTime, int period)
{
    this->reConnTime = reconnectTime;
    this->reconnEnable = enable;
    this->timeOut = period;
    return 0;
}

bool MTLSClient::GetReConnectEnable()
{
    return this->reconnEnable;
}

//判断当前是否正在重连
bool MTLSClient::GetReConnState()
{
    return reconnFlag;
}

void MTLSClient::ResetConnection()
{
    mbedtls_net_free(&net);
    mbedtls_ssl_free(&ssl);
    mbedtls_ssl_config_free(&ssl_cfg);

    mbedtls_net_init(&net);
    mbedtls_ssl_init(&ssl);
    mbedtls_ssl_config_init(&ssl_cfg);
}

int MTLSClient::SetTLSCertPath(std::string path)
{
    certPath = path;
    std::string CA_CERT_FILE = certPath + "ca.crt";
    std::string CLIENT_CERT_FILE = certPath + "client.crt";
    std::string CLIENT_KEY_FILE = certPath + "client.key";
    return 0;
}