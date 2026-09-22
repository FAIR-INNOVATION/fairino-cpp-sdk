#include "FRUdpClient.h"
#include "FrameHandle.h"
#include "DTLSClient.h"
#include "logger.h"
#include "Utility.h"
#include <cstring>


using namespace std;

DTLSClient::DTLSClient()
{
	mbedtls_net_init(&net);
	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&ssl_cfg);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_x509_crt_init(&ca_crt);
	mbedtls_x509_crt_init(&client_crt);
	mbedtls_pk_init(&client_pk);
}

DTLSClient::~DTLSClient()
{
	Close();
}

int DTLSClient::Connect(std::string IP, int port, bool startRecvThread)
{
	robotIP = IP;
	robotPort = port;

	ResetConnection();

	// 1. 一次性资源：只初始化一次
	if (!inited)
	{
		// 1.1 随机数种子
		int rtn = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char*)pers.c_str(), pers.size());
		if (rtn < 0)
		{
			logger_error("ctr_drbg_seed, %d", rtn);
			return -1;
		}

		// 1.2 加载 CA 根证书
		rtn = mbedtls_x509_crt_parse_file(&ca_crt, (const char*)(CA_CERT_FILE.data()));
		if (rtn < 0)
		{
			logger_error("parse CA root cert file, %d", rtn);
			return -3;
		}

		// 1.3 加载客户端证书
		rtn = mbedtls_x509_crt_parse_file(&client_crt, (const char*)(CLIENT_CERT_FILE.data()));
		if (rtn < 0)
		{
			logger_error("parse client cert file, %d", rtn);
			return -4;
		}

		// 1.4 加载客户端私钥
		rtn = mbedtls_pk_parse_keyfile(&client_pk, (const char*)(CLIENT_KEY_FILE.data()),	nullptr, mbedtls_ctr_drbg_random, &ctr_drbg);
		if (rtn < 0)
		{
			logger_error("parse client private key file, %d", rtn);
			return -5;
		}

		inited = true;
	}

	// 2. ★ DTLS 客户端配置：TRANSPORT_DATAGRAM
	int rtn = mbedtls_ssl_config_defaults(&ssl_cfg,	MBEDTLS_SSL_IS_CLIENT,	MBEDTLS_SSL_TRANSPORT_DATAGRAM,	MBEDTLS_SSL_PRESET_DEFAULT);
	if (rtn < 0)
	{
		logger_error("ssl_config_defaults, %d", rtn);
		return -2;
	}

	mbedtls_ssl_conf_rng(&ssl_cfg, mbedtls_ctr_drbg_random, &ctr_drbg);

	// ★ 可选：调整 DTLS 握手超时（初始 ms，最大 ms）
	mbedtls_ssl_conf_handshake_timeout(&ssl_cfg, 1000, 5000);

	// 3. 把 CA 挂到 ssl_cfg（每次都要做，ssl_cfg 被 ResetConnection 清过了）
	mbedtls_ssl_conf_ca_chain(&ssl_cfg, &ca_crt, nullptr);
	mbedtls_ssl_conf_authmode(&ssl_cfg, MBEDTLS_SSL_VERIFY_REQUIRED);

	// 4. 把客户端证书和私钥挂到 ssl_cfg（每次都要做）
	rtn = mbedtls_ssl_conf_own_cert(&ssl_cfg, &client_crt, &client_pk);
	if (rtn < 0)
	{
		logger_error("ssl_conf_own_cert, %d", rtn);
		return -6;
	}

	// 5. SSL setup
	rtn = mbedtls_ssl_setup(&ssl, &ssl_cfg);
	if (rtn < 0)
	{
		logger_error("ssl_setup, %d", rtn);
		return -7;
	}

	// 6. 主机名校验（必须是服务端证书里的 CN）
	rtn = mbedtls_ssl_set_hostname(&ssl, (const char*)(CERT_CN.data()));
	if (rtn < 0)
	{
		logger_error("ssl_set_hostname, %d", rtn);
		return -8;
	}

	// 7. ★ UDP 连接：MBEDTLS_NET_PROTO_UDP
	//rtn = mbedtls_net_connect(&net, (const char*)(robotIP.data()),
	//	std::to_string(robotPort).c_str(),
	//	MBEDTLS_NET_PROTO_UDP);
	//if (rtn < 0)
	//{
	//	logger_error("net_connect, %d", rtn);
	//	return -9;
	//}
	//std::cout << "UDP connect ok (port=" << port << ")\n";
#ifdef WIN32
	WSADATA wsaData;
	int wsa_ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (wsa_ret != 0) 
	{
		std::cerr << "WSAStartup failed: " << wsa_ret << "\n";
		return -9;
	}
#endif
	int fd = socket(AF_INET, SOCK_DGRAM, 0);


	struct sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(20008);            // ← 本地端口,DTLS要求必须绑定本地端口，否则只能连接一次
	addr.sin_addr.s_addr = htonl(INADDR_ANY);  // ← 0.0.0.0
	if (::bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0)
	{
		#ifdef WIN32
		closesocket(fd);
		#else
		close(fd);
		#endif

		return -14;
	}

	addr.sin_port = htons(robotPort);         // ← 服务端端口
	if (inet_pton(AF_INET, robotIP.c_str(), &addr.sin_addr) != 1) 
	{
		#ifdef WIN32
		closesocket(fd);
		#else
		close(fd);
		#endif

		return -15;
	}
	if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) 
	{
		#ifdef WIN32
		closesocket(fd);
		#else
		close(fd);
		#endif
		return -16;
	}

	net.fd = fd;   // 交给 mbedtls，后面 net_free 会自动 close

#ifdef WIN32
	DWORD tv = 500;
	if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) != 0) 
	{
		std::cerr << "setsockopt SO_RCVTIMEO failed, WSA err="<< WSAGetLastError() << "\n";
		return -10;
	}
#else
	struct timeval tv;
	tv.tv_sec = timeout / 1000;
	tv.tv_usec = (timeout % 1000) * 1000;
	if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,	(const char*)&tv, sizeof(tv)) != 0)
	{
		std::cerr << "setsockopt SO_RCVTIMEO failed, errno=" << errno << " (" << strerror(errno) << ")\n";
		return -10;
	}
#endif

	// 8. ★ 绑定 BIO：UDP 必须带 recv_timeout（DTLS 重传靠它）
	mbedtls_ssl_set_bio(&ssl, &net,	mbedtls_net_send, mbedtls_net_recv,	mbedtls_net_recv_timeout);

	// 9. ★ DTLS 必需的定时器回调（重传/超时管理）
	mbedtls_ssl_set_timer_cb(&ssl, &timer,	mbedtls_timing_set_delay, mbedtls_timing_get_delay);

	// 10. ★ DTLS 握手：可能返回 WANT_READ/WANT_WRITE，要重试
	while ((rtn = mbedtls_ssl_handshake(&ssl)) != 0)
	{
		if (!runFlag)
		{
			return -12;   // 主动退出
		}
		if (rtn != MBEDTLS_ERR_SSL_WANT_READ &&
			rtn != MBEDTLS_ERR_SSL_WANT_WRITE)
		{
			logger_error("ssl_handshake, %d", rtn);

			uint32_t flags = mbedtls_ssl_get_verify_result(&ssl);
			if (flags != 0)
			{
				char msg[512];
				mbedtls_x509_crt_verify_info(msg, sizeof(msg), "  cert verify: ", flags);
				std::cerr << msg << "\n";
			}
			return -13;
		}
	}

	// 11. 主动检查服务端证书
	uint32_t verify_flags = mbedtls_ssl_get_verify_result(&ssl);
	if (verify_flags != 0)
	{
		char msg[512];
		mbedtls_x509_crt_verify_info(msg, sizeof(msg), "Server cert verify error: ", verify_flags);
		std::cerr << msg << "\n";
		return -11;
	}

	if (startRecvThread && !recvThreadRunning)
	{
		runFlag = true;
		thread recvThreadA(&DTLSClient::RobotUDPCmdRecvThread, this);
		recvThreadA.detach();
		recvThreadRunning = true;
	}

	return 0;
}

int DTLSClient::Close()
{
	runFlag = false;

	// 1. 先让接收线程退出
	//if (recvThreadRunning && recvThread.joinable())
	//{
	//	recvThread.join();
	//	recvThreadRunning = false;
	//}

	mbedtls_ssl_close_notify(&ssl);
	mbedtls_net_free(&net);
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&ssl_cfg);

	mbedtls_x509_crt_free(&ca_crt);
	mbedtls_x509_crt_free(&client_crt);
	mbedtls_pk_free(&client_pk);

	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);

	return 0;
}

void DTLSClient::RobotUDPCmdRecvThread()
{
	int nRecvNum = 0;

	bool firstData = true;

	char recvBuf[2048] = { 0 };

	while (runFlag)
	{
		

		string recvFrameStr = {0};
		std::string buf;
		int rtn = RecvFrame(recvFrameStr);
		//std::cout << "udp 20007 recv frame, rtn is " << rtn << endl;;
		if (rtn == 0)
		{
			_Sleep(1);
			continue;
		}
		else if (rtn < 0)
		{
			//std::cerr << "DTLS recv failed to reconnect..." << std::endl;
			rtn = ReConnect();
			if (rtn == 0)
			{
				_Sleep(1);
				continue;
			}
			else 
			{
				std::cout << "DTLS reconnect failed\n";
				break;
			}
		}

		std::vector<std::string> allFrames = SplitFrame(recvFrameStr);
		for (int i = 0; i < static_cast<int>(allFrames.size()); i++)
		{
			FRAME recvFrame = UnpacketFrame(allFrames[i]);

			if (UdpRecvFrameCallBack == nullptr)
			{
				std::cout << "UdpRecvFrameCallBack == nullptr\n";
				continue;
			}

			try
			{
				UdpRecvFrameCallBack(COM_UDP, recvFrame.count, recvFrame.cmdID, recvFrame.contentLen, recvFrame.content);
				cout << "recv a frame" << endl;
			}
			catch (...)
			{
				logger_error("Catch error in callback! we now set disable callback funtion! ");
				UdpRecvFrameCallBack = nullptr;
			}
		}
	}
}

int DTLSClient::SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, string))
{
	if (CallBack == nullptr)
	{
		return 1;
	}
	UdpRecvFrameCallBack = CallBack;
	return 0;
}

int DTLSClient::SendFrame(string sendFrame)
{
	std::lock_guard<std::mutex> lock(sslMutex);
	size_t total = sendFrame.size();
	size_t sent = 0;
	while (sent < total)
	{
		int rtn = mbedtls_ssl_write(&ssl, reinterpret_cast<const unsigned char*>(sendFrame.data()) + sent,	total - sent);
		if (rtn == MBEDTLS_ERR_SSL_WANT_READ || rtn == MBEDTLS_ERR_SSL_WANT_WRITE)
		{
			continue;
		}
			
		if (rtn <= 0)
		{
			logger_error("dtls ssl_write, %d", rtn);
			return rtn;
		}
		sent += rtn;
	}

	return 0;
}

int DTLSClient::RecvFrame(std::string& recvFrame)
{
	mbedtls_ssl_conf_read_timeout(&ssl_cfg, 3);
	//std::cout << "udp 20007 recv frame222\n";
	std::lock_guard<std::mutex> lock(sslMutex);
	uint8_t recv_buf[2048];
	int rtn = mbedtls_ssl_read(&ssl, recv_buf, sizeof(recv_buf) - 1);
	//cout << "dtls recv length " << rtn << endl;
	// 1) 收到数据
	if (rtn > 0)
	{
		recvFrame.assign(reinterpret_cast<char*>(recv_buf), rtn);
		return rtn;
	}

	// 2) 暂时没数据 / 读超时：不算断线，交给上层累计判断
	if (rtn == MBEDTLS_ERR_SSL_WANT_READ ||
		rtn == MBEDTLS_ERR_SSL_WANT_WRITE ||
		rtn == MBEDTLS_ERR_SSL_TIMEOUT)
	{
		return 0;
	}

	// 3) 明确的对端关闭
	if (rtn == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
	{
		std::cout << "peer send close notify\n";
		return -1;
	}

	// 4) rtn == 0：底层传输已关闭，必须重连
	if (rtn == 0)
	{
		std::cout << "ssl_read returned 0, transport closed\n";
		return -1;
	}

	// 5) 其余负错误（如 MBEDTLS_ERR_NET_RECV_FAILED）
	logger_error("dtls ssl_read, %d", rtn);
	return -1;
}


int DTLSClient::ReConnect()
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
		int connRtn = Connect(robotIP, robotPort, false);
		if (connRtn != 0)
		{
			logger_error("DTLSClient pord %d reconnect to Robot fail %d/%d, error code is %d", robotPort, i, maxConnTimes, connRtn);
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

void DTLSClient::ResetConnection()
{
	std::lock_guard<std::mutex> lock(sslMutex);
	mbedtls_net_free(&net);
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&ssl_cfg);

	mbedtls_net_init(&net);
	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&ssl_cfg);
}

int DTLSClient::SetTLSCertPath(std::string path)
{
	certPath = path;
	std::string CA_CERT_FILE = certPath + "ca.crt";
	std::string CLIENT_CERT_FILE = certPath + "client.crt";
	std::string CLIENT_KEY_FILE = certPath + "client.key";
	return 0;
}