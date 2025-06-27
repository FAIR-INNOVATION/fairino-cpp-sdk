#ifndef   NETWORK_H_
#define   NETWORK_H_

#ifdef WIN32
    #include <winsock2.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
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

#include <iostream>
#include "XmlRpc.h"
#include "logger.h"
using namespace std;

//将跨平台网络调用封装起来;
//这一块可以考虑引入一个第三方库;
namespace fr_network
{
#ifdef WIN32
    typedef SOCKET socket_fd;
#else
    typedef int socket_fd;
#endif

    socket_fd get_socket_fd();

    void close_fd(socket_fd fd);

    int connect(socket_fd fd, const char *_robot_ip, int port);

    socket_fd get_socket_fd()
    {
#ifdef WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
        socket_fd fd = socket(AF_INET, SOCK_STREAM, 0);
        return fd;
    }

    void close_fd(socket_fd fd)
    {
#ifdef WIN32
        closesocket(fd);
        WSACleanup();
#else
        close(fd);
#endif
    }

    int connect(socket_fd fd, const char *_robot_ip, int port)
    {
        if (nullptr == _robot_ip)
        {
            //logger_error("ip can not be nullptr");
            return -1;
        }
        
#ifdef WIN32
        //logger_info("fd is: %u, robot ip is: %s, port is: %d.", fd, _robot_ip, port);
        sockaddr_in _servaddr;
        memset(&_servaddr, 0, sizeof(_servaddr));
        _servaddr.sin_family = AF_INET;
        _servaddr.sin_addr.S_un.S_addr = inet_addr(_robot_ip); /// 服务器ip;
        _servaddr.sin_port = htons(port);                      /// 服务器端口;
#else
        //logger_info("fd is: %d, robot ip is: %s, port is: %d.", fd, _robot_ip, port);
        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);                 /// 服务器端口;
        servaddr.sin_addr.s_addr = inet_addr(_robot_ip); /// 服务器ip;
#endif

#ifdef WIN32
        /// 连接服务器，成功返回0，错误返回-1;
        if (connect(fd, (SOCKADDR *)&_servaddr, sizeof(SOCKADDR)) < 0)
#else
        if (connect(fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
#endif
        {
            
            #ifdef WIN32
            int connectError = WSAGetLastError();
            logger_error("connect fail, %d.", connectError);
            #else
                logger_error("connect fail, %s.", strerror(errno));
            #endif
            return -1;
        }
        return 0;
    }

}

#endif