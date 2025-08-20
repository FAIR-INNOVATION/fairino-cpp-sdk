#pragma once

#include <iostream>
#include <fstream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

// ��ƽ̨��ȡ�ļ���С�ĺ���
std::streampos GetFileSize(const std::string& filePath);

//���ļ�·����ȡ�ļ�����
std::string GetFileNameInPath(std::string path);

//��鱾���ļ��Ƿ����
bool CheckFileIsExist(std::string path);
