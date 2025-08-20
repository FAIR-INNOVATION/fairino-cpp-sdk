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

// 跨平台获取文件大小的函数
std::streampos GetFileSize(const std::string& filePath);

//从文件路径种取文件名称
std::string GetFileNameInPath(std::string path);

//检查本地文件是否存在
bool CheckFileIsExist(std::string path);
