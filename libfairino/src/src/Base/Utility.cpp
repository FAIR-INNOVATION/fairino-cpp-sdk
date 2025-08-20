#include "Utility.h"
#include "logger.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>

#ifdef WIN32
#include <Shlwapi.h>
#endif

using namespace std;

#define MAX_FILE_PATH_LENGTH 512

// 跨平台获取文件大小的函数
std::streampos GetFileSize(const std::string& filePath) 
{
#ifdef _WIN32
    // Windows平台实现
    WIN32_FILE_ATTRIBUTE_DATA fileInfo;
    if (GetFileAttributesExA(filePath.c_str(), GetFileExInfoStandard, &fileInfo)) {
        LARGE_INTEGER size;
        size.HighPart = fileInfo.nFileSizeHigh;
        size.LowPart = fileInfo.nFileSizeLow;
        return size.QuadPart;
    }
#else
    // Unix-like平台实现(Linux, macOS等)
    struct stat statResult;
    if (stat(filePath.c_str(), &statResult) == 0) {
        return statResult.st_size;
    }
#endif

    // 如果获取失败，返回-1
    return -1;
}

//从文件路径种取文件名称
std::string GetFileNameInPath(std::string filePath)
{
    string point_table_name;
#ifdef WIN32
    point_table_name = PathFindFileNameA(filePath.c_str());
#else
    point_table_name.append(basename(filePath.c_str()));
    logger_info("name is: %s.", point_table_name.c_str());
#endif

    return point_table_name;
}

//检查本地文件是否存在
bool CheckFileIsExist(std::string filePath)
{
#ifdef WIN32
    std::wstring save_path_wide(filePath.begin(), filePath.end());
    if (GetFileAttributesA(filePath.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        return false;
    }
#else
    char save_path[MAX_FILE_PATH_LENGTH];
    memset(save_path, 0, MAX_FILE_PATH_LENGTH);
    snprintf(save_path, MAX_FILE_PATH_LENGTH, "%s", filePath.c_str());
    if (access(save_path, F_OK) != 0)
    {
        logger_error("path %s do not exist.", save_path);
        return false;
    }
#endif

    return true;
}
