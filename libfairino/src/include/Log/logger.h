#ifndef __logger_h__
#define __logger_h__

#include "elog.h"
#include <iostream>
#include <string>

#define ELOG_LVL_ERROR 1
#define ELOG_LVL_WARN 2
#define ELOG_LVL_INFO 3
#define ELOG_LVL_DEBUG 4

#ifdef WIN32
    #define _WINDOWS
#endif

/* 日志入口；
 * 不要直接调用elog系列函数的入口：该模块耦合性强，不适宜直接嵌入C++工程中
 */
namespace fr_logger
{

    int logger_init(bool enable_async_output, bool enable_buf_output, int lvl);

    /**
     * @brief 设置日志过滤等级;
     * @param lvl: 过滤等级值，值越小输出日志越少，默认值是1. 1-error, 2-warnning, 3-inform, 4-debug;
     *
     */
    void set_logger_level(int lvl);

/**
 * @brief 日志接口
 */
// #define LOG_TAG "fairino"
#define logger_error(...) elog_e(LOG_TAG, __VA_ARGS__)
#define logger_warn(...) elog_w(LOG_TAG, __VA_ARGS__)
#define logger_info(...) elog_i(LOG_TAG, __VA_ARGS__)
#define logger_debug(...) elog_d(LOG_TAG, __VA_ARGS__)
#define logger_buf_last(...) elog_v(LOG_TAG, __VA_ARGS__)
    /**
     * @brief 释放初始化的资源;
     */
    void logger_deinit();

    /**
     * @brief 设置日志文件参数
     */
    int set_file_param(std::string file_path, int file_num);

}
#endif /*__logger_h__*/
