#include "logger.h"
int fr_logger::logger_init(bool enable_async_output, bool enable_buf_output, int lvl)
{
    setbuf(stdout, NULL);
    set_enable_buffer_output(enable_buf_output);
    set_enable_async_output(enable_async_output);
    elog_init();

    switch (lvl)
    {
    case ELOG_LVL_ERROR:
        elog_set_filter_lvl(ELOG_LVL_ERROR);
        break;
    case ELOG_LVL_WARN:
        elog_set_filter_lvl(ELOG_LVL_WARN);
        break;
    case ELOG_LVL_INFO:
        elog_set_filter_lvl(ELOG_LVL_INFO);
        break;
    case ELOG_LVL_DEBUG:
        elog_set_filter_lvl(ELOG_LVL_DEBUG);
        break;
    default:
        /* 默认 error 模式*/
        elog_set_filter_lvl(ELOG_LVL_ERROR);
        break;
    }

    /* output format */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);

    elog_start();

    return 0;
}

void fr_logger::set_logger_level(int lvl)
{
    switch (lvl)
    {
    case ELOG_LVL_ERROR:
        elog_set_filter_lvl(ELOG_LVL_ERROR);
        break;
    case ELOG_LVL_WARN:
        elog_set_filter_lvl(ELOG_LVL_WARN);
        break;
    case ELOG_LVL_INFO:
        elog_set_filter_lvl(ELOG_LVL_INFO);
        break;
    case ELOG_LVL_DEBUG:
        elog_set_filter_lvl(ELOG_LVL_DEBUG);
        break;
    default:
        /* 默认 error 模式*/
        elog_set_filter_lvl(ELOG_LVL_ERROR);
        break;
    }
}

void fr_logger::logger_deinit()
{
    logger_buf_last(" ");
    elog_stop();
    elog_deinit();
}

// extern int set_elog_file_init_param(int max_rotate, const char* pname);
int fr_logger::set_file_param(std::string file_path, int file_num)
{
    int retval = 0;
    /* 超限则取上下限 */
    if (file_num < 1)
    {
        file_num = 1;
    }
    else if (file_num > 20)
    {
        file_num = 20;
    }

    /* 为空则使用默认名称，保存到可执行文件所在路径；否则检查文件名称格式，长度  */
    if (file_path.length() == 0)
    {
        std::cout << "log file will be saved into execute path, name will be: fairino.log \n";
        file_path = "fairino.log";
    }
    if (file_path.length() > 256)
    {
        std::cout << "length of file path should less than 256 bytes \n";
        return -1;
    }
    std::size_t pos = file_path.rfind(".log");
    if (std::string::npos == pos)
    {
        std::cout << "file name should be xxx.log \n";
        return -1;
    }

    retval = set_elog_file_init_param(file_num, file_path.c_str());
    if (0 != retval)
    {
        std::cout << "set log file param failed. \n";
        return -1;
    }

    return 0;
}
