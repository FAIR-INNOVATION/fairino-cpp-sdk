/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: It is the configure head file for this library.
 * Created on: 2015-07-30
 */

#ifndef _ELOG_CFG_H_
#define _ELOG_CFG_H_

#define windows_option

/* enable log output. default open this macro */
#define ELOG_OUTPUT_ENABLE
/* enable log write file. default open this macro */
#define ELOG_FILE_ENABLE

/* 输出日志到终端显示 */
#define ELOG_TERMINAL_ENABLE

/* enable flush file cache. default open this macro */
#define ELOG_FILE_FLUSH_CACHE_ENABLE
/* setting static output log level */
#define ELOG_OUTPUT_LVL                      ELOG_LVL_VERBOSE
/* enable assert check */
#define ELOG_ASSERT_ENABLE
/* buffer size for every line's log */
#define ELOG_LINE_BUF_SIZE                   512
/* output line number max length */
#define ELOG_LINE_NUM_MAX_LEN                5
/* output filter's tag max length */
#define ELOG_FILTER_TAG_MAX_LEN              16
/* output filter's keyword max length */
#define ELOG_FILTER_KW_MAX_LEN               16
/* output filter's tag level max num */
#define ELOG_FILTER_TAG_LVL_MAX_NUM          5
/* output newline sign */
#define ELOG_NEWLINE_SIGN                    "\n"

/* 如果需要异步输出，这4个需要全部开启 */
/* enable asynchronous output mode */
#define ELOG_ASYNC_OUTPUT_ENABLE

/* the highest output level for async mode, other level will sync output */
#define ELOG_ASYNC_OUTPUT_LVL                ELOG_LVL_ASSERT

/* buffer size for asynchronous output mode */
#define ELOG_ASYNC_OUTPUT_BUF_SIZE           (ELOG_LINE_BUF_SIZE * 50)

/* asynchronous output mode using POSIX pthread implementation */
/* 异步模式下，posix线程库和win线程库任选其一 */

#ifdef WIN32
    #define ELOG_ASYNC_OUTPUT_USING_WINTHREAD
#else 
    #define ELOG_ASYNC_OUTPUT_USING_PTHREAD
#endif

/* 如果需要缓冲输出，这两个需要全部开启 */
#define ELOG_BUF_OUTPUT_ENABLE
#define ELOG_BUF_OUTPUT_BUF_SIZE 1024*50

#endif /* _ELOG_CFG_H_ */
