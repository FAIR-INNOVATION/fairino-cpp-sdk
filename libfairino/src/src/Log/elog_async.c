/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2016-2017, Armink, <armink.ztl@gmail.com>
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
 * Function: Logs asynchronous output.
 * Created on: 2016-11-06
 */

#include <elog.h>
#include <string.h>
#include <stdio.h>

#ifdef ELOG_ASYNC_OUTPUT_ENABLE

#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
/* thread default stack size */
#ifndef ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE
#if PTHREAD_STACK_MIN > 4*1024
#define ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE     PTHREAD_STACK_MIN
#else
#define ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE     (1*1024)
#endif
/* thread default priority */
#ifndef ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY
#define ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY       (sched_get_priority_max(SCHED_RR) - 1)
#endif
/* output thread poll get log buffer size  */
#ifndef ELOG_ASYNC_LINE_OUTPUT
#ifndef ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE
#define ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE         (ELOG_ASYNC_OUTPUT_BUF_SIZE - 4)
#endif
#else
#ifndef ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE
#define ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE         (ELOG_LINE_BUF_SIZE - 4)
#endif
#endif
#endif /* ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE */

/* asynchronous output log notice */
static sem_t output_notice;
/* asynchronous output pthread thread */
static pthread_t async_output_thread;
#endif /* ELOG_ASYNC_OUTPUT_USING_PTHREAD */

/******************************************************/
/* 使用 win 提供的线程库 */
#ifdef ELOG_ASYNC_OUTPUT_USING_WINTHREAD
#include <windows.h>
/* thread default stack size */
#ifndef ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE
#if PTHREAD_STACK_MIN > 4*1024
#define ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE     PTHREAD_STACK_MIN
#else
#define ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE     (1*1024)
#endif
/* thread default priority */
#ifndef ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY
#define ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY       (sched_get_priority_max(SCHED_RR) - 1)
#endif
/* output thread poll get log buffer size  */
#ifndef ELOG_ASYNC_LINE_OUTPUT
#ifndef ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE
#define ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE         (ELOG_ASYNC_OUTPUT_BUF_SIZE - 4)
#endif
#else
#ifndef ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE
#define ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE         (ELOG_LINE_BUF_SIZE - 4)
#endif
#endif
#endif /* ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE */

/* asynchronous output log notice */
static HANDLE _async_notice_sem;
/* asynchronous output pthread thread */
static DWORD async_output_thread;

/* win下的线程句柄和线程id */
static HANDLE _async_thread;
static bool thread_running = false;

#endif /* ELOG_ASYNC_OUTPUT_USING_WINTHREAD */
/******************************************************/

/* the highest output level for async mode, other level will sync output */
#ifdef ELOG_ASYNC_OUTPUT_LVL
#define OUTPUT_LVL                               ELOG_ASYNC_OUTPUT_LVL
#else
#define OUTPUT_LVL                               ELOG_LVL_ASSERT
#endif /* ELOG_ASYNC_OUTPUT_LVL */

/* buffer size for asynchronous output mode */
#ifdef ELOG_ASYNC_OUTPUT_BUF_SIZE
#define OUTPUT_BUF_SIZE                          ELOG_ASYNC_OUTPUT_BUF_SIZE
#else
#define OUTPUT_BUF_SIZE                          (ELOG_LINE_BUF_SIZE * 10)
#endif /* ELOG_ASYNC_OUTPUT_BUF_SIZE */

/* Initialize OK flag */
static bool init_ok = false;
#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
/* thread running flag */
static bool thread_running = false;
#endif
/* asynchronous output mode enabled flag */
static bool is_enabled = false;
/* asynchronous output mode's ring buffer */
static char log_buf[OUTPUT_BUF_SIZE] = { 0 };
/* log ring buffer write index */
static size_t write_index = 0;
/* log ring buffer read index */
static size_t read_index = 0;
/* log ring buffer full flag */
static bool buf_is_full = false;
/* log ring buffer empty flag */
static bool buf_is_empty = true;

extern void elog_port_output(const char *log, size_t size);
extern void elog_output_lock(void);
extern void elog_output_unlock(void);

/**
 * asynchronous output ring buffer used size
 *
 * @return used size
 */
static size_t elog_async_get_buf_used(void) {
    if (write_index > read_index) {
        return write_index - read_index;
    } else {
        if (!buf_is_full && !buf_is_empty) {
            return OUTPUT_BUF_SIZE - (read_index - write_index);
        } else if (buf_is_full) {
            return OUTPUT_BUF_SIZE;
        } else {
            return 0;
        }
    }
}

/**
 * asynchronous output ring buffer remain space
 *
 * @return remain space
 */
static size_t async_get_buf_space(void) {
    return OUTPUT_BUF_SIZE - elog_async_get_buf_used();
}

/**
 * put log to asynchronous output ring buffer
 *
 * @param log put log buffer
 * @param size log size
 *
 * @return put log size, the log which beyond ring buffer space will be dropped
 */
static size_t async_put_log(const char *log, size_t size) {
    size_t space = 0;

    space = async_get_buf_space();
    /* no space */
    if (!space) {
        size = 0;
        goto __exit;
    }
    /* drop some log */
    if (space <= size) {
        size = space;
        buf_is_full = true;
    }

    /* 小于缓冲区，全部写入；大于缓冲区时，超过部分从缓冲区头开始写入，覆盖掉没有被读取的部分 */
    if (write_index + size < OUTPUT_BUF_SIZE) {
        memcpy(log_buf + write_index, log, size);
        write_index += size;
    } else {
        //printf("-----over write-----------.\n");
        memcpy(log_buf + write_index, log, OUTPUT_BUF_SIZE - write_index);
        memcpy(log_buf, log + OUTPUT_BUF_SIZE - write_index,
                size - (OUTPUT_BUF_SIZE - write_index));
        write_index += size - OUTPUT_BUF_SIZE;
    }

    buf_is_empty = false;

__exit:

    return size;
}

#ifdef ELOG_ASYNC_LINE_OUTPUT
/**
 * Get line log from asynchronous output ring buffer.
 * It will copy all log when the newline sign isn't find.
 *
 * @param log get line log buffer
 * @param size line log size
 *
 * @return get line log size, the log size is less than ring buffer used size
 */
size_t elog_async_get_line_log(char *log, size_t size) {
    size_t used = 0, cpy_log_size = 0;
    /* lock output */
    elog_output_lock();
    used = elog_async_get_buf_used();

    /* no log */
    if (!used || !size) {
        goto __exit;
    }
    /* less log */
    if (used <= size) {
        size = used;
    }

    if (read_index + size < OUTPUT_BUF_SIZE) {
        cpy_log_size = elog_cpyln(log, log_buf + read_index, size);
        read_index += cpy_log_size;
    } else {
        cpy_log_size = elog_cpyln(log, log_buf + read_index, OUTPUT_BUF_SIZE - read_index);
        if (cpy_log_size == OUTPUT_BUF_SIZE - read_index) {
            cpy_log_size += elog_cpyln(log + cpy_log_size, log_buf, size - cpy_log_size);
            read_index += cpy_log_size - OUTPUT_BUF_SIZE;
        } else {
            read_index += cpy_log_size;
        }
    }

    if (used == cpy_log_size) {
        buf_is_empty = true;
    }

    if (cpy_log_size) {
        buf_is_full = false;
    }

__exit:
    /* lock output */
    elog_output_unlock();
    return cpy_log_size;
}
#else
/**
 * get log from asynchronous output ring buffer
 *
 * @param log get log buffer
 * @param size log size
 *
 * @return get log size, the log size is less than ring buffer used size
 */
size_t elog_async_get_log(char *log, size_t size) {
    size_t used = 0;
    /* lock output */
    elog_output_lock();
    /* 使用缓冲区前先检查是否为空，用于处理sempost>>semwait的情况 */
    used = elog_async_get_buf_used();
    /* no log */
    if (!used || !size) {
        size = 0;
        goto __exit;
    }
    /* less log 取走了缓冲区中的全部日志 */
    if (used <= size) {
        size = used;
        buf_is_empty = true;
    }

    /* 如果已经读的 + 将要读的 = 缓冲区大小 + x，那么将缓冲区中剩下的全部读出去，再从缓冲区头开始读x，放入log中 */
    if (read_index + size < OUTPUT_BUF_SIZE) {
        memcpy(log, log_buf + read_index, size);
        read_index += size;
    } else {
        printf("-----------------async output-------------------\n");
        memcpy(log, log_buf + read_index, OUTPUT_BUF_SIZE - read_index);
        memcpy(log + OUTPUT_BUF_SIZE - read_index, log_buf,
                size - (OUTPUT_BUF_SIZE - read_index));
        read_index += size - OUTPUT_BUF_SIZE;
    }

    buf_is_full = false;

__exit:
    /* lock output */
    elog_output_unlock();
    return size;
}
#endif /* ELOG_ASYNC_LINE_OUTPUT */

void elog_async_output(uint8_t level, const char *log, size_t size) {
    /* this function must be implement by user when ELOG_ASYNC_OUTPUT_USING_PTHREAD is not defined */
    extern void elog_async_output_notice(void);
    size_t put_size;

    if (is_enabled) {
        if (level >= OUTPUT_LVL) {
            put_size = async_put_log(log, size);
            /* notify output log thread */
            if (put_size > 0) {
                elog_async_output_notice();
            }
        } else {
            elog_port_output(log, size);
        }
    } else {
        elog_port_output(log, size);
    }
}

#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
void elog_async_output_notice(void) {
    sem_post(&output_notice);
}

static void *async_output(void *arg) {
    size_t get_log_size = 0;
    static char poll_get_buf[ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE];

    while(thread_running) {
        /* waiting log */
        sem_wait(&output_notice);
        /* polling gets and outputs the log */
        while(true) {

#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size = elog_async_get_line_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#else
            get_log_size = elog_async_get_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#endif

            if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
            } else {
                break;
            }
        }
    }
    return NULL;
}
#endif

/**
 * enable or disable asynchronous output mode
 * the log will be output directly when mode is disabled
 *
 * @param enabled true: enabled, false: disabled
 */
void elog_async_enabled(bool enabled) {
    is_enabled = enabled;
}

#ifdef ELOG_ASYNC_OUTPUT_USING_WINTHREAD
static DWORD WINAPI async_output(LPVOID v);
void elog_async_output_notice(void);
#endif
/**
 * asynchronous output mode initialize
 *
 * @return result
 */
ElogErrCode elog_async_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    if (init_ok) {
        return result;
    }
/* 两种方法不共存 */
#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
    pthread_attr_t thread_attr;
    struct sched_param thread_sched_param;

    sem_init(&output_notice, 0, 0);

    thread_running = true;

    pthread_attr_init(&thread_attr);
    //pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&thread_attr, ELOG_ASYNC_OUTPUT_PTHREAD_STACK_SIZE);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_RR);
    thread_sched_param.sched_priority = ELOG_ASYNC_OUTPUT_PTHREAD_PRIORITY;
    pthread_attr_setschedparam(&thread_attr, &thread_sched_param);
    pthread_create(&async_output_thread, &thread_attr, async_output, NULL);
    pthread_attr_destroy(&thread_attr);
#endif
#ifdef ELOG_ASYNC_OUTPUT_USING_WINTHREAD
    /* 信号量 */
    _async_notice_sem = CreateSemaphore(NULL, 0, 100, NULL);
    thread_running = true;

    /* 创建线程 */
    HANDLE _async_thread = CreateThread(NULL, 0, async_output, NULL, 0, &async_output_thread);
    if(NULL == _async_thread)
    {
        printf("frLog CreateThread fail.\n");
        init_ok = false;
        return result;
    }
    // ResumeThread(_async_thread);

#endif  /*ELOG_ASYNC_OUTPUT_USING_WINTHREAD*/

    init_ok = true;

    return result;
}

/**
 * asynchronous output mode deinitialize
 *
 */
void elog_async_deinit(void) {
    if (!init_ok) {
        return ;
    }

#ifdef ELOG_ASYNC_OUTPUT_USING_PTHREAD
    thread_running = false;

    elog_async_output_notice();

    pthread_join(async_output_thread, NULL);
    
    sem_destroy(&output_notice);
#endif
#ifdef ELOG_ASYNC_OUTPUT_USING_WINTHREAD
    /* 线程运行标志停止；发出最后一次信号，防止阻塞；阻塞等待，回收线程；关闭信号量*/
    thread_running = false;

    elog_async_output_notice();

    WaitForSingleObject(_async_thread, INFINITE);

    CloseHandle(_async_thread);

    CloseHandle(_async_notice_sem);
#endif

    init_ok = false;
}

#ifdef ELOG_ASYNC_OUTPUT_USING_WINTHREAD
void elog_async_output_notice(void)
{
    // printf("this is async sem output\n");
    ReleaseSemaphore(_async_notice_sem, 1, NULL);
}

static DWORD WINAPI async_output(LPVOID v)
{
        size_t get_log_size = 0;
    static char poll_get_buf[ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE];

    while(thread_running) {
        /* waiting log */
        WaitForSingleObject(_async_notice_sem, INFINITE);
        /* polling gets and outputs the log */
        while(true) 
        {
#ifdef ELOG_ASYNC_LINE_OUTPUT_do_not_use
            get_log_size = elog_async_get_line_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#else
            get_log_size = elog_async_get_log(poll_get_buf, ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE);
#endif
            // printf("this is async output\n");
            if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
            } else {
                break;
            }
        }
    }
    return 0;
}
#endif /* ELOG_ASYNC_OUTPUT_USING_WINTHREAD */

#endif /* ELOG_ASYNC_OUTPUT_ENABLE */
