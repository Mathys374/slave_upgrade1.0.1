#ifndef SERIAL_H
#define SERIAL_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_PATH_MAX 64

typedef struct {
    int         fd;
    char        path[SERIAL_PATH_MAX];
    unsigned int baud;
} serial_t;

/*
 * 打开并配置串口（8N1、无流控、raw 模式，DTR/RTS 保持无效，
 * 与 Windows 工具 DTR_CONTROL_DISABLE/RTS_CONTROL_DISABLE 行为一致）。
 * 返回：0 成功；-1 打开/配置失败（errbuf 内含人类可读原因）；-2 波特率不支持。
 */
int serial_open(serial_t *s, const char *path, unsigned int baud,
                char *errbuf, size_t errbufsz);

void serial_close(serial_t *s);

/*
 * 在已打开的串口上切换波特率（不重新打开设备、不中断接收）。
 * 返回：0 成功；-1 失败（errbuf 内含人类可读原因）。
 */
int serial_set_baud(serial_t *s, unsigned int baud, char *errbuf, size_t errbufsz);

/*
 * 带超时读。返回：>0 实际读取字节数；0 超时；-1 串口错误；-2 被信号中断。
 */
int serial_read(serial_t *s, void *buf, size_t len, int timeout_ms);

/*
 * 可靠写：循环写直到全部写出或超时。
 * 返回：0 成功；-1 错误/超时；-2 被信号中断。
 */
int serial_write_all(serial_t *s, const void *buf, size_t len, int timeout_ms);

/* 列出 /dev 下的 USB 串口设备（ttyUSB*、ttyACM*），返回数量（已排序） */
int serial_list_ports(char out[][SERIAL_PATH_MAX], int max_ports);

/* 支持的波特率列表（用于错误提示） */
const char *serial_baud_list(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */
