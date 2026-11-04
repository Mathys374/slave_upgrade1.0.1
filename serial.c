/*
 * serial.c —— Linux termios 串口封装
 *
 * 对应 Windows 工具中的 RsTxRx()/writeCom()/readCom()/SearchCom()：
 *   - RsTxRx()  -> serial_open()   （8N1、无流控，波特率 9600~2000000 九档）
 *   - writeCom()-> serial_write_all()
 *   - readCom() -> serial_read()   （poll 实现毫秒级超时）
 *   - SearchCom()-> serial_list_ports()（Windows 下读注册表，Linux 下扫 /dev）
 */

#define _GNU_SOURCE

#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <poll.h>
#include <dirent.h>

/* 与 Windows 工具一致的九档波特率 */
struct baud_entry {
    unsigned int val;
    speed_t      sp;
};

static const struct baud_entry BAUD_TABLE[] = {
    {   9600,   B9600 },
    {  19200,  B19200 },
    {  38400,  B38400 },
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 },
    { 921600, B921600 },
    { 1000000, B1000000 },
    { 2000000, B2000000 },
};

#define BAUD_TABLE_N (sizeof(BAUD_TABLE) / sizeof(BAUD_TABLE[0]))

const char *serial_baud_list(void)
{
    return "9600, 19200, 38400, 115200, 230400, 460800, 921600, 1000000, 2000000";
}

static speed_t baud_to_speed(unsigned int baud)
{
    size_t i;
    for (i = 0; i < BAUD_TABLE_N; i++)
        if (BAUD_TABLE[i].val == baud)
            return BAUD_TABLE[i].sp;
    return 0;
}

int serial_open(serial_t *s, const char *path, unsigned int baud,
                char *errbuf, size_t errbufsz)
{
    struct termios tio;
    speed_t sp;
    int fd, mcs;

    memset(s, 0, sizeof(*s));
    s->fd = -1;   /* 失败路径下保证 fd 无效，避免误关 stdin 等句柄 */
    if (errbuf && errbufsz)
        errbuf[0] = '\0';

    sp = baud_to_speed(baud);
    if (sp == 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "不支持的波特率 %u（可选: %s）",
                     baud, serial_baud_list());
        return -2;
    }

    fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (errbuf) {
            snprintf(errbuf, errbufsz, "无法打开 %s: %s", path, strerror(errno));
            if (errno == EACCES || errno == EPERM) {
                strncat(errbuf, "（当前用户无权限，可执行: "
                        "sudo usermod -aG dialout $USER 并重新登录，或使用 sudo 运行）",
                        errbufsz - strlen(errbuf) - 1);
            } else if (errno == ENOENT) {
                strncat(errbuf, "（设备不存在，可用 --list 查看可用串口）",
                        errbufsz - strlen(errbuf) - 1);
            } else if (errno == ENODEV || errno == ENXIO) {
                strncat(errbuf, "（设备已拔出或驱动未就绪）",
                        errbufsz - strlen(errbuf) - 1);
            }
        }
        return -1;
    }

    if (!isatty(fd)) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "%s 不是串行终端设备", path);
        close(fd);
        return -1;
    }

    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "读取串口属性失败 %s: %s",
                     path, strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);                       /* raw 模式：不做任何输入输出处理 */

    tio.c_cflag |= (CLOCAL | CREAD);       /* 本地连接 + 使能接收 */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;                    /* 8 位数据位 */
    tio.c_cflag &= ~PARENB;                /* 无校验 */
    tio.c_cflag &= ~CSTOPB;                /* 1 位停止位 */
    tio.c_cflag &= ~CRTSCTS;               /* 无硬件流控 */
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);/* 无软件流控 */
    tio.c_cc[VMIN]  = 0;                   /* 非阻塞语义，超时由 poll 控制 */
    tio.c_cc[VTIME] = 0;

    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    /* TCSAFLUSH：配置生效同时清掉残留的输入输出 */
    if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "配置串口失败 %s: %s",
                     path, strerror(errno));
        close(fd);
        return -1;
    }

    /*
     * 清除 DTR/RTS：Linux 打开 tty 默认会置位 DTR/RTS，而 Windows 工具
     * 为 DTR_CONTROL_DISABLE/RTS_CONTROL_DISABLE（线路无效）。部分
     * USB 转接板的 DTR/RTS 接到了模块复位/启动引脚，置位可能触发意外
     * 复位，故此处清除以对齐 Windows 行为（尽力而为，失败不影响使用）。
     */
    mcs = TIOCM_DTR;
    ioctl(fd, TIOCMBIC, &mcs);
    mcs = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &mcs);

    tcflush(fd, TCIOFLUSH);

    s->fd = fd;
    s->baud = baud;
    snprintf(s->path, sizeof(s->path), "%s", path);
    return 0;
}

void serial_close(serial_t *s)
{
    if (s->fd >= 0) {
        tcflush(s->fd, TCIOFLUSH);
        close(s->fd);
        s->fd = -1;
    }
}

int serial_set_baud(serial_t *s, unsigned int baud, char *errbuf, size_t errbufsz)
{
    struct termios tio;
    speed_t sp;

    if (errbuf && errbufsz)
        errbuf[0] = '\0';
    if (s->fd < 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "串口未打开，无法切换波特率");
        return -1;
    }

    sp = baud_to_speed(baud);
    if (sp == 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "不支持的波特率 %u（可选: %s）",
                     baud, serial_baud_list());
        return -1;
    }

    if (tcgetattr(s->fd, &tio) != 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "读取串口属性失败: %s", strerror(errno));
        return -1;
    }

    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    /* TCSANOW：立即生效，不清输出缓冲（避免打断尚在发送的数据） */
    if (tcsetattr(s->fd, TCSANOW, &tio) != 0) {
        if (errbuf)
            snprintf(errbuf, errbufsz, "切换波特率 %u 失败: %s",
                     baud, strerror(errno));
        return -1;
    }

    /* 丢弃旧波特率期间收到的残留字节（对新波特率而言是乱码） */
    tcflush(s->fd, TCIFLUSH);

    s->baud = baud;
    return 0;
}

int serial_read(serial_t *s, void *buf, size_t len, int timeout_ms)
{
    struct pollfd pfd;
    ssize_t r;
    int pr;

    if (s->fd < 0 || len == 0)
        return 0;

    pfd.fd = s->fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    pr = poll(&pfd, 1, timeout_ms);
    if (pr < 0)
        return (errno == EINTR) ? -2 : -1;
    if (pr == 0)
        return 0;                          /* 超时 */

    r = read(s->fd, buf, len);
    if (r > 0)
        return (int)r;
    if (r == 0)
        return 0;                          /* 无数据（如设备重新上电） */
    if (errno == EINTR)
        return -2;
    if (errno == EAGAIN || errno == EWOULDBLOCK)
        return 0;
    return -1;
}

int serial_write_all(serial_t *s, const void *buf, size_t len, int timeout_ms)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t left = len;
    double deadline;
    struct pollfd pfd;
    ssize_t w;

    if (s->fd < 0)
        return -1;
    if (len == 0)
        return 0;

    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        deadline = (double)ts.tv_sec + (double)ts.tv_nsec / 1e9
                   + (double)timeout_ms / 1000.0;
    }

    while (left > 0) {
        w = write(s->fd, p, left);
        if (w > 0) {
            p += w;
            left -= (size_t)w;
            continue;
        }
        if (w < 0) {
            if (errno == EINTR)
                return -2;
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                return -1;
        }

        /* 输出缓冲区满：等待可写 */
        {
            struct timespec ts;
            double now, left_s;
            int tmo;

            clock_gettime(CLOCK_MONOTONIC, &ts);
            now = (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
            left_s = deadline - now;
            if (left_s <= 0)
                return -1;                 /* 写超时 */
            tmo = (int)(left_s * 1000.0);
            if (tmo < 1)
                tmo = 1;

            pfd.fd = s->fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            if (poll(&pfd, 1, tmo) < 0) {
                if (errno == EINTR)
                    return -2;
                return -1;
            }
        }
    }
    return 0;
}

static int cmp_str(const void *a, const void *b)
{
    return strcmp((const char *)a, (const char *)b);
}

static int is_usb_serial_name(const char *name)
{
    return strncmp(name, "ttyUSB", 6) == 0 ||
           strncmp(name, "ttyACM", 6) == 0;
}

int serial_list_ports(char out[][SERIAL_PATH_MAX], int max_ports)
{
    DIR *d;
    struct dirent *e;
    int n = 0;

    d = opendir("/dev");
    if (!d)
        return 0;

    while ((e = readdir(d)) != NULL) {
        if (!is_usb_serial_name(e->d_name))
            continue;
        if (n >= max_ports)
            break;
        /* %.58s：d_name 最长可达 NAME_MAX(255)，超过缓冲；串口名远短于 58 */
        snprintf(out[n], SERIAL_PATH_MAX, "/dev/%.58s", e->d_name);
        n++;
    }
    closedir(d);

    if (n > 1)
        qsort(out, (size_t)n, SERIAL_PATH_MAX, cmp_str);
    return n;
}
