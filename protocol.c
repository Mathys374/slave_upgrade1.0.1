/*
 * protocol.c —— Slave 固件升级协议状态机（Linux CLI 版）
 *
 * 协议与 Windows 工具 FileDownloadDlg.cpp（FileDownload V1.08/20260528）
 * 中的 handle_loop_data() 状态机逐字节兼容：
 *
 *   1. 主机先发 "+++\r\n" 使下位机从数据模式切回命令模式，
 *      再发送复位指令 "AT+S402=1\r\n"，下位机复位并回复 "AT@OK"，进入 bootloader；
 *   2. bootloader 就绪后发送 '#'，主机收到 '#' 开始发送第一个数据块；
 *   3. 数据块 = A1A2A3A4 + 偏移(4B,LE) + 长度(4B,LE) + 数据 + XOR校验(4B,LE)；
 *   4. 下位机回复 '*' 表示确认（继续下一块），'^' 或 '#' 表示异常（重发当前块）；
 *   5. 固件数据发完后，收到 '*' 时发送 MD5 块（偏移 0xFFFFFFFE，长度 16）；
 *   6. 下位机校验 MD5 并写 Flash，完成后回复 "Download OK"，升级成功。
 *
 * 与 Windows 版的差异（均为增强，不改变线上协议）：
 *   - 增加超时与重传上限，失败可退出并报告原因（Windows 版会无限等待/重发）；
 *   - 增加总超时看门狗与用户中断（Ctrl+C）响应；
 *   - 成功后可复位并回显启动信息，用于确认固件版本。
 */

#define _GNU_SOURCE

#include "protocol.h"
#include "serial.h"
#include "md5.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#define DL_BLK_SYNC_WORD  0xA4A3A2A1u

#define RESET_CMD         "AT+S402=1\r\n"
#define QUERY_VER_CMD     "AT+V?\r\n"
#define DL_BAUD           921600   /* 下载波特率：SBL 复位后固定速率，不可配置 */

/*
 * 数据模式 → 命令模式切换指令：
 * "+++\r\n" 按普通 AT 指令格式发送（经实测需带回车换行才生效），
 * 无需 Hayes 转义的静默保护时间。
 */
#define ESCAPE_CMD        "+++\r\n"
#define CMD_ACK_TIMEOUT_MS 2000     /* 发送 +++ 后等待并回显应答的时长 */

#define LINE_MAX_LEN      512
#define WRITE_TIMEOUT_MS  30000   /* 单次完整写出（最多 160KB）的预算 */

/* ---------------- 日志输出 ---------------- */

/* 终端上进度条是否处于活动状态（换行前需要先结束进度行） */
static int g_prog_active;
/* 非终端（输出重定向）模式下的进度里程碑 */
static unsigned g_last_milestone;

static void prog_end(void)
{
    if (g_prog_active) {
        fputc('\n', stdout);
        fflush(stdout);
        g_prog_active = 0;
    }
}

/* 普通信息 -> stdout，quiet 模式抑制 */
static void ulog(const upg_opts_t *o, const char *fmt, ...)
{
    va_list ap;

    if (o->quiet)
        return;
    prog_end();
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
    fputc('\n', stdout);
    fflush(stdout);
}

/* 调试信息 -> stdout，仅 verbose 模式输出 */
static void vlog(const upg_opts_t *o, const char *fmt, ...)
{
    va_list ap;

    if (!o->verbose || o->quiet)
        return;
    prog_end();
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
    fputc('\n', stdout);
    fflush(stdout);
}

/* 警告/错误 -> stderr，始终输出 */
static void ulog_err(const char *fmt, ...)
{
    va_list ap;

    prog_end();
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    fflush(stderr);
}

/* ---------------- 工具函数 ---------------- */

static double now_sec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static uint32_t le32_load(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * XOR 校验：初值 0xA4A3A2A1（同步字），随后异或
 * 偏移(4B) + 长度(4B) + 数据(n 字节，不足 4 字节部分按 0 补齐)。
 * 与 Windows 工具一致（其实现为对小端 u32 数组逐字异或）。
 */
static uint32_t block_checksum(const uint8_t hdr[8], const uint8_t *data, uint32_t n)
{
    uint32_t sum = DL_BLK_SYNC_WORD;
    uint32_t words, i, take;
    uint8_t t[4];

    sum ^= le32_load(hdr);        /* 偏移 */
    sum ^= le32_load(hdr + 4);    /* 长度 */

    words = (n + 3) / 4;
    for (i = 0; i < words; i++) {
        memset(t, 0, 4);
        take = n - i * 4;
        if (take > 4)
            take = 4;
        memcpy(t, data + i * 4, take);
        sum ^= le32_load(t);
    }
    return sum;
}

/* 组包：返回包总长（数据 n + 16 字节头尾） */
static size_t build_block(uint8_t *blk, uint32_t offset,
                          const uint8_t *data, uint32_t n)
{
    uint32_t sum;

    blk[0] = 0xA1;
    blk[1] = 0xA2;
    blk[2] = 0xA3;
    blk[3] = 0xA4;

    blk[4] = (uint8_t)offset;
    blk[5] = (uint8_t)(offset >> 8);
    blk[6] = (uint8_t)(offset >> 16);
    blk[7] = (uint8_t)(offset >> 24);

    blk[8]  = (uint8_t)n;
    blk[9]  = (uint8_t)(n >> 8);
    blk[10] = (uint8_t)(n >> 16);
    blk[11] = (uint8_t)(n >> 24);

    if (n)
        memcpy(blk + 12, data, n);

    sum = block_checksum(blk + 4, data, n);
    blk[12 + n] = (uint8_t)sum;
    blk[13 + n] = (uint8_t)(sum >> 8);
    blk[14 + n] = (uint8_t)(sum >> 16);
    blk[15 + n] = (uint8_t)(sum >> 24);

    return (size_t)n + 16;
}

/* 发送辅助：返回 UPG_OK / UPG_ERR_SERIAL / UPG_ERR_ABORT */
static upg_err_t send_bytes(const upg_opts_t *o, serial_t *ser,
                            const void *p, size_t len)
{
    int r = serial_write_all(ser, p, len, WRITE_TIMEOUT_MS);

    if (r == 0) {
        vlog(o, "[TX] %zu 字节", len);
        return UPG_OK;
    }
    if (r == -2)
        return UPG_ERR_ABORT;
    ulog_err("[错误] 串口写入失败（设备可能已断开）");
    return UPG_ERR_SERIAL;
}

/* ---------------- 按行接收 ---------------- */

/*
 * 行接收器：与 Windows 工具的接收过滤逻辑一致——
 * 只接受可打印字符（0x20~0x7E）与 \r\n；出现其他字节（二进制干扰）
 * 视为脏帧，丢弃当前行。行以 '\n' 结束。
 *
 * 另外处理两类交互式终端的实际情况：
 *   - 一次串口读取可能包含"完整一行 + 后续半行"，剩余字节暂存 pend，
 *     待下次调用继续消费，绝不丢弃；
 *   - 提示符（如 "OAK-128>"）不以换行结尾：缓冲区有未成行数据且持续
 *     LR_IDLE_FLUSH_MS 无新字节时，作为不完整行返回，及时回显。
 */
#define LR_IDLE_FLUSH_MS  300

typedef struct {
    serial_t *ser;
    char      buf[LINE_MAX_LEN];
    size_t    len;
    int       dirty;
    uint8_t   pend[256];    /* 上次读取中尚未消费的剩余字节 */
    size_t    pend_len;
} lreader_t;

static void lr_init(lreader_t *lr, serial_t *ser)
{
    lr->ser = ser;
    lr->len = 0;
    lr->dirty = 0;
    lr->pend_len = 0;
}

/* 送入一个字节；凑成完整行时写入 out 并返回 1，否则返回 0 */
static int lr_feed(lreader_t *lr, uint8_t c, char *out, size_t outsz)
{
    if (c == '\n') {
        if (!lr->dirty && lr->len > 0) {
            lr->buf[lr->len] = '\0';
            snprintf(out, outsz, "%s", lr->buf);
            lr->len = 0;
            lr->dirty = 0;
            return 1;
        }
        lr->len = 0;           
        lr->dirty = 0;
    } else if (c == '\r') {
        /* 忽略回车 */
    } else if (c >= 0x20 && c <= 0x7E) {
        if (lr->len + 1 < sizeof(lr->buf)) {
            lr->buf[lr->len++] = (char)c;

            if (c == '#' || c == '*' || c == '^') {
                lr->buf[lr->len] = '\0';
                snprintf(out, outsz, "%s", lr->buf);
                lr->len = 0;
                lr->dirty = 0;
                return 1;
            }
        } else {
            lr->dirty = 1;    
            lr->len = 0;
        }
    } else {
        /* 修复2：遇二进制乱码只标记脏行，严禁清空 len，保全后续的 # */
        lr->dirty = 1;
    }
    return 0;
}

/* 把缓冲区中未成行的数据作为不完整行输出（无数据时返回 0） */
static int lr_flush_partial(lreader_t *lr, char *out, size_t outsz)
{
    if (lr->len == 0)
        return 0;
    lr->buf[lr->len] = '\0';
    snprintf(out, outsz, "%s", lr->buf);
    lr->len = 0;
    lr->dirty = 0;
    return 1;
}

/*
 * 轮询直到收到一行或超时。
 * 返回：1 收到一行（写入 out，可能是不完整的空闲行）；0 超时；
 *      -1 串口错误；-2 用户中断。
 */
static int lr_line(lreader_t *lr, unsigned int timeout_ms,
                   char *out, size_t outsz)
{
    double deadline = now_sec() + (double)timeout_ms / 1000.0;
    double last_rx = now_sec();
    uint8_t tmp[256];

    for (;;) {
        double left = deadline - now_sec();
        int tmo, n, i;
        uint8_t c;

        if (left <= 0) {
            /* 到期仍有未成行数据：作为不完整行返回（如提示符） */
            if (lr_flush_partial(lr, out, outsz))
                return 1;
            return 0;
        }
        tmo = (int)(left * 1000.0);
        if (tmo > 100)
            tmo = 100;          /* 以 ≤100ms 粒度轮询，便于空闲冲刷 */
        if (tmo < 1)
            tmo = 1;

        if (upg_abort_flag)
            return -2;

        /* 先消费上次读取遗留的字节 */
        for (i = 0; i < (int)lr->pend_len; ) {
            c = lr->pend[i++];
            if (lr_feed(lr, c, out, outsz)) {
                memmove(lr->pend, lr->pend + i, lr->pend_len - (size_t)i);
                lr->pend_len -= (size_t)i;
                return 1;
            }
        }
        lr->pend_len = 0;

        n = serial_read(lr->ser, tmp, sizeof(tmp), tmo);
        if (n == -1)
            return -1;
        if (n == -2) {
            if (upg_abort_flag)
                return -2;
            continue;           /* 非中断信号导致的 EINTR，继续等待 */
        }
        if (n == 0) {
            /* 无新数据：未成行数据持续空闲时作为不完整行返回 */
            if ((now_sec() - last_rx) * 1000.0 >= LR_IDLE_FLUSH_MS &&
                lr_flush_partial(lr, out, outsz))
                return 1;
            continue;           /* 本次 poll 超时，外层重算剩余时间 */
        }

        last_rx = now_sec();
        for (i = 0; i < n; ) {
            c = tmp[i++];
            if (lr_feed(lr, c, out, outsz)) {
                /* 同一读取块中还有剩余字节：暂存，下次调用继续消费 */
                memcpy(lr->pend, tmp + i, (size_t)(n - i));
                lr->pend_len = (size_t)(n - i);
                return 1;
            }
        }
    }
}

/* ---------------- 通用泵：打印并等待 ---------------- */

typedef enum {
    PUMP_TIMEOUT = 0,   /* 到时结束 */
    PUMP_MATCH   = 1,   /* 命中匹配字符 */
    PUMP_ERR     = -1,
    PUMP_ABORT   = -2
} pump_res_t;

/*
 * 在 timeout_ms 内持续读取并打印收到的行（[RX] 前缀）。
 * match_chars 非 NULL 时，某行包含其中任一字符即返回 PUMP_MATCH
 * （该行内容写入 match_out，且该行不打印）；否则一直读到超时。
 * lines_seen 返回期间收到的行数（可为 NULL）。
 */
static pump_res_t pump(upg_opts_t *o, lreader_t *lr, unsigned int timeout_ms,
                       const char *match_chars, char *match_out, size_t match_outsz,
                       int *lines_seen)
{
    double deadline = now_sec() + (double)timeout_ms / 1000.0;
    char line[LINE_MAX_LEN];
    int seen = 0;

    if (lines_seen)
        *lines_seen = 0;

    for (;;) {
        double left = deadline - now_sec();
        int r;

        if (left <= 0)
            return PUMP_TIMEOUT;

        r = lr_line(lr, (unsigned int)(left * 1000.0) + 1,
                    line, sizeof(line));
        if (r == -1)
            return PUMP_ERR;
        if (r == -2)
            return PUMP_ABORT;
        if (r == 0)
            return PUMP_TIMEOUT;

        seen++;
        if (lines_seen)
            *lines_seen = seen;

        if (strstr(line, "AT@OK"))
            ulog(o, "下位机已复位（收到 AT@OK）");

        if (match_chars) {
            const char *pc;
            for (pc = match_chars; *pc; pc++) {
                if (strchr(line, *pc)) {
                    if (match_out && match_outsz)
                        snprintf(match_out, match_outsz, "%s", line);
                    return PUMP_MATCH;
                }
            }
        }
        ulog(o, "[RX] %s", line);
    }
}

/*
 * 发送 "+++\r\n" 使下位机从数据模式切回命令模式，
 * 之后才能执行 AT+S402=1 复位等 AT 指令。
 * 发送后读取并回显下位机应答（[RX] 前缀，最多等 CMD_ACK_TIMEOUT_MS），
 * 便于确认模式切换是否成功（如 "OK" / "ERROR" / 无应答）。
 */
static upg_err_t enter_cmd_mode(upg_opts_t *o, serial_t *ser, lreader_t *lr)
{
    upg_err_t ret;
    pump_res_t pr;
    int seen = 0;

    ulog(o, "发送 +++ 切换到命令模式...");
    ret = send_bytes(o, ser, ESCAPE_CMD, strlen(ESCAPE_CMD));
    if (ret != UPG_OK)
        return ret;

    pr = pump(o, lr, CMD_ACK_TIMEOUT_MS, NULL, NULL, 0, &seen);
    if (pr == PUMP_ERR) {
        ulog_err("[错误] 串口读取失败: %s（设备可能已断开）", strerror(errno));
        return UPG_ERR_SERIAL;
    }
    if (pr == PUMP_ABORT)
        return UPG_ERR_ABORT;
    if (seen == 0)
        ulog(o, "（%u 毫秒内未收到 +++ 的应答）", CMD_ACK_TIMEOUT_MS);
    return UPG_OK;
}

/* ---------------- 进度显示 ---------------- */

static void progress_show(const upg_opts_t *o, uint32_t offset, uint32_t size,
                          unsigned blk_no, unsigned blk_total, double elapsed)
{
    double pct, kbps;

    if (o->quiet || size == 0)
        return;

    pct = (double)offset * 100.0 / (double)size;
    kbps = elapsed > 0.001 ? (double)offset / 1024.0 / elapsed : 0.0;

    if (isatty(STDOUT_FILENO)) {
        printf("\r[进度] %5.1f%%  块 %u/%u  偏移 0x%08X  平均 %6.1f KB/s   ",
               pct, blk_no, blk_total, offset, kbps);
        fflush(stdout);
        g_prog_active = 1;
    } else {
        /* 输出重定向时按 10% 里程碑打印整行，避免日志被 \r 污染 */
        unsigned m = (unsigned)(pct / 10.0);
        if (m > g_last_milestone && m < 10) {
            g_last_milestone = m;
            printf("[进度] %u%%  块 %u/%u\n", (unsigned)pct, blk_no, blk_total);
            fflush(stdout);
        }
    }
}

/* ---------------- 错误描述 ---------------- */

const char *upg_err_str(upg_err_t e)
{
    switch (e) {
    case UPG_OK:          return "成功";
    case UPG_ERR_FILE:    return "固件文件错误（不存在/为空/超过 8MB 上限/读取失败）";
    case UPG_ERR_SERIAL:  return "串口错误（打开/读写失败，设备可能断开）";
    case UPG_ERR_TIMEOUT: return "等待下位机响应超时（链路中断、波特率不匹配或下位机未就绪）";
    case UPG_ERR_RETRY:   return "连续重传次数超限（链路质量差、MD5 校验不匹配或下位机异常）";
    case UPG_ERR_ABORT:   return "用户中断";
    default:              return "未知错误";
    }
}

/* ---------------- 串口打开（含自动探测提示） ---------------- */

static upg_err_t open_serial(upg_opts_t *o, serial_t *ser, unsigned int baud)
{
    char errbuf[512];

    if (serial_open(ser, o->port, baud, errbuf, sizeof(errbuf)) != 0) {
        ulog_err("[错误] %s", errbuf);
        return UPG_ERR_SERIAL;
    }
    ulog(o, "串口已打开: %s（%u 波特率, 8N1, 无流控）", ser->path, ser->baud);
    return UPG_OK;
}

/* ---------------- 升级主流程 ---------------- */

/*
 * 在已打开的串口上切换波特率（不 close/open——重开会清空缓冲，
 * 可能丢失 SBL 复位后立即发出的就绪帧 '#'）。
 * 失败时打印错误并返回 UPG_ERR_SERIAL。
 */
static upg_err_t switch_baud(upg_opts_t *o, serial_t *ser, unsigned int baud)
{
    char eb[160];

    if (serial_set_baud(ser, baud, eb, sizeof(eb)) != 0) {
        ulog_err("[错误] %s", eb);
        return UPG_ERR_SERIAL;
    }
    ulog(o, "串口波特率切换至 %u", baud);
    return UPG_OK;
}

upg_err_t upgrade_run(upg_opts_t *o)
{
    upg_err_t ret = UPG_OK;
    serial_t ser;
    lreader_t lr;
    FILE *fp = NULL;
    uint8_t *filebuf = NULL;
    uint8_t *blk = NULL;
    uint8_t md5[16];
    char md5hex[33];
    char line[LINE_MAX_LEN];
    long fsz;
    uint32_t size, offset = 0, n;
    size_t blk_len = 0;
    unsigned blk_no = 0, blk_total, i;
    int retries = 0, got_ready = 0;
    double t0, overall_deadline = 0;
    pump_res_t pr;
    int r, seen = 0;

    g_prog_active = 0;
    g_last_milestone = 0;
    memset(&ser, 0, sizeof(ser));
    ser.fd = -1;

    /* ---- 1. 读取固件文件并计算 MD5 ---- */
    ulog(o, "读取固件文件: %s", o->filepath);
    fp = fopen(o->filepath, "rb");
    if (!fp) {
        ulog_err("[错误] 无法打开固件文件 %s: %s", o->filepath, strerror(errno));
        ret = UPG_ERR_FILE;
        goto out;
    }
    if (fseek(fp, 0, SEEK_END) != 0 || (fsz = ftell(fp)) < 0 ||
        fseek(fp, 0, SEEK_SET) != 0) {
        ulog_err("[错误] 读取固件文件大小失败: %s", strerror(errno));
        ret = UPG_ERR_FILE;
        goto out;
    }
    if (fsz == 0) {
        ulog_err("[错误] 固件文件为空: %s", o->filepath);
        ret = UPG_ERR_FILE;
        goto out;
    }
    if ((unsigned long)fsz > UPG_FILE_SIZE_MAX) {
        ulog_err("[错误] 固件文件过大: %ld 字节（上限 %u 字节，约 8MB）",
                 fsz, (unsigned)UPG_FILE_SIZE_MAX);
        ret = UPG_ERR_FILE;
        goto out;
    }
    size = (uint32_t)fsz;

    filebuf = (uint8_t *)malloc(size);
    blk = (uint8_t *)malloc((size_t)UPG_TX_PAYLOAD_MAX + 16);
    if (!filebuf || !blk) {
        ulog_err("[错误] 内存分配失败");
        ret = UPG_ERR_FILE;
        goto out;
    }
    if (fread(filebuf, 1, size, fp) != size) {
        ulog_err("[错误] 固件文件读取不完整");
        ret = UPG_ERR_FILE;
        goto out;
    }
    fclose(fp);
    fp = NULL;

    md5_buf(filebuf, size, md5, o->legacy_md5);
    for (i = 0; i < 16; i++)
        snprintf(md5hex + i * 2, 3, "%02x", md5[i]);

    blk_total = (size + UPG_TX_PAYLOAD_MAX - 1) / UPG_TX_PAYLOAD_MAX;
    ulog(o, "文件大小: %u 字节，分 %u 块（每块最大 %u 字节）",
         size, blk_total, (unsigned)UPG_TX_PAYLOAD_MAX);
    ulog(o, "文件 MD5: %s%s", md5hex, o->legacy_md5 ? "（legacy 兼容模式）" : "");

    /* ---- 2. 打开串口（下载波特率固定 921600，复位阶段临时切换） ---- */
    ret = open_serial(o, &ser, DL_BAUD);
    if (ret != UPG_OK)
        goto out;
    lr_init(&lr, &ser);

    t0 = now_sec();
    if (o->overall_timeout_ms > 0)
        overall_deadline = t0 + (double)o->overall_timeout_ms / 1000.0;

    /* ---- 3. 等待 bootloader 就绪（'#'） ---- */
    ulog(o, "等待下位机 bootloader 就绪请求（'#'）...");

    /* 先预监听：下位机可能已停留在 bootloader（如上次升级被中断） */
    if (o->pre_listen_ms > 0) {
        pr = pump(o, &lr, o->pre_listen_ms, "#", line, sizeof(line), &seen);
        if (pr == PUMP_MATCH) {
            got_ready = 1;
        } else if (pr == PUMP_ERR) {
            ulog_err("[错误] 串口读取失败: %s（设备可能已断开）", strerror(errno));
            ret = UPG_ERR_SERIAL;
            goto out;
        } else if (pr == PUMP_ABORT) {
            ret = UPG_ERR_ABORT;
            goto out;
        }
    }

    /* 未就绪则发送复位指令并等待 */
    for (i = 1; !got_ready && (int)i <= o->reset_tries; i++) {
        if (o->init_baud != DL_BAUD) {
            /*
             * 模块固件波特率与 SBL 下载波特率（921600）不同：在已打开的
             * 串口上切换波特率发送复位指令，复位后不重开串口——直接切回
             * 921600 等待 SBL 的 '#'。
             */
            if (switch_baud(o, &ser, o->init_baud) != UPG_OK)
                goto out;
            lr_init(&lr, &ser);      /* 清空行缓冲，丢弃跨波特率残留 */
            ret = enter_cmd_mode(o, &ser, &lr);
            if (ret != UPG_OK)
                goto out;
            ulog(o, "发送复位指令 AT+S402=1（第 %u/%d 次，复位波特率 %u）...",
                 i, o->reset_tries, o->init_baud);
            ret = send_bytes(o, &ser, RESET_CMD, strlen(RESET_CMD));
            if (ret != UPG_OK)
                goto out;
            /* 收到复位回显（含 "S402"）即提前返回，最多等 500ms */
            pump(o, &lr, 500, "S402", line, sizeof(line), &seen);
            /* 复位指令已下发：立即切回下载波特率 921600，等待 SBL 的 '#' */
            if (switch_baud(o, &ser, DL_BAUD) != UPG_OK)
                goto out;
            lr_init(&lr, &ser);      /* 清空行缓冲，丢弃跨波特率残留 */
        } else {
            ret = enter_cmd_mode(o, &ser, &lr);
            if (ret != UPG_OK)
                goto out;
            ulog(o, "发送复位指令 AT+S402=1（第 %u/%d 次），等待下位机进入 bootloader...",
                 i, o->reset_tries);
            ret = send_bytes(o, &ser, RESET_CMD, strlen(RESET_CMD));
            if (ret != UPG_OK)
                goto out;
        }

        pr = pump(o, &lr, o->boot_timeout_ms, "#", line, sizeof(line), &seen);
        if (pr == PUMP_MATCH) {
            got_ready = 1;
        } else if (pr == PUMP_ERR) {
            ulog_err("[错误] 串口读取失败: %s（设备可能已断开）", strerror(errno));
            ret = UPG_ERR_SERIAL;
            goto out;
        } else if (pr == PUMP_ABORT) {
            ret = UPG_ERR_ABORT;
            goto out;
        }
    }

    if (!got_ready) {
        ulog_err("[错误] 等待下位机就绪（'#'）超时。请检查：");
        ulog_err("  1) 任务板与 Slave 之间的 USB 串口链路是否正常；");
        ulog_err("  2) 复位波特率是否正确（当前 %u，可用 -B 调整；SBL 下载固定 921600）；",
                 o->init_baud);
        ulog_err("  3) Slave 模块供电是否正常（可先运行 slave_upgrade -r 测试复位）。");
        ret = UPG_ERR_TIMEOUT;
        goto out;
    }
    ulog(o, "已收到下位机就绪请求（'#'），开始传输固件。");

    /* ---- 4. 传输数据块 ---- */
    n = (size - offset > UPG_TX_PAYLOAD_MAX) ? UPG_TX_PAYLOAD_MAX : size - offset;
    blk_len = build_block(blk, 0, filebuf, n);
    offset = n;
    blk_no = 1;

    ret = send_bytes(o, &ser, blk, blk_len);
    if (ret != UPG_OK)
        goto out;
    vlog(o, "[TX] 块 1/%u 偏移 0x%08X 长度 %u", blk_total, 0u, n);
    progress_show(o, offset, size, blk_no, blk_total, now_sec() - t0);

    for (;;) {
        if (upg_abort_flag) {
            ret = UPG_ERR_ABORT;
            goto out;
        }
        if (overall_deadline > 0 && now_sec() >= overall_deadline) {
            ulog_err("[错误] 总超时（%u 秒）。已传输 %u/%u 字节。",
                     o->overall_timeout_ms / 1000, offset, size);
            ret = UPG_ERR_TIMEOUT;
            goto out;
        }

        r = lr_line(&lr, o->block_timeout_ms, line, sizeof(line));
        if (r == -1) {
            ulog_err("[错误] 串口读取失败: %s（设备可能已断开）", strerror(errno));
            ret = UPG_ERR_SERIAL;
            goto out;
        }
        if (r == -2) {
            ret = UPG_ERR_ABORT;
            goto out;
        }
        if (r == 0) {
            /* 超时无应答：主动重发当前块（Windows 版无此机制，会无限等待） */
            retries++;
            ulog_err("[警告] %u 秒内未收到下位机应答，重发当前块（连续第 %d 次，上限 %d）",
                     o->block_timeout_ms / 1000, retries, o->retry_max);
            if (retries > o->retry_max) {
                ulog_err("[错误] 连续重传次数超限，升级中止。");
                ret = UPG_ERR_RETRY;
                goto out;
            }
            ret = send_bytes(o, &ser, blk, blk_len);
            if (ret != UPG_OK)
                goto out;
            continue;
        }

        /* 应答分派（判序与 Windows 工具一致：'*' > '^' > '#' > 文本） */
        if (strchr(line, '*')) {
            retries = 0;
            if (offset < size) {
                uint32_t off_prev = offset;
                n = (size - offset > UPG_TX_PAYLOAD_MAX) ? UPG_TX_PAYLOAD_MAX
                                                         : size - offset;
                blk_len = build_block(blk, offset, filebuf + offset, n);
                offset += n;
                blk_no++;
                ret = send_bytes(o, &ser, blk, blk_len);
                if (ret != UPG_OK)
                    goto out;
                vlog(o, "[TX] 块 %u/%u 偏移 0x%08X 长度 %u",
                     blk_no, blk_total, off_prev, n);
                progress_show(o, offset, size, blk_no, blk_total, now_sec() - t0);
            } else {
                /* 数据发完：发送（或重发）MD5 校验块 */
                blk_len = build_block(blk, 0xFFFFFFFEu, md5, 16);
                ret = send_bytes(o, &ser, blk, blk_len);
                if (ret != UPG_OK)
                    goto out;
                vlog(o, "[TX] MD5 校验块（0xFFFFFFFE）");
                progress_show(o, size, size, blk_no, blk_total, now_sec() - t0);
            }
        } else if (strchr(line, '^') || strchr(line, '#')) {
            retries++;
            ulog_err("[警告] 下位机请求重发当前块（连续第 %d 次，上限 %d）",
                     retries, o->retry_max);
            if (retries > o->retry_max) {
                ulog_err("[错误] 连续重传次数超限，升级中止。");
                ulog_err("可能原因: 链路干扰、波特率偏高、下位机 MD5 校验不匹配。");
                ulog_err("若怀疑 MD5 兼容性问题，可尝试 --legacy-md5 重新升级。");
                ret = UPG_ERR_RETRY;
                goto out;
            }
            ret = send_bytes(o, &ser, blk, blk_len);
            if (ret != UPG_OK)
                goto out;
        } else {
            if (strstr(line, "Download OK") ||
                strstr(line, "Verify QSPI flash Passed")) {
                break;
            }
            ulog(o, "[RX] %s", line);
        }
    }

    /* ---- 5. 升级成功 ---- */
    prog_end();
    {
        double dt = now_sec() - t0;
        ulog(o, "下位机已确认写入完成（Download OK），耗时 %.1f 秒。", dt);
        ulog(o, "升级成功: %s（%u 字节, MD5 %s）", o->filepath, size, md5hex);
        ulog(o, "请输入 slave_upgrade -V 查询更新后 slave 版本");
    }

    // if (o->listen_secs > 0) {
    //     ulog(o, "复位下位机并监听启动信息 %u 秒（可用于确认固件版本）...",
    //          o->listen_secs);
    //     if (enter_cmd_mode(o, &ser, &lr) == UPG_OK &&
    //         send_bytes(o, &ser, RESET_CMD, strlen(RESET_CMD)) == UPG_OK) {
    //         pump(o, &lr, o->listen_secs * 1000, NULL, NULL, 0, &seen);
    //     }
    //     ulog(o, "启动信息监听结束。");
    // }

out:
    prog_end();
    if (fp)
        fclose(fp);
    free(filebuf);
    free(blk);
    if (ser.fd >= 0)
        serial_close(&ser);
    return ret;
}

/* ---------------- 辅助模式（-V 查询版本 / -r 仅复位） ---------------- */

upg_err_t probe_run(upg_opts_t *o, int query_version)
{
    upg_err_t ret;
    serial_t ser;
    lreader_t lr;
    int seen = 0;

    memset(&ser, 0, sizeof(ser));
    ser.fd = -1;

    /* -V/-r 全程使用复位波特率（-B，默认 115200） */
    ret = open_serial(o, &ser, o->init_baud);
    if (ret != UPG_OK)
        return ret;
    lr_init(&lr, &ser);

    if (query_version) {
        ulog(o, "发送版本查询指令 AT+V? ...");
        ret = send_bytes(o, &ser, QUERY_VER_CMD, strlen(QUERY_VER_CMD));
        if (ret != UPG_OK) {
            serial_close(&ser);
            return ret;
        }

        pump(o, &lr, 3000, NULL, NULL, 0, &seen);
        if (seen > 0) {
            ulog(o, "版本查询完成（见上方 [RX] 输出）。");
            serial_close(&ser);
            return UPG_OK;
        }

        /* 无应答：模块可能处于数据模式，切换命令模式后重试一次 */
        ulog(o, "下位机未响应 AT+V?，切换命令模式后重试...");
        ret = enter_cmd_mode(o, &ser, &lr);
        if (ret != UPG_OK) {
            serial_close(&ser);
            return ret;
        }
        ulog(o, "再次发送版本查询指令 AT+V? ...");
        ret = send_bytes(o, &ser, QUERY_VER_CMD, strlen(QUERY_VER_CMD));
        if (ret != UPG_OK) {
            serial_close(&ser);
            return ret;
        }
        pump(o, &lr, 3000, NULL, NULL, 0, &seen);
        if (seen > 0) {
            ulog(o, "版本查询完成（见上方 [RX] 输出）。");
            serial_close(&ser);
            return UPG_OK;
        }
        ulog(o, "下位机未响应 AT+V?（固件可能不支持该指令）。");
        ulog(o, "尝试复位下位机并读取启动信息（启动信息中通常包含固件版本）...");
    }

    ret = enter_cmd_mode(o, &ser, &lr);
    if (ret != UPG_OK) {
        serial_close(&ser);
        return ret;
    }
    ulog(o, "发送复位指令 AT+S402=1 ...");
    ret = send_bytes(o, &ser, RESET_CMD, strlen(RESET_CMD));
    if (ret != UPG_OK) {
        serial_close(&ser);
        return ret;
    }

    {
        pump_res_t pr = pump(o, &lr, o->listen_secs > 0 ? o->listen_secs * 1000 : 8000,
                             NULL, NULL, 0, &seen);
        if (pr == PUMP_ERR) {
            ulog_err("[错误] 串口读取失败: %s", strerror(errno));
            serial_close(&ser);
            return UPG_ERR_SERIAL;
        }
        if (pr == PUMP_ABORT) {
            serial_close(&ser);
            return UPG_ERR_ABORT;
        }
    }

    serial_close(&ser);
    if (seen == 0) {
        ulog_err("[警告] 复位后未收到任何输出。请检查串口链路与波特率"
                 "（当前 %u，可用 -B 调整），或 Slave 供电。", o->init_baud);
        return UPG_ERR_TIMEOUT;
    }
    ulog(o, "复位完成，启动信息见上方 [RX] 输出。");
    return UPG_OK;
}
