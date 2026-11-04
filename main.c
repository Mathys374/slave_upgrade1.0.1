/*
 * main.c —— slave_upgrade 命令行入口
 *
 * 专网模块 Slave 固件串口升级工具（Linux CLI 版）
 * 升级协议与 Windows 工具 FileDownload V1.08/20260528 完全兼容。
 *
 ********************************************************************************************
 V1.00/20260911
 1、初版：面向机载任务板（Ubuntu 20.04/22.04, aarch64）的命令行升级工具；
 2、支持固件升级（默认）、版本查询（-V）、仅复位（-r）、串口列表（-l）、
    文件校验（--check）五种模式；
 3、新增超时/重传上限/总超时控制，失败时输出原因与退出码，便于脚本化集成；
 4、升级成功后自动复位并回显启动信息，用于确认固件版本。
*/

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>

#include "protocol.h"
#include "serial.h"
#include "md5.h"

#define TOOL_NAME    "slave_upgrade"
#define TOOL_VERSION "1.0.1"

/* 信号处理：置中断标志，协议层在安全的检查点退出 */
volatile sig_atomic_t upg_abort_flag = 0;

static void sig_handler(int sig)
{
    (void)sig;
    upg_abort_flag = 1;
}

static void usage(FILE *out)
{
    fprintf(out,
"slave_upgrade v%s —— 专网模块 Slave 固件串口升级工具\n"
"\n"
"用法:\n"
"  %s [选项] <固件文件.bin>     升级固件（默认模式）\n"
"  %s -V                      查询下位机固件版本\n"
"  %s -r                      仅复位下位机并回显启动信息\n"
"  %s -l                      列出可用串口设备\n"
// "  %s --check <固件文件.bin>  仅校验文件（大小/MD5/分块数，不连接串口）\n"
"\n"
"选项:\n"
"  -p, --port <设备>           串口设备（如 /dev/ttyUSB0）；缺省时自动探测\n"
"  -B, --slave-baud <波特率>   slave模块当前波特率，默认 115200；-V/-r 默认使用该波特率\n"
// "                            可选: 9600/19200/38400/115200/230400/460800/921600/1000000/2000000\n"
// "                            下载波特率固定 921600（SBL 复位后固定速率，不可更改）\n"
// "      --boot-timeout <秒>  每次复位后等待就绪('#')的时长，默认 15\n"
// "      --block-timeout <秒> 发送数据块后等待应答的时长（含 SBL 擦写 Flash 的静默期），默认 120\n"
// "      --reset-tries <次>   复位指令重试次数，默认 3\n"
// "      --retry-max <次>     连续无进展重传上限，默认 20\n"
// "      --timeout <秒>       升级总超时，默认 1800，0 表示不限制\n"
// "      --listen <秒>        -V/-r 复位后监听启动信息的秒数，默认 8，0 表示关闭\n"
// "      --legacy-md5         兼容 Windows 工具在 64 字节整数倍文件上的 MD5 行为\n"
// "  -v, --verbose            输出详细日志（含每块收发）\n"
"  -q, --quiet                静默模式（仅输出最终结果，适合脚本调用）\n"
"  -h, --help                 显示本帮助\n"
"      --version              显示工具版本\n"
"\n"
"示例:\n"
"  %s firmware.bin                    # 最常用：自动探测串口并升级\n"
"  %s -B 115200 firmware.bin          # salve波特率在115200：以115200下复位指令，921600下载\n"
"  %s -p /dev/ttyUSB0 firmware.bin    # 指定串口设备\n"
"  %s -V                              # 查询下位机固件版本\n"
"  %s -r                              # 仅复位下位机\n"
"\n",
// "退出码: 0 成功 | 1 参数错误 | 2 串口错误 | 3 文件错误 | 4 超时 | 5 重传超限 | 6 用户中断\n",
            TOOL_VERSION, TOOL_NAME, TOOL_NAME, TOOL_NAME, TOOL_NAME, 
            TOOL_NAME, TOOL_NAME, TOOL_NAME, TOOL_NAME, TOOL_NAME);
}

/* 解析非负整数参数，失败返回 -1 */
static long parse_nonneg(const char *s, const char *name)
{
    char *end = NULL;
    long v;

    errno = 0;
    v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0' || v < 0) {
        fprintf(stderr, "[错误] 参数 %s 的值无效: %s（应为非负整数）\n", name, s);
        return -1;
    }
    return v;
}

/* 串口自动探测：仅一台设备时自动选用，多台时提示用户指定 */
static int detect_port(char *out, size_t outsz)
{
    char ports[16][SERIAL_PATH_MAX];
    int n = serial_list_ports(ports, 16);
    int i;

    if (n == 0) {
        fprintf(stderr, "[错误] 未发现串口设备（/dev/ttyUSB*、/dev/ttyACM*）。\n");
        fprintf(stderr, "       请确认任务板与 Slave 之间的 USB 串口已连接。\n");
        return -1;
    }
    if (n > 1) {
        fprintf(stderr, "[错误] 发现 %d 个串口设备，请用 -p 指定:\n", n);
        for (i = 0; i < n; i++)
            fprintf(stderr, "       %s\n", ports[i]);
        return -1;
    }
    snprintf(out, outsz, "%s", ports[0]);
    printf("[信息] 自动选择串口设备: %s\n", ports[0]);
    return 0;
}

/* --check 模式：仅校验文件，不连接串口 */
static int check_file(const char *path, int legacy_md5)
{
    FILE *fp;
    long fsz;
    uint8_t *buf;
    uint8_t md5[16];
    char md5hex[33];
    unsigned i, blk_total;

    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "[错误] 无法打开固件文件 %s: %s\n", path, strerror(errno));
        return 3;
    }
    if (fseek(fp, 0, SEEK_END) != 0 || (fsz = ftell(fp)) < 0 ||
        fseek(fp, 0, SEEK_SET) != 0) {
        fprintf(stderr, "[错误] 读取固件文件大小失败: %s\n", strerror(errno));
        fclose(fp);
        return 3;
    }
    if (fsz == 0) {
        fprintf(stderr, "[错误] 固件文件为空: %s\n", path);
        fclose(fp);
        return 3;
    }
    if ((unsigned long)fsz > UPG_FILE_SIZE_MAX) {
        fprintf(stderr, "[错误] 固件文件过大: %ld 字节（上限 %u 字节，约 8MB）\n",
                fsz, (unsigned)UPG_FILE_SIZE_MAX);
        fclose(fp);
        return 3;
    }

    buf = (uint8_t *)malloc((size_t)fsz);
    if (!buf) {
        fprintf(stderr, "[错误] 内存分配失败\n");
        fclose(fp);
        return 3;
    }
    if (fread(buf, 1, (size_t)fsz, fp) != (size_t)fsz) {
        fprintf(stderr, "[错误] 固件文件读取不完整\n");
        free(buf);
        fclose(fp);
        return 3;
    }
    fclose(fp);

    md5_buf(buf, (size_t)fsz, md5, legacy_md5);
    for (i = 0; i < 16; i++)
        snprintf(md5hex + i * 2, 3, "%02x", md5[i]);
    blk_total = (unsigned)(((unsigned long)fsz + UPG_TX_PAYLOAD_MAX - 1) / UPG_TX_PAYLOAD_MAX);

    printf("固件文件: %s\n", path);
    printf("文件大小: %ld 字节\n", fsz);
    printf("分块数量: %u 块（每块最大 %u 字节）\n",
           blk_total, (unsigned)UPG_TX_PAYLOAD_MAX);
    printf("文件 MD5: %s%s\n", md5hex, legacy_md5 ? "（legacy 兼容模式）" : "");
    printf("校验通过。\n");
    free(buf);
    return 0;
}

int main(int argc, char **argv)
{
    static const struct option long_opts[] = {
        { "port",          required_argument, 0, 'p' },
        { "slave-baud",     required_argument, 0, 'B' },
        { "list",          no_argument,       0, 'l' },
        { "query-version", no_argument,       0, 'V' },
        { "reset",         no_argument,       0, 'r' },
        { "verbose",       no_argument,       0, 'v' },
        { "quiet",         no_argument,       0, 'q' },
        { "check",         no_argument,       0, 'C' },
        { "help",          no_argument,       0, 'h' },
        { "boot-timeout",  required_argument, 0, 0 },
        { "block-timeout", required_argument, 0, 0 },
        { "reset-tries",   required_argument, 0, 0 },
        { "retry-max",     required_argument, 0, 0 },
        { "timeout",       required_argument, 0, 0 },
        { "listen",        required_argument, 0, 0 },
        { "legacy-md5",    no_argument,       0, 0 },
        { "version",       no_argument,       0, 0 },
        { 0, 0, 0, 0 }
    };

    upg_opts_t opts;
    const char *port = NULL;
    unsigned long init_baud = 115200;
    int flag_verbose = 0, flag_quiet = 0, flag_legacy_md5 = 1;
    int do_list = 0, do_query = 0, do_reset = 0, do_check = 0;
    int do_help = 0, do_version = 0;
    long boot_timeout_s = 15, block_timeout_s = 120, reset_tries = 3;
    long retry_max = 20, timeout_s = 1800, listen_s = 8;
    const char *filename = NULL;
    char portbuf[SERIAL_PATH_MAX];
    int c, li = -1;
    upg_err_t r;

#ifdef _WIN32
    /* MinGW 语法检查路径；实际目标平台为 Linux */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
#else
    /* Linux: sigaction 不带 SA_RESTART，poll() 被信号打断后立即返回 EINTR，
     * 保证 Ctrl+C 在秒级内响应（signal() 在 glibc 下默认带 SA_RESTART） */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
#endif

    while ((c = getopt_long(argc, argv, "p:B:lVrvqCh", long_opts, &li)) != -1) {
        switch (c) {
        case 0: { /* 仅长选项 */
            const char *name = long_opts[li].name;
            long v;
            if (!strcmp(name, "boot-timeout")) {
                if ((v = parse_nonneg(optarg, "--boot-timeout")) < 0) return 1;
                boot_timeout_s = v;
            } else if (!strcmp(name, "block-timeout")) {
                if ((v = parse_nonneg(optarg, "--block-timeout")) < 0) return 1;
                block_timeout_s = v;
            } else if (!strcmp(name, "reset-tries")) {
                if ((v = parse_nonneg(optarg, "--reset-tries")) < 0) return 1;
                reset_tries = v;
            } else if (!strcmp(name, "retry-max")) {
                if ((v = parse_nonneg(optarg, "--retry-max")) < 0) return 1;
                retry_max = v;
            } else if (!strcmp(name, "timeout")) {
                if ((v = parse_nonneg(optarg, "--timeout")) < 0) return 1;
                timeout_s = v;
            } else if (!strcmp(name, "listen")) {
                if ((v = parse_nonneg(optarg, "--listen")) < 0) return 1;
                listen_s = v;
            } else if (!strcmp(name, "legacy-md5")) {
                flag_legacy_md5 = 1;
            } else if (!strcmp(name, "version")) {
                do_version = 1;
            }
            break;
        }
        case 'p':
            port = optarg;
            break;
        case 'B': {
            /* -B：复位阶段波特率（模块 AT 固件当前波特率），默认 115200 */
            char *end = NULL;
            errno = 0;
            init_baud = strtoul(optarg, &end, 10);
            if (errno != 0 || end == optarg || *end != '\0') {
                fprintf(stderr, "[错误] 波特率无效: %s（可选: %s）\n",
                        optarg, serial_baud_list());
                return 1;
            }
            break;
        }
        case 'l': do_list = 1;   break;
        case 'V': do_query = 1;  break;
        case 'r': do_reset = 1;  break;
        case 'v': flag_verbose = 1;   break;
        case 'q': flag_quiet = 1;     break;
        case 'C': do_check = 1;  break;
        case 'h': do_help = 1;   break;
        case '?':
        default:
            usage(stderr);
            return 1;
        }
    }

    memset(&opts, 0, sizeof(opts));
    opts.verbose = flag_verbose;
    opts.quiet = flag_quiet;
    opts.legacy_md5 = flag_legacy_md5;

    if (do_help) {
        usage(stdout);
        return 0;
    }
    if (do_version) {
        printf("%s v%s\n", TOOL_NAME, TOOL_VERSION);
        return 0;
    }
    if (do_list) {
        char ports[16][SERIAL_PATH_MAX];
        int n = serial_list_ports(ports, 16);
        int i;
        if (n == 0) {
            printf("未发现串口设备（/dev/ttyUSB*、/dev/ttyACM*）\n");
            return 0;
        }
        printf("发现 %d 个串口设备:\n", n);
        for (i = 0; i < n; i++)
            printf("  %s\n", ports[i]);
        return 0;
    }

    if (optind < argc)
        filename = argv[optind];
    if (optind + 1 < argc) {
        fprintf(stderr, "[错误] 多余的参数: %s\n", argv[optind + 1]);
        usage(stderr);
        return 1;
    }

    if (do_check) {
        if (!filename) {
            fprintf(stderr, "[错误] --check 需要指定固件文件\n");
            usage(stderr);
            return 1;
        }
        return check_file(filename, flag_legacy_md5);
    }

    if (!do_query && !do_reset) {
        /* 默认升级模式：必须提供固件文件 */
        if (!filename) {
            fprintf(stderr, "[错误] 缺少固件文件参数\n\n");
            usage(stderr);
            return 1;
        }
    } else if (filename) {
        fprintf(stderr, "[警告] -V/-r 模式不需要固件文件，已忽略: %s\n", filename);
    }

    /* 串口设备：显式指定或自动探测 */
    if (!port) {
        if (detect_port(portbuf, sizeof(portbuf)) != 0)
            return 2;
        port = portbuf;
    }

    opts.port = port;
    opts.init_baud = (unsigned int)init_baud;    /* 复位波特率，默认 115200 */
    opts.filepath = filename;
    opts.pre_listen_ms = 2000;
    opts.boot_timeout_ms = (unsigned int)boot_timeout_s * 1000;
    opts.reset_tries = (int)reset_tries;
    opts.block_timeout_ms = (unsigned int)block_timeout_s * 1000;
    opts.retry_max = (int)retry_max;
    opts.overall_timeout_ms = (unsigned int)timeout_s * 1000;
    opts.listen_secs = (unsigned int)listen_s;

    if (do_query || do_reset) {
        r = probe_run(&opts, do_query);
    } else {
        printf("===== slave_upgrade v%s 开始升级 =====\n",
               TOOL_VERSION);
        r = upgrade_run(&opts);
    }

    switch (r) {
    case UPG_OK:          return 0;
    case UPG_ERR_SERIAL:  return 2;
    case UPG_ERR_FILE:    return 3;
    case UPG_ERR_TIMEOUT: return 4;
    case UPG_ERR_RETRY:   return 5;
    case UPG_ERR_ABORT:   return 6;
    default:              return 1;
    }
}
