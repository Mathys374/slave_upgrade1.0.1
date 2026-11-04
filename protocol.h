#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 升级协议常量（与 Windows 工具 FileDownload V1.08 完全一致）
 *   块格式：A1 A2 A3 A4 + 偏移(4B,LE) + 长度(4B,LE) + 数据 + XOR校验(4B,LE)
 *   XOR 校验初值为 0xA4A3A2A1，覆盖 偏移+长度+数据（数据不足 4 字节按 0 补齐）
 *   MD5 块：偏移 = 0xFFFFFFFE，长度 = 16，数据为固件 MD5 摘要
 */
#define UPG_TX_PAYLOAD_MAX ((uint32_t)(8192  - 16))  /* 单块数据上限 8176 字节（8KB-16） */
#define UPG_FILE_SIZE_MAX  (8u * 1024u * 1024u - 16u)     /* 固件大小上限 8MB-16 */

typedef enum {
    UPG_OK = 0,
    UPG_ERR_FILE = 1,      /* 固件文件读取/尺寸错误 */
    UPG_ERR_SERIAL = 2,    /* 串口打开或读写错误 */
    UPG_ERR_TIMEOUT = 3,   /* 等待下位机响应超时 */
    UPG_ERR_RETRY = 4,     /* 连续重传次数超限 */
    UPG_ERR_ABORT = 5      /* 用户中断（Ctrl+C） */
} upg_err_t;

typedef struct {
    const char   *port;
    unsigned int  init_baud;        /* 复位阶段波特率（模块 AT 固件当前波特率），默认 115200；
                                       下载波特率固定 921600（SBL 复位后固定速率） */
    const char   *filepath;
    unsigned int  pre_listen_ms;    /* 打开串口后预监听时长（下位机可能已在 bootloader） */
    unsigned int  boot_timeout_ms;  /* 每次 AT+S402=1 后等待就绪帧 '#' 的时长 */
    int           reset_tries;      /* AT+S402=1 复位重试次数 */
    unsigned int  block_timeout_ms; /* 发送数据块后等待应答时长 */
    int           retry_max;        /* 连续无进展重传上限 */
    unsigned int  overall_timeout_ms; /* 总超时，0 = 不限制 */
    unsigned int  listen_secs;      /* 成功后监听启动信息的秒数，0 = 关闭 */
    int           legacy_md5;       /* 兼容 Windows 工具在 64 字节整数倍文件上的 MD5 行为 */
    int           quiet;
    int           verbose;
} upg_opts_t;

/* 失败原因（中文，供最终输出） */
const char *upg_err_str(upg_err_t e);

/* 完整升级流程：读文件 -> MD5 -> 复位 -> 分块传输 -> MD5 校验 -> 确认写入 */
upg_err_t upgrade_run(upg_opts_t *o);

/* 辅助模式：
 *   query_version=1: 先发 AT+V? 查询版本，无响应则复位并读取启动信息
 *   query_version=0: 仅复位（AT+S402=1）并回显启动信息
 */
upg_err_t probe_run(upg_opts_t *o, int query_version);

/* main.c 中定义，信号处理函数将其置 1，协议层轮询以响应用户中断 */
extern volatile sig_atomic_t upg_abort_flag;

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
