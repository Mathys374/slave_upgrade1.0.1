#ifndef MD5_H
#define MD5_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t st[4];      /* A/B/C/D 状态字 */
    uint64_t nbytes;     /* 已处理的字节总数 */
    uint8_t  buf[64];    /* 未满 64 字节的缓冲 */
    size_t   fill;       /* buf 中已有字节数 */
} md5_ctx_t;

void md5_init(md5_ctx_t *c);
void md5_update(md5_ctx_t *c, const void *data, size_t len);

/* 标准 MD5 收尾（含填充），输出 16 字节摘要（小端） */
void md5_final(md5_ctx_t *c, uint8_t out[16]);

/*
 * 兼容模式收尾：不追加填充块，直接输出当前状态。
 * 仅用于复现 Windows 工具（FileDownload V1.08）在文件大小恰为
 * 64 字节整数倍时的非常规 MD5 结果（见 --legacy-md5 选项）。
 */
void md5_final_nopad(md5_ctx_t *c, uint8_t out[16]);

/* 对内存缓冲一次性计算。
 * legacy_nopad 非 0 且 len % 64 == 0 时走兼容路径，否则按标准 MD5。 */
void md5_buf(const void *data, size_t len, uint8_t out[16], int legacy_nopad);

#ifdef __cplusplus
}
#endif

#endif /* MD5_H */
