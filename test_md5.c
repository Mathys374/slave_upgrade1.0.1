/*
 * test_md5.c —— md5.c 正确性自测（开发用，不属于交付物）
 * 用法: test_md5 <文件路径>
 * 输出文件的标准 MD5 与 legacy 模式 MD5，与系统工具（md5sum/Get-FileHash）比对。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "md5.h"

int main(int argc, char **argv)
{
    FILE *fp;
    uint8_t *buf;
    long fsz;
    uint8_t md5[16];
    char hex[33];
    unsigned i;

    if (argc != 2) {
        fprintf(stderr, "usage: %s <file>\n", argv[0]);
        return 1;
    }
    fp = fopen(argv[1], "rb");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    fsz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buf = (uint8_t *)malloc(fsz ? (size_t)fsz : 1);
    if (fsz > 0 && fread(buf, 1, (size_t)fsz, fp) != (size_t)fsz) {
        fprintf(stderr, "read error\n");
        return 1;
    }
    fclose(fp);

    /* 标准 MD5 */
    md5_buf(buf, (size_t)fsz, md5, 0);
    for (i = 0; i < 16; i++)
        snprintf(hex + i * 2, 3, "%02x", md5[i]);
    printf("std    %s  %ld  %s\n", hex, fsz, argv[1]);

    /* legacy（64 字节整数倍时不填充） */
    md5_buf(buf, (size_t)fsz, md5, 1);
    for (i = 0; i < 16; i++)
        snprintf(hex + i * 2, 3, "%02x", md5[i]);
    printf("legacy %s  %ld  %s\n", hex, fsz, argv[1]);

    free(buf);
    return 0;
}
