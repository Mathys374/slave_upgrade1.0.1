# slave_upgrade —— 专网模块 Slave 固件串口升级工具（Linux CLI）
#
# 常用目标:
#   make                    本机编译（在任务板上直接编译）
#   make static             静态链接（推荐：单个二进制可在 Ubuntu 20.04/22.04 通用）
#   make CROSS=aarch64-linux-gnu-   交叉编译（x86 主机上编译 aarch64 目标）
#   make clean              清理构建产物
#
# 示例:
#   make CROSS=aarch64-buildroot-linux-gnu- static  # 交叉编译出客户静态 aarch64 二进制



CROSS   ?=
CC      := $(CROSS)gcc

CFLAGS  ?= -O2
CFLAGS  += -std=gnu11 -Wall -Wextra
LDFLAGS ?=

SRCS    := main.c protocol.c serial.c md5.c
OBJS    := $(SRCS:.c=.o)
TARGET  := slave_upgrade

.PHONY: all static clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# 静态链接版本：无运行时依赖，便于 scp 到任务板直接运行
static:
	$(CC) $(CFLAGS) -static $(LDFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -f $(OBJS) $(TARGET)
