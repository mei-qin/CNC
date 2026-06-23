# 五轴伺服系统Makefile（终极版：解决osal.h找不到+纯TAB+适配所有路径）
# 适配环境：
# 1. libsoem.so 路径：/home/server/SOEM/build
# 2. SOEM源码根目录：/home/server/SOEM
# 3. osal.h 路径：/home/server/SOEM/src/osal
SOEM_PATH = /home/meiqin/SOEM
INC_PATH  = ./inc
SRC_PATH  = ./src
BIN_PATH  = ./

CC = gcc
# 核心修正：新增SOEM的src/osal和src/nicdrv路径，解决osal.h/nicdrv.h找不到
CFLAGS = -Wall -O2 \
-I$(INC_PATH) \
-I$(SOEM_PATH)/include \
-I$(SOEM_PATH)/build/include \
-I$(SOEM_PATH)/osal \
-I$(SOEM_PATH)/osal/linux \
-I$(SOEM_PATH)/oshw/linux \
-DLINUX -D_GNU_SOURCE \
-pthread

# 链接参数：libsoem.so在SOEM/build目录，无需修改
LDFLAGS = -L$(SOEM_PATH)/build \
-lsoem \
-lpthread -lrt -lm

TARGET = $(BIN_PATH)/cnc_core
SRC = $(wildcard $(SRC_PATH)/*.c)
OBJ = $(patsubst $(SRC_PATH)/%.c, $(SRC_PATH)/%.o, $(SRC))

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)
	@echo -e "\033[32m编译成功！可执行文件：$@\033[0m"

$(SRC_PATH)/%.o: $(SRC_PATH)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)
	@echo "编译：$< -> $@"

clean:
	rm -rf $(OBJ) $(TARGET)
	@echo -e "\033[32m清理完成！\033[0m"

run:
	sudo $(TARGET) enp7s0
