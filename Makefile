# 五轴伺服系统 Makefile
# 
# 目标:
#   make          — 编译 rpc_server (生产入口, MoveControl 联调)
#   make test     — 编译 cnc_test (测试工具, scanf 交互菜单)
#   make clean    — 清理
#
# ---- SOEM (EtherCAT 主站): 仓库内 vendored linux 子集, 源内编译 (见 third_party/soem/PROVENANCE.md) ----
# 2026-11-24 前为外部预编译库 /home/meiqin/SOEM; 现默认随仓库 clone 即可 make, 可用环境变量临时覆盖
SOEM_PATH ?= ./third_party/soem
OPCUA_PATH = ./third_party/open62541
INC_PATH  = ./inc
SRC_PATH  = ./src
RPC_PATH  = ./rpc
BIN_PATH  = ./

CC = gcc
CFLAGS = -Wall -O2 \
-I$(INC_PATH) \
-I$(RPC_PATH) \
-I$(OPCUA_PATH) \
-I$(SOEM_PATH)/include \
-I$(SOEM_PATH)/osal \
-I$(SOEM_PATH)/osal/linux \
-I$(SOEM_PATH)/oshw/linux \
-DLINUX -D_GNU_SOURCE \
-pthread

LDFLAGS = -lpthread -lrt -lm

# ---- open62541 (OPC UA, v1.5.3 官方 amalgamation, 与 movecontrol 端同版本) ----
OPCUA_OBJ = $(OPCUA_PATH)/open62541.o

# ---- SOEM 对象 (9 核心 + linux OSAL + linux 网卡层, 与上游 soem 库目标同集) ----
SOEM_OBJ = $(patsubst $(SOEM_PATH)/%.c, $(SOEM_PATH)/%.o, $(wildcard $(SOEM_PATH)/src/*.c)) \
$(SOEM_PATH)/osal/linux/osal.o \
$(SOEM_PATH)/oshw/linux/nicdrv.o \
$(SOEM_PATH)/oshw/linux/oshw.o

# ---- 公共源文件 (src/*.c 除 main.c) ----
KERNEL_SRC = $(filter-out $(SRC_PATH)/main.c, $(wildcard $(SRC_PATH)/*.c))
KERNEL_OBJ = $(patsubst $(SRC_PATH)/%.c, $(SRC_PATH)/%.o, $(KERNEL_SRC))

# ---- rpc_server (生产入口) ----
# P0-a: 改用 wildcard 自动收集 rpc/*.c (rpc_server.c + rpc_push_server.c)
# .cpp 文件 (SmcControllerSdk / snapshot_subscriber) 是 Windows 客户端 SDK, 不进 Linux 服务端
RPC_TARGET = $(BIN_PATH)/rpc_server
RPC_SRC    = $(wildcard $(RPC_PATH)/*.c)
RPC_OBJ    = $(patsubst $(RPC_PATH)/%.c, $(RPC_PATH)/%.o, $(RPC_SRC))

# ---- cnc_test (测试工具) ----
TEST_TARGET = $(BIN_PATH)/cnc_test
TEST_SRC    = $(SRC_PATH)/main.c
TEST_OBJ    = $(SRC_PATH)/main.o

# 默认: 编译 rpc_server
all: $(RPC_TARGET)

# 编译测试工具
test: $(TEST_TARGET)

# 编译两者
both: $(RPC_TARGET) $(TEST_TARGET)

$(RPC_TARGET): $(KERNEL_OBJ) $(RPC_OBJ) $(OPCUA_OBJ) $(SOEM_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)
	@echo -e "\033[32m编译成功！可执行文件：$@\033[0m"
	@echo "  启动: sudo ./rpc_server sim"

$(TEST_TARGET): $(KERNEL_OBJ) $(TEST_OBJ) $(SOEM_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)
	@echo -e "\033[32m编译成功！可执行文件：$@\033[0m"
	@echo "  启动: sudo ./cnc_test sim"

$(SRC_PATH)/%.o: $(SRC_PATH)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)
	@echo "编译: $< -> $@"

$(RPC_PATH)/%.o: $(RPC_PATH)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)
	@echo "编译: $< -> $@"

# open62541 amalgamation 单文件编译 (仅需 pthread, 无外部依赖)
$(OPCUA_PATH)/%.o: $(OPCUA_PATH)/%.c
	$(CC) -c $< -o $@ -O2 -pthread
	@echo "编译: $< -> $@"

# SOEM 源内编译 (vendored 子集, 复用上方 CFLAGS 的 soem 头路径)
$(SOEM_PATH)/%.o: $(SOEM_PATH)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)
	@echo "编译: $< -> $@"

# ---- opcua_probe: OPC UA 冒烟测试客户端 (纯客户端, 不链接内核/SOEM) ----
# 模拟 movecontrol: connect → Browse CNC.Axis → 读契约节点 → 写倍率 → 调方法
PROBE_TARGET = $(BIN_PATH)/opcua_probe

probe: $(PROBE_TARGET)

$(PROBE_TARGET): scripts/opcua_probe.c $(OPCUA_OBJ)
	$(CC) -o $@ $^ -I$(OPCUA_PATH) -pthread
	@echo -e "\033[32m编译成功！可执行文件：$@\033[0m"
	@echo "  前置: sudo ./rpc_server sim 已在跑"

clean:
	rm -f $(KERNEL_OBJ) $(SRC_PATH)/main.o $(RPC_PATH)/*.o $(RPC_TARGET) $(TEST_TARGET) $(OPCUA_OBJ) $(PROBE_TARGET) \
	      $(SOEM_PATH)/src/*.o $(SOEM_PATH)/osal/linux/*.o $(SOEM_PATH)/oshw/linux/*.o
	@echo -e "\033[32m清理完成！\033[0m"

run:
	sudo $(RPC_TARGET) sim

run-test:
	sudo $(TEST_TARGET) sim
