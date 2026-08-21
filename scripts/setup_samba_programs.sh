#!/usr/bin/env bash
# =====================================================================
# setup_samba_programs.sh —— 程序共享目录 (SMB) 一键部署
#
# Syntec 生态对等的文件通道 (新代 = 共享文件系统 + 路径触发加载):
#   上位 movecontrol (Windows) 经 \\<本机IP>\cnc-programs 写入 .nc 文件,
#   再调 OPC UA CNC.Program.Load(路径) —— 传 Windows 形式路径/UNC/裸文件
#   名均可, CNC 端按 basename 在共享根目录解析 (rpc/opcua_server.c
#   resolve_program_path)。
#
# 用法 (实机 Linux PC):
#   sudo bash scripts/setup_samba_programs.sh          # 默认 /home/meiqin/nc
#   sudo SMC_PROGRAM_DIR=/srv/cnc bash scripts/setup_samba_programs.sh
#
# 前置: samba 已安装 (Ubuntu: sudo apt install samba)
# 幂等: 配置段已存在则跳过追加
# =====================================================================
set -e

SHARE_DIR="${SMC_PROGRAM_DIR:-/home/meiqin/nc}"
SHARE_NAME="cnc-programs"
SMB_CONF="/etc/samba/smb.conf"
MARKER="# --- CNC program share (opcua_server) ---"

echo "[setup] 共享目录: $SHARE_DIR"
mkdir -p "$SHARE_DIR"
chmod 775 "$SHARE_DIR"

if ! command -v smbd >/dev/null 2>&1; then
    echo "[setup] samba 未安装。请先执行:"
    echo "        sudo apt install samba"
    exit 1
fi

if [ ! -f "$SMB_CONF" ]; then
    echo "[setup] 未找到 $SMB_CONF, 请确认 samba 安装完整"
    exit 1
fi

if grep -qF "$MARKER" "$SMB_CONF"; then
    echo "[setup] smb.conf 已含配置段, 跳过追加"
else
    # 备份 + 追加共享段 (登录访问, 仅授权账号可写)
    cp "$SMB_CONF" "$SMB_CONF.bak.$(date +%Y%m%d%H%M%S)"
    cat >> "$SMB_CONF" << EOF

$MARKER
[$SHARE_NAME]
   path = $SHARE_DIR
   browseable = yes
   read only = no
   guest ok = no
   create mask = 0664
   directory mask = 0775
EOF
    echo "[setup] 已追加共享段 [$SHARE_NAME] -> $SHARE_DIR"
fi

echo "[setup] 提示: 上位访问需要 samba 账号, 如未设置过执行:"
echo "        sudo smbpasswd -a \$USER"
echo
echo "[setup] 重启 smbd 生效..."
if command -v systemctl >/dev/null 2>&1; then
    systemctl restart smbd
    systemctl enable smbd 2>/dev/null || true
else
    service smbd restart
fi

IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "[setup] 完成。Windows 侧访问: \\\\${IP:-<本机IP>}\\$SHARE_NAME"
echo "[setup] 加载调用示例: CNC.Program.Load(\"\\\\\\\\${IP:-<本机IP>}\\\\$SHARE_NAME\\\\test.nc\")"
echo "[setup]           或: CNC.Program.Load(\"test.nc\")  (basename 回退同一文件)"
