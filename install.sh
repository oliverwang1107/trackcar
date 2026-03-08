#!/usr/bin/env bash
# =============================================================================
# AprilTag 追蹤小車 — 安裝腳本
# 請在 Raspberry Pi 5 上以 sudo 權限執行：
#   chmod +x install.sh && sudo ./install.sh
# =============================================================================

set -euo pipefail

echo "=========================================="
echo "  AprilTag 追蹤小車 — 環境安裝"
echo "=========================================="

# ---- 系統層依賴 ----
echo ""
echo "[1/4] 更新套件清單..."
apt-get update -y

echo ""
echo "[2/4] 安裝系統依賴..."
apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    libopencv-dev \
    v4l-utils \
    libatlas-base-dev \
    libhdf5-dev \
    libharfbuzz0b \
    liblapack-dev \
    libcblas-dev

# ---- Python 虛擬環境 ----
echo ""
echo "[3/4] 建立 Python 虛擬環境..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/venv"

if [ ! -d "${VENV_DIR}" ]; then
    python3 -m venv "${VENV_DIR}" --system-site-packages
    echo "    虛擬環境建立於：${VENV_DIR}"
else
    echo "    虛擬環境已存在，跳過建立。"
fi

# ---- Python 套件 ----
echo ""
echo "[4/4] 安裝 Python 依賴..."
"${VENV_DIR}/bin/pip" install --upgrade pip
"${VENV_DIR}/bin/pip" install -r "${SCRIPT_DIR}/requirements.txt"

echo ""
echo "=========================================="
echo "  安裝完成！"
echo ""
echo "  啟用虛擬環境："
echo "    source ${VENV_DIR}/bin/activate"
echo ""
echo "  執行小車程式："
echo "    python3 main.py"
echo "=========================================="
