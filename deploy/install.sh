#!/usr/bin/env bash
# 安装 / 卸载 iec104-slave.service（边缘 104 子站）
#
# ⚠️ 默认装的是**共存模式**（config/points-pi.json）：不占串口，
#    可以和 cems-* 服务同时跑。切独占模式请改 unit 里的 --points，并先停 P1。
#
# 用法：
#   sudo bash deploy/install.sh            # 安装（不自动 enable）
#   sudo bash deploy/install.sh --enable   # 安装并开机自启
#   sudo bash deploy/uninstall.sh          # 卸载

set -euo pipefail

NAME="iec104-slave.service"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${NAME}"
DEST="/etc/systemd/system/${NAME}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_USER="${SUDO_USER:-ddf}"

if [[ ! -f "${SRC}" ]]; then
    echo "找不到 unit 文件：${SRC}" >&2
    exit 1
fi

echo "仓库路径：${REPO}"
echo "运行用户：${RUN_USER}"

# unit 里写死了 /home/ddf/iec104-rtu；仓库不在那儿就按实际路径替换，避免装了跑不起来。
sed -e "s#/home/ddf/iec104-rtu#${REPO}#g" -e "s#^User=.*#User=${RUN_USER}#" \
    -e "s#^Group=.*#Group=${RUN_USER}#" "${SRC}" > "${DEST}"
chmod 644 "${DEST}"

systemctl daemon-reload
echo "已安装：${DEST}"
echo
echo "启动：  sudo systemctl start ${NAME}"
echo "状态：  systemctl status ${NAME}"
echo "日志：  journalctl -u ${NAME} -f"

if [[ "${1:-}" == "--enable" ]]; then
    systemctl enable "${NAME}"
    echo "已设为开机自启。"
fi
