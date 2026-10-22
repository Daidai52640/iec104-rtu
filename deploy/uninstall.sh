#!/usr/bin/env bash
# 卸载 iec104-slave.service（单元文件备份到 /var/backups/iec104-systemd/）

set -euo pipefail

NAME="iec104-slave.service"
DEST="/etc/systemd/system/${NAME}"
BACKUP="/var/backups/iec104-systemd"

systemctl stop "${NAME}" 2>/dev/null || true
systemctl disable "${NAME}" 2>/dev/null || true

if [[ -f "${DEST}" ]]; then
    mkdir -p "${BACKUP}"
    mv "${DEST}" "${BACKUP}/${NAME}.$(date +%Y%m%d-%H%M%S)"
    echo "已备份并移除：${DEST} -> ${BACKUP}"
else
    echo "未安装：${DEST}"
fi

systemctl daemon-reload
