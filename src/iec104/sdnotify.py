"""systemd sd_notify：向 systemd 报「就绪」和「心跳」。

⚠️ 纯标准库：直接往 `$NOTIFY_SOCKET` 这个 unix 数据报套接字写文本。
   不在 systemd 下运行时，所有函数都是**空操作** —— 本地调试完全不受影响。

⭐ 为什么要有心跳：`Restart=always` 只能救"进程死了"。
   "进程活着、但采集循环卡住不出数"这种故障它救不了 —— 那才是现场最难查的。
   `Type=notify` + `WatchdogSec` 就是为这个存在的。
"""

import os
import socket
from typing import Optional

#: systemd 注入的套接字地址；不在 systemd 下时为 None
_NOTIFY_SOCKET: Optional[str] = os.environ.get("NOTIFY_SOCKET")


def _send(message: str) -> bool:
    """发一条通知。成功返回 True；不在 systemd 下或发送失败返回 False。"""
    if not _NOTIFY_SOCKET:
        return False
    if not hasattr(socket, "AF_UNIX"):       # Windows 等平台直接跳过
        return False
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # 抽象命名空间以 '@' 开头，内核侧要换成前导 \0
        addr = "\0" + _NOTIFY_SOCKET[1:] if _NOTIFY_SOCKET.startswith("@") else _NOTIFY_SOCKET
        sock.connect(addr)
        sock.sendall(message.encode("utf-8"))
        sock.close()
        return True
    except OSError:
        return False


def ready() -> bool:
    """告诉 systemd：服务已经起来了（`Type=notify` 必须发这个，否则超时被杀）。"""
    return _send("READY=1")


def watchdog() -> bool:
    """喂狗。必须在 `WatchdogSec` 之内周期性发，否则 systemd 判定"活着但不动"。"""
    return _send("WATCHDOG=1")


def stopping() -> bool:
    """告诉 systemd：正在优雅停止。"""
    return _send("STOPPING=1")


def status(text: str) -> bool:
    """把一行状态挂到 unit 上（`systemctl status` 可见）。"""
    return _send(f"STATUS={text}")


def enabled() -> bool:
    """当前是否跑在 systemd 的 notify 模式下。"""
    return bool(_NOTIFY_SOCKET)
