"""systemd sd_notify 测试：不在 systemd 下必须完全无害，在 systemd 下必须真发得出去。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_sdnotify.py
"""

import os
import socket
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import sdnotify  # noqa: E402


def test_noop_without_notify_socket():
    """不在 systemd 下 ⇒ 所有调用都是空操作，不抛异常、返回 False。"""
    saved = sdnotify._NOTIFY_SOCKET
    try:
        sdnotify._NOTIFY_SOCKET = None
        assert sdnotify.enabled() is False
        assert sdnotify.ready() is False
        assert sdnotify.watchdog() is False
        assert sdnotify.stopping() is False
        assert sdnotify.status("x") is False
    finally:
        sdnotify._NOTIFY_SOCKET = saved


def test_sends_when_notify_socket_set():
    """在 systemd 下 ⇒ 真的把 'READY=1' 发到那个 unix 数据报套接字上。"""
    if not hasattr(socket, "AF_UNIX"):
        print("      (跳过：本平台没有 AF_UNIX)")
        return

    tmpdir = tempfile.mkdtemp()
    path = os.path.join(tmpdir, "notify.sock")
    try:
        rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        rx.bind(path)
        rx.settimeout(2.0)

        saved = sdnotify._NOTIFY_SOCKET
        try:
            sdnotify._NOTIFY_SOCKET = path
            assert sdnotify.enabled() is True
            assert sdnotify.ready() is True
            data = rx.recv(256)
            assert data == b"READY=1", data

            assert sdnotify.watchdog() is True
            assert rx.recv(256) == b"WATCHDOG=1"

            assert sdnotify.stopping() is True
            assert rx.recv(256) == b"STOPPING=1"

            assert sdnotify.status("运行中") is True
            assert rx.recv(256).decode("utf-8") == "STATUS=运行中"
        finally:
            sdnotify._NOTIFY_SOCKET = saved
            rx.close()
    finally:
        for name in ("notify.sock",):
            try:
                os.unlink(os.path.join(tmpdir, name))
            except OSError:
                pass
        try:
            os.rmdir(tmpdir)
        except OSError:
            pass


def test_never_raises_on_bad_socket():
    """套接字地址不存在时也不能把服务搞崩——只是返回 False。"""
    saved = sdnotify._NOTIFY_SOCKET
    try:
        sdnotify._NOTIFY_SOCKET = "/definitely/not/here/notify.sock"
        assert sdnotify.ready() is False
    finally:
        sdnotify._NOTIFY_SOCKET = saved


def _run_all():
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed.append((name, e))
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print("-" * 60)
    print(f"{len(fns) - len(failed)}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
