"""链路层定时器测试：t1 / t2 / t3 / W。

用**故障注入**验证，不依赖第三方库：裸 socket 故意不回确认、不回 TESTFR con，
看子站会不会按时反应（该发的发、该断的断）。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_link_timers.py
"""

import os
import select
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    Iec104Slave, MemoryActuator, Station, TypeID, decode_apdu, iter_apdus,
)

CA = 1
STARTDT_ACT = bytes.fromhex("680407000000")
STARTDT_CON = bytes.fromhex("68040B000000")
TESTFR_ACT = bytes.fromhex("680443000000")
TESTFR_CON = bytes.fromhex("680483000000")
GI_FRAME = bytes.fromhex("680E0000000064010600FFFF00000014")

# 真正"没人处理"的类型：TypeID=121（不在 1/13/30/45/100/103 之内）——
# 子站不解码、不回复，只记接收序号，正好用来观察 t2 / W 的确认行为。
# ⚠️ 以前这里用的是 C_CS_NA_1（对时），那属于拿真实协议帧当测试道具：
#    对时接进会话后子站会对它回确认，t2/W 用例就被顺带确认掉了。
UNHANDLED_T1 = bytes.fromhex("681400000000" "7901060001000000000000000000000000")
UNHANDLED_T2 = bytes.fromhex("681402000000" "7901060001000000000000000000000000")
UNHANDLED_T3 = bytes.fromhex("681404000000" "7901060001000000000000000000000000")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def recv_frames(sock: socket.socket, idle: float = 0.4, max_total: float = 3.0):
    buf = bytearray()
    out = []
    hard_end = time.time() + max_total
    end = time.time() + idle
    while True:
        if time.time() > hard_end:
            break
        left = min(end - time.time(), hard_end - time.time())
        r, _, _ = select.select([sock], [], [], left if left > 0 else 0.01)
        if not r:
            break
        chunk = sock.recv(65536)
        if not chunk:
            out.append(None)      # 标记对端关闭
            break
        buf.extend(chunk)
        while True:
            i = buf.find(0x68)
            if i < 0:
                buf.clear()
                break
            if i:
                del buf[:i]
            if len(buf) < 2:
                break
            total = buf[1] + 2
            if len(buf) < total:
                break
            out.append(bytes(buf[:total]))
            del buf[:total]
        end = time.time() + idle
    return out


def u_codes(frames):
    codes = []
    for f in frames:
        if f is None:
            continue
        a = decode_apdu(f)
        if a is not None and a.is_u:
            codes.append(a.u_code)
    return codes


def s_frames(frames):
    out = []
    for f in frames:
        if f is None:
            continue
        a = decode_apdu(f)
        if a is not None and a.is_s:
            out.append(a)
    return out


def wait_closed(sock: socket.socket, timeout: float) -> bool:
    """等对端关闭（recv 返回空）。"""
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([sock], [], [], max(0.01, end - time.time()))
        if not r:
            continue
        try:
            if not sock.recv(65536):
                return True
        except OSError:
            return True
    return False


def s_frame(rx: int) -> bytes:
    """手工拼一个 S 帧（确认到 rx）。"""
    v = (rx % 32768) << 1
    return bytes((0x68, 0x04, 0x01, 0x00, v & 0xFF, (v >> 8) & 0xFF))


def i_frame_txs(frames):
    out = []
    for f in frames:
        if f is None:
            continue
        a = decode_apdu(f)
        if a is not None and a.is_i:
            out.append(a.tx)
    return out


class Harness:
    def __init__(self, **timers):
        self.port = free_port()
        self.station = Station(CA)
        self.station.add(1, int(TypeID.M_ME_NC_1), 23.7)
        self.station.add(5, int(TypeID.M_SP_NA_1), True)

        self.act = MemoryActuator(channels=8)
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, actuator=self.act, **timers)
        self.slave.start()
        time.sleep(0.15)
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.sock.sendall(STARTDT_ACT)
        recv_frames(self.sock, idle=0.2)

    def session(self):
        deadline = time.time() + 2
        while time.time() < deadline:
            s = self.slave.sessions
            if s:
                return s[0]
            time.sleep(0.02)
        return None

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.slave.stop()
        time.sleep(0.1)


# ================================================================ t1
def test_t1_aborts_when_ack_never_comes():
    """发出 I 帧后一直不确认 → t1 到点断链。"""
    h = Harness(t1=0.4, t2=0.3, t3=60)
    try:
        h.sock.sendall(GI_FRAME)
        frames = recv_frames(h.sock, idle=0.2)
        assert any(f is not None for f in frames), "总召唤应有回帧"

        closed = wait_closed(h.sock, timeout=2.0)
        assert closed, "t1 超时后应主动断链"

        sess = None
        for s in h.slave._sessions:
            if s.abort_reason:
                sess = s
        assert sess is not None, "应记录断链原因"
        assert "t1" in sess.abort_reason, sess.abort_reason
    finally:
        h.close()


def test_t1_not_triggered_when_ack_arrives():
    """正常确认时不应误断。"""
    h = Harness(t1=0.4, t2=0.3, t3=60)
    try:
        h.sock.sendall(GI_FRAME)
        frames = recv_frames(h.sock, idle=0.2)
        txs = i_frame_txs(frames)
        assert txs, "总召唤应回 I 帧"
        # 按子站实际发出的序号确认（超前确认会被子站当异常忽略）
        h.sock.sendall(s_frame((max(txs) + 1) % 32768))
        time.sleep(0.6)
        assert not wait_closed(h.sock, timeout=0.3), "已确认不应断链"
    finally:
        h.close()


# ================================================================ t3
def test_t3_sends_testfr_when_idle():
    """空闲到 t3 → 发 TESTFR act；回 con 后链路保持。"""
    h = Harness(t1=1.5, t2=0.3, t3=0.3)
    try:
        frames = recv_frames(h.sock, idle=0.8, max_total=2.0)
        codes = u_codes(frames)
        assert 0x43 in codes, f"应发 TESTFR act(0x43)，实际收到 {[hex(c) for c in codes]}"

        h.sock.sendall(TESTFR_CON)
        time.sleep(0.3)
        assert not wait_closed(h.sock, timeout=0.3), "回了 con 就不该断链"
    finally:
        h.close()


def test_t3_aborts_when_testfr_unanswered():
    """TESTFR act 没有 con → 按 t1 断链。"""
    h = Harness(t1=0.5, t2=0.3, t3=0.3)
    try:
        closed = wait_closed(h.sock, timeout=3.0)
        assert closed, "TESTFR 无应答应断链"
        reasons = [s.abort_reason for s in h.slave._sessions if s.abort_reason]
        assert any("TESTFR" in r for r in reasons), reasons
    finally:
        h.close()


# ================================================================ t2 / W
def test_t2_delayed_ack():
    """收到 I 帧但没有回帧 → t2 到点补发 S 帧。"""
    h = Harness(t1=60, t2=0.3, t3=60, w=8)
    try:
        h.sock.sendall(UNHANDLED_T1)
        started = time.time()
        frames = recv_frames(h.sock, idle=0.5, max_total=2.0)
        elapsed = time.time() - started
        sfs = s_frames(frames)
        assert sfs, "t2 到点应发 S 帧"
        assert elapsed >= 0.2, f"S 帧来得太早（{elapsed:.2f}s），说明没走 t2"
        assert sfs[0].rx == 1, f"S 帧 rx 应为 1，实际 {sfs[0].rx}"
    finally:
        h.close()


def test_w_triggers_ack_before_t2():
    """攒够 W 个 I 帧 → 不等 t2，立即确认。"""
    h = Harness(t1=60, t2=30, t3=60, w=2)
    try:
        h.sock.sendall(UNHANDLED_T1)
        recv_frames(h.sock, idle=0.15, max_total=0.5)
        h.sock.sendall(UNHANDLED_T2)

        started = time.time()
        frames = recv_frames(h.sock, idle=0.5, max_total=2.0)
        elapsed = time.time() - started
        sfs = s_frames(frames)
        assert sfs, "第 W 个 I 帧后应确认"
        assert elapsed < 2.0, f"确认太慢（{elapsed:.2f}s）"
        assert sfs[0].rx == 2, f"应确认到 2，实际 {sfs[0].rx}"
    finally:
        h.close()


def test_t2_anchors_on_first_unacked_frame():
    """⭐ t2 锚点 = 第一帧未确认 I 帧到达的时刻，不是"最近收到帧"。

    慢速点滴场景：每 0.8s 一帧、t2=0.3s。
    正确：第 1 帧后一个 0.2s 定时拍内就发 S 帧（实测约 0.4s，第 2 帧 0.8s 才来），
          且此时只确认到 rx=1。
    旧实现：每收一帧就刷新 `_last_recv`，S 帧被推到第 2 帧之后 ——
    主站侧 t1 先超时断链，现场表现是"慢速数据偶尔掉线"。
    """
    h = Harness(t1=60, t2=0.3, t3=60, w=8)
    try:
        h.sock.sendall(UNHANDLED_T1)       # 第 1 帧（tx=0），t2 从此刻起算
        t0 = time.time()

        def drip():
            for frame in (UNHANDLED_T2, UNHANDLED_T3):
                time.sleep(0.8)
                h.sock.sendall(frame)

        threading.Thread(target=drip, daemon=True).start()

        first_s = None       # 第一帧 S 帧到达的相对时刻
        first_s_rx = None
        buf = bytearray()
        end = t0 + 2.2
        while time.time() < end:
            r, _, _ = select.select([h.sock], [], [], 0.03)
            if not r:
                continue
            chunk = h.sock.recv(65536)
            if not chunk:
                break
            buf.extend(chunk)
            for frame in iter_apdus(buf):
                a = decode_apdu(frame)
                if a is not None and a.is_s and first_s is None:
                    first_s = time.time() - t0
                    first_s_rx = a.rx

        assert first_s is not None, "t2 到点应发 S 帧"
        assert first_s < 0.8, (
            f"S 帧应在第 2 帧之前（t2=0.3s + 一个 0.2s 定时拍）出现，实际 {first_s:.2f}s；"
            f"≥0.8s 说明 t2 锚点被后续帧刷新了")
        assert first_s >= 0.2, f"S 帧来得太早（{first_s:.2f}s），说明没走 t2"
        assert first_s_rx == 1, f"此时只该确认到 1，实际 {first_s_rx}"
    finally:
        h.close()


def test_wrong_ns_aborts_session():
    """接收序号 N(S) 不连续（重放 / 乱序 / fuzz）→ 直接断链。

    连接上期望 N(S)=0，却收到一帧 tx=2 的 I 帧（UNHANDLED_T3 的控制域是 04 00）。
    序号失步会让后续所有确认错位，所以必须断链重来，不能"继续猜"。
    """
    h = Harness(t1=60, t2=0.3, t3=60)
    try:
        h.sock.sendall(UNHANDLED_T3)       # tx=2，跳过了 0/1
        assert wait_closed(h.sock, timeout=2.0), "N(S) 不连续应直接断链"
        reasons = [s.abort_reason for s in h.slave._sessions if s.abort_reason]
        assert any("N(S)" in r for r in reasons), reasons
        assert any("期望 0" in r and "收到 2" in r for r in reasons), reasons
    finally:
        h.close()


def test_window_k_limits_inflight():
    """窗口 K 限制：未确认帧数不会超过 k。"""
    h = Harness(t1=60, t2=30, t3=60, k=2, w=8)
    try:
        h.sock.sendall(GI_FRAME)          # 总召唤会一次回 3~4 帧
        time.sleep(0.5)
        sess = h.session()
        assert sess is not None
        assert sess._tx_unacked() <= 2, f"窗口占用 {sess._tx_unacked()} 超过 k=2"
    finally:
        h.close()


# ================================================================ 跑
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
