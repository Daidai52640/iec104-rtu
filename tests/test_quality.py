"""品质位映射测试：设备掉线 → IV（无效），绝不把旧值当好值。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_quality.py
"""

import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    COT, Iec104Slave, MemoryActuator, QDS, Station, StubSource, TypeID,
    clear_faults, decode_apdu, decode_asdu, describe, is_good, mark_invalid, mark_stale,
)

CA = 1
STARTDT_ACT = bytes.fromhex("680407000000")
GI_FRAME = bytes.fromhex("680E0000000064010600FFFF00000014")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def recv_frames(sock: socket.socket, idle: float = 0.5, max_total: float = 3.0):
    """收帧，直到连续 idle 秒没数据；**并有硬上限 max_total**，防止刷屏卡死。"""
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


def asdus(frames):
    out = []
    for f in frames:
        a = decode_apdu(f)
        if a is not None and a.is_i:
            d = decode_asdu(a.asdu)
            if d is not None:
                out.append(d)
    return out


# ================================================================ 品质位工具
def test_quality_helpers():
    assert is_good(0) is True
    assert is_good(QDS.IV) is False

    q = mark_invalid(0)
    assert q == QDS.IV and is_good(q) is False

    q = mark_stale(0)
    assert q == QDS.NT

    # 读到了 → 清掉 IV/NT，但保留 BL/SB/OV
    q = clear_faults(QDS.IV | QDS.NT | QDS.BL)
    assert q == QDS.BL, hex(q)

    assert describe(0) == "good"
    assert "iv" in describe(QDS.IV)


# ================================================================ 点表落地
def test_apply_marks_missing_points_invalid():
    """⭐ 读不到的点 → IV，且不能保留旧值当"好值"。"""
    st = Station(1)
    st.add(1, int(TypeID.M_ME_NC_1), 0.0, name="T")
    st.add(2, int(TypeID.M_ME_NC_1), 0.0, name="H")

    # ① 正常读到
    changed = st.apply({1: (23.7, 0), 2: (62.4, 0)})
    assert {p.ioa for p in changed} == {1, 2}
    assert is_good(st.get(1).quality) and is_good(st.get(2).quality)

    # ② 只读到 1（值保持 23.7 不变）→ 只有 2 变成 IV
    changed = st.apply({1: (23.7, 0)})
    assert [p.ioa for p in changed] == [2], [p.ioa for p in changed]
    assert is_good(st.get(1).quality) is True
    assert st.get(2).quality & QDS.IV, "读不到的点必须标 IV"
    assert st.get(2).value == 62.4, "旧值保留但品质标坏（不冒充新值）"

    # ③ 全部读不到 → 两个都 IV
    st.apply({})
    assert st.get(1).quality & QDS.IV

    # ④ 恢复 → IV 被清掉
    st.apply({1: (24.0, 0), 2: (63.0, 0)})
    assert is_good(st.get(1).quality) and is_good(st.get(2).quality)


def test_apply_no_change_returns_empty():
    st = Station(1)
    st.add(1, int(TypeID.M_ME_NC_1), 23.7)
    st.apply({1: (23.7, 0)})
    assert st.apply({1: (23.7, 0)}) == [], "值没变不应产生变化点"


def test_invalidate_all():
    st = Station(1)
    st.add(1, int(TypeID.M_ME_NC_1), 0.0)
    st.apply({1: (1.0, 0)})
    changed = st.invalidate_all()
    assert len(changed) == 1 and changed[0].quality & QDS.IV


def test_command_points_are_not_invalidated_by_polling():
    """⭐ 命令点不是采集来的，不能因为"数据源没返回"就被标成 IV。"""
    st = Station(1)
    st.add(1, int(TypeID.M_ME_NC_1), 0.0)
    st.add(10, int(TypeID.C_SC_NA_1), False)
    st.bind_command(10, 0, 1)

    st.apply({1: (23.7, 0)})
    assert is_good(st.get(10).quality), "命令点品质不应被采集循环改动"

    st.apply({})   # 数据源全挂
    assert st.get(1).quality & QDS.IV, "遥测应无效"
    assert is_good(st.get(10).quality), "命令点仍不应被标无效"


# ================================================================ 端到端
class Harness:
    def __init__(self, poll_s: float = 0.1):
        self.port = free_port()
        self.station = Station(CA)
        self.station.add(1, int(TypeID.M_ME_NC_1), 0.0, cyclic=True, name="T")
        self.station.add(2, int(TypeID.M_ME_NC_1), 0.0, cyclic=True, name="H")
        self.station.add(5, int(TypeID.M_SP_NA_1), False, name="DI1")
        self.station.add(6, int(TypeID.M_SP_NA_1), True, name="DI2")

        self.act = MemoryActuator(channels=8)
        self.source = StubSource(actuator=self.act, readback_map={0: 5, 1: 6})
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, actuator=self.act,
                                 source=self.source, poll_s=poll_s)
        self.slave.start()
        time.sleep(0.15)
        self.tx = 0
        self.rx = 0
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.sock.sendall(STARTDT_ACT)
        recv_frames(self.sock, idle=0.2)

    def gi(self):
        """发一次总召唤。

        ⚠️ N(S) / N(R) 必须递增：子站会校验接收序号连续性，
           **同一个 tx 重发**（比如每次总召唤都用 GI_FRAME 里的 tx=0）
           会被当成重放/乱序直接断链。
        """
        tx = (self.tx % 32768) << 1
        rx = (self.rx % 32768) << 1
        frame = (GI_FRAME[:2]
                 + bytes((tx & 0xFF, (tx >> 8) & 0xFF, rx & 0xFF, (rx >> 8) & 0xFF))
                 + GI_FRAME[6:])
        self.sock.sendall(frame)
        self.tx = (self.tx + 1) % 32768

        frames = recv_frames(self.sock, idle=0.5)
        for f in frames:
            a = decode_apdu(f)
            if a is not None and a.is_i:
                self.rx = (a.tx + 1) % 32768
        return asdus(frames)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.slave.stop()
        time.sleep(0.1)


def test_interrogation_reports_invalid_when_device_lost():
    """设备掉线后总召唤 → 所有点带 IV（而不是继续报旧值当好值）。"""
    h = Harness()
    try:
        good = h.gi()
        vals = [o for a in good for o in a.objects if a.cot == int(COT.INTERROGATED_BY_STATION)]
        assert vals, "总召唤应有数据"
        assert all(is_good(o.quality) for o in vals), \
            [(o.ioa, describe(o.quality)) for o in vals]

        # 模拟设备掉线
        h.source.fail = True
        time.sleep(0.4)                      # 让采集循环跑几轮
        recv_frames(h.sock, idle=0.2)        # 清掉随之而来的突发帧

        got = h.gi()
        vals = [o for a in got for o in a.objects if a.cot == int(COT.INTERROGATED_BY_STATION)]
        assert vals, "总召唤应有数据"
        bad = [o for o in vals if not is_good(o.quality)]
        assert len(bad) == len(vals), \
            f"掉线后所有点都应为 IV，实际 {[(o.ioa, describe(o.quality)) for o in vals]}"
        assert all(o.quality & QDS.IV for o in vals)
    finally:
        h.close()


def test_quality_change_is_reported_spontaneously():
    """品质变化（好→坏）本身要被突发上送。"""
    h = Harness()
    try:
        h.gi()
        recv_frames(h.sock, idle=0.2)

        h.source.fail = True
        frames = asdus(recv_frames(h.sock, idle=0.8))

        assert frames, "品质变化后应有突发上送"
        spont = [a for a in frames if a.cot == int(COT.SPONTANEOUS)]
        assert spont, "应有 COT=3 的突发帧"
        ios = {o.ioa: o for a in spont for o in a.objects}
        assert 1 in ios and 2 in ios, f"遥测点应被上报，实际 {list(ios)}"
        assert ios[1].quality & QDS.IV, "突发帧里品质应为 IV"
    finally:
        h.close()


def test_recovery_clears_invalid():
    """设备恢复 → 品质回到良好，并再次突发上送。"""
    h = Harness()
    try:
        h.source.fail = True
        time.sleep(0.4)
        recv_frames(h.sock, idle=0.2)
        assert h.station.get(1).quality & QDS.IV

        h.source.fail = False
        frames = asdus(recv_frames(h.sock, idle=0.8))
        assert not (h.station.get(1).quality & QDS.IV), "恢复后应清掉 IV"
        assert any(a.cot == int(COT.SPONTANEOUS) for a in frames), "恢复应有突发上送"
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
