"""规模化测试：点表变大之后，总召唤还跑得动吗？

点表刚配置化，现在可以随便加点数——但**没验证过大量点时的行为**。
这个文件回答三个问题：
    ① 一个 ASDU 最多 127 个对象，连续段有没有正确切开（不切就直接抛异常）
    ② 500 个点的总召唤要多久（窗口 k=12 时，确认机制会不会把它拖死）
    ③ 对端不确认时，窗口 k 是不是真的把发送卡住

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_scale.py
"""

import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    Iec104Slave, Station, TypeID, decode_apdu, decode_asdu,
    encode_s_frame,
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


def recv_apdus(sock: socket.socket, buf: bytearray):
    """把缓冲区里能拆的帧全拆出来。"""
    out = []
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
        frame = bytes(buf[:total])
        del buf[:total]
        apdu = decode_apdu(frame)
        if apdu is not None:
            out.append(apdu)
    return out


def run_gi(port: int, total: int, ack: bool, timeout: float):
    """发总召唤并收完。

    返回 (耗时, 收到 I 帧数, 数据对象数, 是否收到激活终止)
    """
    sock = socket.create_connection(("127.0.0.1", port), timeout=3)
    sock.sendall(STARTDT_ACT)
    time.sleep(0.1)
    buf = bytearray()
    sock.recv(4096)          # 吃掉 STARTDT con

    sock.sendall(GI_FRAME)
    t0 = time.monotonic()
    rx = 0
    frames = 0
    objs = 0
    done = False

    while time.monotonic() - t0 < timeout:
        r, _, _ = select.select([sock], [], [], 0.2)
        if not r:
            continue
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf.extend(chunk)
        for apdu in recv_apdus(sock, buf):
            if not apdu.is_i:
                continue
            frames += 1
            rx = (rx + 1) % 32768
            asdu = decode_asdu(apdu.asdu)
            if asdu is None:
                continue
            if asdu.cot == 20:
                objs += len(asdu.objects)
            if asdu.type_id == 100 and asdu.cot == 10:
                done = True
            if ack:
                sock.sendall(encode_s_frame(rx))
        if done:
            break

    elapsed = time.monotonic() - t0
    sock.close()
    return elapsed, frames, objs, done


class Harness:
    def __init__(self, n_meas: int = 250, n_single: int = 250, k: int = 12):
        self.port = free_port()
        self.station = Station(CA)
        for i in range(n_meas):
            self.station.add(i + 1, int(TypeID.M_ME_NC_1), 23.7)
        for i in range(n_single):
            self.station.add(n_meas + i + 1, int(TypeID.M_SP_NA_1), True)
        self.n_total = n_meas + n_single
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, cyclic_s=0, poll_s=0, k=k, w=8)
        self.slave.start()
        time.sleep(0.2)

    def close(self):
        self.slave.stop()
        time.sleep(0.1)


# ================================================================ ① 分组上限
def test_grouping_splits_at_127_objects():
    """⭐ 一个 ASDU 最多 127 个对象，连续段必须在 127 处切开。"""
    st = Station(1)
    for ioa in range(1, 251):
        st.add(ioa, int(TypeID.M_ME_NC_1), 1.0)

    groups = st.group_consecutive(st.all())
    assert [len(g) for g in groups] == [127, 123], [len(g) for g in groups]

    # 每个组都必须能真的编成 ASDU（这才是 bug 的真正症状）
    from iec104 import InformationObject, encode_asdu
    for g in groups:
        encode_asdu(int(TypeID.M_ME_NC_1), 20, 1,
                    [InformationObject(p.ioa, p.value) for p in g],
                    sq=len(g) > 1)


def test_127_exactly_stays_one_group():
    st = Station(1)
    for ioa in range(1, 128):
        st.add(ioa, int(TypeID.M_ME_NC_1), 1.0)
    groups = st.group_consecutive(st.all())
    assert [len(g) for g in groups] == [127]


# ================================================================ ② 500 点总召唤
def test_500_points_interrogation_all_arrive():
    """500 个点全部上送，且能收到激活终止。"""
    h = Harness(250, 250)
    try:
        elapsed, frames, objs, done = run_gi(h.port, h.n_total, ack=True, timeout=30)
        print(f"      → 500 点：{frames} 帧 / {objs} 对象 / {elapsed * 1000:.1f} ms")
        assert done, f"{elapsed:.1f}s 内没收到激活终止"
        assert objs == h.n_total, f"只收到 {objs} 个对象，期望 {h.n_total}"
        assert frames <= 10, f"分组后应该是少数几帧，实际 {frames} 帧"
    finally:
        h.close()


def test_500_points_interrogation_is_fast():
    """500 点总召唤应在秒级完成（本地回环）。"""
    h = Harness(250, 250)
    try:
        elapsed, _, objs, done = run_gi(h.port, h.n_total, ack=True, timeout=30)
        assert done and objs == h.n_total
        assert elapsed < 5.0, f"总召唤耗时 {elapsed:.2f}s，偏慢"
    finally:
        h.close()


# ================================================================ ③ 窗口 k
def test_window_k_blocks_sending_without_ack():
    """⭐ 对端不确认时，窗口 k 必须把发送卡住（不能让未确认帧无限增长）。

    ⚠️ 必须把 k 调小才有意义：500 个点打包后只有 10 帧，k=12 时根本不会触顶。
    """
    h = Harness(250, 250, k=3)
    try:
        elapsed, frames, objs, done = run_gi(h.port, h.n_total, ack=False, timeout=2.0)
        print(f"      → 不确认 2s：只发出 {frames} 帧（k=3）")
        assert not done, "不确认的情况下不该完成"
        assert frames <= 3, f"未确认帧 {frames} 超过窗口 k=3"
        assert frames >= 1, "至少应该有确认帧发出来"
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
