"""遥控「选择-执行」两段式的字节级测试。

思路：**不依赖任何第三方库**，直接用裸 socket 发手工拼好的字节，
看子站回什么字节。所有期望帧都是硬编码的 16 进制，避免"自己测自己"。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_sbo.py
"""

import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    COT, Iec104Slave, MemoryActuator, Station, TypeID,
    decode_apdu, decode_asdu,
)

CA = 1
CMD_IOA = 10        # 遥控点
OUT_IDX = 0         # 执行器通道
RB_IOA = 5          # 回读映射到的遥信点


def H(s: str) -> bytes:
    return bytes.fromhex(s.replace(" ", ""))


# ---------------------------------------------------------------- 手工拼的帧
STARTDT_ACT = H("68 04 07 00 00 00")
STOPDT_ACT = H("68 04 13 00 00 00")

# 选择：C_SC_NA_1, COT=6, CA=1, IOA=10, SCO=0x81(选择+合)
SEL_FRAME = H("68 0E 00 00 00 00  2D 01 06 00 01 00 0A 00 00 81")
# 执行：同上去掉选择位，SCO=0x01(执行+合)；tx=1 rx=1
EXE_FRAME = H("68 0E 02 00 02 00  2D 01 06 00 01 00 0A 00 00 01")
# 同一条执行帧，但作为连接上的**第一帧**时 N(S) 必须是 0（子站会校验接收序号）
EXE_FRAME_FIRST = H("68 0E 00 00 00 00  2D 01 06 00 01 00 0A 00 00 01")
# 未知 IOA 的选择
SEL_UNKNOWN = H("68 0E 00 00 00 00  2D 01 06 00 01 00 63 00 00 81")


# ---------------------------------------------------------------- 工具
def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def recv_frames(sock: socket.socket, idle: float = 0.6) -> list:
    """收到连续帧，直到空闲 idle 秒为止。"""
    buf = bytearray()
    out = []
    end = time.time() + idle
    while True:
        left = end - time.time()
        if left <= 0:
            # 再等一个很短的窗口，确认真的没数据了
            r, _, _ = select.select([sock], [], [], 0.05)
        else:
            r, _, _ = select.select([sock], [], [], left)
        if not r:
            break
        chunk = sock.recv(4096)
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
    """把帧列表解成 ASDU 列表（跳过 S/U 帧）。"""
    out = []
    for f in frames:
        a = decode_apdu(f)
        if a is not None and a.is_i:
            d = decode_asdu(a.asdu)
            if d is not None:
                out.append(d)
    return out


class Harness:
    """起一个子站 + 一个裸 socket 连接。"""

    def __init__(self, select_timeout: float = 30.0):
        self.port = free_port()
        self.station = Station(CA, name="sbo")
        self.station.add(5, int(TypeID.M_SP_NA_1), False, name="DI1")
        self.station.add(6, int(TypeID.M_SP_NA_1), True, name="DI2")
        self.station.add(CMD_IOA, int(TypeID.C_SC_NA_1), False, name="DO1")
        self.station.bind_command(CMD_IOA, OUT_IDX, RB_IOA)

        self.act = MemoryActuator(channels=8)
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, actuator=self.act,
                                 select_timeout=select_timeout)
        self.slave.start()
        time.sleep(0.2)

        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.sock.sendall(STARTDT_ACT)
        recv_frames(self.sock, idle=0.3)

    def send(self, frame: bytes):
        self.sock.sendall(frame)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.slave.stop()
        time.sleep(0.1)


# ---------------------------------------------------------------- 用例
def test_select_then_execute_success():
    """正常两段式：选择→确认，执行→确认+变位上送。"""
    h = Harness()
    try:
        # ① 选择
        h.send(SEL_FRAME)
        got = asdus(recv_frames(h.sock))
        assert len(got) >= 1, "选择后应有确认"
        sel_ack = got[0]
        assert sel_ack.type_id == int(TypeID.C_SC_NA_1)
        assert sel_ack.cot == int(COT.ACTIVATION_CONFIRM)
        assert sel_ack.negative is False, "选择应被肯定确认"
        assert sel_ack.objects[0].ioa == CMD_IOA
        assert sel_ack.objects[0].select is True, "确认应回显选择位"
        assert h.act.snapshot()[OUT_IDX] is False, "只选择不应动作"

        # ② 执行
        h.send(EXE_FRAME)
        got = asdus(recv_frames(h.sock))
        assert len(got) >= 2, "执行后应有确认 + 变位上送"

        exe_ack = got[0]
        assert exe_ack.type_id == int(TypeID.C_SC_NA_1)
        assert exe_ack.cot == int(COT.ACTIVATION_CONFIRM)
        assert exe_ack.negative is False, "执行应被肯定确认"
        assert exe_ack.objects[0].select is False, "确认应回显执行位"

        spont = got[1]
        # 变位的 TypeID 跟点表声明走：本用例 IOA 5 声明为 M_SP_NA_1 → 突发帧仍是 1
        assert spont.type_id == int(TypeID.M_SP_NA_1), \
            "M_SP_NA_1 的变位应仍按 TypeID=1 上送"
        assert spont.cot == int(COT.SPONTANEOUS), "变位的 COT 应为 3(突发)"
        assert spont.objects[0].ioa == RB_IOA
        assert spont.objects[0].value is True
        assert spont.objects[0].timestamp is None, "M_SP_NA_1 的帧不该带时标"

        # ③ 硬件侧状态
        assert h.act.snapshot()[OUT_IDX] is True, "执行器通道应被拉高"
        assert h.station.get(RB_IOA).value is True, "回读应写回遥信点"
        assert h.station.get(RB_IOA).timestamp is not None, "点表里应记下变位时刻"
    finally:
        h.close()


def test_execute_without_select_is_rejected():
    """没先选择直接执行 → 否定确认，且不动作。"""
    h = Harness()
    try:
        # 这一帧是连接上的第一帧，N(S) 必须是 0（子站校验接收序号）
        h.send(EXE_FRAME_FIRST)
        got = asdus(recv_frames(h.sock))
        assert got, "应有回复"
        assert got[0].cot == int(COT.ACTIVATION_CONFIRM)
        assert got[0].negative is True, "未选择就执行应被否定"
        assert h.act.snapshot()[OUT_IDX] is False, "被拒绝时不应动作"
    finally:
        h.close()


def test_select_timeout_is_rejected():
    """选择超时后执行 → 否定确认。"""
    h = Harness(select_timeout=0.3)
    try:
        h.send(SEL_FRAME)
        asdus(recv_frames(h.sock, idle=0.2))
        time.sleep(0.5)                      # 超过选择超时
        h.send(EXE_FRAME)
        got = asdus(recv_frames(h.sock))
        assert got, "应有回复"
        assert got[0].negative is True, "选择超时后执行应被否定"
        assert h.act.snapshot()[OUT_IDX] is False
    finally:
        h.close()


def test_unknown_ioa_is_rejected():
    """未绑定的 IOA → 否定确认。"""
    h = Harness()
    try:
        h.send(SEL_UNKNOWN)
        got = asdus(recv_frames(h.sock))
        assert got, "应有回复"
        assert got[0].negative is True
    finally:
        h.close()


def test_stopdt_clears_pending_select():
    """STOPDT 后未执行的选择作废：重新 STARTDT 再执行 → 未先选择。"""
    h = Harness()
    try:
        h.send(SEL_FRAME)                     # 选择（tx=0）
        asdus(recv_frames(h.sock, idle=0.2))
        h.send(STOPDT_ACT)                    # U 帧，不影响 N(S)
        asdus(recv_frames(h.sock, idle=0.2))
        h.send(STARTDT_ACT)
        asdus(recv_frames(h.sock, idle=0.2))

        h.send(EXE_FRAME)                     # 执行（tx=1）
        got = asdus(recv_frames(h.sock))
        assert got, "应有回复"
        assert got[0].negative is True, "STOPDT 后选择状态应被清空"
        assert h.act.snapshot()[OUT_IDX] is False, "被拒绝时不应动作"
    finally:
        h.close()


def test_apply_failure_is_reported_as_negative():
    """执行器拒绝 → 否定确认，不上送变位。"""
    h = Harness()
    try:
        h.act.fail_apply = True
        h.send(SEL_FRAME)
        asdus(recv_frames(h.sock, idle=0.2))
        h.send(EXE_FRAME)
        got = asdus(recv_frames(h.sock))
        assert got[0].negative is True, "执行失败应否定确认"
        assert len(got) == 1, "失败时不应有变位上送"
    finally:
        h.close()


def test_readback_unavailable_is_not_success():
    """回读读不到 → 不能当作成功（否定确认，不上送变位）。"""
    h = Harness()
    try:
        h.act.fail_readback = True
        h.send(SEL_FRAME)
        asdus(recv_frames(h.sock, idle=0.2))
        h.send(EXE_FRAME)
        got = asdus(recv_frames(h.sock))
        assert got[0].negative is True, "回读不到必须报失败"
        assert len(got) == 1, "回读失败时不应上送变位"
    finally:
        h.close()


def test_select_is_per_connection():
    """选择状态是每连接独立的：A 选择不影响 B。"""
    h = Harness()
    try:
        h.send(SEL_FRAME)
        asdus(recv_frames(h.sock, idle=0.2))

        sock2 = socket.create_connection(("127.0.0.1", h.port), timeout=3)
        sock2.sendall(STARTDT_ACT)
        recv_frames(sock2, idle=0.2)
        sock2.sendall(H("68 0E 00 00 00 00  2D 01 06 00 01 00 0A 00 00 01"))
        got2 = asdus(recv_frames(sock2))
        assert got2 and got2[0].negative is True, "另一个连接没选择过，执行应被拒绝"
        sock2.close()
    finally:
        h.close()


# ---------------------------------------------------------------- 跑
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
