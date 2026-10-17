"""遥控「动作约束」的测试：执行前判定，任一不满足就否定确认且**不驱动执行器**。

四条约束：
    ① 设备离线：绑定的回读点品质带 IV
    ② 互斥组：同组内**其它**通道当前回读为 ON
    ③ 最小间隔：同一 IOA 距上次成功执行不足 min_interval
    ④ IOA 未绑定遥控（原有行为，一并回归）

⚠️ 本文件只验"约束判定 + 拒绝时不动作"，不验 104 字节编码 ——
   ASDU/APCI 的逐字节对拍在 `test_iec104.py` 与 `test_sbo.py` 里，
   这里拼命令帧用工程自带的编码函数，避免重复维护两份十六进制。

跑法（在项目根目录，pwsh）：
    $env:PYTHONPATH="src"
    python tests\\test_constraints.py
"""

import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    COT, Iec104Slave, MemoryActuator, Session, Station, TypeID,
    decode_apdu, decode_asdu, mark_invalid,
)
from iec104.asdu import InformationObject, encode_asdu  # noqa: E402
from iec104.config import ConfigError, station_from_dict  # noqa: E402
from iec104.link import encode_i_frame  # noqa: E402

CA = 1
DI1_IOA = 5          # DO1 的回读点
DI2_IOA = 6          # DO2 的回读点
DO1_IOA = 10         # 遥控点 1 → 通道 0
DO2_IOA = 11         # 遥控点 2 → 通道 1
CH1, CH2 = 0, 1
MUTEX = "relay"
STARTDT_ACT = bytes.fromhex("680407000000")


# ---------------------------------------------------------------- 工具
def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def recv_frames(sock: socket.socket, idle: float = 0.4, max_total: float = 3.0) -> list:
    """收帧，直到连续 idle 秒没数据；有硬上限 max_total，防止刷屏卡死。"""
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


class Harness:
    """起一个子站（两个互斥的遥控点）+ 一个裸 socket 连接。"""

    def __init__(self, min_interval: float = 5.0):
        self.port = free_port()
        self.station = Station(CA, name="constraints")
        # DI2 初值给 False：互斥用例里再显式把它置 ON，
        # 免得"其它通道默认就 ON"把所有用例都带偏。
        self.station.add(DI1_IOA, int(TypeID.M_SP_NA_1), False, name="DI1")
        self.station.add(DI2_IOA, int(TypeID.M_SP_NA_1), False, name="DI2")
        self.station.add(DO1_IOA, int(TypeID.C_SC_NA_1), False, name="DO1")
        self.station.add(DO2_IOA, int(TypeID.C_SC_NA_1), False, name="DO2")
        self.station.bind_command(DO1_IOA, CH1, DI1_IOA, mutex_group=MUTEX)
        self.station.bind_command(DO2_IOA, CH2, DI2_IOA, mutex_group=MUTEX)

        self.act = MemoryActuator(channels=8)
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, actuator=self.act,
                                 min_interval=min_interval)
        self.slave.start()
        time.sleep(0.15)

        self.tx = 0
        self.rx = 0
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.sock.sendall(STARTDT_ACT)
        self.recv_asdus()

    def recv_asdus(self) -> list:
        """收一轮帧，解成 ASDU；顺手把 rx 推进到对端 tx+1（别让发送窗口堵住）。"""
        out = []
        for f in recv_frames(self.sock):
            apdu = decode_apdu(f)
            if apdu is None:
                continue
            if apdu.is_i:
                self.rx = (apdu.tx + 1) % 32768
                d = decode_asdu(apdu.asdu)
                if d is not None:
                    out.append(d)
        return out

    def command(self, ioa: int, select: bool, on: bool = True) -> list:
        """发一条 C_SC_NA_1（选择或执行），返回子站的回复 ASDU 列表。"""
        asdu = encode_asdu(int(TypeID.C_SC_NA_1), COT.ACTIVATION, CA,
                           [InformationObject(ioa, on, select=select)])
        self.sock.sendall(encode_i_frame(self.tx, self.rx, asdu))
        self.tx += 1
        return self.recv_asdus()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.slave.stop()
        time.sleep(0.05)


def ack_of(got: list):
    """从回复里取命令确认帧（COT=7）。"""
    for a in got:
        if a.type_id == int(TypeID.C_SC_NA_1) and a.cot == int(COT.ACTIVATION_CONFIRM):
            return a
    return None


def _unit_session(st: Station, act: MemoryActuator,
                  min_interval: float = 1.0) -> Session:
    """造一个**不跑线程**的 Session，只用来直接调 `_check_constraints`。

    conn 给 None 是安全的：判定逻辑不碰 socket，`run()` 才会。
    """
    return Session(conn=None, addr=("unit", 0), station=st, verbose=False,
                   actuator=act, min_interval=min_interval)


def _base_config(do_extra=None) -> dict:
    """一份最小点表：遥信 IOA=5 + 遥控 IOA=10（可附加字段）。"""
    cmd = {"ioa": 10, "type": "C_SC_NA_1", "actuator": 0, "readback_ioa": 5}
    cmd.update(do_extra or {})
    return {"common_address": 1, "points": [
        {"ioa": 5, "type": "M_SP_NA_1"},
        cmd,
    ]}


def expect_config_error(cfg, *tokens):
    """断言 `station_from_dict` 抛 ConfigError，且错误信息包含所有 tokens。"""
    try:
        station_from_dict(cfg)
    except ConfigError as e:
        msg = str(e)
    else:
        raise AssertionError("非法配置应当抛 ConfigError，但实际载入成功了")
    for token in tokens:
        assert token in msg, f"错误信息应包含 {token!r}，实际：{msg}"
    return msg


# ---------------------------------------------------------------- ① 设备离线
def test_offline_readback_rejected():
    """回读点品质带 IV → 拒绝执行，且执行器不动作。"""
    h = Harness()
    try:
        # 模拟设备离线：回读点品质变无效
        h.station.get(DI1_IOA).quality = mark_invalid(0)

        got = h.command(DO1_IOA, select=True)
        assert ack_of(got) is not None and ack_of(got).negative is False, "选择应仍能确认"

        got = h.command(DO1_IOA, select=False, on=True)
        ack = ack_of(got)
        assert ack is not None, "执行应有确认"
        assert ack.negative is True, "设备离线时执行应被否定确认"
        assert h.act.snapshot()[CH1] is False, "被拒绝时执行器绝不能动作"
        assert h.station.get(DI1_IOA).value is False, "回读点不应被改写"
    finally:
        h.close()


# ---------------------------------------------------------------- ② 互斥组
def test_mutex_group_rejected():
    """同组内其它通道当前为 ON → 拒绝执行，且执行器不动作。"""
    h = Harness()
    try:
        # 另一路（DO2/通道 1）当前是 ON —— 现场状态由它的回读点表示
        h.station.get(DI2_IOA).value = True

        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=True)
        ack = ack_of(got)
        assert ack is not None
        assert ack.negative is True, "互斥组内已有通道为 ON 时执行应被否定确认"
        assert h.act.snapshot()[CH1] is False, "被拒绝时执行器绝不能动作"
        assert h.act.snapshot()[CH2] is False, "另一路也不该被碰"

        # 反向确认互斥不是"一刀切"：别的组/不同组不参与判定（这里先关掉对方）
        h.station.get(DI2_IOA).value = False
        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=True)
        assert ack_of(got).negative is False, "对方为 OFF 后应允许动作"
        assert h.act.snapshot()[CH1] is True
    finally:
        h.close()


# ---------------------------------------------------------------- ③ 最小间隔
def test_min_interval_rejected():
    """同一 IOA 两次成功执行间隔不足 → 第二次拒绝，且执行器保持第一次的状态。"""
    h = Harness(min_interval=5.0)
    try:
        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=True)
        assert ack_of(got).negative is False, "第一次执行应成功"
        assert h.act.snapshot()[CH1] is True

        # 立刻反向执行（期望 False）：若真的动作了，通道会变成 False
        time.sleep(0.05)
        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=False)
        ack = ack_of(got)
        assert ack is not None
        assert ack.negative is True, "间隔不足时第二次执行应被否定确认"
        assert h.act.snapshot()[CH1] is True, "被拒绝时执行器应保持上一次的状态"
    finally:
        h.close()


# ---------------------------------------------------------------- ④ 未绑定 IOA
def test_unbound_ioa_rejected():
    """IOA 未绑定遥控 → 否定确认，且没有任何通道被驱动。"""
    h = Harness()
    try:
        got = h.command(99, select=True)
        ack = ack_of(got)
        assert ack is not None
        assert ack.negative is True, "未绑定 IOA 应被否定确认"
        assert ack.objects[0].ioa == 99, "确认应回显原 IOA"
        assert h.act.snapshot() == [False] * 8, "未绑定 IOA 不应驱动任何通道"
    finally:
        h.close()


# ---------------------------------------------------------------- ⑤ 选择值 ≠ 执行值
def test_select_value_mismatch_rejected():
    """选择「合」后执行「分」→ 否定确认，且执行器不动。

    ⭐ 选择是"预演"，执行必须与预演一致；不一致还照做，等于两段式白做。
    """
    h = Harness()
    try:
        h.command(DO1_IOA, select=True, on=True)          # 选择：合
        got = h.command(DO1_IOA, select=False, on=False)  # 执行：分
        ack = ack_of(got)
        assert ack is not None
        assert ack.negative is True, "选择值与执行值不一致应被否定确认"
        assert h.act.snapshot()[CH1] is False, "被拒绝时执行器绝不能动作"

        # 反向再验一次：选择「分」后执行「合」同样拒绝
        h.command(DO1_IOA, select=True, on=False)
        got = h.command(DO1_IOA, select=False, on=True)
        assert ack_of(got).negative is True
        assert h.act.snapshot()[CH1] is False
    finally:
        h.close()


# ---------------------------------------------------------------- 拒绝原因可区分
def test_reason_codes_are_distinct():
    """三条约束给出不同的短码，便于日志排查；且点表可覆盖会话默认间隔。"""
    st = Station(CA)
    st.add(DI1_IOA, int(TypeID.M_SP_NA_1), False)
    st.add(DI2_IOA, int(TypeID.M_SP_NA_1), False)
    st.add(DO1_IOA, int(TypeID.C_SC_NA_1), False)
    st.add(DO2_IOA, int(TypeID.C_SC_NA_1), False)
    st.bind_command(DO1_IOA, CH1, DI1_IOA, mutex_group=MUTEX)
    st.bind_command(DO2_IOA, CH2, DI2_IOA, mutex_group=MUTEX)

    act = MemoryActuator(channels=8)
    sess = _unit_session(st, act, min_interval=5.0)
    b = st.binding(DO1_IOA)

    assert sess._check_constraints(b) is None, "条件都满足时应放行"

    st.get(DI1_IOA).quality = mark_invalid(0)
    assert sess._check_constraints(b)[0] == "offline"
    st.get(DI1_IOA).quality = 0

    st.get(DI2_IOA).value = True
    assert sess._check_constraints(b)[0] == "mutex"
    st.get(DI2_IOA).value = False

    sess._last_exec[DO1_IOA] = time.monotonic()
    reason = sess._check_constraints(b)
    assert reason[0] == "interval", reason
    assert "5.00s" in reason[1], reason[1].lower()

    # 点表里写 min_interval=0 覆盖会话默认值 → 不再拦间隔
    st2 = Station(CA)
    st2.add(DI1_IOA, int(TypeID.M_SP_NA_1), False)
    st2.add(DO1_IOA, int(TypeID.C_SC_NA_1), False)
    st2.bind_command(DO1_IOA, CH1, DI1_IOA, min_interval=0.0)
    sess2 = _unit_session(st2, act, min_interval=5.0)
    sess2._last_exec[DO1_IOA] = time.monotonic()
    assert sess2._check_constraints(st2.binding(DO1_IOA)) is None


# ---------------------------------------------------------------- 配置校验
def test_config_declares_constraints():
    """点表能声明 mutex_group / min_interval，并落到绑定上。"""
    st = station_from_dict(_base_config({"mutex_group": " relay ", "min_interval": 0.5}))
    b = st.binding(10)
    assert b.mutex_group == "relay", "组名两端空白应被去掉"
    assert b.min_interval == 0.5

    # 不写：两者都是 None（用会话默认值）
    b2 = station_from_dict(_base_config()).binding(10)
    assert b2.mutex_group is None and b2.min_interval is None


def test_config_rejects_bad_mutex_group():
    for bad in ("", "   ", 123, True, ["relay"]):
        expect_config_error(_base_config({"mutex_group": bad}),
                            "points[1]", "mutex_group")

    # 只有命令点可以声明
    cfg = _base_config()
    cfg["points"][0]["mutex_group"] = MUTEX
    expect_config_error(cfg, "points[0]", "mutex_group", "C_SC_NA_1")


def test_config_rejects_bad_min_interval():
    for bad in (-1, "1", True, [1]):
        expect_config_error(_base_config({"min_interval": bad}),
                            "points[1]", "min_interval")

    cfg = _base_config()
    cfg["points"][0]["min_interval"] = 1.0
    expect_config_error(cfg, "points[0]", "min_interval", "C_SC_NA_1")


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
