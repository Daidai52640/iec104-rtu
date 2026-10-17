"""对时（C_CS_NA_1）与 SOE 时标上送的端到端测试。

⭐ 本文件的核心规矩：**一个点的线类型由点表声明的 `type` 一锤定音，全路径一致**。
    同一个 IOA 不能"总召唤发 M_SP_NA_1、突发发 M_SP_TB_1"——
    主站按自己点表里的类型收帧，对不上就丢弃（互操作实测丢过变位）。

覆盖：
    ① 收到 103 + COT=6 → 回 COT=7 的确认，**时标逐字节回显**
    ② 对时偏移确实被记住（`ClockOffset`），后续时标走"对时后的钟"
    ③ 点声明 `M_SP_NA_1`：总召唤与突发**都是 TypeID=1、都不带时标**（互操作回归守门）
    ④ 点声明 `M_SP_TB_1`：总召唤与突发**都是 TypeID=30、都带时标**（含遥控成功后的变位）
    ⑤ 遥测（M_ME_NC_1）的突发仍是 13、不带时标
    ⑥ 时标非法（CP56Time2a 解不出来）的对时帧回**否定确认**并把 IOC 原样回显

跑法（在项目根目录，pwsh）：
    $env:PYTHONPATH="src"
    python tests\\test_soe.py
"""

import os
import select
import socket
import sys
import time
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    COT, Iec104Slave, MemoryActuator, Station, TypeID,
    decode_apdu, decode_asdu,
)
from iec104.asdu import InformationObject, encode_asdu  # noqa: E402
from iec104.clock import ClockOffset  # noqa: E402
from iec104.config import station_from_dict  # noqa: E402
from iec104.link import encode_i_frame  # noqa: E402

CA = 1
TEMP_IOA = 1         # 遥测（M_ME_NC_1）：突发仍不带时标
DI1_IOA = 5
DI2_IOA = 6
DO1_IOA = 10         # 通道 0，回读 IOA 5
DO2_IOA = 11         # 通道 1，回读 IOA 6
CH1, CH2 = 0, 1
STARTDT_ACT = bytes.fromhex("680407000000")
GI_FRAME = bytes.fromhex("680E0000000064010600FFFF00000014")

# 手工拼的对时帧：TypeID=0x67(103), VSQ=1(SQ=0,n=1), COT=6, OA=0, CA=1,
# IOA=0, CP56Time2a = 2026-10-17 18:14:16.636 UTC（FC 40 0E 12 11 0A 1A）
CLOCK_SYNC_FRAME = bytes.fromhex(
    "68 14 00 00 00 00  67 01 06 00 01 00  00 00 00  FC 40 0E 12 11 0A 1A")
# 期望的确认 ASDU：只把 COT 换成 7，其余（含时标）原样回显
CLOCK_SYNC_ACK = bytes.fromhex(
    "67 01 07 00 01 00  00 00 00  FC 40 0E 12 11 0A 1A")
SYNC_DT = datetime(2026, 10, 17, 18, 14, 16, 636000, tzinfo=timezone.utc)

# 时标非法（全 0 → 2000-00-00）的对时帧：用来验证"回否定确认 + IOC 原样回显"
CLOCK_SYNC_BAD = bytes.fromhex(
    "68 14 00 00 00 00  67 01 06 00 01 00  00 00 00  00 00 00 00 00 00 00")
# 期望的确认 ASDU：COT 06→47（7 | P/N），IOC（IOA+7 字节时标）逐字节原样
CLOCK_SYNC_BAD_ACK = bytes.fromhex(
    "67 01 47 00 01 00  00 00 00  00 00 00 00 00 00 00")


# ---------------------------------------------------------------- 工具
def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def recv_frames(sock: socket.socket, idle: float = 0.4, max_total: float = 3.0) -> list:
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
    """起一个子站（遥测 + 遥信 + 两个遥控点）+ 一个裸 socket 连接。

    `di_type` 决定两个遥信点在点表里声明成什么类型：
        - `M_SP_NA_1`（默认）：全路径 TypeID=1、不带时标（第三方主站互通的那种）
        - `M_SP_TB_1`：全路径 TypeID=30、带时标（SOE）
    """

    def __init__(self, di_type: int = int(TypeID.M_SP_NA_1)):
        self.port = free_port()
        self.di_type = di_type
        self.station = Station(CA, name="soe")
        self.station.add(TEMP_IOA, int(TypeID.M_ME_NC_1), 0.0, name="Temp")
        self.station.add(DI1_IOA, di_type, False, name="DI1")
        self.station.add(DI2_IOA, di_type, False, name="DI2")
        self.station.add(DO1_IOA, int(TypeID.C_SC_NA_1), False, name="DO1")
        self.station.add(DO2_IOA, int(TypeID.C_SC_NA_1), False, name="DO2")
        self.station.bind_command(DO1_IOA, CH1, DI1_IOA)
        self.station.bind_command(DO2_IOA, CH2, DI2_IOA)

        self.act = MemoryActuator(channels=8)
        # min_interval=0：本文件不测动作间隔，别让它挡住连续两次执行
        self.slave = Iec104Slave(self.station, host="127.0.0.1", port=self.port,
                                 verbose=False, actuator=self.act,
                                 min_interval=0.0)
        self.slave.start()
        time.sleep(0.15)

        self.tx = 0
        self.rx = 0
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.sock.sendall(STARTDT_ACT)
        self.recv_asdus()

    # ---- 收发
    def send_raw(self, frame: bytes) -> list:
        """发一帧原始 APDU（手工拼的帧也从这里走）。

        ⚠️ 发完把 `tx` 推进到该帧 N(S)+1：子站会校验接收序号连续性，
           手工帧和后面用 `send_i` 拼的帧不能撞号。
        """
        self.sock.sendall(frame)
        apdu = decode_apdu(frame)
        if apdu is not None and apdu.is_i:
            self.tx = (apdu.tx + 1) % 32768
        return self.recv_asdus()

    def recv_asdus(self) -> list:
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

    def send_i(self, asdu: bytes) -> list:
        return self.send_raw(encode_i_frame(self.tx, self.rx, asdu))

    def command(self, ioa: int, select: bool, on: bool = True) -> list:
        return self.send_i(encode_asdu(
            int(TypeID.C_SC_NA_1), COT.ACTIVATION, CA,
            [InformationObject(ioa, on, select=select)]))

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
        time.sleep(0.05)


def clock_sync_asdu(dt: datetime) -> bytes:
    """对时 ASDU（用工程自带编码器；ASDU 编码本身另有逐字节对拍用例）。"""
    return encode_asdu(int(TypeID.C_CS_NA_1), COT.ACTIVATION, CA,
                       [InformationObject(0, timestamp=dt)])


def spont_of(got: list) -> list:
    return [a for a in got if a.cot == int(COT.SPONTANEOUS)]


# ---------------------------------------------------------------- ClockOffset 单元
def test_clock_offset_unit():
    """偏移 = 主站时间 − 本地 UTC；没对时前 now() 就是本地 UTC。"""
    c = ClockOffset()
    assert c.offset == 0.0
    assert c.synced_at is None
    assert abs((c.now() - datetime.now(timezone.utc)).total_seconds()) < 1.0

    remote = datetime.now(timezone.utc).replace(microsecond=0) + timedelta(seconds=120)
    off = c.update(remote)
    assert abs(off - 120.0) < 2.0, off
    assert c.offset == off, "offset 属性应反映 update 的结果"
    assert c.synced_at is not None
    assert abs((c.now() - remote).total_seconds()) < 2.0, "now() 应回到主站时间附近"

    # 不带时区的时标按 UTC 解释
    c2 = ClockOffset()
    c2.update(remote.replace(tzinfo=None))
    assert abs(c2.offset - off) < 0.5


# ---------------------------------------------------------------- ① 对时确认
def test_clock_sync_confirmed_and_echoed():
    """发一帧 103 给子站 → 收到 COT=7 的确认，时标逐字节回显。"""
    h = Harness()
    try:
        got = h.send_raw(CLOCK_SYNC_FRAME)
        acks = [a for a in got if a.type_id == int(TypeID.C_CS_NA_1)]
        assert len(acks) == 1, f"对时应回且只回一帧确认，实际 {len(acks)} 帧"
        ack = acks[0]
        assert ack.cot == int(COT.ACTIVATION_CONFIRM), f"COT 应为 7，实际 {ack.cot}"
        assert ack.negative is False, "对时确认应为肯定"
        assert ack.ca == CA
        assert ack.objects[0].ioa == 0
        assert ack.objects[0].timestamp == SYNC_DT, "回显时标应与收到的一致"
        # 逐字节：只有 COT 从 06 变成 07，其余原样
        assert ack_raw(got) == CLOCK_SYNC_ACK

        # 偏移 = 主站时标 − 本地 UTC
        sess = h.session()
        assert sess is not None
        expect = (SYNC_DT - datetime.now(timezone.utc)).total_seconds()
        assert abs(sess.clock.offset - expect) < 2.0, \
            f"偏移 {sess.clock.offset:.1f}s 与预期 {expect:.1f}s 差太多"
    finally:
        h.close()


def test_clock_sync_updates_offset():
    """对时后 `Session.clock.now()` 应跟着主站走。"""
    h = Harness()
    try:
        remote = (datetime.now(timezone.utc) + timedelta(seconds=90)).replace(microsecond=0)
        got = h.send_i(clock_sync_asdu(remote))
        assert any(a.type_id == int(TypeID.C_CS_NA_1) for a in got), "应有对时确认"

        sess = h.session()
        assert sess is not None
        assert abs(sess.clock.offset - 90.0) < 2.0, sess.clock.offset
        assert abs((sess.clock.now() - remote).total_seconds()) < 2.0
    finally:
        h.close()


# ---------------------------------------------------------------- ③ SOE 变位
def test_spontaneous_after_sync_is_type30_and_monotonic():
    """声明成 M_SP_TB_1 的点：对时后两次遥控变位 TypeID=30、COT=3，
    时标取主站时间且单调不减。"""
    h = Harness(di_type=int(TypeID.M_SP_TB_1))
    try:
        remote = (datetime.now(timezone.utc) + timedelta(seconds=60)).replace(microsecond=0)
        h.send_i(clock_sync_asdu(remote))

        h.command(DO1_IOA, select=True)
        got1 = h.command(DO1_IOA, select=False, on=True)
        sp1 = [a for a in spont_of(got1) if a.type_id == int(TypeID.M_SP_TB_1)]
        assert sp1, "遥控成功后应有 M_SP_TB_1 变位上送"
        o1 = sp1[0].objects[0]
        assert o1.ioa == DI1_IOA
        assert o1.value is True
        assert o1.timestamp is not None, "SOE 帧必须带时标"
        # 对时 +60s 后打的时标必须落在主站时间附近（不是本地钟）
        assert o1.timestamp >= remote, f"时标 {o1.timestamp} 应不早于主站时标 {remote}"
        assert (o1.timestamp - remote).total_seconds() < 2.0

        h.command(DO2_IOA, select=True)
        got2 = h.command(DO2_IOA, select=False, on=True)
        sp2 = [a for a in spont_of(got2) if a.type_id == int(TypeID.M_SP_TB_1)]
        assert sp2, "第二次变位也应有 SOE 上送"
        o2 = sp2[0].objects[0]
        assert o2.timestamp >= o1.timestamp, \
            f"时标应单调不减：{o1.timestamp} -> {o2.timestamp}"
    finally:
        h.close()


# ---------------------------------------------------------------- ③ M_SP_NA_1 全路径
def test_na_point_keeps_type1_on_all_paths():
    """⭐ 互操作回归守门：点声明 M_SP_NA_1 → 总召唤与突发**都必须是 TypeID=1**，
    且任何帧都不带时标。

    背景：曾经"总召唤发 1、突发发 30"，主站按自己点表里的类型收帧，
    30 被当成未知点丢弃 —— 变位全丢（c104 互操作实测）。
    """
    h = Harness()                          # 默认 M_SP_NA_1
    try:
        got = h.send_raw(GI_FRAME)
        data = [a for a in got if a.cot == int(COT.INTERROGATED_BY_STATION)]
        assert data, "总召唤应有数据帧"
        types = {a.type_id for a in data}
        assert int(TypeID.M_SP_NA_1) in types, f"遥信数据应为 TypeID=1，实际 {types}"
        assert int(TypeID.M_ME_NC_1) in types, f"遥测数据应为 TypeID=13，实际 {types}"
        assert int(TypeID.M_SP_TB_1) not in types, "M_SP_NA_1 的点不能出现在 30 帧里"
        for a in data:
            for o in a.objects:
                assert o.timestamp is None, "总召唤数据不应带时标"
        ios = {o.ioa: o for a in data
               if a.type_id == int(TypeID.M_SP_NA_1) for o in a.objects}
        assert DI1_IOA in ios and DI2_IOA in ios

        # 突发（遥控成功后的变位）也必须是 1，且不带时标
        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=True)
        sp = spont_of(got)
        assert len(sp) == 1, f"执行成功应有一帧变位，实际 {len(sp)}"
        assert sp[0].type_id == int(TypeID.M_SP_NA_1), \
            f"M_SP_NA_1 的变位必须仍用 TypeID=1，实际 {sp[0].type_id}"
        o = sp[0].objects[0]
        assert o.ioa == DI1_IOA and o.value is True
        assert o.timestamp is None, "M_SP_NA_1 的帧不该带时标"
        assert h.station.get(DI1_IOA).timestamp is not None, \
            "点表里仍应记下变位时刻（只是不上线）"
    finally:
        h.close()


# ---------------------------------------------------------------- ④ M_SP_TB_1 全路径
def test_tb_point_keeps_type30_on_all_paths():
    """点声明 M_SP_TB_1 → 总召唤与突发**都是 TypeID=30、都带时标**。

    "突发"这里走的是遥控成功后的变位：变位类型跟随回读点的声明。
    """
    h = Harness(di_type=int(TypeID.M_SP_TB_1))
    try:
        got = h.send_raw(GI_FRAME)
        data = [a for a in got if a.cot == int(COT.INTERROGATED_BY_STATION)]
        tb = [a for a in data if a.type_id == int(TypeID.M_SP_TB_1)]
        assert tb, f"M_SP_TB_1 的点在总召唤里也应按 30 上送，实际 {[a.type_id for a in data]}"
        ios = {o.ioa: o for a in tb for o in a.objects}
        assert DI1_IOA in ios and DI2_IOA in ios
        assert all(o.timestamp is not None for o in ios.values()), \
            "总召唤的 30 帧必须带时标"
        assert int(TypeID.M_SP_NA_1) not in {a.type_id for a in data}, \
            "M_SP_TB_1 的点不能出现在 1 帧里"

        h.command(DO1_IOA, select=True)
        got = h.command(DO1_IOA, select=False, on=True)
        sp = [a for a in spont_of(got) if a.type_id == int(TypeID.M_SP_TB_1)]
        assert sp, "遥控成功后的变位也应按回读点声明的 30 上送"
        o = sp[0].objects[0]
        assert o.ioa == DI1_IOA and o.value is True
        assert o.timestamp is not None, "30 帧必须带时标"
    finally:
        h.close()


# ---------------------------------------------------------------- 点表声明
def test_config_accepts_tb_readback_point():
    """点表允许把 readback_ioa 指向 M_SP_TB_1 的点：上送类型由该点声明决定。"""
    st = station_from_dict({
        "common_address": 1,
        "points": [
            {"ioa": 5, "type": "M_SP_TB_1", "name": "DI1"},
            {"ioa": 10, "type": "C_SC_NA_1", "actuator": 0, "readback_ioa": 5},
        ],
    })
    assert st.get(5).type_id == int(TypeID.M_SP_TB_1)
    assert st.binding(10).readback_ioa == 5


def test_spontaneous_telemetry_keeps_unstamped_type():
    """⭐ 遥测的突发**不带时标**：仍是 M_ME_NC_1(13)，不是带时标类型。"""
    h = Harness()
    try:
        h.send_i(clock_sync_asdu(datetime.now(timezone.utc)))
        p = h.station.get(TEMP_IOA)
        p.value = 23.7
        p.timestamp = None
        sess = h.session()
        assert sess is not None
        sess.send_spontaneous_points([p])
        got = h.recv_asdus()

        tele = [a for a in spont_of(got) if a.type_id == int(TypeID.M_ME_NC_1)]
        assert tele, "遥测变化应有突发帧（M_ME_NC_1）"
        o = tele[0].objects[0]
        assert o.ioa == TEMP_IOA
        assert o.timestamp is None, "遥测突发不应带时标"
        assert abs(o.value - 23.7) < 1e-3
    finally:
        h.close()


# ---------------------------------------------------------------- 非法时标
def test_invalid_clock_sync_is_negatively_confirmed():
    """时标解不出来的对时帧：回否定确认，IOC 原样回显，且不改时钟偏移。

    ⚠️ 回显的是**收到的原字节**（全 0 时标也照抄），不能用编码器重编 ——
       103 的编码器只能编出一个合法时标。
    """
    h = Harness()
    try:
        h.sock.sendall(CLOCK_SYNC_BAD)      # 手工帧：自己收原始字节
        raw_asdus = []
        for f in recv_frames(h.sock):
            a = decode_apdu(f)
            if a is not None and a.is_i:
                raw_asdus.append(a.asdu)
        assert raw_asdus == [CLOCK_SYNC_BAD_ACK], \
            [b.hex(" ").upper() for b in raw_asdus]

        ack = decode_asdu(raw_asdus[0])
        assert ack is not None
        assert ack.type_id == int(TypeID.C_CS_NA_1)
        assert ack.cot == int(COT.ACTIVATION_CONFIRM)
        assert ack.negative is True, "非法时标必须回否定确认"

        sess = h.session()
        assert sess is not None
        assert sess.clock.synced_at is None, "非法时标不能改动时钟偏移"
    finally:
        h.close()


# ---------------------------------------------------------------- 帧 hex 自检
def ack_raw(got: list) -> bytes:
    """把收到的确认 ASDU 重新编码，供逐字节比对。

    ⚠️ 用 decode→encode 不是"自己测自己"：编码器本身在 `test_iec104.py`
       里有手拼十六进制对拍；这里比的是**子站真的回了这 16 个字节**。
    """
    for a in got:
        if a.type_id == int(TypeID.C_CS_NA_1) and a.cot == int(COT.ACTIVATION_CONFIRM):
            return encode_asdu(a.type_id, a.cot, a.ca, a.objects,
                               sq=a.sq, oa=a.oa, negative=a.negative)
    raise AssertionError("没有对时确认帧")


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
