"""104 子站骨架的单元测试。

核心思路：**用实测抓包的真实字节做逐字节对拍**。
向量来源：`_p2_iec104/实测抓包-104.md`（c104 ↔ QTester104 互操作时抓的线上字节）。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_iec104.py
或
    python -m pytest tests\\test_iec104.py -q
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104.asdu import (  # noqa: E402
    InformationObject, decode_asdu, decode_cp56, encode_asdu, encode_cp56,
)
from iec104.link import (  # noqa: E402
    decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus,
)
from iec104.station import Station  # noqa: E402
from iec104.consts import BROADCAST_CA, COT, FrameFormat, TypeID, UFrame  # noqa: E402


def H(s: str) -> bytes:
    return bytes.fromhex(s.replace(" ", ""))


# ================================================================ 链路层
def test_u_frames_match_capture():
    assert encode_u_frame(UFrame.STARTDT_ACT) == H("680407000000")
    assert encode_u_frame(UFrame.STARTDT_CON) == H("68040B000000")
    assert encode_u_frame(UFrame.TESTFR_ACT) == H("680443000000")
    assert encode_u_frame(UFrame.TESTFR_CON) == H("680483000000")


def test_s_frame_matches_capture():
    # 实测：68 04 01 00 08 00 -> Rx = 4
    assert encode_s_frame(4) == H("680401000800")
    a = decode_apdu(H("680401000800"))
    assert a is not None and a.is_s and a.rx == 4


def test_decode_u_frames():
    act = decode_apdu(H("680407000000"))
    con = decode_apdu(H("68040B000000"))
    assert act is not None and act.is_u and act.u_code == UFrame.STARTDT_ACT
    assert con is not None and con.is_u and con.u_code == UFrame.STARTDT_CON


def test_i_frame_roundtrip():
    raw = H("68 0E 00 00 00 00 64 01 06 00 FF FF 00 00 00 14")
    a = decode_apdu(raw)
    assert a is not None and a.is_i
    assert a.tx == 0 and a.rx == 0
    assert len(a.asdu) == 10
    assert encode_i_frame(0, 0, a.asdu) == raw


def test_i_frame_seq_encoding():
    # 序号在控制域里左移 1 位、低字节在前
    raw = encode_i_frame(2, 1, b"")
    assert raw[2:6] == H("04 00 02 00")
    a = decode_apdu(raw)
    assert a is not None and a.tx == 2 and a.rx == 1


def test_seq_wraps_at_32768():
    raw = encode_i_frame(32767, 32767, b"")
    a = decode_apdu(raw)
    assert a is not None and a.tx == 32767 and a.rx == 32767
    raw = encode_i_frame(0, 0, b"")
    a = decode_apdu(raw)
    assert a is not None and a.tx == 0


def test_iter_apdus_streaming():
    blob = H("680407000000") + H("680401000800") + H("68040B000000")
    buf = bytearray(blob[:5])          # 故意只喂半个帧
    frames = list(iter_apdus(buf))
    assert frames == []                 # 半个帧不吐出来
    buf.extend(blob[5:])
    frames = list(iter_apdus(buf))
    assert len(frames) == 3
    assert buf == bytearray()           # 全部消费掉


def test_iter_apdus_skips_garbage():
    buf = bytearray(b"\xff\xee" + H("680407000000"))
    frames = list(iter_apdus(buf))
    assert len(frames) == 1


# ================================================================ 应用层
def test_decode_interrogation_activation_broadcast():
    # ⭐ 实测：c104 客户端总召唤用广播 CA=0xFFFF
    a = decode_asdu(H("64 01 06 00 FF FF 00 00 00 14"))
    assert a is not None
    assert a.type_id == int(TypeID.C_IC_NA_1)
    assert a.cot == int(COT.ACTIVATION)
    assert a.ca == BROADCAST_CA
    assert a.objects[0].value == 20          # QOI = 站召唤


def test_encode_interrogation_confirm_matches_capture():
    got = encode_asdu(int(TypeID.C_IC_NA_1), COT.ACTIVATION_CONFIRM, 1,
                      [InformationObject(0, 20)])
    assert got == H("64 01 07 00 01 00 00 00 00 14")


def test_encode_interrogation_termination_matches_capture():
    got = encode_asdu(int(TypeID.C_IC_NA_1), COT.ACTIVATION_TERMINATION, 1,
                      [InformationObject(0, 20)])
    assert got == H("64 01 0A 00 01 00 00 00 00 14")


def test_encode_single_point_matches_capture():
    # 实测：68 0E ... 01 81 14 00 01 00 05 00 00 01
    got = encode_asdu(int(TypeID.M_SP_NA_1), COT.INTERROGATED_BY_STATION, 1,
                      [InformationObject(5, True)], sq=True)
    assert got == H("01 81 14 00 01 00 05 00 00 01")


def test_encode_float_matches_capture():
    # ⭐ 实测：23.7 的短浮点是小端 9A 99 BD 41
    got = encode_asdu(int(TypeID.M_ME_NC_1), COT.INTERROGATED_BY_STATION, 1,
                      [InformationObject(1, 23.7)], sq=True)
    assert got == H("0D 81 14 00 01 00 01 00 00 9A 99 BD 41 00")


def test_float_is_little_endian():
    import struct
    assert struct.pack("<f", 23.7) == H("9A 99 BD 41")
    # 大端会是这么一坨，显然不是 —— 留作反例
    assert struct.pack(">f", 23.7) == H("41 BD 99 9A")


def test_sq1_multi_object_encodes_single_ioa():
    got = encode_asdu(int(TypeID.M_ME_NC_1), COT.PERIODIC, 1,
                      [InformationObject(1, 23.7), InformationObject(2, 62.4)], sq=True)
    assert got[1] == 0x82                       # VSQ: SQ=1, n=2
    assert got[6:9] == H("01 00 00")            # 只有第一个 IOA
    a = decode_asdu(got)
    assert a is not None
    assert [o.ioa for o in a.objects] == [1, 2]  # 第二个靠 +1 推出


def test_sq0_multi_object_encodes_each_ioa():
    got = encode_asdu(int(TypeID.M_ME_NC_1), COT.PERIODIC, 1,
                      [InformationObject(1, 23.7), InformationObject(9, 62.4)], sq=False)
    assert got[1] == 0x02                        # VSQ: SQ=0, n=2
    a = decode_asdu(got)
    assert a is not None
    assert [o.ioa for o in a.objects] == [1, 9]


def test_sq1_rejects_non_consecutive_ioa():
    try:
        encode_asdu(int(TypeID.M_ME_NC_1), COT.PERIODIC, 1,
                    [InformationObject(1, 1.0), InformationObject(9, 2.0)], sq=True)
    except ValueError:
        return
    raise AssertionError("SQ=1 应该拒绝不连续的 IOA")


def test_decode_single_command_select_execute_bit():
    # SCO bit7 = S/E（1=选择，0=执行）；这里只验证解码不崩、值能取到
    a = decode_asdu(H("2D 01 06 00 01 00 0A 00 00 81"))
    assert a is not None
    assert a.type_id == int(TypeID.C_SC_NA_1)
    assert a.objects[0].ioa == 10


def test_cp56_roundtrip():
    from datetime import datetime, timezone
    dt = datetime(2026, 10, 17, 18, 14, 16, 636000, tzinfo=timezone.utc)
    back = decode_cp56(encode_cp56(dt))
    assert back is not None
    assert (back.year, back.month, back.day, back.hour, back.minute, back.second) == \
           (2026, 10, 17, 18, 14, 16)


def test_clock_sync_roundtrip():
    """C_CS_NA_1（对时）：数据段是纯 7 字节 CP56Time2a，往返应还原到同一时刻。"""
    from datetime import datetime, timezone
    dt = datetime(2026, 10, 17, 18, 14, 16, 636000, tzinfo=timezone.utc)
    got = encode_asdu(int(TypeID.C_CS_NA_1), COT.ACTIVATION, 1,
                      [InformationObject(0, timestamp=dt)])
    assert len(got) == 16                 # 6(头) + 3(IOA) + 7(时标)
    a = decode_asdu(got)
    assert a is not None
    assert a.type_id == int(TypeID.C_CS_NA_1)
    assert a.cot == int(COT.ACTIVATION)
    assert a.ca == 1
    assert a.sq is False                  # 默认 SQ=0
    assert a.objects[0].ioa == 0
    assert a.objects[0].value is None     # 对时对象只有时标，没有值
    assert a.objects[0].timestamp == dt   # CP56Time2a 分辨率到毫秒，往返应精确相等


def test_clock_sync_matches_handmade_frame():
    """手拼 C_CS_NA_1 帧逐字节对拍（字节全部按本实现规则手算，不抄抓包）。"""
    from datetime import datetime, timezone
    # 67          TypeID = 103(0x67) C_CS_NA_1
    # 01          VSQ    = SQ=0, n=1（对时只带一个时标对象）
    # 06 00       COT    = 6（激活）| OA = 0
    # 01 00       CA     = 1（低字节在前）
    # 00 00 00    IOA    = 0（低字节在前）
    # FC 40       毫秒   = 16*1000 + 636 = 16636 = 0x40FC（低字节在前）
    # 0E 12 11 0A 1A     分=14 时=18 日=17 月=10 年=26（=2026-2000）
    raw = H("67 01 06 00 01 00 00 00 00 FC 40 0E 12 11 0A 1A")
    assert len(raw) == 16
    dt = datetime(2026, 10, 17, 18, 14, 16, 636000, tzinfo=timezone.utc)
    got = encode_asdu(int(TypeID.C_CS_NA_1), COT.ACTIVATION, 1,
                      [InformationObject(0, timestamp=dt)])
    assert got == raw
    # 反向：手工字节解回来也是同一时刻
    a = decode_asdu(raw)
    assert a is not None
    assert a.type_id == int(TypeID.C_CS_NA_1)
    assert a.objects[0].timestamp == dt


def test_clock_sync_defaults_to_now_utc():
    """对时对象不给 timestamp 时，按当前 UTC 时刻编码（秒级容差）。"""
    from datetime import datetime, timezone
    got = encode_asdu(int(TypeID.C_CS_NA_1), COT.ACTIVATION, 1,
                      [InformationObject(0)])         # timestamp=None
    assert len(got) == 16
    a = decode_asdu(got)
    assert a is not None
    ts = a.objects[0].timestamp
    assert ts is not None and ts.tzinfo is timezone.utc
    drift = abs(ts.timestamp() - datetime.now(timezone.utc).timestamp())
    assert drift <= 5.0


def test_unknown_type_keeps_raw_tail():
    """未登记类型仍走 raw_tail 原样保留，不硬解、不抛异常。"""
    a = decode_asdu(H("FF 01 06 00 01 00 DE AD BE EF"))
    assert a is not None
    assert a.type_id == 0xFF
    assert a.objects == []
    assert a.raw_tail == H("DE AD BE EF")


# ================================================================ 点表
def test_station_grouping():
    st = Station(1)
    for ioa in (1, 2, 5, 6, 9):
        st.add(ioa, int(TypeID.M_ME_NC_1), 0.0)
    groups = st.group_consecutive(st.all())
    assert [[p.ioa for p in g] for g in groups] == [[1, 2], [5, 6], [9]]


def test_station_rejects_duplicate_ioa():
    st = Station(1)
    st.add(1, int(TypeID.M_SP_NA_1), True)
    try:
        st.add(1, int(TypeID.M_SP_NA_1), False)
    except ValueError:
        return
    raise AssertionError("重复 IOA 应该报错")


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
