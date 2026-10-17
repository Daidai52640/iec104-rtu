"""应用层（ASDU）：编解码 + 信息对象。

ASDU 骨架（实测确认）：
    TypeID(1) | VSQ(1) | COT(2, 低字节在前) | CA(2, 低字节在前) | <对象>
    对象 = IOA(3, 低字节在前) + 数据

VSQ（可变结构限定词）：
    bit7 = SQ（顺序标志）
    bit0~6 = 对象数量
    ⭐ SQ=1：**只发第一个对象的 IOA**，后续对象按 IOA+1 隐含递增
    ⭐ SQ=0：每个对象都自带 IOA
    （实测：c104 对单对象也用 SQ=1，VSQ=0x81）

COT（传送原因）2 字节：
    第 1 字节 = 原因码(bits0~5) | P/N(bit6) | TEST(bit7)
    第 2 字节 = 源发站地址 OA（实测为 0）

各类数据长度（不含 IOA）：
    M_SP_NA_1  1 字节 SIQ（bit0=值，bit4~7=品质）
    M_ME_NC_1  5 字节 = 4 字节短浮点 + 1 字节 QDS  ⚠️ 浮点是**小端**（实测 23.7 = 9A 99 BD 41）
    C_SC_NA_1  1 字节 SCO（bit0=SCS，bit7=S/E）
    C_IC_NA_1  1 字节 QOI
    C_CS_NA_1  7 字节 CP56Time2a（对时：纯时标，无品质位）

⚠️ 不依赖任何第三方 104 库。数值定义见 types.py，需与标准原文核对。
"""

import struct
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, List, Optional

from .consts import BROADCAST_CA, COT, TypeID


# ---------------------------------------------------------------- 对象
@dataclass
class InformationObject:
    """一个信息对象：地址 + 值 + 品质/限定词。"""

    ioa: int                       # 信息对象地址（3 字节）
    value: Any = None              # bool / float / int
    quality: int = 0               # QDS（遥测）或 SIQ 高位（遥信）
    select: bool = False           # 仅遥控：S/E 位（True=选择，False=执行）
    timestamp: Optional[datetime] = None  # 仅带时标类型（M_SP_TB_1 等；C_CS_NA_1 只有它）

    def __repr__(self) -> str:
        extra = " SEL" if self.select else ""
        return f"<IOA={self.ioa} v={self.value!r} q={self.quality:#04x}{extra}>"


@dataclass
class Asdu:
    type_id: int
    cot: int
    ca: int
    objects: List[InformationObject] = field(default_factory=list)
    sq: bool = False
    oa: int = 0
    negative: bool = False
    test: bool = False
    raw_tail: bytes = b""          # 未知类型的原始数据（便于排查）

    def __repr__(self) -> str:
        try:
            tname = TypeID(self.type_id).name
        except ValueError:
            tname = f"TypeID={self.type_id}"
        try:
            cname = COT(self.cot).name
        except ValueError:
            cname = f"COT={self.cot}"
        return (f"<ASDU {tname} {cname} CA={self.ca} sq={self.sq} "
                f"n={len(self.objects)} {self.objects}>")


# ---------------------------------------------------------------- 各类型数据
# 长度（不含 IOA）
DATA_LEN = {
    int(TypeID.M_SP_NA_1): 1,
    int(TypeID.M_SP_TB_1): 8,     # 1 字节 SIQ + 7 字节 CP56Time2a（SOE）
    int(TypeID.M_ME_NC_1): 5,
    int(TypeID.C_SC_NA_1): 1,
    int(TypeID.C_IC_NA_1): 1,
    int(TypeID.C_CS_NA_1): 7,     # 纯 7 字节 CP56Time2a（对时，无 SIQ/QDS）
}


def _encode_data(type_id: int, obj: InformationObject) -> bytes:
    """把一个对象的值编码成"不含 IOA"的数据段。"""
    if type_id == int(TypeID.M_SP_NA_1):
        siq = (0x01 if obj.value else 0x00) | (obj.quality & 0xF0)
        return bytes((siq,))

    if type_id == int(TypeID.M_SP_TB_1):
        siq = (0x01 if obj.value else 0x00) | (obj.quality & 0xF0)
        return bytes((siq,)) + encode_cp56(obj.timestamp)

    if type_id == int(TypeID.M_ME_NC_1):
        # ⭐ 小端短浮点 + 1 字节 QDS —— 实测：23.7 -> 9A 99 BD 41 00
        return struct.pack("<f", float(obj.value)) + bytes((obj.quality & 0xFF,))

    if type_id == int(TypeID.C_SC_NA_1):
        # SCO：bit0 = SCS，bit7 = S/E（1=选择，0=执行）
        sco = (0x01 if obj.value else 0x00) | (0x80 if obj.select else 0x00)
        return bytes((sco,))

    if type_id == int(TypeID.C_IC_NA_1):
        return bytes((int(obj.value) & 0xFF,))

    if type_id == int(TypeID.C_CS_NA_1):
        # 对时：数据段就是 7 字节 CP56Time2a（不像 M_SP_TB_1 那样带 SIQ）。
        # 未指定时刻时 encode_cp56(None) 取当前 UTC。
        return encode_cp56(obj.timestamp)

    raise ValueError(f"unsupported TypeID {type_id}")


def _decode_data(type_id: int, chunk: bytes, obj: InformationObject) -> None:
    """把数据段解回对象。"""
    if type_id == int(TypeID.M_SP_NA_1):
        obj.value = bool(chunk[0] & 0x01)
        obj.quality = chunk[0] & 0xF0
        return

    if type_id == int(TypeID.M_SP_TB_1):
        obj.value = bool(chunk[0] & 0x01)
        obj.quality = chunk[0] & 0xF0
        obj.timestamp = decode_cp56(chunk[1:8])
        return

    if type_id == int(TypeID.M_ME_NC_1):
        obj.value = struct.unpack("<f", chunk[:4])[0]
        obj.quality = chunk[4]
        return

    if type_id == int(TypeID.C_SC_NA_1):
        obj.value = bool(chunk[0] & 0x01)
        obj.select = bool(chunk[0] & 0x80)
        obj.quality = chunk[0] & 0x7C
        return

    if type_id == int(TypeID.C_IC_NA_1):
        obj.value = chunk[0]
        return

    if type_id == int(TypeID.C_CS_NA_1):
        # 对时：只有时标，value 保持 None；时标非法时 decode_cp56 返回 None。
        obj.timestamp = decode_cp56(chunk)
        return


# ---------------------------------------------------------------- 编码
def encode_asdu(
    type_id: int,
    cot: int,
    ca: int,
    objects: List[InformationObject],
    sq: bool = False,
    oa: int = 0,
    negative: bool = False,
) -> bytes:
    """把一个 ASDU 编码成字节。

    sq=True 时按"顺序 IOA"打包（只写第一个对象的 IOA，其余隐含 +1）。
    ⚠️ 用 sq=True 时对象 IOA 必须连续递增，否则抛 ValueError。
    negative=True 时置 COT 的 P/N 位（否定确认）——遥控被拒绝时用。
    """
    if type_id not in DATA_LEN:
        raise ValueError(f"unsupported TypeID {type_id}")

    n = len(objects)
    if n == 0:
        raise ValueError("ASDU needs at least one object")
    if n > 127:
        raise ValueError("too many objects for one ASDU (max 127)")

    if sq:
        for i, obj in enumerate(objects):
            if obj.ioa != objects[0].ioa + i:
                raise ValueError("sq=True requires consecutive IOA")

    vsq = (0x80 if sq else 0) | (n & 0x7F)
    cot_byte = (cot & 0x3F) | (0x40 if negative else 0x00)
    header = bytes((
        type_id & 0xFF,
        vsq,
        cot_byte,
        oa & 0xFF,           # OA（实测为 0）
        ca & 0xFF,
        (ca >> 8) & 0xFF,
    ))

    body = bytearray()
    for i, obj in enumerate(objects):
        if (not sq) or i == 0:
            body += bytes((obj.ioa & 0xFF, (obj.ioa >> 8) & 0xFF, (obj.ioa >> 16) & 0xFF))
        body += _encode_data(type_id, obj)

    return header + bytes(body)


# ---------------------------------------------------------------- 解码
def decode_asdu(data: bytes) -> Optional[Asdu]:
    """解析一个 ASDU。不合法返回 None；未知类型保留 raw_tail。"""
    if len(data) < 6:
        return None

    type_id = data[0]
    vsq = data[1]
    cot_lo = data[2]
    oa = data[3]
    ca = data[4] | (data[5] << 8)

    asdu = Asdu(
        type_id=type_id,
        cot=cot_lo & 0x3F,
        ca=ca,
        sq=bool(vsq & 0x80),
        oa=oa,
        negative=bool(cot_lo & 0x40),
        test=bool(cot_lo & 0x80),
    )

    if type_id not in DATA_LEN:
        asdu.raw_tail = data[6:]
        return asdu

    size = DATA_LEN[type_id]
    n = vsq & 0x7F
    pos = 6
    ioa = 0

    # ⚠️ 标准里 n=0 是"无效"；实测对端不会这么发，这里保守处理成 1 个对象
    count = n if n > 0 else 1
    for i in range(count):
        if (not asdu.sq) or i == 0:
            if pos + 3 > len(data):
                break
            ioa = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16)
            pos += 3
        else:
            ioa += 1

        if pos + size > len(data):
            break
        chunk = data[pos:pos + size]
        pos += size

        obj = InformationObject(ioa=ioa)
        _decode_data(type_id, chunk, obj)
        asdu.objects.append(obj)

    asdu.raw_tail = data[pos:]
    return asdu


# ---------------------------------------------------------------- 时标
def encode_cp56(dt: Optional[datetime] = None) -> bytes:
    """CP56Time2a：7 字节时标（毫秒 2 字节小端 + 分/时/日/月/年）。

    ⚠️ 位布局（IV/SU/星期）需按标准原文逐位核对；本实现未置这些标志位。
    """
    if dt is None:
        dt = datetime.now(timezone.utc)
    ms = dt.second * 1000 + dt.microsecond // 1000
    return bytes((
        ms & 0xFF, (ms >> 8) & 0xFF,
        dt.minute & 0x3F,
        dt.hour & 0x1F,
        dt.day & 0x1F,
        dt.month & 0x0F,
        (dt.year - 2000) & 0x7F,
    ))


def decode_cp56(b: bytes) -> Optional[datetime]:
    """7 字节 CP56Time2a → datetime（UTC）。"""
    if len(b) < 7:
        return None
    ms = b[0] | (b[1] << 8)
    try:
        return datetime(
            2000 + (b[6] & 0x7F), b[5] & 0x0F, b[4] & 0x1F,
            b[3] & 0x1F, b[2] & 0x3F, ms // 1000, (ms % 1000) * 1000,
            tzinfo=timezone.utc,
        )
    except ValueError:
        return None


# ---------------------------------------------------------------- 自检
def _self_check() -> None:
    """用实测抓包的原始字节做逐字节对拍（按 bytes 比，避免大小写歧义）。"""
    def H(s: str) -> bytes:
        return bytes.fromhex(s.replace(" ", ""))

    a = decode_asdu(H("64 01 06 00 FF FF 00 00 00 14"))
    assert a is not None and a.type_id == 100 and a.cot == 6
    assert a.ca == BROADCAST_CA and a.objects[0].ioa == 0 and a.objects[0].value == 20

    got = encode_asdu(int(TypeID.C_IC_NA_1), COT.ACTIVATION_CONFIRM, 1, [InformationObject(0, 20)])
    assert got == H("64 01 07 00 01 00 00 00 00 14"), got.hex()

    got = encode_asdu(int(TypeID.M_SP_NA_1), COT.INTERROGATED_BY_STATION, 1,
                      [InformationObject(5, True)], sq=True)
    assert got == H("01 81 14 00 01 00 05 00 00 01"), got.hex()

    got = encode_asdu(int(TypeID.M_ME_NC_1), COT.INTERROGATED_BY_STATION, 1,
                      [InformationObject(1, 23.7)], sq=True)
    assert got == H("0D 81 14 00 01 00 01 00 00 9A 99 BD 41 00"), got.hex()


if __name__ == "__main__":
    _self_check()
    print("asdu.py self-check OK")
