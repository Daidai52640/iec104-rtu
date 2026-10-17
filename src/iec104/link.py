"""链路层（APCI）：I / S / U 三种帧的编解码 + 流式拆帧。

APDU 骨架（实测确认）：
    68 | L | <控制域 4 字节> | <ASDU L 字节>
    L = 4 + len(ASDU)；整帧长度 = L + 2

控制域（实测确认）：
    I 格式：byte0 bit0 = 0；Tx = (b0 | b1<<8) >> 1；Rx = (b2 | b3<<8) >> 1
    S 格式：b0 = 0x01；Rx = (b2 | b3<<8) >> 1
    U 格式：b0 = 控制码（见 types.UFrame）；b1~b3 = 0

⚠️ 序号是 15 位、低字节在前、左移 1 位存放（最低位留给格式标志）。
"""

from dataclasses import dataclass
from typing import Iterator, Optional

from .consts import FrameFormat, UFrame

START_BYTE = 0x68
CONTROL_LEN = 4
SEQ_MOD = 32768
SEQ_MASK = 0x7FFF


@dataclass
class Apdu:
    """一帧解析结果。I 帧才有 asdu；S 帧只有 rx；U 帧只有 u_code。"""

    fmt: FrameFormat
    tx: int = 0
    rx: int = 0
    u_code: int = 0
    asdu: bytes = b""

    @property
    def is_i(self) -> bool:
        return self.fmt is FrameFormat.I

    @property
    def is_s(self) -> bool:
        return self.fmt is FrameFormat.S

    @property
    def is_u(self) -> bool:
        return self.fmt is FrameFormat.U

    def __repr__(self) -> str:
        if self.is_i:
            return f"<I tx={self.tx} rx={self.rx} asdu={len(self.asdu)}B>"
        if self.is_s:
            return f"<S rx={self.rx}>"
        name = UFrame(self.u_code).name if self.u_code in set(UFrame) else hex(self.u_code)
        return f"<U {name}>"


# ---------------------------------------------------------------- 编码
def _seq_bytes(seq: int) -> bytes:
    v = (seq % SEQ_MOD) << 1
    return bytes((v & 0xFF, (v >> 8) & 0xFF))


def encode_i_frame(tx: int, rx: int, asdu: bytes) -> bytes:
    """I 格式帧：承载 ASDU。"""
    control = _seq_bytes(tx) + _seq_bytes(rx)
    return _wrap(control, asdu)


def encode_s_frame(rx: int) -> bytes:
    """S 格式帧：只确认接收序号。"""
    control = b"\x01\x00" + _seq_bytes(rx)
    return _wrap(control, b"")


def encode_u_frame(code: int) -> bytes:
    """U 格式帧：链路控制。"""
    control = bytes((code & 0xFF, 0, 0, 0))
    return _wrap(control, b"")


def _wrap(control: bytes, asdu: bytes) -> bytes:
    body = control + asdu
    if len(body) > 253:
        raise ValueError(f"APDU too long: {len(body)} > 253")
    return bytes((START_BYTE, len(body))) + body


# ---------------------------------------------------------------- 解码
def decode_apdu(frame: bytes) -> Optional[Apdu]:
    """解析**一整帧**（长度必须正好）。不合法返回 None。"""
    if len(frame) < 2 or frame[0] != START_BYTE:
        return None
    length = frame[1]
    if len(frame) != length + 2:
        return None
    control = frame[2:6]
    if len(control) < 4:
        return None

    b0, b1, b2, b3 = control
    low = b0 & 0x03

    if low == FrameFormat.U:
        return Apdu(fmt=FrameFormat.U, u_code=b0)

    if low == FrameFormat.S:
        rx = ((b2 | (b3 << 8)) >> 1) & SEQ_MASK
        return Apdu(fmt=FrameFormat.S, rx=rx)

    # I 格式（low == 0）
    tx = ((b0 | (b1 << 8)) >> 1) & SEQ_MASK
    rx = ((b2 | (b3 << 8)) >> 1) & SEQ_MASK
    return Apdu(fmt=FrameFormat.I, tx=tx, rx=rx, asdu=frame[6:])


def iter_apdus(buf: bytearray) -> Iterator[bytes]:
    """从缓冲区里按 68+L 流式拆帧，**并从 buf 里消费掉已取出的帧**。

    注意：遇到不属于帧的字节会跳过，避免脏数据把整条链路卡死。
    """
    while True:
        # 找起始字节
        idx = buf.find(START_BYTE)
        if idx < 0:
            if len(buf) > 512:
                del buf[: len(buf) - 512]
            return
        if idx > 0:
            del buf[:idx]

        if len(buf) < 2:
            return
        total = buf[1] + 2
        if total < 6:  # 长度字段异常（至少要 4 字节控制域）
            del buf[:1]
            continue
        if len(buf) < total:
            return

        frame = bytes(buf[:total])
        del buf[:total]
        yield frame


# ---------------------------------------------------------------- 自检
def _self_check() -> None:
    """用实测抓包里的字节做往返自检（不依赖任何框架，直接跑）。"""
    cases = [
        ("68 04 07 00 00 00", "<U STARTDT_ACT>"),
        ("68 04 0B 00 00 00", "<U STARTDT_CON>"),
        ("68 04 01 00 08 00", "<S rx=4>"),
    ]
    for hexstr, want_repr in cases:
        raw = bytes.fromhex(hexstr.replace(" ", ""))
        apdu = decode_apdu(raw)
        assert apdu is not None, hexstr
        assert repr(apdu) == want_repr, f"{hexstr} -> {apdu!r} != {want_repr}"

    # I 帧往返
    raw = bytes.fromhex("680E0000000064010600FFFF00000014")
    apdu = decode_apdu(raw)
    assert apdu is not None and apdu.is_i
    assert apdu.tx == 0 and apdu.rx == 0 and len(apdu.asdu) == 10
    assert encode_i_frame(apdu.tx, apdu.rx, apdu.asdu) == raw


if __name__ == "__main__":
    _self_check()
    print("link.py self-check OK")
