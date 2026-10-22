"""Modbus RTU 链路测试：用一个"假的串口"模拟从站，不需要真硬件。

验的是三件事：
    ① CRC / 帧格式对不对（用已知向量钉死）
    ② 三种失败能不能区分开：超时 / 异常码 / CRC 错
    ③ 多个从站挂在一条总线上时，一个坏了不影响另一个

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_rtu.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104.rtu import (  # noqa: E402
    ModbusRtuActuator, ModbusRtuSource, RtuError, RtuLink, crc16_modbus,
)


# ---------------------------------------------------------------- 假串口
class FakeUnit:
    """一个假从站：保持寄存器 + 输入寄存器 + 线圈。"""

    def __init__(self, registers=None, coils=None, exception=None,
                 input_registers=None):
        self.registers = dict(registers or {})
        self.input_registers = dict(input_registers or {})
        self.coils = dict(coils or {})          # 地址 -> bool
        self.exception = exception              # 非 None 时一律回该异常码


class FakeSerial:
    """够用的串口替身：master 写请求 → 这里生成应答 → master 再读走。"""

    def __init__(self, units, no_response=False, corrupt_crc=False):
        self.units = units
        self.no_response = no_response
        self.corrupt_crc = corrupt_crc
        self._in = b""
        self.requests = []

    # 串口接口
    def reset_input_buffer(self):
        self._in = b""

    def flush(self):
        pass

    def close(self):
        pass

    def read(self, n):
        out = self._in[:n]
        self._in = self._in[n:]
        return out

    def write(self, data):
        data = bytes(data)
        self.requests.append(data)
        self._in = b""
        if self.no_response:
            return

        unit, fc = data[0], data[1]
        body, crc = data[:-2], data[-2:]
        want = crc16_modbus(body)
        if crc != bytes((want & 0xFF, (want >> 8) & 0xFF)):
            return                              # 请求本身 CRC 错：从站不理

        u = self.units.get(unit)
        if u is None:
            return

        if u.exception is not None:
            resp = bytes((unit, fc | 0x80, u.exception))
        elif fc in (3, 4):
            addr = (data[2] << 8) | data[3]
            count = (data[4] << 8) | data[5]
            src = u.registers if fc == 3 else u.input_registers
            payload = bytearray()
            for i in range(count):
                v = src.get(addr + i, 0) & 0xFFFF
                payload += bytes((v >> 8 & 0xFF, v & 0xFF))
            resp = bytes((unit, fc, count * 2)) + bytes(payload)
        elif fc == 1:
            addr = (data[2] << 8) | data[3]
            count = (data[4] << 8) | data[5]
            nbytes = (count + 7) // 8
            raw = bytearray(nbytes)
            for i in range(count):
                if u.coils.get(addr + i, False):
                    raw[i // 8] |= 1 << (i % 8)
            resp = bytes((unit, 1, nbytes)) + bytes(raw)
        elif fc == 5:
            addr = (data[2] << 8) | data[3]
            on = data[4] == 0xFF
            u.coils[addr] = on
            resp = data[:-2]                    # FC05 原样回显
        else:
            return

        crc = crc16_modbus(resp)
        crc_bytes = bytes((crc & 0xFF, (crc >> 8) & 0xFF))
        if self.corrupt_crc:
            crc_bytes = bytes((crc_bytes[1], crc_bytes[0]))
        self._in = resp + crc_bytes


def make_link(fake):
    # baud 调高只为让帧间静默短一点，测试跑得快
    return RtuLink(transport=fake, baud=115200, timeout=0.1, retries=0)


# ================================================================ CRC
def test_crc_known_vector():
    """已知向量：01 03 00 00 00 0A → CRC = C5 CD（低字节在前）。"""
    assert crc16_modbus(bytes.fromhex("01030000000A")) == 0xCDC5
    assert crc16_modbus(bytes.fromhex("010300000002")) == 0x0BC4


# ================================================================ 读
def test_fc03_read_registers():
    fake = FakeSerial({1: FakeUnit({6: 259, 7: 585})})
    link = make_link(fake)
    assert link.read_registers(1, 6, 2) == [259, 585]
    link.close()


def test_fc01_read_coils():
    fake = FakeSerial({2: FakeUnit(coils={1280: True, 1281: False, 1282: True})})
    link = make_link(fake)
    assert link.read_coils(2, 1280, 3) == [True, False, True]
    link.close()


# ================================================================ 写
def test_fc05_write_coil_then_readback():
    """⭐ 继电器必须用 FC05 写线圈；写完再读回确认。"""
    fake = FakeSerial({2: FakeUnit(coils={1280: False})})
    link = make_link(fake)
    link.write_coil(2, 1280, True)
    assert fake.units[2].coils[1280] is True
    assert link.read_coils(2, 1280, 1) == [True]
    link.write_coil(2, 1280, False)
    assert link.read_coils(2, 1280, 1) == [False]
    link.close()


def test_write_coil_uses_fc05_not_fc06():
    """守住那条实测教训：写继电器必须是 05，不能是 06。"""
    fake = FakeSerial({2: FakeUnit(coils={1280: False})})
    link = make_link(fake)
    link.write_coil(2, 1280, True)
    assert fake.requests[-1][1] == 5, f"应发 FC05，实际 FC{fake.requests[-1][1]}"
    link.close()


# ================================================================ 三种失败
def test_timeout_is_distinguishable():
    fake = FakeSerial({1: FakeUnit()}, no_response=True)
    link = make_link(fake)
    try:
        link.read_registers(1, 0, 1)
    except RtuError as e:
        assert e.kind == "timeout", e.kind
    else:
        raise AssertionError("应超时")
    link.close()


def test_exception_code_is_distinguishable():
    fake = FakeSerial({2: FakeUnit(exception=2)})
    link = make_link(fake)
    try:
        link.read_registers(2, 0, 1)
    except RtuError as e:
        assert e.kind == "exception", e.kind
        assert "0x02" in str(e)
    else:
        raise AssertionError("应回异常码")
    link.close()


def test_crc_error_is_distinguishable():
    fake = FakeSerial({1: FakeUnit({6: 100})}, corrupt_crc=True)
    link = make_link(fake)
    try:
        link.read_registers(1, 6, 1)
    except RtuError as e:
        assert e.kind == "crc", e.kind
    else:
        raise AssertionError("CRC 错应报错")
    link.close()


# ================================================================ 数据源
def test_source_reads_multiple_units():
    """一条总线上两个从站：变送器(unit1) 和 继电器板(unit2)。"""
    fake = FakeSerial({
        1: FakeUnit({6: 259, 7: 585}),
        2: FakeUnit(coils={1280: True, 1281: False}),
    })
    link = make_link(fake)
    src = ModbusRtuSource(link, [
        (1, 1, "hr", 6, 10.0, True),        # 温度
        (2, 1, "hr", 7, 10.0, False),       # 湿度
        (5, 2, "coil", 1280, 1.0, False),   # Y0
        (6, 2, "coil", 1281, 1.0, False),   # Y1
    ])
    out = src.poll()
    assert abs(out[1][0] - 25.9) < 1e-6, out
    assert abs(out[2][0] - 58.5) < 1e-6, out
    assert out[5][0] is True and out[6][0] is False, out
    assert src.last_error == "", src.last_error
    link.close()


def test_source_isolates_failed_unit():
    """⭐ 继电器板坏了，变送器的温度不该跟着变成无效。"""
    fake = FakeSerial({
        1: FakeUnit({6: 259}),
        2: FakeUnit(exception=4),           # 从站故障
    })
    link = make_link(fake)
    src = ModbusRtuSource(link, [
        (1, 1, "hr", 6, 10.0, True),
        (5, 2, "coil", 1280, 1.0, False),
    ])
    out = src.poll()
    assert 1 in out, "变送器的点应仍然有值"
    assert 5 not in out, "故障从站的点应缺席（上层标 IV）"
    assert "从站2" in src.last_error, src.last_error
    link.close()


# ================================================================ 执行器
def test_actuator_apply_and_readback():
    fake = FakeSerial({2: FakeUnit(coils={1280: False, 1281: False})})
    link = make_link(fake)
    act = ModbusRtuActuator(link, unit=2, coil_base=1280, channels=4)

    assert act.apply(0, True) is True
    assert act.readback(0) is True
    assert act.apply(0, False) is True
    assert act.readback(0) is False
    link.close()


def test_actuator_readback_none_when_unreachable():
    """⭐ 读不到必须返回 None，不能返回 False 冒充"已断开"。"""
    fake = FakeSerial({2: FakeUnit()}, no_response=True)
    link = make_link(fake)
    act = ModbusRtuActuator(link, unit=2, coil_base=1280, channels=4)
    assert act.apply(0, True) is False, "写失败应返回 False"
    assert act.readback(0) is None, "读不到应返回 None"
    link.close()


# ================================================================ 位提取
def test_bit_kind_extracts_from_input_register():
    """⭐ DI 聚在输入寄存器的字里：按位取出来当遥信。

    继电器板实测：X 输入在 0x0400，用 FC04 读，一个寄存器装多个开关量。
    """
    # 0x0400 = 1024；bit0=1, bit1=0, bit2=1 → 0b101 = 5
    fake = FakeSerial({2: FakeUnit(input_registers={1024: 0b101})})
    link = make_link(fake)
    src = ModbusRtuSource(link, [
        (7, 2, "bit", 1024, 1.0, False, 0),
        (8, 2, "bit", 1024, 1.0, False, 1),
        (9, 2, "bit", 1024, 1.0, False, 2),
    ])
    out = src.poll()
    assert out[7][0] is True, out
    assert out[8][0] is False, out
    assert out[9][0] is True, out
    link.close()


def test_multiple_bits_share_one_request():
    """同一个寄存器的多位只读一次，不是每位发一帧。"""
    fake = FakeSerial({2: FakeUnit(input_registers={1024: 0xFF})})
    link = make_link(fake)
    src = ModbusRtuSource(link, [
        (7, 2, "bit", 1024, 1.0, False, 0),
        (8, 2, "bit", 1024, 1.0, False, 1),
        (9, 2, "bit", 1024, 1.0, False, 2),
    ])
    src.poll()
    assert fake.requests and all(r[1] == 4 for r in fake.requests), "应全部走 FC04"
    assert len(fake.requests) == 1, f"应只发 1 帧，实际 {len(fake.requests)} 帧"
    link.close()


def test_bit_kind_requires_bit_field():
    fake = FakeSerial({2: FakeUnit()})
    link = make_link(fake)
    try:
        ModbusRtuSource(link, [(7, 2, "bit", 1024, 1.0, False)])
    except ValueError as e:
        assert "bit" in str(e), e
    else:
        raise AssertionError("kind=bit 缺少 bit 应报错")
    link.close()


def test_bit_field_rejected_on_other_kinds():
    fake = FakeSerial({2: FakeUnit()})
    link = make_link(fake)
    try:
        ModbusRtuSource(link, [(7, 2, "coil", 1024, 1.0, False, 0)])
    except ValueError as e:
        assert "不接受" in str(e), e
    else:
        raise AssertionError("kind=coil 给 bit 应报错")
    link.close()


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
