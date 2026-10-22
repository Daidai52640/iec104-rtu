"""Modbus RTU 链路：CRC + 串口收发 + 常用功能码。

⚠️ **本项目唯一需要第三方库的地方**：串口访问用 `pyserial`。
   104 协议核心（link / asdu / server）仍然零依赖；只有走 RTU 南向时才需要它。

⭐ 为什么自己写而不用 pymodbus：
   我们要的是"能控制字节、能看见每一帧"的最小实现，
   和本项目"自己按标准写"的定位一致；pymodbus 会把这层藏起来。

设计要点（都是现场会咬人的地方）：
- **帧间静默**：RS485 半双工，上一帧没发完就发下一帧会撞。按 3.5 字符时间留间隔。
- **回显**：有些 485 转接头会把自己发的字节回读一份，收之前先清空输入缓冲。
- **异常码**：从站回 `fc|0x80` + 异常码，必须和"超时无响应"区分开——
  前者说明链路通了但请求不对，后者说明链路/地址有问题。
"""

import struct
import time
from typing import List, Optional

from .actuator import Actuator
from .source import Source

try:                                    # 只有 RTU 模式才需要
    import serial
except ImportError:                     # pragma: no cover - 无串口库时仍可导入本模块
    serial = None                       # type: ignore[assignment]


# ---------------------------------------------------------------- CRC
def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS：多项式 0xA001（反射），初值 0xFFFF。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _with_crc(body: bytes) -> bytes:
    crc = crc16_modbus(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


class RtuError(IOError):
    """RTU 层错误。`kind` 区分超时 / 异常码 / CRC 错，便于上层判性质。"""

    def __init__(self, message: str, kind: str = "other") -> None:
        super().__init__(message)
        self.kind = kind


# ---------------------------------------------------------------- 链路
class RtuLink:
    """一条 RS485 总线上的 Modbus RTU 主站。

    ⚠️ 一个串口只能有一个实例（半双工总线，多实例会互相踩）。
       需要同时读写多个从站时，**共用同一个 RtuLink**。

    `transport` 可注入（测试用）：只要提供 read(n) / write(bytes) / reset_input_buffer()
    / close() 四个方法即可。
    """

    def __init__(self, port: Optional[str] = None, baud: int = 4800,
                 bytesize: int = 8, parity: str = "N", stopbits: int = 1,
                 timeout: float = 0.5, retries: int = 1,
                 transport=None) -> None:
        self.timeout = float(timeout)
        self.retries = max(0, int(retries))

        if transport is not None:
            self._ser = transport
            self._chars_per_frame = 11       # 假定 8N1：1 起始 + 8 数据 + 1 停止 + 余量
            self.baud = baud
        else:
            if serial is None:
                raise RtuError("未安装 pyserial，无法使用 Modbus RTU（pip install pyserial）",
                               kind="nodep")
            if not port:
                raise RtuError("RTU 模式必须指定串口名", kind="config")
            self._ser = serial.Serial(
                port=port, baudrate=baud, bytesize=bytesize, parity=parity,
                stopbits=stopbits, timeout=self.timeout,
            )
            self.baud = baud
            self._chars_per_frame = 1 + bytesize + (0 if parity == "N" else 1) + stopbits

        # 3.5 字符时间：4800 波特下约 8 ms
        self._frame_gap = 3.5 * self._chars_per_frame / float(self.baud)
        self._last_tx = 0.0

    # ------------------------------------------------------------ 底层收发
    def _sleep_gap(self) -> None:
        wait = self._frame_gap - (time.monotonic() - self._last_tx)
        if wait > 0:
            time.sleep(wait)

    def _xfer(self, body: bytes, expect: int) -> bytes:
        """发一帧、收一帧。返回**去掉 CRC 的 PDU**。"""
        frame = _with_crc(body)
        last_err: Optional[RtuError] = None

        for _ in range(self.retries + 1):
            self._sleep_gap()
            try:
                self._ser.reset_input_buffer()      # 清掉上一帧残留/自身回显
            except AttributeError:
                pass
            self._ser.write(frame)
            try:
                self._ser.flush()
            except AttributeError:
                pass
            self._last_tx = time.monotonic()

            try:
                head = self._ser.read(2)
            except Exception as e:                  # 串口异常统一转 RTU 错误
                last_err = RtuError(f"读串口失败：{type(e).__name__}: {e}", kind="io")
                continue
            if len(head) < 2:
                last_err = RtuError("超时无响应", kind="timeout")
                continue

            unit, fc = head[0], head[1]
            if fc & 0x80:                           # 异常应答：2 字节
                rest = self._ser.read(3)            # code + CRC(2)
                if len(rest) < 3:
                    last_err = RtuError("异常应答不完整", kind="timeout")
                    continue
                raise RtuError(f"从站 {unit} 异常码 {rest[0]:#04x}",
                               kind="exception")

            need = max(0, expect - 2)
            rest = self._ser.read(need)
            if len(rest) < need:
                last_err = RtuError("应答不完整（可能被截断）", kind="timeout")
                continue

            pdu = head[1:] + rest
            payload, crc = pdu[:-2], pdu[-2:]
            want = crc16_modbus(bytes((unit,)) + payload)
            if crc != bytes((want & 0xFF, (want >> 8) & 0xFF)):
                last_err = RtuError("CRC 校验失败", kind="crc")
                continue
            return payload

        assert last_err is not None
        raise last_err

    # ------------------------------------------------------------ 功能码
    def read_registers(self, unit: int, address: int, count: int) -> List[int]:
        """FC03：读保持寄存器。"""
        pdu = self._xfer(bytes((unit, 3, address >> 8, address & 0xFF,
                                count >> 8, count & 0xFF)), expect=5 + count * 2)
        nbytes = pdu[1]
        data = pdu[2:2 + nbytes]
        if len(data) != count * 2:
            raise RtuError(f"寄存器字节数不符：期望 {count * 2}，实得 {len(data)}", kind="format")
        return [struct.unpack(">H", data[i:i + 2])[0] for i in range(0, len(data), 2)]

    def read_input_registers(self, unit: int, address: int, count: int) -> List[int]:
        """FC04：读输入寄存器。"""
        pdu = self._xfer(bytes((unit, 4, address >> 8, address & 0xFF,
                                count >> 8, count & 0xFF)), expect=5 + count * 2)
        nbytes = pdu[1]
        data = pdu[2:2 + nbytes]
        return [struct.unpack(">H", data[i:i + 2])[0] for i in range(0, len(data), 2)]

    def read_coils(self, unit: int, address: int, count: int) -> List[bool]:
        """FC01：读线圈（位软元件，例如 Y 输出）。"""
        nbytes = (count + 7) // 8
        pdu = self._xfer(bytes((unit, 1, address >> 8, address & 0xFF,
                                count >> 8, count & 0xFF)), expect=5 + nbytes)
        data = pdu[2:2 + pdu[1]]
        if len(data) < nbytes:
            raise RtuError("线圈数据不完整", kind="format")
        bits: List[bool] = []
        for i in range(count):
            bits.append(bool(data[i // 8] >> (i % 8) & 1))
        return bits

    def write_coil(self, unit: int, address: int, on: bool) -> None:
        """FC05：写单个线圈。

        ⭐ 继电器输出是**位软元件**，必须用 FC05；
           用 FC06 写保持寄存器会"能写能回读但继电器不动"（实测过的坑）。
        """
        self._xfer(bytes((unit, 5, address >> 8, address & 0xFF,
                          0xFF if on else 0x00, 0x00)), expect=8)

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    def __repr__(self) -> str:
        return f"<RtuLink baud={self.baud} timeout={self.timeout}>"


# ---------------------------------------------------------------- 数据源
class ModbusRtuSource(Source):
    """一条 RTU 总线上**多个从站**的数据源。

    映射每项：`(ioa, unit, kind, address, scale, signed)`
    - `kind`：`"hr"` 保持寄存器(FC03) / `"ir"` 输入寄存器(FC04) / `"coil"` 线圈(FC01)
    - 线圈返回 `bool`，`scale` 忽略

    ⭐ **按从站隔离故障**：某个从站读失败，只让它的点缺席（上层标 IV），
       不影响同一总线上其它从站 —— 真的现场就是这样：
       继电器板坏了不该让温度也跟着变无效。
    """

    KINDS = ("hr", "ir", "coil")
    CHUNK = 120

    def __init__(self, link: RtuLink, mapping, default_unit: int = 1) -> None:
        self.link = link
        self.default_unit = int(default_unit)
        self._map = []
        for item in mapping:
            ioa, unit, kind, address, scale, signed = item
            if kind not in self.KINDS:
                raise ValueError(f"IOA {ioa} 的 kind={kind!r} 不支持（{self.KINDS}）")
            if address < 0:
                raise ValueError(f"IOA {ioa} 的地址不能为负")
            if scale <= 0:
                raise ValueError(f"IOA {ioa} 的 scale 必须 > 0")
            self._map.append((int(ioa), int(unit or self.default_unit),
                              kind, int(address), float(scale), bool(signed)))
        self.last_error: str = ""

    def _read_group(self, unit: int, kind: str, items):
        start = min(a for _, _, _, a, _, _ in items)
        end = max(a for _, _, _, a, _, _ in items)
        raw = {}
        addr = start
        while addr <= end:
            count = min(self.CHUNK, end - addr + 1)
            if kind == "hr":
                vals = self.link.read_registers(unit, addr, count)
            elif kind == "ir":
                vals = self.link.read_input_registers(unit, addr, count)
            else:
                vals = self.link.read_coils(unit, addr, count)
            for offset, v in enumerate(vals):
                raw[addr + offset] = v
            addr += count
        return raw

    def poll(self):
        # 按 (从站, 区) 分组，每个组合一次连续读
        groups = {}
        for entry in self._map:
            ioa, unit, kind, address, scale, signed = entry
            groups.setdefault((unit, kind), []).append(entry)

        out = {}
        errors = []
        for (unit, kind), items in groups.items():
            try:
                raw = self._read_group(unit, kind, items)
            except Exception as e:                  # ⚠️ 只影响这个从站
                errors.append(f"从站{unit}/{kind}: {type(e).__name__}: {e}")
                continue
            for ioa, _u, _k, address, scale, signed in items:
                if address not in raw:
                    continue
                v = raw[address]
                if kind == "coil":
                    out[ioa] = (bool(v), 0)
                else:
                    if signed and v >= 0x8000:
                        v -= 0x10000
                    out[ioa] = (round(v / scale, 3), 0)

        self.last_error = "；".join(errors)
        return out

    def close(self) -> None:
        self.link.close()

    def __repr__(self) -> str:
        return f"<ModbusRtuSource 点={len(self._map)}>"


# ---------------------------------------------------------------- 执行器
class ModbusRtuActuator(Actuator):
    """通过 RTU 写继电器输出（FC05 写线圈）。

    ⭐ **必须用 FC05**：Y 是位软元件。用 FC06 写保持寄存器会"能写能回读但继电器不动"。
    ⚠️ `readback()` 读不到时返回 `None`（由上层判失败），绝不返回 False 冒充"已断开"。
    """

    def __init__(self, link: RtuLink, unit: int = 2, coil_base: int = 0,
                 channels: int = 4) -> None:
        super().__init__(channels=channels)
        self.link = link
        self.unit = int(unit)
        self.coil_base = int(coil_base)
        self.last_error: str = ""

    def apply(self, index: int, on: bool) -> bool:
        if not (0 <= index < self.channels):
            self.last_error = f"通道 {index} 越界"
            return False
        try:
            self.link.write_coil(self.unit, self.coil_base + index, on)
            return True
        except Exception as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return False

    def readback(self, index: int):
        if not (0 <= index < self.channels):
            return None
        try:
            bits = self.link.read_coils(self.unit, self.coil_base + index, 1)
            return bool(bits[0]) if bits else None
        except Exception as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return None

    def __repr__(self) -> str:
        return f"<ModbusRtuActuator unit={self.unit} coil_base={self.coil_base}>"
