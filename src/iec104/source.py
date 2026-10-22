"""南向数据源接口：**协议层与现场之间的"读"接缝**。

⭐ 与 `actuator.py` 对称：

    Actuator  协议层 → 现场（写：遥控动作）
    Source    现场 → 协议层（读：采集值 + 品质）

    无硬件时用 `StubSource`；板子到货后写 `ModbusSource` 实现同一接口，
    **协议层与品质位逻辑一行都不用改**。

⭐ 契约（很重要）：
    - `poll()` 返回 {ioa: (value, quality)}
    - **读不到的点不要出现在返回结果里** —— 由上层统一标记为"无效"，
      而不是让数据源自己随便塞个旧值。
"""

from abc import ABC, abstractmethod
import socket
import struct
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .actuator import Actuator


# ---------------------------------------------------------------- 接口
class Source(ABC):
    """南向数据源。"""

    @abstractmethod
    def poll(self) -> Dict[int, Tuple[Any, int]]:
        """读一轮。返回 {ioa: (value, quality)}；读不到的点**不要**出现。"""

    def close(self) -> None:  # 默认无资源
        pass


# ---------------------------------------------------------------- 无硬件实现
class StubSource(Source):
    """无硬件时的数据源。

    - 遥测（IOA 1/2）：模拟温湿度，缓慢变化
    - 遥信（IOA 5/6）：**从执行器回读**（stub 世界里执行器就是继电器模块）
    - `fail = True` 时模拟设备掉线：poll 返回空 → 上层把点标成 IV
    """

    def __init__(self, actuator: Optional[Actuator] = None,
                 readback_map: Optional[Dict[int, int]] = None,
                 step_s: float = 2.0) -> None:
        # readback_map: {输出通道号: 遥信点 IOA}
        self.actuator = actuator
        self.readback_map = readback_map or {0: 5, 1: 6}
        self.step_s = step_s
        self.fail = False
        self._t0 = time.monotonic()

    def poll(self) -> Dict[int, Tuple[Any, int]]:
        if self.fail:
            return {}

        # ⚠️ 值按"墙钟时间"阶梯变化（每 step_s 跳一档），**不是每次轮询都变**。
        #    否则每次 poll 都产生"变化点"→ 突发帧刷屏，把链路和测试都拖死。
        elapsed = time.monotonic() - self._t0
        phase = round(int(elapsed // self.step_s) % 10 * 0.1, 2)

        out: Dict[int, Tuple[Any, int]] = {
            1: (round(23.7 + phase, 2), 0),     # 温度
            2: (round(62.4 + phase, 2), 0),     # 湿度
        }

        # DI 从执行器回读（读不到就不上报 → 上层标 IV）
        if self.actuator is not None:
            for out_idx, ioa in self.readback_map.items():
                rb = self.actuator.readback(out_idx)
                if rb is not None:
                    out[ioa] = (bool(rb), 0)
        return out

    def __repr__(self) -> str:
        return f"<StubSource fail={self.fail}>"


# ---------------------------------------------------------------- Modbus TCP 实现
class ModbusTcpSource(Source):
    """从 **Modbus TCP 从站**读数据（纯标准库，不引入 pymodbus）。

    ⭐ 为什么用 TCP 而不是 RTU：现场那条 485 总线**只允许一个进程独占**。
       已经被另一个采集进程接走时，我们从它暴露的 Modbus TCP 从站上读，
       **不抢串口、不改对方**。这是同一台机器上两个系统共存的唯一干净做法。

    ⚠️ 契约同 `Source`：**出错就整轮返回空**，由上层把点标成 IV。
       绝不返回旧值冒充新值。
    """

    #: 一帧最多读多少个寄存器（PDU 长度所限，留余量）
    CHUNK = 120

    def __init__(self, host: str, port: int, unit: int,
                 mapping: Iterable[Tuple[int, int, float, bool]],
                 timeout: float = 2.0) -> None:
        """mapping: 可迭代的 (ioa, register, scale, signed)。

        - `ioa`      ：子站点表中的信息对象地址
        - `register` ：Modbus 保持寄存器地址（0 基）
        - `scale`    ：除数（寄存器里存的是 值×scale）
        - `signed`   ：是否按有符号 16 位解释（温度可能为负）
        """
        self.host = host
        self.port = int(port)
        self.unit = int(unit)
        self.timeout = float(timeout)

        self._map: List[Tuple[int, int, float, bool]] = []
        for item in mapping:
            ioa, reg, scale, signed = item
            if scale <= 0:
                raise ValueError(f"IOA {ioa} 的 scale 必须 > 0")
            if reg < 0:
                raise ValueError(f"IOA {ioa} 的寄存器地址不能为负")
            self._map.append((int(ioa), int(reg), float(scale), bool(signed)))

        self._sock: Optional[socket.socket] = None
        self._tid = 0
        self.last_error: str = ""

    # ------------------------------------------------------------ 连接
    def _ensure_sock(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self._sock = s
        return s

    def _drop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("对端关闭连接")
            buf += chunk
        return buf

    # ------------------------------------------------------------ 读寄存器
    def _read_registers(self, start: int, count: int) -> List[int]:
        self._tid = (self._tid + 1) & 0xFFFF
        pdu = struct.pack(">BHH", 3, start, count)
        mbap = struct.pack(">HHHB", self._tid, 0, len(pdu) + 1, self.unit)
        sock = self._ensure_sock()
        sock.sendall(mbap + pdu)

        head = self._recv_exact(7)          # MBAP：事务号2 + 协议2 + 长度2 + 单元1
        _tid, _proto, length, _unit = struct.unpack(">HHHB", head)
        body = self._recv_exact(max(0, length - 1))
        if not body:
            raise ConnectionError("空应答")

        fc = body[0]
        if fc == (0x03 | 0x80):
            raise IOError(f"Modbus 异常码 {body[1]:#04x}")
        if fc != 0x03:
            raise IOError(f"意外功能码 {fc:#04x}")

        nbytes = body[1]
        data = body[2:2 + nbytes]
        if len(data) != count * 2:
            raise IOError(f"寄存器字节数不符：期望 {count * 2}，实得 {len(data)}")
        return [struct.unpack(">H", data[i:i + 2])[0] for i in range(0, len(data), 2)]

    # ------------------------------------------------------------ 采集
    def poll(self) -> Dict[int, Tuple[Any, int]]:
        if not self._map:
            return {}

        start = min(r for _, r, _, _ in self._map)
        end = max(r for _, r, _, _ in self._map)

        # 按块读（本项目通常一次就够）
        raw: Dict[int, int] = {}
        try:
            addr = start
            while addr <= end:
                count = min(self.CHUNK, end - addr + 1)
                for offset, value in enumerate(self._read_registers(addr, count)):
                    raw[addr + offset] = value
                addr += count
        except Exception as e:                       # ⚠️ 任何失败都整轮作废
            self.last_error = f"{type(e).__name__}: {e}"
            self._drop()                             # 下次重连
            return {}

        out: Dict[int, Tuple[Any, int]] = {}
        for ioa, reg, scale, signed in self._map:
            if reg not in raw:
                continue
            v = raw[reg]
            if signed and v >= 0x8000:
                v -= 0x10000
            out[ioa] = (round(v / scale, 3), 0)
        return out

    def close(self) -> None:
        self._drop()

    def __repr__(self) -> str:
        return (f"<ModbusTcpSource {self.host}:{self.port} unit={self.unit} "
                f"点={len(self._map)}>")
