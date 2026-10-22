"""Modbus TCP 数据源测试：用微型 Modbus TCP 服务端当对端，纯标准库。

这一层验的是"接缝"——数据源读不到时必须整轮返回空（上层据此标 IV），
不能返回旧值冒充新值。

跑法（在项目根目录）：
    set PYTHONPATH=src
    python tests\\test_source_modbus.py
"""

import os
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import ModbusTcpSource, Station, TypeID  # noqa: E402
from iec104.config import ConfigError, source_spec_from_dict  # noqa: E402


# ---------------------------------------------------------------- 微型从站
class MockSlave(threading.Thread):
    """极简 Modbus TCP 从站：只实现 FC03 读保持寄存器。

    - `registers`：地址 -> 值
    - `exception_code`：非 None 时一律回该异常码
    """

    def __init__(self, registers, unit=1, exception_code=None, drop_after=None):
        super().__init__(daemon=True)
        self.registers = dict(registers)
        self.unit = unit
        self.exception_code = exception_code
        self.drop_after = drop_after          # 服务 N 个请求后主动断链（测重连）
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(4)
        self._srv.settimeout(0.3)
        self.port = self._srv.getsockname()[1]
        self._running = True
        self.requests = 0

    def _serve(self, conn):
        conn.settimeout(1.0)
        served = 0
        try:
            while self._running:
                head = b""
                while len(head) < 7:
                    chunk = conn.recv(7 - len(head))
                    if not chunk:
                        return
                    head += chunk
                tid, _proto, length, unit = struct.unpack(">HHHB", head)
                body = b""
                while len(body) < max(0, length - 1):
                    chunk = conn.recv(length - 1 - len(body))
                    if not chunk:
                        return
                    body += chunk
                self.requests += 1
                served += 1
                fc = body[0]
                start, count = struct.unpack(">HH", body[1:5])
                if fc != 3:
                    pdu = bytes((fc | 0x80, 1))
                elif self.exception_code is not None:
                    pdu = bytes((0x83, self.exception_code))
                else:
                    data = b"".join(struct.pack(">H", self.registers.get(start + i, 0))
                                    for i in range(count))
                    pdu = bytes((3, len(data))) + data
                conn.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu)
                if self.drop_after is not None and served >= self.drop_after:
                    return                       # 主动断链，模拟对端重启
        except (OSError, socket.timeout):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def run(self):
        while self._running:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def stop(self):
        self._running = False
        try:
            self._srv.close()
        except OSError:
            pass


def make_station():
    st = Station(1)
    st.add(1, int(TypeID.M_ME_NC_1), 0.0, name="Temp")
    st.add(2, int(TypeID.M_ME_NC_1), 0.0, name="Humidity")
    return st


# ================================================================ 采集
def test_reads_registers_with_scale():
    """常态：寄存器 ÷ scale，映射到 IOA。"""
    slave = MockSlave({6: 259, 7: 585})       # 25.9°C / 58.5%RH
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1,
                              [(1, 6, 10.0, True), (2, 7, 10.0, False)])
        out = src.poll()
        assert out[1][0] == 25.9, out
        assert out[2][0] == 58.5, out
        assert out[1][1] == 0 and out[2][1] == 0, "品质应为 good"
        src.close()
    finally:
        slave.stop()


def test_signed_temperature():
    """有符号：温度可以为负。"""
    slave = MockSlave({6: 0x10000 - 53})      # -5.3°C
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1, [(1, 6, 10.0, True)])
        out = src.poll()
        assert abs(out[1][0] - (-5.3)) < 1e-6, out
        src.close()
    finally:
        slave.stop()


def test_unsigned_high_bit_stays_positive():
    """无符号：最高位不该被当负数。"""
    slave = MockSlave({7: 0xFFF0})            # 65520 → 6552.0
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1, [(2, 7, 10.0, False)])
        out = src.poll()
        assert out[2][0] == 6552.0, out
        src.close()
    finally:
        slave.stop()


def test_single_frame_covers_whole_range():
    """一次请求覆盖 起点..终点 整段，不做 N 次单点读。"""
    slave = MockSlave({i: i * 10 for i in range(9)})
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1,
                              [(1, 6, 10.0, False), (2, 7, 10.0, False)])
        src.poll()
        assert slave.requests == 1, f"实际发了 {slave.requests} 次请求"
        src.close()
    finally:
        slave.stop()


# ================================================================ 故障
def test_connection_refused_returns_empty():
    """⭐ 连不上 ⇒ 整轮返回空（上层标 IV），绝不返回旧值。"""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                  # 立刻释放，保证没人监听

    src = ModbusTcpSource("127.0.0.1", port, 1, [(1, 6, 10.0, False)], timeout=0.5)
    out = src.poll()
    assert out == {}, f"应返回空，实际 {out}"
    assert src.last_error, "应记录错误原因"


def test_exception_response_returns_empty():
    """从站回异常码 ⇒ 整轮返回空。"""
    slave = MockSlave({6: 100}, exception_code=2)
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1, [(1, 6, 10.0, False)])
        out = src.poll()
        assert out == {}, out
        assert "异常码" in src.last_error, src.last_error
        src.close()
    finally:
        slave.stop()


def test_recovers_after_connection_drop():
    """⭐ 对端把连接断了以后能自动重连：一次失败 → 下一轮恢复。"""
    slave = MockSlave({6: 237}, drop_after=1)
    slave.start()
    try:
        src = ModbusTcpSource("127.0.0.1", slave.port, 1,
                              [(1, 6, 10.0, False)], timeout=0.5)
        assert abs(src.poll()[1][0] - 23.7) < 1e-6      # 第 1 次成功
        time.sleep(0.25)                                 # 让服务端把连接关掉

        out = src.poll()                                 # 旧连接已死 → 应失败
        assert out == {}, f"连接被关掉后应返回空，实际 {out}"

        time.sleep(0.1)
        out = src.poll()                                 # 下一轮应重连成功
        assert abs(out[1][0] - 23.7) < 1e-6, f"应自动重连，实际 {out}"
        src.close()
    finally:
        slave.stop()


# ================================================================ 配置校验
def test_source_spec_valid():
    st = make_station()
    spec = source_spec_from_dict({
        "type": "modbus_tcp", "host": "127.0.0.1", "port": 5020, "unit": 1,
        "map": [{"ioa": 1, "register": 6, "scale": 10, "signed": True},
                {"ioa": 2, "register": 7, "scale": 10}],
    }, station=st)
    assert spec["port"] == 5020 and spec["unit"] == 1
    assert spec["mapping"] == [(1, 6, 10.0, True), (2, 7, 10.0, False)]


def test_source_spec_rejects_bad_map():
    st = make_station()
    bad_cases = [
        ({"type": "modbus_tcp", "map": [{"ioa": 99, "register": 6}]}, "不存在"),
        ({"type": "modbus_tcp", "map": [{"ioa": 1, "register": -1}]}, "register"),
        ({"type": "modbus_tcp", "map": [{"ioa": 1, "register": 6, "scale": 0}]}, "scale"),
        ({"type": "modbus_tcp", "map": [{"ioa": 1, "register": 6},
                                        {"ioa": 1, "register": 7}]}, "重复"),
        ({"type": "modbus_tcp", "map": []}, "map"),
        ({"type": "modbus_tcp", "port": 70000, "map": [{"ioa": 1, "register": 6}]}, "port"),
        ({"type": "modbus_tcp", "unit": 300, "map": [{"ioa": 1, "register": 6}]}, "unit"),
        ({"type": "mqtt", "map": [{"ioa": 1, "register": 6}]}, "不支持"),
        ({"type": "modbus_tcp", "map": [{"ioa": 1, "register": 6, "typo": 1}]}, "未知字段"),
    ]
    for cfg, keyword in bad_cases:
        try:
            source_spec_from_dict(cfg, station=st)
        except ConfigError as e:
            assert keyword in str(e), f"{keyword!r} 未出现在错误里：{e}"
        else:
            raise AssertionError(f"应报错但没报：{cfg}")


def test_points_without_source_key_is_fine():
    """点表没有 source 段 ⇒ 合法（走桩数据源）。"""
    import json
    import tempfile

    from iec104.config import load_source_spec

    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False,
                                     encoding="utf-8") as f:
        json.dump({"common_address": 1, "points": [
            {"ioa": 1, "type": "M_ME_NC_1", "name": "T"}]}, f)
        path = f.name
    try:
        assert load_source_spec(path) is None
    finally:
        os.unlink(path)


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
