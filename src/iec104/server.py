"""104 子站服务端：TCP + 会话状态机。

职责边界（骨架阶段）：
    ✅ 建链（STARTDT）、总召唤（激活/确认/数据/终止）、S 帧确认、TESTFR、周期上送
    ✅ 收到遥控命令时记录并打日志
    ⬜ 遥控的「选择-执行」两段式与继电器回读 —— 下一步
    ⬜ 南向 Modbus RTU —— 下一步
    ⬜ 品质位映射、SOE 时标上送 —— 下一步

⚠️ 不依赖任何第三方 104 库。
"""

import socket
import threading
import time
from datetime import datetime, timezone
from typing import List, Optional

from . import quality as q_
from .actuator import Actuator, MemoryActuator
from .asdu import Asdu, InformationObject, decode_asdu, encode_asdu
from .link import Apdu, decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus
from .source import Source
from .station import Station
from .consts import BROADCAST_CA, COT, FrameFormat, TypeID, UFrame


# ---------------------------------------------------------------- 工具
def _hex(b: bytes, limit: int = 64) -> str:
    h = b.hex(" ").upper()
    return h if len(b) <= limit else h[: limit * 3] + " ..."


def _tname(tid: int) -> str:
    try:
        return TypeID(tid).name
    except ValueError:
        return f"TypeID={tid}"


def _cname(cot: int) -> str:
    try:
        return COT(cot).name
    except ValueError:
        return f"COT={cot}"


# ---------------------------------------------------------------- 会话
class Session(threading.Thread):
    """一个 TCP 连接上的 104 会话。"""

    def __init__(self, conn: socket.socket, addr, station: Station,
                 verbose: bool = True, actuator: Optional[Actuator] = None,
                 select_timeout: float = 30.0):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr
        self.station = station
        self.verbose = verbose
        self.actuator = actuator or MemoryActuator()
        self.select_timeout = select_timeout

        self.tx = 0                 # 我们发出的 I 帧序号
        self.rx = 0                 # 我们收到的 I 帧序号（下一个期望值）
        self.started = False        # 是否已 STARTDT
        self.closed = False
        self._selected = {}         # 遥控选择状态：IOA -> 选择时刻（monotonic）
        self._send_lock = threading.Lock()
        self._buf = bytearray()

    # ------------------------------------------------------------ 日志
    def _log(self, direction: str, text: str) -> None:
        if self.verbose:
            print(f"{time.strftime('%H:%M:%S')} {direction} {text}", flush=True)

    # ------------------------------------------------------------ 收发
    def _send(self, raw: bytes) -> None:
        with self._send_lock:
            if self.closed:
                return
            try:
                self.conn.sendall(raw)
            except OSError:
                self.closed = True

    def send_u(self, code: int) -> None:
        self._send(encode_u_frame(code))
        self._log("T-->", f"U {UFrame(code).name if code in set(UFrame) else hex(code)}")

    def send_s(self) -> None:
        self._send(encode_s_frame(self.rx))
        self._log("T-->", f"S rx={self.rx}")

    def send_asdu(self, asdu: bytes) -> None:
        frame = encode_i_frame(self.tx, self.rx, asdu)
        self._send(frame)
        a = decode_asdu(asdu)
        desc = (f"I tx={self.tx} rx={self.rx} | {_tname(a.type_id)} {_cname(a.cot)} "
                f"CA={a.ca} n={len(a.objects)} | {_hex(asdu)}") if a else _hex(asdu)
        self._log("T-->", desc)
        self.tx = (self.tx + 1) % 32768

    # ------------------------------------------------------------ 主循环
    def run(self) -> None:
        self._log("---", f"连接建立来自 {self.addr}")
        self.conn.settimeout(0.5)
        try:
            while not self.closed:
                try:
                    chunk = self.conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                self._buf.extend(chunk)

                for frame in iter_apdus(self._buf):
                    apdu = decode_apdu(frame)
                    if apdu is None:
                        self._log("<--", f"无法解析的帧 {_hex(frame)}")
                        continue
                    self._log("<--", f"{apdu!r}" + (f" | {_hex(apdu.asdu)}" if apdu.asdu else ""))
                    try:
                        self._handle(apdu)
                    except Exception as e:      # 骨架阶段：单帧异常不弄死会话
                        self._log("ERR", f"处理帧异常: {type(e).__name__}: {e}")
        finally:
            self.closed = True
            try:
                self.conn.close()
            except OSError:
                pass
            self._log("---", "连接关闭")

    # ------------------------------------------------------------ 帧处理
    def _handle(self, apdu: Apdu) -> None:
        if apdu.is_u:
            self._handle_u(apdu.u_code)
        elif apdu.is_s:
            # 对端在确认我们发出的 I 帧；骨架阶段只记录
            pass
        else:
            self._handle_i(apdu)

    def _handle_u(self, code: int) -> None:
        if code == UFrame.STARTDT_ACT:
            self.send_u(UFrame.STARTDT_CON)
            self.started = True
        elif code == UFrame.STOPDT_ACT:
            self.send_u(UFrame.STOPDT_CON)
            self.started = False
        elif code == UFrame.TESTFR_ACT:
            self.send_u(UFrame.TESTFR_CON)

    def _handle_i(self, apdu: Apdu) -> None:
        asdu = decode_asdu(apdu.asdu)
        if asdu is None:
            self.rx = (self.rx + 1) % 32768
            return

        if asdu.test:
            self._log("---", "收到 TEST 帧，骨架阶段忽略")
            self.rx = (self.rx + 1) % 32768
            self.send_s()
            return

        # ⭐ 总召唤：TypeID=100, COT=6(激活)
        if asdu.type_id == int(TypeID.C_IC_NA_1) and asdu.cot == COT.ACTIVATION:
            self.rx = (self.rx + 1) % 32768
            self._do_interrogation(asdu)
            self.send_s()
            return

        # 遥控：TypeID=45 —— ⭐ 选择 → 确认；执行 → 确认 + 变位上送
        if asdu.type_id == int(TypeID.C_SC_NA_1):
            self.rx = (self.rx + 1) % 32768
            self._handle_command(asdu)
            self.send_s()
            return

        # 其它未处理类型：确认收到，避免对端等超时
        self.rx = (self.rx + 1) % 32768
        self.send_s()

    # ------------------------------------------------------------ 遥控（选择-执行）
    def _handle_command(self, asdu: Asdu) -> None:
        """C_SC_NA_1 两段式。

        ⭐ 两段式的意义：防误操作。手一抖点错了，第一步就停住。
        """
        ca = self.station.ca
        for obj in asdu.objects:
            binding = self.station.binding(obj.ioa)
            if binding is None:
                self._log("CMD", f"拒绝：IOA={obj.ioa} 未绑定遥控")
                self._reply_command(obj, ca, negative=True)
                continue

            if obj.select:
                # ① 选择：记下时刻，回肯定确认
                self._selected[obj.ioa] = time.monotonic()
                self._log("CMD", f"选择 IOA={obj.ioa} -> 确认（等待执行）")
                self._reply_command(obj, ca, negative=False)
                continue

            # ② 执行：必须有未超时的选择
            t = self._selected.pop(obj.ioa, None)
            if t is None:
                self._log("CMD", f"拒绝：IOA={obj.ioa} 未先选择")
                self._reply_command(obj, ca, negative=True)
                continue
            if (time.monotonic() - t) > self.select_timeout:
                self._log("CMD", f"拒绝：IOA={obj.ioa} 选择超时"
                                 f"（>{self.select_timeout:.0f}s）")
                self._reply_command(obj, ca, negative=True)
                continue

            ok = self._execute(binding, obj.value)
            self._reply_command(obj, ca, negative=not ok)
            if ok:
                self._send_spontaneous(binding.readback_ioa)

    def _reply_command(self, obj: InformationObject, ca: int, negative: bool) -> None:
        """回命令确认：TypeID 相同、COT=7；negative 时置 P/N 位。"""
        self.send_asdu(encode_asdu(
            int(TypeID.C_SC_NA_1), COT.ACTIVATION_CONFIRM, ca,
            [InformationObject(obj.ioa, obj.value, select=obj.select)],
            negative=negative,
        ))

    def _execute(self, binding, value) -> bool:
        """执行：驱动执行器 → **独立回读** → 校验 → 更新遥信点。

        ⭐ 关键：执行成功与否**不看写入是否被接受**，而看回读是否一致。
        """
        idx = binding.output_index
        if not self.actuator.apply(idx, bool(value)):
            self._log("CMD", f"执行失败：通道 {idx} 拒绝")
            return False

        rb = self.actuator.readback(idx)
        if rb is None:
            self._log("CMD", f"回读失败：通道 {idx} 读不到（不能当作成功）")
            return False
        if bool(rb) != bool(value):
            self._log("CMD", f"回读不一致：期望 {value} 实得 {rb}")
            return False

        if binding.readback_ioa is not None:
            p = self.station.get(binding.readback_ioa)
            if p is not None:
                p.value = bool(rb)
                p.timestamp = datetime.now(timezone.utc)

        self._log("CMD", f"执行成功：通道 {idx} -> {bool(value)}（回读一致）")
        return True

    def _send_spontaneous(self, ioa: Optional[int]) -> None:
        """执行成功后主动上送变位（COT=3 突发）。"""
        if ioa is None:
            return
        p = self.station.get(ioa)
        if p is None:
            return
        self.send_asdu(encode_asdu(
            int(TypeID.M_SP_NA_1), COT.SPONTANEOUS, self.station.ca,
            [InformationObject(p.ioa, p.value, p.quality)],
        ))

    def send_spontaneous_points(self, points) -> None:
        """把一批变化点按突发（COT=3）上送。**每点一帧**，便于日志与排查。"""
        for p in points:
            self.send_asdu(encode_asdu(
                p.type_id, COT.SPONTANEOUS, self.station.ca,
                [InformationObject(p.ioa, p.value, p.quality)],
            ))

    # ------------------------------------------------------------ 总召唤
    def _do_interrogation(self, req: Asdu) -> None:
        """激活 → 确认 → 数据 → 终止。"""
        # 实测：主站可能用广播 CA=0xFFFF 发起
        ca = self.station.ca
        qoi = req.objects[0].value if req.objects else 20

        # ① 激活确认（COT=7）
        self.send_asdu(encode_asdu(
            int(TypeID.C_IC_NA_1), COT.ACTIVATION_CONFIRM, ca,
            [InformationObject(0, qoi)],
        ))

        # ② 数据上送（COT=20），按 IOA 连续性用 SQ=1 打包
        for group in self.station.interrogation_groups():
            self.send_asdu(encode_asdu(
                group[0].type_id, COT.INTERROGATED_BY_STATION, ca,
                [InformationObject(p.ioa, p.value, p.quality) for p in group],
                sq=len(group) > 1,
            ))

        # ③ 激活终止（COT=10）
        self.send_asdu(encode_asdu(
            int(TypeID.C_IC_NA_1), COT.ACTIVATION_TERMINATION, ca,
            [InformationObject(0, qoi)],
        ))

    # ------------------------------------------------------------ 周期上送
    def send_cyclic_once(self) -> None:
        if not (self.started and not self.closed):
            return
        groups = Station.group_consecutive(self.station.cyclic_points())
        for group in groups:
            self.send_asdu(encode_asdu(
                group[0].type_id, COT.PERIODIC, self.station.ca,
                [InformationObject(p.ioa, p.value, p.quality) for p in group],
                sq=len(group) > 1,
            ))


# ---------------------------------------------------------------- 服务端
class Iec104Slave:
    """104 子站服务端。可以同时接多个主站（骨架阶段不做冗余判断）。"""

    def __init__(self, station: Station, host: str = "0.0.0.0",
                 port: int = 2404, cyclic_s: float = 0.0, verbose: bool = True,
                 actuator: Optional[Actuator] = None,
                 select_timeout: float = 30.0,
                 source: Optional[Source] = None,
                 poll_s: float = 0.0) -> None:
        self.station = station
        self.host = host
        self.port = port
        self.cyclic_s = cyclic_s
        self.verbose = verbose
        self.actuator = actuator or MemoryActuator()
        self.select_timeout = select_timeout
        self.source = source
        self.poll_s = poll_s
        self._poll_thread: Optional[threading.Thread] = None

        self._srv: Optional[socket.socket] = None
        self._sessions: List[Session] = []
        self._running = False
        self._accept_thread: Optional[threading.Thread] = None
        self._cyclic_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------ 生命周期
    def start(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(8)
        srv.settimeout(0.5)
        self._srv = srv
        self._running = True

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        if self.cyclic_s > 0:
            self._cyclic_thread = threading.Thread(target=self._cyclic_loop, daemon=True)
            self._cyclic_thread.start()

        if self.source is not None and self.poll_s > 0:
            self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll_thread.start()

        if self.verbose:
            print("=" * 68)
            print(f"104 子站已启动  {self.host}:{self.port}  CA={self.station.ca}")
            print(f"点表（{len(self.station)} 个）：")
            for p in self.station.all():
                print(f"   IOA {p.ioa:<4} {_tname(p.type_id):<12} {p.name:<10} = {p.value!r}")
            if self.cyclic_s:
                print(f"周期上送：每 {self.cyclic_s}s 一次（COT=1 CYCLIC）")
            print("等待主站连接…  Ctrl+C 停止")
            print("=" * 68)

    def stop(self) -> None:
        self._running = False
        for s in list(self._sessions):
            s.closed = True
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass

    # ------------------------------------------------------------ 循环
    def _accept_loop(self) -> None:
        assert self._srv is not None
        while self._running:
            try:
                conn, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            sess = Session(conn, addr, self.station, verbose=self.verbose,
                           actuator=self.actuator, select_timeout=self.select_timeout)
            self._sessions = [s for s in self._sessions if s.is_alive()] + [sess]
            sess.start()

    def _cyclic_loop(self) -> None:
        while self._running:
            time.sleep(self.cyclic_s)
            for sess in list(self._sessions):
                if sess.is_alive():
                    try:
                        sess.send_cyclic_once()
                    except Exception:
                        pass

    def _poll_loop(self) -> None:
        """采集循环：source.poll() → 落点表 → 变化点突发上送。

        ⭐ 品质位规则在这里落地：
           数据源没返回的点，`Station.apply` 会把它标成 IV（无效），
           并且因为"品质变了"，会被当成变化点突发上送出去。
           主站因此能看到"这个点坏了"，而不是继续看到过期的假数。
        """
        while self._running:
            time.sleep(self.poll_s)
            values = {}
            try:
                values = self.source.poll()
            except Exception as e:
                if self.verbose:
                    print(f"    [POLL] 数据源异常: {type(e).__name__}: {e}", flush=True)

            changed = self.station.apply(values)
            if not changed:
                continue

            if self.verbose:
                state = "正常" if values else "❌ 数据源无数据"
                print(f"    [POLL] {state}｜变化 {len(changed)} 点: "
                      + ", ".join(f"IOA{p.ioa}={p.value!r}"
                                  f"({q_.describe(p.quality)})" for p in changed),
                      flush=True)

            for sess in list(self._sessions):
                if sess.is_alive() and sess.started:
                    try:
                        sess.send_spontaneous_points(changed)
                    except Exception:
                        pass
