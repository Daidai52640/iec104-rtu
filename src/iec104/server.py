"""104 子站服务端：TCP + 会话状态机。

职责边界（骨架阶段）：
    ✅ 建链（STARTDT）、总召唤（激活/确认/数据/终止）、S 帧确认、TESTFR、周期上送
    ✅ 接收序号 N(S) 连续性校验（重放/乱序直接断链）
    ✅ 遥控「选择-执行」两段式 + 独立回读 + 执行前的动作约束（离线/互斥组/最小间隔）
    ✅ 对时 C_CS_NA_1（记录偏移、回确认）
    ✅ SOE 时标上送：线类型由点表声明的 type 决定（M_SP_TB_1 全路径带时标）
    ⬜ 南向 Modbus RTU —— 下一步
    ⬜ 品质位映射细节（BL/SB 的人工置位）—— 下一步

⚠️ 不依赖任何第三方 104 库。
"""

import socket
import threading
import time
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from . import quality as q_
from .actuator import Actuator, MemoryActuator
from .asdu import Asdu, InformationObject, decode_asdu, encode_asdu
from .clock import ClockOffset
from .link import Apdu, decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus
from .source import Source
from .station import CommandBinding, Station, max_objects_per_frame
from .consts import BROADCAST_CA, COT, COT_PN_BIT, QDS, FrameFormat, TypeID, UFrame


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
                 select_timeout: float = 30.0,
                 min_interval: float = 1.0,
                 k: int = 12, w: int = 8,
                 t1: float = 15.0, t2: float = 10.0, t3: float = 20.0):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr
        self.station = station
        self.verbose = verbose
        self.actuator = actuator or MemoryActuator()
        self.select_timeout = select_timeout
        self.min_interval = min_interval    # 同一 IOA 两次成功执行的最小间隔（秒）

        # ---- 链路层参数
        self.k = k                  # 未确认 I 帧上限（发送窗口）
        self.w = w                  # 收到这么多 I 帧就发确认
        self.t1 = t1                # 等确认超时
        self.t2 = t2                # 延迟确认超时（t2 < t1）
        self.t3 = t3                # 空闲多久发 TESTFR

        self.tx = 0                 # 我们发出的 I 帧序号
        self.rx = 0                 # 我们收到的 I 帧序号（下一个期望值）
        self.peer_rx = 0            # 对端已确认到我们第几帧（取自对端的 rx 字段）
        self.started = False        # 是否已 STARTDT
        self.closed = False
        self.abort_reason = ""      # 断链原因（t1/t3 超时填这里）

        self._selected: Dict[int, Tuple[float, bool, bool]] = {}
        # 遥控选择状态：IOA -> (选择时刻 monotonic, 预演的 SCS 值, 选择位)
        # ⭐ 存值是为了执行时校验：选择是"预演"，执行与预演不一致绝不能动作。
        self._last_exec: Dict[int, float] = {}   # IOA -> 上次成功执行时刻（monotonic）
        self.clock = ClockOffset()  # 对时偏移（SOE 打时标用）
        self._send_lock = threading.Lock()
        self._buf = bytearray()

        # ---- 发送队列：采集线程也会往里放，所以要加锁；真正的写 socket 只在
        #      会话线程的 _pump() 里发生，避免跨线程写。
        self._outq: deque = deque()
        self._q_lock = threading.Lock()

        # ---- 定时器状态（一律用 monotonic）
        now = time.monotonic()
        self._t1_base = now          # 最早未确认 I 帧的发送时刻
        self._last_send = now        # 最近发出任何帧
        self._last_recv = now        # 最近收到任何帧
        self._pending_ack = 0        # 已收到但还没确认的 I 帧数
        self._t2_anchor: Optional[float] = None  # 第一帧未确认 I 帧到达的时刻（t2 锚点）
        self._test_at: Optional[float] = None   # 发过 TESTFR act 的时刻（等 con）

    # ------------------------------------------------------------ 窗口/序号
    def _tx_unacked(self) -> int:
        """窗口已占用多少格：我方已发 vs 对端已确认。

        ⚠️ 对端可能送来"超前"的确认（确认到我们还没发的序号）。
           按模运算会得到接近 32768 的大数，直接把发送卡死，
           所以这里把它当 0 处理——宁可多发，不可卡死。
        """
        delta = (self.tx - self.peer_rx) % 32768
        return 0 if delta > 16384 else delta

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
        self._last_send = time.monotonic()

    def send_s(self) -> None:
        self._send(encode_s_frame(self.rx))
        self._log("T-->", f"S rx={self.rx}")
        self._pending_ack = 0
        self._t2_anchor = None          # 确认发过了，t2 重新从下一轮算
        self._last_send = time.monotonic()

    def send_asdu(self, asdu: bytes) -> None:
        """把一帧 ASDU 放进发送队列。

        ⚠️ **可从任意线程调用**（采集线程也会放）。
        真正的 socket 写只发生在会话线程的 `_pump()` 里，
        这样窗口 K 满时可以排队，也不会跨线程写 socket。
        """
        with self._q_lock:
            self._outq.append(asdu)

    def _pump(self) -> None:
        """在窗口 K 允许的范围内，把队列里的帧发出去。"""
        while True:
            with self._q_lock:
                if not self._outq or self._tx_unacked() >= self.k:
                    return
                asdu = self._outq.popleft()
            self._send_i(asdu)

    def _send_i(self, asdu: bytes) -> None:
        if self._tx_unacked() == 0:
            self._t1_base = time.monotonic()      # 新一轮窗口的计时起点
        frame = encode_i_frame(self.tx, self.rx, asdu)
        self._send(frame)
        a = decode_asdu(asdu)
        desc = (f"I tx={self.tx} rx={self.rx} | {_tname(a.type_id)} {_cname(a.cot)} "
                f"CA={a.ca} n={len(a.objects)} | {_hex(asdu)}") if a else _hex(asdu)
        self._log("T-->", desc)
        self.tx = (self.tx + 1) % 32768
        self._pending_ack = 0        # I 帧自带 rx，等于顺带确认了收到的帧
        self._t2_anchor = None       # 同上：本轮已确认，t2 重新开始
        self._last_send = time.monotonic()

    # ------------------------------------------------------------ 主循环
    def run(self) -> None:
        self._log("---", f"连接建立来自 {self.addr}")
        self.conn.settimeout(0.2)      # 0.2s 一拍，兼顾响应速度与定时精度
        try:
            while not self.closed:
                self._pump()
                if self._tick():
                    break
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
                    except Exception as e:      # 单帧异常不弄死会话
                        self._log("ERR", f"处理帧异常: {type(e).__name__}: {e}")
                    if self.closed:             # 已断链：同一批里的后续帧不再处理
                        break
                self._pump()
        finally:
            self.closed = True
            self._selected.clear()      # 会话结束：未执行的选择一律作废
            try:
                self.conn.close()
            except OSError:
                pass
            tail = f"（{self.abort_reason}）" if self.abort_reason else ""
            self._log("---", f"连接关闭{tail}")

    # ------------------------------------------------------------ 定时器
    def _tick(self) -> bool:
        """检查 t1 / t2 / t3。返回 True 表示该断链了。"""
        now = time.monotonic()

        # ---- t1：发出 I 帧后迟迟拿不到确认
        if self._tx_unacked() > 0 and (now - self._t1_base) > self.t1:
            self._abort(f"t1 超时：{self.t1:.1f}s 未收到确认，窗口占用 {self._tx_unacked()}")
            return True

        # ---- t2：有未确认的接收 → 攒够 W 个或从**第一帧未确认 I 帧**起算 t2 到点
        # ⚠️ 锚点必须是 `_t2_anchor`，不能用 `_last_recv`：后者每收到任何帧都刷新，
        #    主站慢速点滴（比如 1 帧/3s）时 S 帧会被无限推后，
        #    主站侧 t1 先超时断链 —— 现场表现是"慢速数据偶尔掉线"，极难查。
        if self._pending_ack > 0:
            if self._pending_ack >= self.w:
                self.send_s()
            elif self._t2_anchor is not None and (now - self._t2_anchor) >= self.t2:
                self.send_s()

        # ---- t3：空闲太久发 TESTFR act；发出后按 t1 等 con
        if self._test_at is None:
            if (now - max(self._last_send, self._last_recv)) >= self.t3:
                self.send_u(UFrame.TESTFR_ACT)
                self._test_at = time.monotonic()
        elif (now - self._test_at) > self.t1:
            self._abort(f"TESTFR act 无应答：{self.t1:.1f}s 未收到 con")
            return True

        return False

    def _abort(self, reason: str) -> None:
        self.abort_reason = reason
        self._log("!!!", f"断链：{reason}")
        self.closed = True

    # ------------------------------------------------------------ 帧处理
    def _handle(self, apdu: Apdu) -> None:
        self._last_recv = time.monotonic()
        self._test_at = None            # 收到任何帧都证明链路活着

        if apdu.is_u:
            self._handle_u(apdu.u_code)
        elif apdu.is_s:
            self._note_ack(apdu.rx)
        else:
            # ⭐ 先校验接收序号 N(S)：重放 / 乱序 / 重复帧会让 rx 失步，
            #    之后所有确认全部错位。宁可直接断链，也不带着错位的序号跑。
            #    （与 lib60870 行为一致）
            if apdu.tx != self.rx:
                self._abort(f"N(S) 不连续：期望 {self.rx} 收到 {apdu.tx}")
                return
            self._note_ack(apdu.rx)     # I 帧同样携带对端的接收序号
            first_unacked = self._pending_ack == 0
            self._pending_ack += 1
            if first_unacked:
                # t2 锚点 = 第一帧未确认 I 帧到达的时刻（收到后续帧不刷新它）
                self._t2_anchor = time.monotonic()
            self._handle_i(apdu)

    def _note_ack(self, rx: int) -> None:
        """对端确认到 rx：窗口前移；窗口空了就重置 t1 计时。"""
        if rx == self.peer_rx:
            return
        # 超前确认（确认了我们还没发的序号）不可信，忽略并留痕
        if ((self.tx - rx) % 32768) > 16384:
            self._log("!!!", f"忽略异常确认 rx={rx}（我方 tx={self.tx}）")
            return
        self.peer_rx = rx
        if self._tx_unacked() == 0:
            self._t1_base = time.monotonic()

    def _handle_u(self, code: int) -> None:
        if code == UFrame.STARTDT_ACT:
            self.send_u(UFrame.STARTDT_CON)
            self.started = True
        elif code == UFrame.STOPDT_ACT:
            self.send_u(UFrame.STOPDT_CON)
            self.started = False
            # STOPDT 后链路停传，未执行的选择作废：再 STARTDT 必须先重新选择
            if self._selected:
                self._log("CMD", f"STOPDT：作废 {len(self._selected)} 个未执行的选择")
                self._selected.clear()
        elif code == UFrame.TESTFR_ACT:
            self.send_u(UFrame.TESTFR_CON)

    def _handle_i(self, apdu: Apdu) -> None:
        asdu = decode_asdu(apdu.asdu)
        if asdu is None:
            self.rx = (self.rx + 1) % 32768
            return

        if asdu.test:
            self._log("---", "收到 TEST 帧，忽略")
            self.rx = (self.rx + 1) % 32768
            return

        # ⭐ 总召唤：TypeID=100, COT=6(激活)
        if asdu.type_id == int(TypeID.C_IC_NA_1) and asdu.cot == COT.ACTIVATION:
            self.rx = (self.rx + 1) % 32768
            self._do_interrogation(asdu)
            return

        # 遥控：TypeID=45 —— ⭐ 选择 → 确认；执行 → 确认 + 变位上送
        if asdu.type_id == int(TypeID.C_SC_NA_1):
            self.rx = (self.rx + 1) % 32768
            self._handle_command(asdu)
            return

        # 对时：TypeID=103, COT=6(激活) —— 记录偏移 + 回确认（回显时标）
        if asdu.type_id == int(TypeID.C_CS_NA_1) and asdu.cot == COT.ACTIVATION:
            self.rx = (self.rx + 1) % 32768
            self._handle_clock_sync(asdu, apdu.asdu)
            return

        # 其它未处理类型：只记接收序号，由 t2/W 负责回确认
        self.rx = (self.rx + 1) % 32768

    # ------------------------------------------------------------ 遥控（选择-执行）
    def _handle_command(self, asdu: Asdu) -> None:
        """C_SC_NA_1 两段式 + 执行前的动作约束。

        ⭐ 两段式的意义：防误操作。手一抖点错了，第一步就停住。
        ⭐ 执行必须与选择**同值**：选择是"预演"，执行与预演不一致一律拒绝
           （`value_mismatch`）—— 先选"合"再执行"分"绝不能真的动。
        ⭐ 执行前还有四条**动作约束**（顺序判定，任一不满足就否定确认且不驱动执行器）：
            ① IOA 未绑定遥控（本函数开头）
            ② 设备离线：回读点品质带 IV
            ③ 互斥组：同组内其它通道当前回读为 ON
            ④ 最小间隔：同一 IOA 距上次成功执行不足 min_interval
        ⚠️ 这些是"动作前判断"，秒级，不承担功能安全 —— 人身安全靠外部硬件回路。
        """
        ca = self.station.ca
        for obj in asdu.objects:
            binding = self.station.binding(obj.ioa)
            if binding is None:
                self._log("CMD", f"拒绝（unbound）：IOA={obj.ioa} 未绑定遥控")
                self._reply_command(obj, ca, negative=True)
                continue

            if obj.select:
                # ① 选择：记下时刻**与预演的值**，回肯定确认
                self._selected[obj.ioa] = (time.monotonic(), bool(obj.value), True)
                self._log("CMD", f"选择 IOA={obj.ioa} -> 确认（等待执行）")
                self._reply_command(obj, ca, negative=False)
                continue

            # ② 执行：必须有未超时的选择
            entry = self._selected.pop(obj.ioa, None)
            if entry is None:
                self._log("CMD", f"拒绝（no_select）：IOA={obj.ioa} 未先选择")
                self._reply_command(obj, ca, negative=True)
                continue
            select_at, select_value, _select_bit = entry
            if (time.monotonic() - select_at) > self.select_timeout:
                self._log("CMD", f"拒绝（select_timeout）：IOA={obj.ioa} 选择超时"
                                 f"（>{self.select_timeout:.0f}s）")
                self._reply_command(obj, ca, negative=True)
                continue

            # ⭐ 执行值必须等于选择时预演的值（选择-执行闭环）
            if bool(obj.value) != select_value:
                self._log("CMD", f"拒绝（value_mismatch）：IOA={obj.ioa} "
                                 f"选择 {select_value} 执行 {bool(obj.value)}")
                self._reply_command(obj, ca, negative=True)
                continue

            # ③ 动作约束：任一不满足就不驱动执行器
            blocked = self._check_constraints(binding)
            if blocked is not None:
                reason, detail = blocked
                # ⚠️ 拒绝原因要能一眼区分（排查时先看括号里的短码，再看细节）
                self._log("CMD", f"拒绝（{reason}）：IOA={obj.ioa} {detail}")
                self._reply_command(obj, ca, negative=True)
                continue

            ok = self._execute(binding, obj.value)
            self._reply_command(obj, ca, negative=not ok)
            if ok:
                self._last_exec[obj.ioa] = time.monotonic()
                self._send_spontaneous(binding.readback_ioa)

    def _check_constraints(self, binding: CommandBinding) -> Optional[Tuple[str, str]]:
        """执行前的动作约束判定。

        返回 `None` 表示可以执行；否则返回 `(短码, 人话细节)`，短码用于日志区分：
            offline  回读点品质带 IV（不知道现场是什么状态，不能动）
            mutex    互斥组内其它通道当前为 ON
            interval 距上次成功执行太近
        """
        # ---- ① 设备离线：回读点带 IV → 现场状态不可知，绝不能动
        if binding.readback_ioa is not None:
            p = self.station.get(binding.readback_ioa)
            if p is not None and (p.quality & QDS.IV):
                return "offline", (f"回读点 IOA={p.ioa} 品质 "
                                   f"{q_.describe(p.quality)}（设备离线，禁止动作）")

        # ---- ② 互斥组：同组内**其它**通道当前回读为 ON → 拒绝
        if binding.mutex_group is not None:
            for other in self.station.all_bindings():
                if other.ioa == binding.ioa or other.mutex_group != binding.mutex_group:
                    continue
                if self._channel_readback_on(other):
                    return "mutex", (f"互斥组 {binding.mutex_group!r} 内 IOA={other.ioa}"
                                     f"（通道 {other.output_index}）当前为 ON")

        # ---- ③ 最小间隔：同一 IOA 距上次成功执行不足 min_interval
        last = self._last_exec.get(binding.ioa)
        if last is not None:
            limit = (binding.min_interval if binding.min_interval is not None
                     else self.min_interval)
            gap = time.monotonic() - last
            if gap < limit:
                return "interval", f"距上次成功执行 {gap:.2f}s < {limit:.2f}s（最小间隔）"

        return None

    def _channel_readback_on(self, binding: CommandBinding) -> bool:
        """某个通道当前是不是 ON。

        优先看该通道的遥信点（协议侧看到的状态），没绑回读点就直接问执行器。
        ⚠️ 读不到按"不是 ON"处理 —— 互斥判定只拦"确信为 ON"的情况。
        """
        if binding.readback_ioa is not None:
            p = self.station.get(binding.readback_ioa)
            if p is not None:
                return bool(p.value)
        return bool(self.actuator.readback(binding.output_index))

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
                # 时标取"对时后的钟"，这样 SOE 与主站时间对齐（秒级）
                p.timestamp = self.clock.now()

        self._log("CMD", f"执行成功：通道 {idx} -> {bool(value)}（回读一致）")
        return True

    # ------------------------------------------------------------ 突发上送（SOE）
    def _point_timestamp(self, p) -> datetime:
        """给**带时标类型**的点打时标：点表里已有（刚变位时记的）就用它，
        否则取当前的钟（含对时偏移）。

        取完写回点表：同一事件被多个会话上送时用同一个时标。
        """
        ts = p.timestamp or self.clock.now()
        p.timestamp = ts
        return ts

    def _info_object(self, p) -> InformationObject:
        """点表对象 → ASDU 信息对象。

        ⭐ **线类型完全由点表声明的 type 决定**：
           - `M_SP_NA_1`(1)：不带时标（总召唤 / 突发 / 周期都是 1）
           - `M_SP_TB_1`(30)：带 CP56Time2a 时标（总召唤 / 突发 / 周期都是 30）
        ⚠️ 只有 30 才附时标；遥测（M_ME_NC_1）等类型保持原样。
        """
        if p.type_id == int(TypeID.M_SP_TB_1):
            return InformationObject(p.ioa, p.value, p.quality,
                                     timestamp=self._point_timestamp(p))
        return InformationObject(p.ioa, p.value, p.quality)

    def _spont_frame(self, p) -> bytes:
        """一个变化点的突发帧（COT=3），类型跟点表声明走。

        ⚠️ 同一个点**不能**"总召唤发一种类型、突发发另一种"：主站按点表配的
           类型收帧，类型对不上会当成未知点直接丢弃 —— 变位就丢了（互操作实测踩过）。
        """
        return encode_asdu(
            p.type_id, COT.SPONTANEOUS, self.station.ca,
            [self._info_object(p)],
        )

    def _send_spontaneous(self, ioa: Optional[int]) -> None:
        """执行成功后主动上送变位（COT=3 突发，类型跟点表声明走）。"""
        if ioa is None:
            return
        p = self.station.get(ioa)
        if p is None:
            return
        self.send_asdu(self._spont_frame(p))

    def send_spontaneous_points(self, points) -> None:
        """把一批变化点按突发（COT=3）上送。**每点一帧**，便于日志与排查。"""
        for p in points:
            self.send_asdu(self._spont_frame(p))

    # ------------------------------------------------------------ 对时
    def _handle_clock_sync(self, asdu: Asdu, raw_asdu: bytes) -> None:
        """C_CS_NA_1 对时：记录偏移 → 回确认（回显收到的时标）。

        ⚠️ 时标缺失/非法（CP56Time2a 解不出来，例如全 0）时回**否定确认**，
           并把收到的 IOC（IOA + 7 字节时标）原样回显 —— 子站不能假装收到了
           一个自己看不懂的时刻，但也不能对主站的报文"装死"。
        """
        obj = asdu.objects[0] if asdu.objects else None
        ts = obj.timestamp if obj is not None else None
        if ts is None:
            self._log("TIME", "对时帧时标非法/缺失（CP56Time2a 解析失败），回否定确认")
            self.send_asdu(self._clock_confirm_raw(raw_asdu, negative=True))
            return

        offset = self.clock.update(ts)
        self._log("TIME", f"对时：主站时标 {ts.isoformat()}，偏移 {offset:+.3f}s")
        if abs(offset) > 5.0:
            self._log("WARN", f"时钟偏移 {offset:+.3f}s 超过 5s：两端时钟源可能不一致")

        # 回确认：TypeID 不变、COT=7、**回显收到的时标与 IOA**
        self.send_asdu(encode_asdu(
            int(TypeID.C_CS_NA_1), COT.ACTIVATION_CONFIRM, self.station.ca,
            [InformationObject(obj.ioa, timestamp=ts)],
        ))

    def _clock_confirm_raw(self, raw_asdu: bytes, negative: bool) -> bytes:
        """拼 103 的确认帧：**IOC 原样照抄**（时标非法/缺失时用）。

        ⚠️ 为什么不走 `encode_asdu`：`asdu.py` 里 103 的编码只能把 datetime
           编成一个**合法**时标（timestamp=None 会取当前 UTC），做不到原样
           回显一个非法/缺失的 CP56Time2a。这里只换 COT 字节（可置 P/N），
           其余 CA/IOA/时标与收到的报文逐字节一致。
        """
        ca = self.station.ca
        cot = int(COT.ACTIVATION_CONFIRM) | (COT_PN_BIT if negative else 0)
        header = bytes((int(TypeID.C_CS_NA_1), 0x01, cot, 0x00,
                        ca & 0xFF, (ca >> 8) & 0xFF))
        ioc = raw_asdu[6:16]           # 第一个对象的 IOC：IOA(3) + CP56(7)
        if len(ioc) < 10:
            ioc = bytes(10)            # 报文被截断：补 0，至少有 IE 与 IOC 长度
        return header + ioc

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

        # ② 数据上送（COT=20），按类型 + IOA 连续性用 SQ=1 打包
        # ⚠️ 类型由点表声明决定：M_SP_NA_1 发 1（不带时标），M_SP_TB_1 发 30（带时标）
        for group in self.station.interrogation_groups():
            self.send_asdu(encode_asdu(
                group[0].type_id, COT.INTERROGATED_BY_STATION, ca,
                [self._info_object(p) for p in group],
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
        # ⚠️ 先按**类型**分组再按 IOA 连续性打包：SQ=1 的一帧只能有一种 TypeID，
        #    混着打包会把不同声明的点按第一个点的类型编出去。
        by_type: Dict[int, List] = {}
        for p in self.station.cyclic_points():
            by_type.setdefault(p.type_id, []).append(p)
        for type_id in sorted(by_type):
            for group in Station.group_consecutive(
                    by_type[type_id],
                    max_objects=max_objects_per_frame(type_id)):
                self.send_asdu(encode_asdu(
                    type_id, COT.PERIODIC, self.station.ca,
                    [self._info_object(p) for p in group],
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
                 poll_s: float = 0.0,
                 k: int = 12, w: int = 8,
                 t1: float = 15.0, t2: float = 10.0, t3: float = 20.0,
                 min_interval: float = 1.0) -> None:
        self.station = station
        self.host = host
        self.port = port
        self.cyclic_s = cyclic_s
        self.verbose = verbose
        self.actuator = actuator or MemoryActuator()
        self.select_timeout = select_timeout
        self.min_interval = min_interval      # 同一 IOA 两次成功执行的最小间隔（秒）
        self.source = source
        self.poll_s = poll_s
        self.k = k
        self.w = w
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self._poll_thread: Optional[threading.Thread] = None

        self._srv: Optional[socket.socket] = None
        self._sessions: List[Session] = []
        self._running = False
        self._accept_thread: Optional[threading.Thread] = None
        self._cyclic_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------ 生命周期
    @property
    def sessions(self) -> List["Session"]:
        """当前会话列表（测试与排障用）。"""
        return [s for s in self._sessions if s.is_alive()]

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
        # 等会话线程收尾，避免解释器退出时 daemon 线程还在写 stdout
        deadline = time.monotonic() + 2.0
        for s in list(self._sessions):
            left = deadline - time.monotonic()
            if left <= 0:
                break
            s.join(timeout=left)
        self._sessions = []

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
                           actuator=self.actuator, select_timeout=self.select_timeout,
                           min_interval=self.min_interval,
                           k=self.k, w=self.w, t1=self.t1, t2=self.t2, t3=self.t3)
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
