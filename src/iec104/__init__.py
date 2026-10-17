"""IEC 60870-5-104 子站

不依赖任何第三方 104 库。当前能力：
    ✅ 建链 STARTDT / STOPDT / TESTFR
    ✅ 会话层定时器 t1 / t2 / t3 + 发送窗口 k / 确认阈值 w
    ✅ 总召唤：激活 → 确认 → 数据(COT=20) → 终止(COT=10)
    ✅ SQ=0 / SQ=1 两种打包
    ✅ 周期上送 COT=1
    ✅ 遥控「选择-执行」两段式 + 独立回读 + 变位上送
    ✅ 品质位映射（掉线 → IV）+ 采集循环 + 突发上送
    ✅ 短浮点(小端) / 单点 / 带时标单点 / 单点命令 编解码
    ⬜ 南向 Modbus RTU（板子到货）      ← 下一步
    ⬜ 对时 C_CS_NA_1 / SOE 时标上送     ← 下一步
"""

from .actuator import Actuator, MemoryActuator
from .asdu import Asdu, InformationObject, decode_asdu, decode_cp56, encode_asdu, encode_cp56
from .link import Apdu, decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus
from .quality import clear_faults, describe, is_good, mark_invalid, mark_stale
from .server import Iec104Slave, Session
from .source import Source, StubSource
from .station import CommandBinding, Point, Station
from .consts import BROADCAST_CA, COT, QDS, SCO, SIQ, FrameFormat, SessionParam, TypeID, UFrame

__all__ = [
    "Actuator", "MemoryActuator",
    "Asdu", "InformationObject", "decode_asdu", "encode_asdu", "decode_cp56", "encode_cp56",
    "Apdu", "decode_apdu", "encode_i_frame", "encode_s_frame", "encode_u_frame", "iter_apdus",
    "clear_faults", "describe", "is_good", "mark_invalid", "mark_stale",
    "Iec104Slave", "Session", "Source", "StubSource", "CommandBinding", "Point", "Station",
    "BROADCAST_CA", "COT", "QDS", "SCO", "SIQ", "FrameFormat", "SessionParam", "TypeID", "UFrame",
]
