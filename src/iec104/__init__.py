"""IEC 60870-5-104 子站

不依赖任何第三方 104 库。当前能力：
    ✅ 建链 STARTDT / STOPDT / TESTFR
    ✅ 总召唤：激活 → 确认 → 数据(COT=20) → 终止(COT=10)
    ✅ SQ=0 / SQ=1 两种打包
    ✅ 周期上送 COT=1
    ✅ 遥控「选择-执行」两段式 + 独立回读 + 变位上送
    ✅ 短浮点(小端) / 单点 / 带时标单点 / 单点命令 编解码
    ⬜ 南向 Modbus RTU（板子到货）      ← 下一步
    ⬜ 品质位映射（设备掉线 → IV/NT）   ← 下一步
    ⬜ t1/t2/t3 超时与重传              ← 下一步
"""

from .actuator import Actuator, MemoryActuator
from .asdu import Asdu, InformationObject, decode_asdu, decode_cp56, encode_asdu, encode_cp56
from .link import Apdu, decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus
from .server import Iec104Slave, Session
from .station import CommandBinding, Point, Station
from .consts import BROADCAST_CA, COT, QDS, SCO, SIQ, FrameFormat, SessionParam, TypeID, UFrame

__all__ = [
    "Actuator", "MemoryActuator",
    "Asdu", "InformationObject", "decode_asdu", "encode_asdu", "decode_cp56", "encode_cp56",
    "Apdu", "decode_apdu", "encode_i_frame", "encode_s_frame", "encode_u_frame", "iter_apdus",
    "Iec104Slave", "Session", "CommandBinding", "Point", "Station",
    "BROADCAST_CA", "COT", "QDS", "SCO", "SIQ", "FrameFormat", "SessionParam", "TypeID", "UFrame",
]
