"""IEC 60870-5-104 子站

不依赖任何第三方 104 库。当前能力：
    ✅ 建链 STARTDT / STOPDT / TESTFR
    ✅ 会话层定时器 t1 / t2 / t3 + 发送窗口 k / 确认阈值 w
    ✅ 接收序号 N(S) 连续性校验（不连续直接断链）
    ✅ 总召唤：激活 → 确认 → 数据(COT=20) → 终止(COT=10)
    ✅ SQ=0 / SQ=1 两种打包
    ✅ 周期上送 COT=1
    ✅ 遥控「选择-执行」两段式 + 独立回读 + 变位上送
    ✅ 执行前动作约束（设备离线 / 互斥组 / 最小间隔）+ 选择-执行值必须一致
    ✅ 品质位映射（掉线 → IV）+ 采集循环 + 突发上送
    ✅ 对时 C_CS_NA_1 + SOE 时标上送
       ⭐ 线类型由点表声明的 type 决定：M_SP_NA_1 全路径 1（不带时标）、
          M_SP_TB_1 全路径 30（带时标，SOE）
       ⚠️ SOE 时标精度受本地钟限制，不是毫秒级
    ✅ 短浮点(小端) / 单点 / 带时标单点 / 单点命令 编解码
    ✅ 南向数据源：桩 / Modbus TCP（从现场采集进程暴露的从站读，不抢串口）
    ✅ 南向数据源：Modbus RTU（自己开串口扫 485 总线，可挂多个从站）
    ✅ 南向执行器：Modbus RTU 写线圈（FC05）+ 回读（FC01）
"""

from .actuator import Actuator, MemoryActuator
from .asdu import Asdu, InformationObject, decode_asdu, decode_cp56, encode_asdu, encode_cp56
from .clock import ClockOffset
from .link import Apdu, decode_apdu, encode_i_frame, encode_s_frame, encode_u_frame, iter_apdus
from .quality import clear_faults, describe, is_good, mark_invalid, mark_stale
from .server import Iec104Slave, Session
from .rtu import (
    ModbusRtuActuator, ModbusRtuSource, RtuError, RtuLink, crc16_modbus,
)
from .source import ModbusTcpSource, Source, StubSource
from .station import CommandBinding, Point, Station
from .consts import BROADCAST_CA, COT, QDS, SCO, SIQ, FrameFormat, SessionParam, TypeID, UFrame

__all__ = [
    "Actuator", "MemoryActuator",
    "Asdu", "InformationObject", "decode_asdu", "encode_asdu", "decode_cp56", "encode_cp56",
    "Apdu", "decode_apdu", "encode_i_frame", "encode_s_frame", "encode_u_frame", "iter_apdus",
    "clear_faults", "describe", "is_good", "mark_invalid", "mark_stale",
    "ClockOffset",
    "Iec104Slave", "Session", "Source", "StubSource", "ModbusTcpSource",
    "ModbusRtuSource", "ModbusRtuActuator", "RtuLink", "RtuError", "crc16_modbus",
    "CommandBinding", "Point", "Station",
    "BROADCAST_CA", "COT", "QDS", "SCO", "SIQ", "FrameFormat", "SessionParam", "TypeID", "UFrame",
]
