"""点表配置载入：把"有哪些点、遥控接到哪一路"从代码搬到 JSON 文件。

⭐ 真实远动装置的点表是**运维改的，不是开发改的**。所以这里的目标有两条：
    ① 代码里不出现任何具体 IOA；
    ② 配置非法时**启动即报错**，且错误信息能直接定位到 `points[i]` / 字段名。

⚠️ 只用标准库 `json` —— 本项目零第三方依赖，不引入 PyYAML 之类的库。

配置形态（`common_address` / `points` 必填，其余可选）：

    {
      "common_address": 1,          # 0~65535
      "name": "demo",               # 子站名，仅用于日志
      "points": [
        {
          "ioa": 1,                 # 1~16777215（IOA 0 按标准保留）
          "type": "M_ME_NC_1",      # 见 SUPPORTED_TYPES
          "name": "Temp",           # 仅用于日志/展示
          "value": 23.7,            # 初始值：遥测=数字，遥信/命令=true/false
          "cyclic": true            # 参与周期上送；命令点不允许
        },
        {
          "ioa": 6,
          "type": "M_SP_NA_1",
          "name": "DI2",
          "value": true,
          "source_channel": 1       # 可选：该遥信从数据源哪一路读（无硬件时=执行器回读）
        },
        {
          "ioa": 10,
          "type": "C_SC_NA_1",
          "name": "DO1",
          "actuator": 0,            # 命令点必填：执行器通道 0~7
          "readback_ioa": 5,        # 可选：执行后回读到哪个遥信点
          "mutex_group": "relay",   # 可选：互斥组名（同组其它通道回读为 ON 时拒绝动作）
          "min_interval": 1.0       # 可选：本通道两次成功执行的最小间隔（秒）
        }
      ]
    }

⚠️ `mutex_group` / `min_interval` 是**动作约束**的声明，只能写在命令点（C_SC_NA_1）上：
    - `mutex_group` 必须是**非空字符串**。同组内任一通道当前回读为 ON 时，
      其它通道的命令在执行前被拒（否定确认，不驱动执行器）；
    - `min_interval` 必须是**非负数字**（秒）。不写就用会话默认值（1.0s），
      写 0 表示不限制。⚠️ 这是"动作前判断"，秒级，不承担功能安全。

⚠️ 支持的类型受 ASDU 编码能力约束，**不是标准全集**：
    遥信 M_SP_NA_1 / M_SP_TB_1、遥测 M_ME_NC_1、遥控 C_SC_NA_1。
   `M_ME_NA_1` 等类型 `asdu.py` 还没实现编码，这里直接按未知类型报错，
   免得配置能写进去、跑起来上送时才炸。

⭐ **点的 TypeID 由这里的 `type` 一锤定音，全路径一致**：
   - `M_SP_NA_1`：总召唤、突发变位、周期上送**全部按 TypeID=1**，任何路径都不带时标；
   - `M_SP_TB_1`：三条路径**全部按 TypeID=30**，都带 CP56Time2a 时标（SOE）。
   ⚠️ 同一个点绝不能"一条路径发 1、另一条发 30"：主站按它自己点表里的类型收帧，
      类型对不上会当成未知点丢弃 —— 互操作实测过，变位会丢。
   默认示例点表用 M_SP_NA_1（已与 c104 / QTester104 两套第三方主站验证互通）；
   需要 SOE 时把该点的 `type` 改成 `M_SP_TB_1` 即可。

⚠️ `source_channel`（只能用于单点遥信）与命令点的 `readback_ioa` 共同决定
   "执行器通道 -> 采集点"的映射（见 `readback_map`）。两者指向同一通道却给出
   不同 IOA 时按配置错误处理 —— 接线自相矛盾，越早报越好。
   `readback_ioa` 可以指向 M_SP_NA_1 或 M_SP_TB_1 的点，变位上送按该点声明的类型编码。
"""

import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

from .consts import TypeID
from .station import Station

# 执行器通道数。启动脚本也用它构造执行器，避免"校验按 8、运行时按别的数"。
ACTUATOR_CHANNELS = 8

# 配置里的类型名 -> TypeID。⚠️ 只列 ASDU 编码已实现的类型（见模块 docstring）。
SUPPORTED_TYPES: Dict[str, int] = {
    "M_SP_NA_1": int(TypeID.M_SP_NA_1),
    "M_SP_TB_1": int(TypeID.M_SP_TB_1),
    "M_ME_NC_1": int(TypeID.M_ME_NC_1),
    "C_SC_NA_1": int(TypeID.C_SC_NA_1),
}

_COMMAND_TYPE = int(TypeID.C_SC_NA_1)
# 单点遥信类型（可声明 source_channel）。
_SINGLE_TYPES = frozenset({int(TypeID.M_SP_NA_1), int(TypeID.M_SP_TB_1)})
# 值取 true/false 的类型（遥信 + 遥控）；其余按数字处理。
_BOOL_VALUE_TYPES = frozenset({
    int(TypeID.M_SP_NA_1), int(TypeID.M_SP_TB_1), _COMMAND_TYPE,
})
# 回读点只允许单点遥信（带不带时标都行）：上送 TypeID 由该点声明的类型决定。
_READBACK_TYPES = frozenset({int(TypeID.M_SP_NA_1), int(TypeID.M_SP_TB_1)})

_TOP_KEYS = frozenset({"common_address", "name", "points", "source", "actuator"})
_POINT_KEYS = frozenset({
    "ioa", "type", "name", "value", "cyclic", "source_channel",
    "actuator", "readback_ioa", "mutex_group", "min_interval",
})

_IOA_MIN, _IOA_MAX = 1, 0xFFFFFF
_CA_MIN, _CA_MAX = 0, 0xFFFF


class ConfigError(ValueError):
    """点表配置错误。

    继承 `ValueError`：调用方既能专门捕获它给出人话提示，
    也能按普通参数错误处理（例如 argparse 之后的顶层 try）。
    """


def _err(origin: str, msg: str) -> ConfigError:
    """统一错误前缀：把"哪个文件错了"也带出去，现场排障少猜一轮。"""
    return ConfigError(f"点表配置错误（{origin}）：{msg}")


def _allowed(keys) -> str:
    """把允许的字段名排成一句可读的话（顺序稳定，便于比对）。"""
    return "、".join(sorted(keys))


def _is_int(v: Any) -> bool:
    """JSON 里的 true/false 在 Python 中是 bool，而 bool 是 int 的子类，要单独排除。"""
    return isinstance(v, int) and not isinstance(v, bool)


def _point_value(spec: Dict[str, Any], type_id: int,
                 label: str, origin: str) -> Any:
    """取点的初始值；没写就按类型给默认值（遥测 0.0，遥信/命令 False）。"""
    if "value" not in spec:
        return False if type_id in _BOOL_VALUE_TYPES else 0.0

    v = spec["value"]
    if type_id in _BOOL_VALUE_TYPES:
        if not isinstance(v, bool):
            raise _err(origin, f"{label}.value = {v!r} 必须是 true/false")
        return v
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        raise _err(origin, f"{label}.value = {v!r} 必须是数字")
    return float(v)


def station_from_dict(raw: Any, origin: str = "<内存>") -> Station:
    """从已解析的 dict 建 Station。

    `load_station` 与测试共用这一套校验，避免"文件入口"和"dict 入口"两套规则。
    """
    if not isinstance(raw, dict):
        raise _err(origin, f"顶层必须是 JSON 对象，实际是 {type(raw).__name__}")

    unknown = sorted(set(raw) - _TOP_KEYS)
    if unknown:
        raise _err(origin, f"顶层含未知字段 {unknown}"
                            f"（允许：{_allowed(_TOP_KEYS)}）")

    # ---------------------------------------------------------- 公共地址 / 子站名
    if "common_address" not in raw:
        raise _err(origin, '缺少顶层字段 "common_address"')
    ca = raw["common_address"]
    if not _is_int(ca):
        raise _err(origin, f"common_address = {ca!r} 必须是整数")
    if not (_CA_MIN <= ca <= _CA_MAX):
        raise _err(origin, f"common_address = {ca} 越界"
                            f"（允许 {_CA_MIN}~{_CA_MAX}）")

    name = raw.get("name", "")
    if not isinstance(name, str):
        raise _err(origin, f"name = {name!r} 必须是字符串")

    if "points" not in raw:
        raise _err(origin, '缺少顶层字段 "points"')
    specs = raw["points"]
    if not isinstance(specs, list):
        raise _err(origin, f"points 必须是数组，实际是 {type(specs).__name__}")

    station = Station(common_address=ca, name=name)
    seen: Dict[int, int] = {}                      # ioa -> 首次出现的下标
    bindings: List[Tuple[int, Dict[str, Any], str]] = []   # 待所有点建好后解析引用

    # ---------------------------------------------------------- 逐点校验
    for i, spec in enumerate(specs):
        label = f"points[{i}]"
        if not isinstance(spec, dict):
            raise _err(origin, f"{label} 必须是 JSON 对象，实际是 {type(spec).__name__}")

        unknown = sorted(set(spec) - _POINT_KEYS)
        if unknown:
            raise _err(origin, f"{label} 含未知字段 {unknown}"
                                f"（允许：{_allowed(_POINT_KEYS)}）")

        # ---- IOA
        if "ioa" not in spec:
            raise _err(origin, f'{label} 缺少必填字段 "ioa"')
        ioa = spec["ioa"]
        if not _is_int(ioa):
            raise _err(origin, f"{label}.ioa = {ioa!r} 必须是整数")
        if not (_IOA_MIN <= ioa <= _IOA_MAX):
            raise _err(origin, f"{label}.ioa = {ioa} 越界"
                                f"（允许 {_IOA_MIN}~{_IOA_MAX}）")
        label = f"{label}（ioa={ioa}）"
        if ioa in seen:
            raise _err(origin, f"{label} 重复：points[{seen[ioa]}] 已使用同一 IOA")
        seen[ioa] = i

        # ---- 类型
        if "type" not in spec:
            raise _err(origin, f'{label} 缺少必填字段 "type"')
        type_name = spec["type"]
        if not isinstance(type_name, str):
            raise _err(origin, f"{label}.type = {type_name!r} 必须是字符串")
        type_id = SUPPORTED_TYPES.get(type_name)
        if type_id is None:
            raise _err(origin, f'{label}.type = "{type_name}" 未知'
                                f'（当前支持：{_allowed(SUPPORTED_TYPES)}）')

        # ---- 展示名
        point_name = spec.get("name", "")
        if not isinstance(point_name, str):
            raise _err(origin, f"{label}.name = {point_name!r} 必须是字符串")

        # ---- 周期上送
        cyclic = spec.get("cyclic", False)
        if not isinstance(cyclic, bool):
            raise _err(origin, f"{label}.cyclic = {cyclic!r} 必须是 true/false")
        if cyclic and type_id == _COMMAND_TYPE:
            raise _err(origin, f"{label}.cyclic = true 不合法："
                                f"C_SC_NA_1 是命令点，不参与周期上送")

        # ---- 初始值
        value = _point_value(spec, type_id, label, origin)

        # ---- 数据源通道（只有单点遥信能声明）
        source_channel = spec.get("source_channel")
        if source_channel is not None:
            if type_id not in _SINGLE_TYPES:
                raise _err(origin, f"{label}.source_channel 不合法："
                                    f"{type_name} 不是单点遥信，"
                                    f"只有 M_SP_NA_1 / M_SP_TB_1 可以声明")
            if not _is_int(source_channel):
                raise _err(origin, f"{label}.source_channel = {source_channel!r} "
                                    f"必须是整数")
            if not (0 <= source_channel < ACTUATOR_CHANNELS):
                raise _err(origin, f"{label}.source_channel = {source_channel} 越界"
                                    f"（允许 0~{ACTUATOR_CHANNELS - 1}）")

        # ---- 遥控绑定字段：命令点必须有，其它类型不许有
        if type_id == _COMMAND_TYPE:
            if "actuator" not in spec:
                raise _err(origin, f'{label} 缺少必填字段 "actuator"'
                                    f'（命令点必须绑定执行器通道）')
            actuator = spec["actuator"]
            if not _is_int(actuator):
                raise _err(origin, f"{label}.actuator = {actuator!r} 必须是整数")
            if not (0 <= actuator < ACTUATOR_CHANNELS):
                raise _err(origin, f"{label}.actuator = {actuator} 越界"
                                    f"（允许 0~{ACTUATOR_CHANNELS - 1}）")

            # ---- 动作约束（可选）：互斥组 / 最小执行间隔
            mutex_group = spec.get("mutex_group")
            if mutex_group is not None:
                if not isinstance(mutex_group, str):
                    raise _err(origin, f"{label}.mutex_group = {mutex_group!r}"
                                        f" 必须是字符串（互斥组名）")
                mutex_group = mutex_group.strip()
                if not mutex_group:
                    raise _err(origin, f"{label}.mutex_group 不能为空字符串"
                                        f"（组里没有名字就没法和别的通道对上）")

            min_interval = spec.get("min_interval")
            if min_interval is not None:
                if not isinstance(min_interval, (int, float)) or isinstance(min_interval, bool):
                    raise _err(origin, f"{label}.min_interval = {min_interval!r}"
                                        f" 必须是数字（秒）")
                if not math.isfinite(min_interval):
                    raise _err(origin, f"{label}.min_interval = {min_interval} 不是有限数")
                if min_interval < 0:
                    raise _err(origin, f"{label}.min_interval = {min_interval} 不能为负"
                                        f"（0 表示不限制）")
                min_interval = float(min_interval)

            bindings.append((ioa, spec, label, mutex_group, min_interval))
        else:
            for field in ("actuator", "readback_ioa", "mutex_group", "min_interval"):
                if field in spec:
                    raise _err(origin, f"{label}.{field} 不合法："
                                        f"只有 C_SC_NA_1 可以声明")

        station.add(ioa, type_id, value=value, cyclic=cyclic, name=point_name,
                    source_channel=source_channel)

    # ---------------------------------------------------------- 解析回读引用
    # ⚠️ 放到所有点都建好之后再解析 —— 配置里允许前面的命令点引用后面的遥信点。
    for ioa, spec, label, mutex_group, min_interval in bindings:
        actuator = spec["actuator"]
        readback_ioa = spec.get("readback_ioa")
        if readback_ioa is not None:
            if not _is_int(readback_ioa):
                raise _err(origin, f"{label}.readback_ioa = {readback_ioa!r} 必须是整数")
            target = station.get(readback_ioa)
            if target is None:
                raise _err(origin, f"{label}.readback_ioa = {readback_ioa}"
                                    f" 指向不存在的点")
            if target.type_id not in _READBACK_TYPES:
                target_name = TypeID(target.type_id).name
                raise _err(origin, f"{label}.readback_ioa = {readback_ioa}"
                                    f" 指向 IOA={readback_ioa} 的 {target_name}；"
                                    f"回读点必须是 M_SP_NA_1 或 M_SP_TB_1"
                                    f"（变位上送按该点声明的类型编码）")
        station.bind_command(ioa, output_index=actuator,
                             readback_ioa=readback_ioa,
                             mutex_group=mutex_group,
                             min_interval=min_interval)

    # ---------------------------------------------------------- 通道映射自检
    # 命令绑定的 readback_ioa 与各点声明的 source_channel 会合成
    # "数据源通道 -> 采集点"的映射；同一通道指向不同点属接线矛盾，
    # 配置阶段就报错（`readback_map` 里也做同样的检查，防止绕过配置直接建站）。
    try:
        readback_map(station)
    except ConfigError as e:
        raise _err(origin, str(e)) from e

    return station


def load_station(path: str) -> Station:
    """从 JSON 文件读点表。

    读不到文件 / JSON 语法错 / 校验不过，一律抛 `ConfigError`，
    启动脚本捕获后直接退出 —— 宁可不启动，也不带着半张错点表跑。
    """
    path = os.fspath(path)
    try:
        # utf-8-sig：允许 Windows 记事本存出的 UTF-8 BOM，否则 json.load 会报
        # "Unexpected UTF-8 BOM" —— 这是现场改配置时最常见的低级坑。
        with open(path, "r", encoding="utf-8-sig") as f:
            raw = json.load(f)
    except FileNotFoundError:
        raise _err(path, "配置文件不存在") from None
    except OSError as e:
        raise _err(path, f"无法读取配置文件：{e}") from e
    except UnicodeDecodeError:
        # 现场最常见的低级坑：记事本另存成 ANSI/GBK 或 UTF-16。
        # 不猜编码自动转换 —— 猜错了中文点表名会静默变乱码。
        raise _err(path, "文件不是 UTF-8 编码（请另存为 UTF-8）") from None
    except json.JSONDecodeError as e:
        raise _err(path, f"JSON 语法错误（第 {e.lineno} 行第 {e.colno} 列）："
                          f"{e.msg}") from e

    return station_from_dict(raw, origin=path)


def readback_map(station: Station) -> Dict[int, int]:
    """数据源通道 -> 采集点 IOA（给 `StubSource` 构造用）。

    ⭐ 两个来源取并集，替代原先写死在启动脚本里的 READBACK_MAP：
        - 命令绑定的回读：`{actuator_index: readback_ioa}`
        - 遥信点声明的 `source_channel`：`{source_channel: ioa}`
    同一通道被两处指向不同 IOA 时抛 `ConfigError` —— 那是接线矛盾，
    不猜"谁优先"。
    """
    mapping: Dict[int, int] = {}
    declared: Dict[int, str] = {}       # 通道 -> 谁声明的（报错用）

    for p in station.all():
        channel = p.source_channel
        if channel is None:
            continue
        if channel in mapping and mapping[channel] != p.ioa:
            raise ConfigError(
                f"数据源通道 {channel} 冲突：IOA={mapping[channel]} 与 "
                f"IOA={p.ioa} 都声明了 source_channel")
        mapping[channel] = p.ioa
        declared[channel] = f"IOA={p.ioa} 的 source_channel"

    for b in station.all_bindings():
        channel = b.output_index
        if b.readback_ioa is None:
            continue
        if channel in mapping and mapping[channel] != b.readback_ioa:
            raise ConfigError(
                f"数据源通道 {channel} 冲突：{declared[channel]} 指向 "
                f"IOA={mapping[channel]}，而命令点 IOA={b.ioa} 的 "
                f"readback_ioa 指向 IOA={b.readback_ioa}")
        mapping[channel] = b.readback_ioa
        declared[channel] = f"命令点 IOA={b.ioa} 的 readback_ioa"

    return mapping


# ---------------------------------------------------------------- 数据源配置
#: `source` 段允许的字段
_SOURCE_KEYS = frozenset({"type", "host", "port", "baud", "unit", "default_unit",
                          "timeout", "map"})
_SOURCE_MAP_KEYS = frozenset({"ioa", "unit", "kind", "register", "address",
                              "scale", "signed", "bit"})
_SOURCE_TYPES = frozenset({"modbus_tcp", "modbus_rtu"})
_RTU_KINDS = frozenset({"hr", "ir", "coil", "bit"})


def source_spec_from_dict(raw: Any, station: Optional[Station] = None,
                          origin: str = "<内存>") -> Dict[str, Any]:
    """校验点表里的 `source` 段。

    ⭐ 为什么数据源也进点表：现场设备地址、寄存器、系数都是"点表级"的事实，
       和 IOA 一样属于运维改的东西，不该写死在启动脚本里。

    两种形态：
        `modbus_tcp` —— 从某个 Modbus TCP 从站读（同一台机器上已有采集进程时用）
        `modbus_rtu` —— 自己开串口扫 485 总线（独占窗口用，可接多个从站）

    返回可直接喂给数据源构造器的参数。
    """
    if not isinstance(raw, dict):
        raise _err(origin, f'"source" 必须是 JSON 对象，实际是 {type(raw).__name__}')
    unknown = sorted(set(raw) - _SOURCE_KEYS)
    if unknown:
        raise _err(origin, f'"source" 含未知字段 {unknown}'
                          f'（允许：{_allowed(_SOURCE_KEYS)}）')

    stype = raw.get("type")
    if stype not in _SOURCE_TYPES:
        raise _err(origin, f'"source.type" 不支持 {stype!r}'
                          f'（当前支持：{_allowed(_SOURCE_TYPES)}）')

    timeout = raw.get("timeout", 2.0)
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise _err(origin, f'"source.timeout" 必须是数字，实际是 {type(timeout).__name__}')
    if timeout <= 0:
        raise _err(origin, f'"source.timeout" 必须 > 0：{timeout}')

    maps = raw.get("map")
    if not isinstance(maps, list) or not maps:
        raise _err(origin, '"source.map" 必须是非空数组')

    out: Dict[str, Any] = {"type": stype, "timeout": float(timeout)}

    if stype == "modbus_tcp":
        host = raw.get("host", "127.0.0.1")
        if not isinstance(host, str) or not host:
            raise _err(origin, '"source.host" 必须是非空字符串')

        def _int_field(name: str, lo: int, hi: int, default: Optional[int] = None) -> int:
            v = raw.get(name, default)
            if v is None:
                raise _err(origin, f'"source" 缺少字段 "{name}"')
            if isinstance(v, bool) or not isinstance(v, int):
                raise _err(origin, f'"source.{name}" 必须是整数，实际是 {type(v).__name__}')
            if not (lo <= v <= hi):
                raise _err(origin, f'"source.{name}" 超出范围 [{lo}, {hi}]：{v}')
            return v

        out.update(host=host, port=_int_field("port", 1, 65535, 502),
                   unit=_int_field("unit", 1, 247, 1))
        mapping = []
        seen_ioa = set()
        for i, m in enumerate(maps):
            label = f"source.map[{i}]"
            if not isinstance(m, dict):
                raise _err(origin, f"{label} 必须是 JSON 对象")
            unk = sorted(set(m) - {"ioa", "register", "address", "scale", "signed"})
            if unk:
                raise _err(origin, f"{label} 含未知字段 {unk}")
            ioa = m.get("ioa")
            if isinstance(ioa, bool) or not isinstance(ioa, int):
                raise _err(origin, f"{label}.ioa 必须是整数")
            if station is not None and station.get(ioa) is None:
                raise _err(origin, f"{label}.ioa={ioa} 在点表里不存在")
            if ioa in seen_ioa:
                raise _err(origin, f"{label}.ioa={ioa} 重复")
            seen_ioa.add(ioa)
            reg = m.get("register", m.get("address"))
            if isinstance(reg, bool) or not isinstance(reg, int) or reg < 0:
                raise _err(origin, f"{label}.register 必须是 >= 0 的整数")
            scale = m.get("scale", 1)
            if isinstance(scale, bool) or not isinstance(scale, (int, float)) or scale <= 0:
                raise _err(origin, f"{label}.scale 必须是 > 0 的数字")
            signed = m.get("signed", False)
            if not isinstance(signed, bool):
                raise _err(origin, f"{label}.signed 必须是 true/false")
            mapping.append((ioa, reg, float(scale), signed))
        out["mapping"] = mapping
        return out

    # ---------------- modbus_rtu
    port = raw.get("port")
    if not isinstance(port, str) or not port:
        raise _err(origin, '"source.port" 在 RTU 模式下必须是非空字符串'
                          '（Linux 形如 /dev/ttyUSB0，Windows 形如 COM3）')
    baud = raw.get("baud", 4800)
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise _err(origin, f'"source.baud" 必须是正整数，实际是 {baud!r}')
    default_unit = raw.get("default_unit", raw.get("unit", 1))
    if isinstance(default_unit, bool) or not isinstance(default_unit, int) \
            or not (1 <= default_unit <= 247):
        raise _err(origin, f'"source.default_unit" 必须在 [1, 247]：{default_unit!r}')

    mapping = []
    seen_ioa = set()
    for i, m in enumerate(maps):
        label = f"source.map[{i}]"
        if not isinstance(m, dict):
            raise _err(origin, f"{label} 必须是 JSON 对象")
        unk = sorted(set(m) - _SOURCE_MAP_KEYS)
        if unk:
            raise _err(origin, f"{label} 含未知字段 {unk}"
                              f"（允许：{_allowed(_SOURCE_MAP_KEYS)}）")
        ioa = m.get("ioa")
        if isinstance(ioa, bool) or not isinstance(ioa, int):
            raise _err(origin, f"{label}.ioa 必须是整数")
        if station is not None and station.get(ioa) is None:
            raise _err(origin, f"{label}.ioa={ioa} 在点表里不存在")
        if ioa in seen_ioa:
            raise _err(origin, f"{label}.ioa={ioa} 重复")
        seen_ioa.add(ioa)

        unit = m.get("unit", default_unit)
        if isinstance(unit, bool) or not isinstance(unit, int) or not (1 <= unit <= 247):
            raise _err(origin, f"{label}.unit 必须在 [1, 247]：{unit!r}")

        kind = m.get("kind", "hr")
        if kind not in _RTU_KINDS:
            raise _err(origin, f"{label}.kind 不支持 {kind!r}"
                              f"（允许：{_allowed(_RTU_KINDS)}）")

        addr = m.get("address", m.get("register"))
        if isinstance(addr, bool) or not isinstance(addr, int) or addr < 0:
            raise _err(origin, f"{label}.address 必须是 >= 0 的整数")

        scale = m.get("scale", 1)
        if isinstance(scale, bool) or not isinstance(scale, (int, float)) or scale <= 0:
            raise _err(origin, f"{label}.scale 必须是 > 0 的数字")

        signed = m.get("signed", False)
        if not isinstance(signed, bool):
            raise _err(origin, f"{label}.signed 必须是 true/false")

        bit = m.get("bit")
        if kind == "bit":
            if bit is None:
                raise _err(origin, f'{label}.kind="bit" 必须同时给 bit（0~15）')
            if isinstance(bit, bool) or not isinstance(bit, int) or not (0 <= bit <= 15):
                raise _err(origin, f"{label}.bit={bit!r} 超出 0~15")
        elif bit is not None:
            raise _err(origin, f'{label}.kind={kind!r} 不接受 bit 字段')

        mapping.append((ioa, unit, kind, addr, float(scale), signed, bit))

    out.update(port=port, baud=baud, default_unit=default_unit, mapping=mapping)
    return out


def load_source_spec(path: str, station: Optional[Station] = None) -> Optional[Dict[str, Any]]:
    """从点表 JSON 里读 `source` 段。

    没有该段 ⇒ 返回 `None`（表示用桩数据源，无硬件也能跑）。
    """
    path = os.fspath(path)
    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            raw = json.load(f)
    except FileNotFoundError:
        raise _err(path, "配置文件不存在") from None
    except json.JSONDecodeError as e:
        raise _err(path, f"JSON 语法错误（第 {e.lineno} 行第 {e.colno} 列）：{e.msg}") from e
    except (OSError, UnicodeDecodeError) as e:
        raise _err(path, f"无法读取配置文件：{e}") from e

    if not isinstance(raw, dict) or "source" not in raw:
        return None
    return source_spec_from_dict(raw["source"], station=station, origin=path)


#: `actuator` 段允许的字段（只有 RTU 模式用）
_ACTUATOR_KEYS = frozenset({"type", "unit", "coil_base", "channels"})
_ACTUATOR_TYPES = frozenset({"modbus_rtu"})


def actuator_spec_from_dict(raw: Any, origin: str = "<内存>") -> Dict[str, Any]:
    """校验点表里可选的 `actuator` 段（遥控落到哪一路继电器）。"""
    if not isinstance(raw, dict):
        raise _err(origin, f'"actuator" 必须是 JSON 对象，实际是 {type(raw).__name__}')
    unknown = sorted(set(raw) - _ACTUATOR_KEYS)
    if unknown:
        raise _err(origin, f'"actuator" 含未知字段 {unknown}'
                          f'（允许：{_allowed(_ACTUATOR_KEYS)}）')

    atype = raw.get("type")
    if atype not in _ACTUATOR_TYPES:
        raise _err(origin, f'"actuator.type" 不支持 {atype!r}'
                          f'（当前支持：{_allowed(_ACTUATOR_TYPES)}）')

    def _int(name: str, lo: int, hi: int, default=None) -> int:
        v = raw.get(name, default)
        if v is None:
            raise _err(origin, f'"actuator" 缺少字段 "{name}"')
        if isinstance(v, bool) or not isinstance(v, int):
            raise _err(origin, f'"actuator.{name}" 必须是整数')
        if not (lo <= v <= hi):
            raise _err(origin, f'"actuator.{name}" 超出范围 [{lo}, {hi}]：{v}')
        return v

    return {
        "type": atype,
        "unit": _int("unit", 1, 247, 2),
        "coil_base": _int("coil_base", 0, 65535, 0),
        "channels": _int("channels", 1, 64, ACTUATOR_CHANNELS),
    }


def load_actuator_spec(path: str) -> Optional[Dict[str, Any]]:
    """从点表 JSON 里读可选的 `actuator` 段；没有就返回 None。"""
    path = os.fspath(path)
    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            raw = json.load(f)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as e:
        raise _err(path, f"无法读取配置文件：{e}") from e
    if not isinstance(raw, dict) or "actuator" not in raw:
        return None
    return actuator_spec_from_dict(raw["actuator"], origin=path)


__all__ = [
    "ACTUATOR_CHANNELS", "ConfigError", "SUPPORTED_TYPES",
    "actuator_spec_from_dict", "load_actuator_spec", "load_source_spec",
    "load_station", "readback_map", "source_spec_from_dict", "station_from_dict",
]
