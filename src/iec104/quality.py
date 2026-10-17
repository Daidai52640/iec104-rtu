"""品质位工具：QDS（遥测）与 SIQ 高位（遥信）的读写。

语义（⚠️ 位定义需与标准原文逐位核对）：

    IV  0x80  无效       值不可用：设备读不到、越界、校验失败
    NT  0x40  非当前值   值过期：曾经有效，但已经不新鲜
    SB  0x20  被取代     值被人工/其他来源取代
    BL  0x10  被闭锁     值被闭锁
    OV  0x01  溢出       仅 QDS（遥测）有

⭐ 纪律：**设备读不到时不能把旧值当好值上送**。
   宁可上送"无效"，也不能给主站一个看起来正常、实际过期的数。
"""

from .consts import QDS


def is_good(quality: int) -> bool:
    """没有任何异常标志 = 品质良好。"""
    return (quality & 0xF1) == 0


def mark_invalid(quality: int) -> int:
    """置 IV：值不可用。"""
    return quality | QDS.IV


def mark_stale(quality: int) -> int:
    """置 NT：值不新鲜（但仍保留数值）。"""
    return quality | QDS.NT


def clear_faults(quality: int) -> int:
    """清掉 IV / NT（设备恢复、读到新值时用）。保留 BL / SB / OV。"""
    return quality & ~(QDS.IV | QDS.NT)


def with_flags(base: int, *, invalid: bool = False, stale: bool = False) -> int:
    """按布尔条件构造品质。"""
    q = clear_faults(base)
    if invalid:
        q = mark_invalid(q)
    elif stale:
        q = mark_stale(q)
    return q


def describe(quality: int) -> str:
    """给人看的品质描述（日志/证据用）。"""
    if is_good(quality):
        return "good"
    names = []
    if quality & QDS.IV:
        names.append("iv")
    if quality & QDS.NT:
        names.append("nt")
    if quality & QDS.SB:
        names.append("sb")
    if quality & QDS.BL:
        names.append("bl")
    if quality & QDS.OV:
        names.append("ov")
    return "|".join(names) or f"unknown({quality:#04x})"
