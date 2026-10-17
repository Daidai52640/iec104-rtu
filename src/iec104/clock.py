"""时钟：记住主站对时得到的偏移，供 SOE 打时标用。

⭐ 为什么单独一个类，而不是直接改系统时间：
    - 子站**不调** `settimeofday` / `adjtime` —— 那要 root，而且会把日志、
      systemd 的时间戳一起改掉，现场排障时反而分不清"是设备时间不对还是主站时间不对"；
    - 对时只改变**我们给数据打时标用的基准**，系统时间保持原样。

⭐ 用法（秒级，不做锁相/平滑）：
    clock = ClockOffset()
    offset = clock.update(master_utc)   # 收到 C_CS_NA_1 时调用一次
    ts = clock.now()                    # 之后所有 SOE 时标都从这里取

⚠️ 边界：这里只做"主站时间 − 本地 UTC"的一次性差值，没有晶振漂移补偿，
   也没有往返时延（RTT）扣除。104 的对时精度本来就只到秒级，
   毫秒级的时间同步不是本模块的职责。
"""

from datetime import datetime, timedelta, timezone
from typing import Optional


class ClockOffset:
    """主站时间相对本地 UTC 的偏移。

    偏移 = 主站时间 − 本地 UTC。收到对时前偏为 0（即完全使用本地 UTC）。
    """

    def __init__(self) -> None:
        self._offset = 0.0                       # 秒，可为负
        self._synced_at: Optional[datetime] = None   # 最近一次对时的本地时刻

    @property
    def offset(self) -> float:
        """当前生效的偏移（秒）。只读：只能由 `update` 改写。"""
        return self._offset

    @property
    def synced_at(self) -> Optional[datetime]:
        """最近一次对时的本地 UTC 时刻；从没对过时返回 None。"""
        return self._synced_at

    def update(self, remote_utc: datetime) -> float:
        """记录主站时标，返回偏移秒数（主站时间 − 本地 UTC）。

        ⚠️ 偏移按**收到这一帧的本地时刻**计算，所以它天然包含了链路时延；
           秒级的对时精度下这点时延可以忽略，但如果哪天要算毫秒级，
           必须改成"发请求时记 t0、收确认时取 (t1+t0)/2"。
        """
        if remote_utc.tzinfo is None:
            # 不带时区的时标按 UTC 解释（CP56Time2a 本来就是 UTC）
            remote_utc = remote_utc.replace(tzinfo=timezone.utc)
        local = datetime.now(timezone.utc)
        self._offset = (remote_utc - local).total_seconds()
        self._synced_at = local
        return self._offset

    def now(self) -> datetime:
        """校正后的 UTC 时间（= 本地 UTC + 偏移）。对时前就等于本地 UTC。"""
        return datetime.now(timezone.utc) + timedelta(seconds=self._offset)

    def __repr__(self) -> str:
        if self._synced_at is None:
            return "<ClockOffset 未对时>"
        return f"<ClockOffset {self._offset:+.3f}s>"


__all__ = ["ClockOffset"]
