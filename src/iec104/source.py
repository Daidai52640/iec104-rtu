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
import time
from typing import Any, Dict, Optional, Tuple

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
