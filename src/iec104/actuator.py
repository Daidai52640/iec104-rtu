"""执行器：协议层与硬件层的接缝。

⭐ 现在用**内存执行器**（无硬件）把协议跑通；
   板子到货后写一个 `ModbusActuator`（写 `0x0000` 高 8 位、读低 8 位）实现同一接口，
   **协议层与遥控状态机一行都不用改**。

接口只要求两件事：
    apply(index, on)  -> bool          执行动作，返回是否成功
    readback(index)   -> Optional[bool] **物理回读**；None = 读不到（失败/陈旧）
"""

from abc import ABC, abstractmethod
from typing import List, Optional


class Actuator(ABC):
    """执行器抽象。"""

    def __init__(self, channels: int = 8) -> None:
        self.channels = channels

    @abstractmethod
    def apply(self, index: int, on: bool) -> bool:
        """把第 index 路输出设为 on。返回 True 表示命令已被接受执行。"""

    @abstractmethod
    def readback(self, index: int) -> Optional[bool]:
        """读第 index 路的**物理回读**。None 表示读不到（不能当作 False）。"""


class MemoryActuator(Actuator):
    """内存执行器：没有硬件时用它跑通协议。

    ⚠️ 它的 readback 就是内存状态 —— **没有共模保护**，只用于协议验证。
       真实硬件上"命令路径"和"回读路径"必须互相独立（见执行准则 3.3）。
    """

    def __init__(self, channels: int = 8) -> None:
        super().__init__(channels)
        self._state: List[bool] = [False] * channels
        self.fail_apply = False        # 置 True 可模拟"执行失败"
        self.fail_readback = False     # 置 True 可模拟"回读失败/读不到"

    def apply(self, index: int, on: bool) -> bool:
        if self.fail_apply or not (0 <= index < self.channels):
            return False
        self._state[index] = bool(on)
        return True

    def readback(self, index: int) -> Optional[bool]:
        if self.fail_readback or not (0 <= index < self.channels):
            return None
        return self._state[index]

    def snapshot(self) -> List[bool]:
        """测试用：看当前各路状态。"""
        return list(self._state)

    def __repr__(self) -> str:
        return f"<MemoryActuator {''.join('1' if s else '0' for s in self._state)}>"
