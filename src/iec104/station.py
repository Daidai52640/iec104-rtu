"""对象模型：点表 + 快照。

一个 Station 对应一个公共地址（CA）。点按 IOA 索引。
本文件不含任何协议字节级逻辑，只描述"我有哪些点、值是什么"。
"""

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from .consts import TypeID


@dataclass
class Point:
    """一个信息对象。"""

    ioa: int
    type_id: int
    value: Any = None
    quality: int = 0
    cyclic: bool = False          # True = 参与周期上送（COT=1）
    name: str = ""

    def __repr__(self) -> str:
        try:
            t = TypeID(self.type_id).name
        except ValueError:
            t = str(self.type_id)
        return f"<Point {self.name or ''} IOA={self.ioa} {t}={self.value!r} q={self.quality:#x}>"


class Station:
    """一个子站（公共地址）。点表是"代码里的配置"，暂不支持热加载。"""

    def __init__(self, common_address: int, name: str = "") -> None:
        if not (0 <= common_address <= 0xFFFF):
            raise ValueError("common_address out of range")
        self.ca = common_address
        self.name = name
        self._points: Dict[int, Point] = {}

    # ------------------------------------------------------------ 增删查
    def add(self, ioa: int, type_id: int, value: Any = None,
            quality: int = 0, cyclic: bool = False, name: str = "") -> Point:
        """加一个点。同一 IOA 重复添加会报错（避免静默覆盖）。"""
        if ioa in self._points:
            raise ValueError(f"IOA {ioa} already exists")
        p = Point(ioa=ioa, type_id=type_id, value=value,
                  quality=quality, cyclic=cyclic, name=name)
        self._points[ioa] = p
        return p

    def get(self, ioa: int) -> Optional[Point]:
        return self._points.get(ioa)

    def all(self) -> List[Point]:
        return [self._points[k] for k in sorted(self._points)]

    def by_type(self, type_id: int) -> List[Point]:
        return [p for p in self.all() if p.type_id == type_id]

    def __len__(self) -> int:
        return len(self._points)

    # ------------------------------------------------------------ 分组（供 SQ=1 打包）
    @staticmethod
    def group_consecutive(points: Iterable[Point]) -> List[List[Point]]:
        """把 IOA 连续的同类点分成一组，用于 SQ=1 打包。

        例：[1,2,5,6] -> [[1,2],[5,6]]
        ⚠️ SQ=1 时帧里只写第一个 IOA，所以只有连续的点才能同组。
        """
        ordered = sorted(points, key=lambda p: p.ioa)
        groups: List[List[Point]] = []
        for p in ordered:
            if groups and p.ioa == groups[-1][-1].ioa + 1:
                groups[-1].append(p)
            else:
                groups.append([p])
        return groups

    # ------------------------------------------------------------ 总召唤内容
    def interrogation_groups(self) -> List[List[Point]]:
        """总召唤要上送的所有点，按类型 + IOA 连续性分组。"""
        groups: List[List[Point]] = []
        for type_id in (int(TypeID.M_SP_NA_1), int(TypeID.M_SP_TB_1),
                        int(TypeID.M_ME_NA_1), int(TypeID.M_ME_NC_1)):
            pts = self.by_type(type_id)
            if pts:
                groups.extend(self.group_consecutive(pts))
        return groups

    def cyclic_points(self) -> List[Point]:
        """参与周期上送的点。"""
        return [p for p in self.all() if p.cyclic]
