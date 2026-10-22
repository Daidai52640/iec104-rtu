"""启动 104 子站。

用法：
    # 在项目根目录下（pwsh）
    $env:PYTHONPATH="src"
    python scripts\\run_slave.py --port 24040 --poll 2 --cyclic 2

点表从 JSON 文件读（默认 `config/points.json`），换一套点表只改配置、不改代码：
    python scripts\\run_slave.py --points config\\points.json

然后用 QTester104 连 127.0.0.1:24040，点 [GI]。

参数：
    --points  点表 JSON 路径（默认：<项目根>/config/points.json）
    --ca      公共地址；给了就覆盖配置文件里的 common_address
    --poll    采集周期（秒），0=不采集
    --cyclic  周期上送间隔（秒），0=关闭
    --no-sim  不模拟数值变化（静态值）
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import Iec104Slave, MemoryActuator, ModbusTcpSource, StubSource  # noqa: E402
from iec104.config import (  # noqa: E402
    ACTUATOR_CHANNELS, ConfigError, load_source_spec, load_station, readback_map,
)

# 默认点表按"项目根"解析，这样从任何目录启动都能找到同一份配置。
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_POINTS = os.path.join(ROOT, "config", "points.json")


def main() -> int:
    ap = argparse.ArgumentParser(description="IEC 60870-5-104 子站")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=24040)
    ap.add_argument("--points", default=DEFAULT_POINTS,
                    help="点表 JSON 路径（默认：<项目根>/config/points.json）")
    ap.add_argument("--ca", type=int, default=None,
                    help="公共地址；不给就用配置文件里的 common_address")
    ap.add_argument("--poll", type=float, default=2.0,
                    help="采集周期（秒），0=不采集")
    ap.add_argument("--cyclic", type=float, default=0.0,
                    help="周期上送间隔（秒），0=关闭")
    ap.add_argument("--no-sim", action="store_true",
                    help="不模拟数值变化（静态值）")
    args = ap.parse_args()

    # ⚠️ 点表非法就别启动：带着半张错点表跑，比不跑更难查。
    try:
        station = load_station(args.points)
        # 通道映射也在这里算：source_channel / 命令回读冲突会在这一步报错。
        rb_map = readback_map(station)
        # 点表里声明了 `source` 段 ⇒ 走真实数据源；没有 ⇒ 走桩数据源。
        source_spec = load_source_spec(args.points, station=station)
    except ConfigError as e:
        print(f"启动失败：{e}", file=sys.stderr)
        return 2

    if args.ca is not None:
        if not (0 <= args.ca <= 0xFFFF):
            print(f"启动失败：--ca {args.ca} 越界（允许 0~65535）", file=sys.stderr)
            return 2
        station.ca = args.ca

    print(f"点表：{args.points}（CA={station.ca}，{len(station)} 个点）")

    actuator = MemoryActuator(channels=ACTUATOR_CHANNELS)
    if source_spec is not None:
        # ⭐ 真实数据源：从 Modbus TCP 从站读（不抢串口，可与现场采集进程共存）
        source = ModbusTcpSource(
            host=source_spec["host"], port=source_spec["port"],
            unit=source_spec["unit"], mapping=source_spec["mapping"],
            timeout=source_spec["timeout"],
        )
        print(f"数据源：Modbus TCP {source_spec['host']}:{source_spec['port']} "
              f"unit={source_spec['unit']}（{len(source_spec['mapping'])} 个映射点）")
    else:
        # 回读接线也从点表推导，脚本里不再出现具体 IOA
        source = StubSource(actuator=actuator, readback_map=rb_map)
        print("数据源：桩（无硬件，模拟数值）")
    if args.no_sim:
        source.fail = False
        # 静态值：仍然采集，但值不变（StubSource 内部相位靠轮次推进，
        # 这里通过把采集周期设 0 来避免"值在变"）
        args.poll = 0.0

    slave = Iec104Slave(station, host=args.host, port=args.port,
                        cyclic_s=args.cyclic, verbose=True,
                        actuator=actuator,
                        source=source if args.poll > 0 else None,
                        poll_s=args.poll)
    slave.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n停止中…")
    finally:
        slave.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
