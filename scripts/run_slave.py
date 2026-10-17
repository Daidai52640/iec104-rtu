"""启动 104 子站。

用法：
    # 在项目根目录下
    set PYTHONPATH=src
    python scripts\\run_slave.py --port 24040 --poll 2 --cyclic 2

然后用 QTester104 连 127.0.0.1:24040，点 [GI]。

参数：
    --poll    采集周期（秒），0=不采集
    --cyclic  周期上送间隔（秒），0=关闭
    --no-sim  不模拟数值变化（静态值）
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import (  # noqa: E402
    Iec104Slave, MemoryActuator, Station, StubSource, TypeID,
)


MEAS_BASE = {1: 23.7, 2: 62.4}     # 遥测：IOA -> 基准值
SINGLE_BASE = {5: False, 6: True}  # 遥信：IOA -> 基准值（会由执行器回读覆盖）
CMD_IOA = 10                       # 遥控点：输出通道 0，回读映射到遥信 IOA 5
READBACK_MAP = {0: 5, 1: 6}        # 执行器通道 -> 回读到的遥信点 IOA


def build_station(ca: int) -> Station:
    """点表：遥测 IOA 1/2、遥信 IOA 5/6、遥控 IOA 10（绑定输出通道 0）。"""
    st = Station(common_address=ca, name="demo")

    for ioa, base in MEAS_BASE.items():
        st.add(ioa, int(TypeID.M_ME_NC_1), base, cyclic=True,
               name="Temp" if ioa == 1 else "Humidity")
    for ioa, val in SINGLE_BASE.items():
        st.add(ioa, int(TypeID.M_SP_NA_1), val,
               name="DI1" if ioa == 5 else "DI2")

    # ⭐ 遥控：命令点 → 执行器通道 0 → 回读到遥信 IOA 5
    st.add(CMD_IOA, int(TypeID.C_SC_NA_1), False, name="DO1")
    st.bind_command(CMD_IOA, output_index=0, readback_ioa=READBACK_MAP[0])

    return st


def main() -> int:
    ap = argparse.ArgumentParser(description="IEC 60870-5-104 子站")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=24040)
    ap.add_argument("--ca", type=int, default=1, help="公共地址")
    ap.add_argument("--poll", type=float, default=2.0,
                    help="采集周期（秒），0=不采集")
    ap.add_argument("--cyclic", type=float, default=0.0,
                    help="周期上送间隔（秒），0=关闭")
    ap.add_argument("--no-sim", action="store_true",
                    help="不模拟数值变化（静态值）")
    args = ap.parse_args()

    station = build_station(args.ca)
    actuator = MemoryActuator(channels=8)
    source = StubSource(actuator=actuator, readback_map=READBACK_MAP)
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
