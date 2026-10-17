"""启动 104 子站（骨架版）。

用法：
    # 在项目根目录下
    set PYTHONPATH=src
    python scripts\\run_slave.py --port 24040 --cyclic 2

然后用 QTester104 连 127.0.0.1:24040，点 [GI]。
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import Iec104Slave, Station, TypeID  # noqa: E402


MEAS_BASE = {1: 23.7, 2: 62.4}   # 遥测：IOA -> 基准值
SINGLE_BASE = {5: True, 6: False}  # 遥信：IOA -> 基准值


def build_station(ca: int) -> Station:
    """和演示服务端一致的点表：遥测 IOA 1/2、遥信 IOA 5/6。"""
    st = Station(common_address=ca, name="demo")

    for ioa, base in MEAS_BASE.items():
        st.add(ioa, int(TypeID.M_ME_NC_1), base, cyclic=True,
               name="Temp" if ioa == 1 else "Humidity")
    for ioa, val in SINGLE_BASE.items():
        st.add(ioa, int(TypeID.M_SP_NA_1), val,
               name="DI1" if ioa == 5 else "DI2")

    return st


def simulate(station: Station, period: float = 2.0) -> None:
    """让遥测值缓慢变化，方便主站那边看到刷新（骨架阶段的假数据）。"""
    t = 0.0
    while True:
        time.sleep(period)
        t += 1.0
        for ioa, base in MEAS_BASE.items():
            p = station.get(ioa)
            if p is not None:
                p.value = round(base + (t % 10) * 0.1, 2)


def main() -> int:
    ap = argparse.ArgumentParser(description="IEC 60870-5-104 子站（骨架）")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=24040)
    ap.add_argument("--ca", type=int, default=1, help="公共地址")
    ap.add_argument("--cyclic", type=float, default=0.0,
                    help="周期上送间隔（秒），0=关闭")
    ap.add_argument("--no-sim", action="store_true", help="不模拟数值变化")
    args = ap.parse_args()

    station = build_station(args.ca)
    if not args.no_sim:
        threading.Thread(target=simulate, args=(station,), daemon=True).start()

    slave = Iec104Slave(station, host=args.host, port=args.port,
                        cyclic_s=args.cyclic, verbose=True)
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
