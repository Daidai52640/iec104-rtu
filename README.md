# iec104-rtu

IEC 60870-5-104 远动终端（子站）实现：四遥采集、遥控「选择-执行」闭环、第三方主站互操作。

**纯标准库，零第三方依赖。**

---

## 这是什么

在真实的电力/工业远动系统里，这个仓库对应这样一格：

```
调度主站 ── IEC 60870-5-104 (TCP) ──► 【远动终端 / 通信管理机】 ── Modbus RTU ──► 现场设备
                                          ↑
                                     本仓库
```

调度主站不会去连一个 Modbus 板子。中间那个"把设备语言翻译成调度语言、并保证遥控可靠"的盒子，就是本项目。

**本项目只做协议与交互语义，不含工艺逻辑**（联锁、时序、策略在 PLC 里）。

## 当前能力

| 层 | 能力 | 状态 |
|---|---|---|
| 链路层 | APCI 三种帧（I/S/U）、流式拆帧、15 位序号回绕、**N(S) 连续性校验**、窗口 k / 确认阈值 w、**t1 / t2 / t3** | ✅ |
| 应用层 | ASDU 编解码：`M_SP_NA_1` / `M_SP_TB_1` / `M_ME_NC_1` / `C_SC_NA_1` / `C_IC_NA_1` / `C_CS_NA_1`；SQ=0 与 SQ=1；短浮点小端；CP56Time2a | ✅ |
| 会话 | STARTDT / STOPDT / TESTFR、总召唤四步（COT 6→7→20→10）、周期上送、突发上送 | ✅ |
| 遥控 | 「选择-执行」两段式、选择超时、**选择值与执行值一致校验**、执行后独立回读、失败不上报成功 | ✅ |
| 动作约束 | 未绑定 / 设备离线(IV) / 互斥组 / 最小间隔 | ✅ |
| 品质位 | 读不到 → `IV`，不把旧值当好值；品质变化也触发突发上送 | ✅ |
| 对时 | `C_CS_NA_1` 记录偏移、回显时标、偏移超阈值告警 | ✅ |
| 点表 | JSON 配置驱动，含类型/周期/遥控绑定/数据源通道/互斥组；21 类非法配置校验 | ✅ |
| 南向 Modbus | 设备侧实现（`ModbusActuator` / `ModbusSource`） | ⬜ 待硬件 |
| 部署 | 树莓派 + systemd | ⬜ 未做 |

> ⚠️ **控制器（`actuator.py`）与数据源（`source.py`）目前是内存 / 桩实现**：
> 协议与交互语义已完整验证，但**真实继电器动作、触点回读、设备掉线**尚未在实机上验证。

## 快速开始

```bash
# 需要 Python 3.8+，无第三方依赖
export PYTHONPATH=src
python scripts/run_slave.py --port 24040 --poll 2
```

然后在任意 IEC 60870-5-104 主站上连接 `127.0.0.1:24040`（公共地址 1），发起总召唤。

点表换了只改配置，不改代码：

```bash
python scripts/run_slave.py --points config/points.json
```

## 点表配置

```json
{
  "common_address": 1,
  "points": [
    {"ioa": 1,  "type": "M_ME_NC_1", "name": "Temp", "value": 23.7, "cyclic": true},
    {"ioa": 5,  "type": "M_SP_NA_1", "name": "DI1",  "value": false, "source_channel": 0},
    {"ioa": 10, "type": "C_SC_NA_1", "name": "DO1",  "actuator": 0, "readback_ioa": 5,
     "mutex_group": "relay", "min_interval": 1.0}
  ]
}
```

⭐ **一个点的类型由 `type` 决定，并在所有路径一致**：声明 `M_SP_NA_1` 就一路发 TypeID 1（不带时标）；声明 `M_SP_TB_1` 就一路发 TypeID 30（带 CP56Time2a）。主站按点表配类型，中途换类型会被丢弃。

## 测试

```bash
export PYTHONPATH=src
for f in tests/test_*.py; do python "$f"; done
```

| 文件 | 项数 | 覆盖 |
|---|---|---|
| `test_iec104.py` | 25 | 帧编解码，用实测抓包字节逐字节对拍 |
| `test_link_timers.py` | 9 | t1 / t2 / t3 / 窗口 k / N(S) 校验（故障注入） |
| `test_sbo.py` | 8 | 遥控选择-执行与全部拒绝路径 |
| `test_constraints.py` | 9 | 动作约束四条 |
| `test_quality.py` | 8 | 掉线 → IV、恢复、品质变化突发 |
| `test_soe.py` | 9 | 对时、非法时标、NA/TB 全路径一致性 |
| `test_config.py` | 27 | 点表载入与 21 类非法配置 |
| `test_scale.py` | 5 | 单帧容量上限、大规模总召唤、窗口阻塞 |

**共 100 项**，验收原始输出见 [`docs/evidence/`](docs/evidence/)。

### 规模实测（本地回环）

| 点数 | 总召唤帧数 | 耗时 |
|---|---|---|
| 500 | 13 | < 1 ms |
| 5,000 | 107 | 15 ms |
| 20,000 | 419 | 47 ms |

单帧容量受两个限制同时约束：VSQ 只有 7 位（≤127 个对象），链路层长度字节只有 1 字节（ASDU ≤ 249 字节）。按类型算，`M_ME_NC_1` 每帧 48 个、`M_SP_TB_1` 每帧 30 个、`M_SP_NA_1` 每帧 127 个。

## 目录结构

```
src/iec104/
├── consts.py     协议常量（TypeID / COT / 品质位 / 会话参数）
├── quality.py    品质位工具
├── link.py       链路层：APCI I/S/U 帧、拆帧、序号
├── asdu.py       应用层：ASDU 编解码、CP56Time2a
├── station.py    点表、遥控绑定、采集落地、单帧容量计算
├── actuator.py   执行器接缝（协议层 → 现场，写）
├── source.py     数据源接缝（现场 → 协议层，读）
├── config.py     点表 JSON 载入与校验
├── clock.py      对时偏移
└── server.py     会话状态机、定时器、采集循环

scripts/run_slave.py   启动入口
config/points.json     点表示例
docs/                  定位与边界、拓扑、执行准则、待办
diagrams/              拓扑图与架构图
```

⭐ `actuator.py` 与 `source.py` 是**协议层与硬件层之间唯一的接缝**。接真机只需实现这两个接口，协议层一行不动。

## 互操作

与两套**独立实现**的第三方主站验证过（总召唤 + 遥控 SBO 全链）：

- 一套基于 lib60870
- 一套独立的 C++/Qt 实现

验收记录见 [`docs/evidence/`](docs/evidence/)。

## 明确不做

| 不做 | 原因 |
|---|---|
| 冗余（双机热备 / 双通道） | 超出范围，且无法验证 |
| 规约一致性测试认证 | 需送检测机构 |
| 电力二次系统安全防护（纵向加密认证） | 需专用装置 |
| SIL / 功能安全相关 | 本实现不是安全相关设备 |
| 毫秒级 SOE 精度 | 无硬件对时源 |
| 平台层 / Web UI | 不提供人机界面，主站即界面 |
| 工艺逻辑（联锁、时序、策略） | 远动终端不应内置工艺逻辑，否则换现场就要改网关 |

## 许可

**尚未选定**（仓库中暂无 LICENSE 文件）。项目成形后再定，避免过早承诺。

## 状态

开发中。协议与交互语义已完成并通过测试；**硬件链路与部署形态待设备到位后补**。详见 [`docs/03-待办与优先级.md`](docs/03-待办与优先级.md)。
