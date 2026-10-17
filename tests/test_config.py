"""点表配置载入与校验的测试。

思路：一份正常配置 + 每一类非法配置各一条。**错误信息必须能定位到字段**
（`points[i]` / `ioa` / `type` / `actuator` / `readback_ioa` /
`source_channel` / `common_address`），否则运维拿到一句"配置错了"根本改不动。
另外有一条回归守门：默认配置首轮采集后不能出现 IV 点。

跑法（在项目根目录，pwsh）：
    $env:PYTHONPATH="src"
    python tests\\test_config.py
"""

import json
import os
import sys
import tempfile
from contextlib import contextmanager

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from iec104 import MemoryActuator, StubSource  # noqa: E402
from iec104.config import (  # noqa: E402
    ACTUATOR_CHANNELS, ConfigError, load_station, readback_map,
)
from iec104.consts import QDS, TypeID  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(ROOT, "config", "points.json")


# ---------------------------------------------------------------- 工具
@contextmanager
def config_file(data=None, raw=None, raw_bytes=None, bom=False):
    """写一个临时 JSON 配置文件，用完删掉。"""
    fd, path = tempfile.mkstemp(prefix="iec104-points-", suffix=".json")
    os.close(fd)
    try:
        if raw_bytes is not None:
            with open(path, "wb") as f:
                f.write(raw_bytes)
        else:
            if raw is None:
                raw = json.dumps(data, ensure_ascii=False, indent=2)
            with open(path, "w", encoding="utf-8-sig" if bom else "utf-8") as f:
                f.write(raw)
        yield path
    finally:
        try:
            os.remove(path)
        except OSError:
            pass


def expect_error(data, *tokens):
    """断言 load_station 抛 ConfigError，且错误信息包含所有 tokens。"""
    with config_file(data) as path:
        try:
            load_station(path)
        except ConfigError as e:
            msg = str(e)
        else:
            raise AssertionError("非法配置应当抛 ConfigError，但实际载入成功了")
    for token in tokens:
        assert token in msg, f"错误信息应包含 {token!r}，实际：{msg}"
    return msg


def valid_config():
    """一份最小合法配置：遥测 / 遥信 / 遥控各一个。"""
    return {
        "common_address": 1,
        "name": "unit",
        "points": [
            {"ioa": 1, "type": "M_ME_NC_1", "name": "Temp",
             "value": 23.7, "cyclic": True},
            {"ioa": 5, "type": "M_SP_NA_1", "name": "DI1", "value": False},
            {"ioa": 10, "type": "C_SC_NA_1", "name": "DO1",
             "actuator": 0, "readback_ioa": 5},
        ],
    }


# ---------------------------------------------------------------- 正常载入
def test_example_config_equivalent_to_hardcoded_table():
    """config/points.json 与原来写死在 run_slave 里的点表等价。"""
    st = load_station(EXAMPLE)

    assert st.ca == 1
    assert st.name == "demo"
    assert len(st) == 5
    assert [p.ioa for p in st.all()] == [1, 2, 5, 6, 10]

    temp = st.get(1)
    assert temp.type_id == int(TypeID.M_ME_NC_1)
    assert temp.name == "Temp"
    assert temp.value == 23.7
    assert temp.cyclic is True

    humidity = st.get(2)
    assert humidity.type_id == int(TypeID.M_ME_NC_1)
    assert humidity.name == "Humidity"
    assert humidity.value == 62.4
    assert humidity.cyclic is True

    assert st.get(5).type_id == int(TypeID.M_SP_NA_1)
    assert st.get(5).value is False
    assert st.get(5).source_channel is None
    assert st.get(6).type_id == int(TypeID.M_SP_NA_1)
    assert st.get(6).value is True
    assert st.get(6).source_channel == 1

    cmd = st.get(10)
    assert cmd.type_id == int(TypeID.C_SC_NA_1)
    assert cmd.name == "DO1"
    binding = st.binding(10)
    assert binding is not None
    assert binding.output_index == 0
    assert binding.readback_ioa == 5

    assert [p.ioa for p in st.cyclic_points()] == [1, 2]
    # 两个来源的并集：命令回读(0->5) + IOA6 声明的 source_channel(1->6)
    assert readback_map(st) == {0: 5, 1: 6}


def test_minimal_config_uses_defaults():
    """可选字段不写：取类型默认值，不因为缺省炸掉。"""
    cfg = {
        "common_address": 0xFFFF,
        "points": [
            {"ioa": 1, "type": "M_ME_NC_1"},
            {"ioa": 5, "type": "M_SP_NA_1"},
            {"ioa": 6, "type": "M_SP_TB_1", "cyclic": True, "source_channel": 1},
        ],
    }
    with config_file(cfg) as path:
        st = load_station(path)

    assert st.ca == 0xFFFF
    assert st.name == ""
    assert st.get(1).value == 0.0
    assert st.get(1).cyclic is False
    assert st.get(1).name == ""
    assert st.get(5).value is False
    assert st.get(6).type_id == int(TypeID.M_SP_TB_1)
    assert st.get(6).cyclic is True
    assert st.get(6).source_channel == 1
    assert readback_map(st) == {1: 6}


def test_utf8_bom_and_chinese_text_accepted():
    """Windows 记事本存出的 UTF-8 BOM 配置要能读，中文名不能乱码。"""
    cfg = {
        "common_address": 1,
        "name": "演示站",
        "points": [{"ioa": 1, "type": "M_ME_NC_1", "name": "温度"}],
    }
    with config_file(cfg, bom=True) as path:
        st = load_station(path)

    assert st.name == "演示站"
    assert st.get(1).name == "温度"


def test_default_config_first_poll_has_no_invalid_point():
    """回归守门：默认点表首轮采集后不能出现 IV 点。

    背景：曾经只从命令绑定推导回读接线，IOA 6 没被采集，开箱就是 IV
    并会突发上送无效值。`source_channel` 就是为了补上这条接线。
    """
    st = load_station(EXAMPLE)
    assert readback_map(st) == {0: 5, 1: 6}

    actuator = MemoryActuator(channels=ACTUATOR_CHANNELS)
    source = StubSource(actuator=actuator, readback_map=readback_map(st))

    values = source.poll()
    assert sorted(values) == [1, 2, 5, 6], sorted(values)

    st.apply(values)
    bad = [(p.ioa, hex(p.quality)) for p in st.all() if p.quality & int(QDS.IV)]
    assert not bad, f"首轮采集不应有无效点，实际：{bad}"


# ---------------------------------------------------------------- 非法配置
def test_duplicate_ioa_rejected():
    cfg = valid_config()
    cfg["points"][1]["ioa"] = 1          # 与 points[0] 重复
    expect_error(cfg, "points[1]", "IOA", "重复")


def test_unknown_type_rejected():
    cfg = valid_config()
    cfg["points"][0]["type"] = "M_ME_NA_1"   # asdu.py 尚未实现编码，按未知处理
    expect_error(cfg, "points[0]", "M_ME_NA_1", "未知", "M_ME_NC_1")


def test_readback_ioa_must_exist():
    cfg = valid_config()
    cfg["points"][2]["readback_ioa"] = 99
    expect_error(cfg, "points[2]", "readback_ioa", "99", "不存在")


def test_source_channel_only_on_single_point():
    """遥测点声明 source_channel → 报错（它不是一个开关量输入通道）。"""
    cfg = valid_config()
    cfg["points"][0]["source_channel"] = 0
    expect_error(cfg, "points[0]", "source_channel", "M_ME_NC_1")


def test_source_channel_out_of_range_rejected():
    for bad in (-1, ACTUATOR_CHANNELS, 99, "1", True):
        cfg = valid_config()
        cfg["points"][1]["source_channel"] = bad
        expect_error(cfg, "points[1]", "source_channel")


def test_duplicate_source_channel_rejected():
    """两个点声明同一通道 → 报错，且错误里带两个 IOA。"""
    cfg = valid_config()
    cfg["points"][1]["source_channel"] = 0
    cfg["points"].insert(1, {"ioa": 6, "type": "M_SP_NA_1",
                             "source_channel": 0})
    expect_error(cfg, "source_channel", "冲突", "IOA=5", "IOA=6")


def test_source_channel_conflicts_with_command_readback():
    """点声明的通道 vs 命令回读的通道指向不同点 → 报错。"""
    cfg = valid_config()
    cfg["points"][1]["source_channel"] = 0      # 通道 0 采集 IOA 5
    cfg["points"][2]["readback_ioa"] = 6        # 通道 0 回读却指向 IOA 6
    cfg["points"].append({"ioa": 6, "type": "M_SP_NA_1"})
    expect_error(cfg, "冲突", "IOA=5", "IOA=6", "readback_ioa")


def test_actuator_out_of_range_rejected():
    for bad in (-1, ACTUATOR_CHANNELS, 99):
        cfg = valid_config()
        cfg["points"][2]["actuator"] = bad
        expect_error(cfg, "points[2]", "actuator", str(bad), "越界")


def test_common_address_out_of_range_rejected():
    for bad in (-1, 0x10000):
        cfg = valid_config()
        cfg["common_address"] = bad
        expect_error(cfg, "common_address", str(bad), "越界")


def test_missing_top_level_field_rejected():
    for field in ("common_address", "points"):
        cfg = valid_config()
        del cfg[field]
        expect_error(cfg, field, "缺少顶层字段")


def test_json_syntax_error_rejected():
    raw = '{"common_address": 1, "points": [ }'
    with config_file(raw=raw) as path:
        try:
            load_station(path)
        except ConfigError as e:
            msg = str(e)
        else:
            raise AssertionError("JSON 语法错误应当抛 ConfigError")
    assert "JSON 语法错误" in msg
    assert "行" in msg, f"语法错误应给出行列号，实际：{msg}"


def test_missing_file_rejected():
    assert issubclass(ConfigError, ValueError)
    path = os.path.join(tempfile.gettempdir(), "iec104-points-not-here-9527.json")
    try:
        load_station(path)
    except ConfigError as e:
        assert "配置文件不存在" in str(e)
    else:
        raise AssertionError("配置文件不存在应当抛 ConfigError")


def test_non_utf8_config_rejected():
    """GBK 配置要报"不是 UTF-8"，不能甩一个 UnicodeDecodeError 出来。"""
    raw = '{"common_address": 1, "name": "演示站", "points": []}'.encode("gbk")
    with config_file(raw_bytes=raw) as path:
        try:
            load_station(path)
        except ConfigError as e:
            msg = str(e)
        else:
            raise AssertionError("非 UTF-8 配置应当抛 ConfigError")
    assert "UTF-8" in msg


def test_top_level_must_be_json_object():
    expect_error([1, 2, 3], "顶层", "list")


def test_unknown_top_level_field_rejected():
    cfg = valid_config()
    cfg["config_version"] = 1
    expect_error(cfg, "config_version", "未知字段")


def test_unknown_point_field_rejected():
    cfg = valid_config()
    cfg["points"][0]["redback_ioa"] = 5      # 拼错字段名，不能静默忽略
    expect_error(cfg, "points[0]", "redback_ioa", "未知字段")


def test_missing_point_field_rejected():
    for field in ("ioa", "type"):
        cfg = valid_config()
        del cfg["points"][0][field]
        expect_error(cfg, "points[0]", field, "缺少必填字段")


def test_command_point_requires_actuator():
    cfg = valid_config()
    del cfg["points"][2]["actuator"]
    expect_error(cfg, "points[2]", "actuator", "缺少必填字段")


def test_command_fields_only_on_command_point():
    for field in ("actuator", "readback_ioa"):
        cfg = valid_config()
        cfg["points"][0][field] = 0          # 遥测点上绑执行器 → 明显配错了
        expect_error(cfg, "points[0]", field, "C_SC_NA_1")


def test_cyclic_command_point_rejected():
    cfg = valid_config()
    cfg["points"][2]["cyclic"] = True
    expect_error(cfg, "points[2]", "cyclic", "C_SC_NA_1")


def test_readback_must_point_to_single_point():
    cfg = valid_config()
    cfg["points"][2]["readback_ioa"] = 1      # 指向遥测点
    expect_error(cfg, "points[2]", "readback_ioa", "M_SP_NA_1", "M_ME_NC_1")


def test_invalid_ioa_rejected():
    for bad in (0, -1, 0x1000000, "1", True):
        cfg = valid_config()
        cfg["points"][0]["ioa"] = bad
        expect_error(cfg, "points[0]", "ioa")


def test_invalid_value_type_rejected():
    cfg = valid_config()
    cfg["points"][0]["value"] = "23.7"       # 遥测值写成字符串
    expect_error(cfg, "points[0]", "value", "必须是数字")

    cfg = valid_config()
    cfg["points"][1]["value"] = 1            # 遥信值写成 0/1
    expect_error(cfg, "points[1]", "value", "true/false")


# ---------------------------------------------------------------- 跑
def _run_all():
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed.append((name, e))
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print("-" * 60)
    print(f"{len(fns) - len(failed)}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
