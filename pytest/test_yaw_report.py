#!/usr/bin/env python
# coding=utf-8
"""yaw 上报用例: loc_base 的 yaw 与 mavros odom 的 gpsYaw 分别验证。

数据源约定 ( mavlink_telemetry.cpp ):
  - /loc_base                -> mapLocation / roll / pitch / yaw
  - /mavros/local_position/odom -> 仅 gpsYaw (yaw_enu_)
ENU->NED 转换当前处于注释状态, yaw 值原样直通上报。
"""
from conftest import OdometryPublisher  # noqa: F401  (fixtures 在 conftest 中)

TOL = 0.05


def approx(a, b):
    return abs(a - b) <= TOL


def angle_close(a, b):
    """按角度回绕比较 (yaw 解算后可能差 2pi)。"""
    import math
    diff = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) <= TOL


def test_loc_base_yaw_report(loc_base_pub, mqtt_state_collector):
    """只发 /loc_base: yaw 上报为 loc_base 值, 且不出现 gpsYaw 字段。"""
    loc_base_pub.set_yaw(1.0)
    state = mqtt_state_collector.wait_state(
        lambda s: "yaw" in s and approx(s["yaw"], 1.0),
        "yaw 上报为 loc_base 的 1.0 rad",
    )
    assert "gpsYaw" not in state, f"仅 loc_base 时不应有 gpsYaw: {state}"


def test_odom_gps_yaw_report(odom_pub, mqtt_state_collector):
    """只发 /mavros/local_position/odom: gpsYaw 上报为 odom 值, yaw 保持 0。"""
    odom_pub.set_yaw(2.0)
    state = mqtt_state_collector.wait_state(
        lambda s: "gpsYaw" in s and approx(s["gpsYaw"], 2.0),
        "gpsYaw 上报为 odom 的 2.0 rad",
    )
    assert approx(state.get("yaw", 0.0), 0.0), (
        f"无 loc_base 时 yaw 应缺省或为 0: {state}")


def test_yaw_sources_independent(loc_base_pub, odom_pub, mqtt_state_collector):
    """双源同时在线: yaw 跟随 loc_base, gpsYaw 跟随 odom, 互不影响。"""
    loc_base_pub.set_yaw(0.5)
    odom_pub.set_yaw(2.5)
    state = mqtt_state_collector.wait_state(
        lambda s: "yaw" in s and "gpsYaw" in s
        and angle_close(s["yaw"], 0.5) and angle_close(s["gpsYaw"], 2.5),
        "yaw=0.5 (loc_base) 与 gpsYaw=2.5 (odom) 同时正确上报",
    )
    # 反向再各改一次, 确认两个字段始终各随其源 (3.5 rad 验证 2pi 回绕)
    loc_base_pub.set_yaw(1.5)
    odom_pub.set_yaw(3.5)
    state = mqtt_state_collector.wait_state(
        lambda s: angle_close(s.get("yaw", 99), 1.5)
        and angle_close(s.get("gpsYaw", 99), 3.5),
        "修改后 yaw=1.5 / gpsYaw=3.5 (回绕后 -2.783) 依旧各自跟随",
    )
