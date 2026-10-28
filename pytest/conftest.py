#!/usr/bin/env python
# coding=utf-8
"""pytest 公共 fixture: mosquitto broker + api_routes 节点 + ROS 话题伪造发布。

不启动 mavros / 飞控, 只在 ROS topic 层伪造 /loc_base 与
/mavros/local_position/odom 两个 odometry 话题; 通过 mosquitto broker
在 MQTT 链路上观察节点上报的 yaw / gpsYaw (0.5Hz 全量心跳 + 差量上报)。

节点 paho C++ 用 MQTT v5 连接, amqtt 不支持 v5, 故用本机 mosquitto。

依赖: rospy (需 source /opt/ros/noetic/setup.bash + devel/setup.bash)、
paho-mqtt、/usr/sbin/mosquitto。
"""
import json
import subprocess
import threading
import time

import pytest
import rospy
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt

MQTT_PORT = 41884
WEB_PORT = 8901
LOC_BASE_TOPIC = "/loc_base"
ODOM_TOPIC = "/mavros/local_position/odom"


class MosquittoBroker:
    """subprocess 拉起的 mosquitto broker (唯一支持 MQTT v5 的本地方案)。"""

    def __init__(self, port):
        self.port = port
        self.proc = None

    def start(self):
        self.proc = subprocess.Popen(
            ["/usr/sbin/mosquitto", "-p", str(self.port), "-v"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self._wait_ready()
        print(f"\n[mosquitto] broker 已启动, 监听 127.0.0.1:{self.port} "
              f"(pid={self.proc.pid})")

    def _wait_ready(self, timeout=15):
        """用 paho 同步客户端确认 broker 可连。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f"mosquitto 启动即退出 (exit={self.proc.returncode}), "
                    "端口可能被占用"
                )
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            try:
                client.connect("127.0.0.1", self.port, keepalive=5)
                client.loop_start()
                client.disconnect()
                client.loop_stop()
                return
            except Exception:
                client.loop_stop()
                time.sleep(0.5)
        raise RuntimeError(f"mosquitto 在 {timeout}s 内未就绪 (port={self.port})")

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class MqttStateCollector:
    """订阅 device/<code>/state, 线程安全地累积收到的 JSON 状态。"""

    def __init__(self, port, device_code):
        self.states = []
        self._lock = threading.Lock()
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.connect("127.0.0.1", port, keepalive=30)
        self._client.subscribe(f"device/{device_code}/state", qos=0)
        self._client.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"\n[collector] 已连接 broker (code={reason_code}), "
              f"订阅 device/*/state")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"\n[collector] 与 broker 断开 (code={reason_code})")

    def _on_message(self, client, userdata, msg):
        print(f"\n[collector] 收到 state: {msg.payload[:200]}")
        try:
            data = json.loads(msg.payload)
        except Exception:
            return
        with self._lock:
            self.states.append(data)

    def latest(self):
        with self._lock:
            return self.states[-1] if self.states else None

    def wait_state(self, pred, desc, timeout=30.0):
        """轮询最新 state 直到条件满足 (差量抑制 0.01 + 0.5Hz 心跳, 需要等)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self.latest()
            if state is not None and pred(state):
                return state
            time.sleep(0.5)
        raise AssertionError(
            f"等待超时: {desc}, 最后一次状态: "
            f"{json.dumps(self.latest(), ensure_ascii=False)}"
        )

    def stop(self):
        self._client.loop_stop()
        self._client.disconnect()


class OdometryPublisher:
    """20Hz 定频发布 odometry 的话题伪造器, 姿态 yaw 可随时修改。"""

    def __init__(self, topic):
        import math

        self._math = math
        self.topic = topic
        self.yaw = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._pub = rospy.Publisher(topic, Odometry, queue_size=10)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def set_yaw(self, yaw):
        with self._lock:
            self.yaw = yaw

    def _spin(self):
        rate = rospy.Rate(20)
        while not self._stop.is_set() and not rospy.is_shutdown():
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = 1.0
            msg.pose.pose.position.y = 2.0
            msg.pose.pose.position.z = 0.0
            with self._lock:
                yaw = self.yaw
            msg.pose.pose.orientation.z = self._math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = self._math.cos(yaw / 2.0)
            self._pub.publish(msg)
            rate.sleep()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)


def _node_running(name="api_routes"):
    out = subprocess.run(["rosnode", "list"], capture_output=True, text=True)
    return any(name in line for line in out.stdout.splitlines())


@pytest.fixture(scope="session", autouse=True)
def mqtt_broker():
    """整测期间的 mosquitto broker (支持节点 paho C++ 的 MQTT v5)。"""
    broker = MosquittoBroker(MQTT_PORT)
    broker.start()
    yield broker
    broker.stop()


@pytest.fixture(scope="session", autouse=True)
def ros_node(mqtt_broker):
    """保证 roscore + api_routes 节点存活, 并读出 device_code。"""
    core = None
    if subprocess.run(["pgrep", "-f", "rosmaster"], capture_output=True).returncode != 0:
        core = subprocess.Popen(
            ["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        deadline = time.time() + 30
        while time.time() < deadline:
            if subprocess.run(
                ["pgrep", "-f", "rosmaster"], capture_output=True
            ).returncode == 0:
                break
            time.sleep(1)

    # 总是重启 api_routes: 已在跑的节点连的是自己配置的真实 broker,
    # 必须带测试参数 (指向测试 mosquitto) 拉起全新节点
    from pathlib import Path

    subprocess.run(["pkill", "-f", "api_routes_node"], check=False)
    time.sleep(2)
    node_log = Path(__file__).parent / "logs" / "api_routes.log"
    node_log.parent.mkdir(parents=True, exist_ok=True)
    node_log_f = open(node_log, "ab")
    node = subprocess.Popen(
        [
            "rosrun", "api_routes", "api_routes_node",
            f"_web_port:={WEB_PORT}",
            "_mqtt_host:=127.0.0.1", f"_mqtt_port:={MQTT_PORT}",
        ],
        stdout=node_log_f, stderr=subprocess.STDOUT,
    )
    time.sleep(1)
    if node.poll() is not None:
        raise RuntimeError(
            f"api_routes 节点启动即退出 (exit={node.returncode}), "
            f"日志尾部:\n{node_log.read_text(errors='replace')[-2000:]}"
        )

    rospy.init_node("yaw_report_test", anonymous=True, disable_signals=True)

    deadline = time.time() + 60
    while time.time() < deadline:
        if _node_running() and rospy.has_param("/device_code"):
            break
        time.sleep(1)
    else:
        raise RuntimeError("api_routes 节点启动或 device_code 加载超时")
    time.sleep(2)  # 等 MQTT 连接与 WS 服务就绪
    yield rospy.get_param("/device_code")
    if node is not None:
        node.terminate()


@pytest.fixture
def mqtt_state_collector(ros_node):
    """订阅 device/<code>/state 的状态收集器 (ros_node 的值即 device_code)。"""
    collector = MqttStateCollector(MQTT_PORT, ros_node)
    yield collector
    collector.stop()


@pytest.fixture
def loc_base_pub(ros_node):
    pub = OdometryPublisher(LOC_BASE_TOPIC)
    yield pub
    pub.stop()


@pytest.fixture
def odom_pub(ros_node):
    pub = OdometryPublisher(ODOM_TOPIC)
    yield pub
    pub.stop()
