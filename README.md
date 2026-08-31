# api_routes

`api_routes` 是一个集成了 MAVLink/MAVROS 遥测数据处理、MQTT & WebSocket 状态实时推送、设备动态注册、HTTP API 接口、异常报告上报以及 ROS Parameter 动态路由扩展功能的 ROS 核心通信功能包。

## 核心功能

1. **MAVLink 遥测处理与坐标同步**
   - 自动订阅 MAVROS 状态与姿态/位置话题（`/mavros/state`、`/mavros/local_position/odom`、`/mavros/global_position/global` 等）。
   - 使用 `DatumSynchronizer` 算法对高频 ENU Odometry 与低频 GPS 进行时间戳插值对齐与稳态校验。

2. **MQTT & WebSocket 状态推送**
   - 内存级实时整理遥测状态与映射字段（`gpsLocation`, `mapLocation`, `xVel`, `yVel`, `relAlt` 等）。
   - 基于 `StateDiffTracker` 实时比对差量，自动附加 `deviceCode` 与 `timestamp` 后直接分发至 **MQTT**（主题 `/device/$/state`）和 **WebSocket**（路径 `/ws`）。
   - 支持网络断线重连全量重发与 0.5Hz 心跳全量推送。

3. **HTTP 接口服务**
   - `POST/GET /get_gps`：直接通过 HTTP 接口查询计算后的准确实时 GPS 坐标。

4. **异常报告上报 (`/abnormal/report`)**
   - 监听 ROS 话题 `/abnormal/report`（`std_msgs/String` JSON）。
   - 自动注入 `deviceCode`、`mapCoordinate` 与 `gpsLocation`。
   - 解析 `files` 字段：支持网络 URL（cURL 异步下载）与本地路径（读取内容后**自动删除本地源文件**）。
   - 组装 `multipart/form-data` HTTP POST 请求发送至云端上报接口。

5. **设备注册与动态路由**
   - 支持通过 `config/device_code.yaml` 加载或通过 MQTT `$exclusive/register` 动态注册设备代码。
   - 支持通过 ROS Parameter 动态配置自定义 MQTT / WebSocket / HTTP Bridge / 图片上传路由。

---

## 动态路由配置 (rosparam 注册说明)

除了内置的遥测与异常上报以外，`api_routes` 可以解析 `/api_routes/api/` 命名空间下的 `rosparam` 字典参数，动态开启 ROS 话题与 MQTT/WebSocket/HTTP 之间的转发桥梁：

### 参数结构说明

在 YAML 或 launch 文件中，每个自定义路由任务配置在 `/api_routes/api/` 目录下：

```yaml
/api_routes:
  api:
    # 示例 1: ROS Topic -> MQTT 差量状态发布
    mqtt_state_task:
      protocol: "mqtt"         # 支持: mqtt | websocket (或 ws) | http | upload_image
      topic_type: "pub_state"  # 支持: pub (普通发布) | pub_state (差量状态发布) | sub (订阅转发至ROS)
      ros_topic: "/my/status"  # 本地 ROS 话题名称 (std_msgs/String JSON)
      remote_uri: "/device/$/my_state" # 远程 MQTT Topic ($ 将自动替换为 deviceCode)
      qos: 0                   # 可选, MQTT QoS (默认 0)
      retain: false            # 可选, MQTT Retain (默认 false)

    # 示例 2: MQTT -> ROS Topic 订阅转发
    mqtt_sub_task:
      protocol: "mqtt"
      topic_type: "sub"
      ros_topic: "/my/command"
      remote_uri: "/device/$/cmd"
      qos: 1

    # 示例 3: ROS Service -> HTTP Bridge 转换
    http_bridge_task:
      protocol: "http"
      ros_topic: "/my_service"  # 对应的 ROS 服务名称 (api_routes/StringSrv)
      remote_uri: "/api/my_service" # 对应的 HTTP 路由路径

    # 示例 4: ROS Image Topic 自动上传至 Web/Cloud
    upload_image_task:
      protocol: "upload_image"
      ros_topic: "/camera/image_raw/compressed" # 图像话题
      topic_type: "compressed" # compressed 或 raw
      remote_uri: "/upload/image" # 上传的目标 HTTP 路径
      form_field_name: "file" # multipart/form-data 字段名
      interval_ms: 1000      # 上传最小时间间隔 (毫秒)
```

---

## 快速启动

```bash
roslaunch api_routes api_routes.launch
```

## 核心配置参数

| 参数名 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `publish_rate` | `20.0` | 遥测推送与主循环频率 (Hz) |
| `use_degrees` | `false` | 姿态角单位（true: 角度, false: 弧度） |
| `odom_topic` | `/mavros/local_position/odom` | 里程计话题路径 |
| `mqtt_host` / `mqtt_port` | `localhost` / `1883` | MQTT Broker 地址与端口 |
| `web_port` | `8000` | HTTP 与 WebSocket 服务端口 |
| `cloud_host` / `cloud_port` | `localhost` / `8000` | 云端 HTTP 上报主机与端口 |
| `abnormal_report_topic` | `/abnormal/report` | 异常报告 ROS 监听话题 |
| `abnormal_report_uri` | `/abnormal/report` | 异常报告远程 HTTP 路径 |
