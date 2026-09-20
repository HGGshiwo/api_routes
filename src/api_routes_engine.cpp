#include "api_routes/api_routes_engine.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <iomanip>
#include <sstream>

void ApiRoutesEngine::on_start() {
  try {
    std::string str = ros::package::getPath(ROSNODE_NAME);
    fs::path p(str);
    target_file_ = p / "config" / "device_code.yaml";

    std::string mqtt_host =
        private_nh_.param("mqtt_host", std::string("localhost"));
    unsigned short mqtt_port = private_nh_.param("mqtt_port", 1883);

    load_param();

    mqtt_adapter_ =
        std::make_shared<MqttAdapter>(shared_from_this(), mqtt_host, mqtt_port);
    ROS_INFO_STREAM("[ApiRoutes] MQTT starting at " << mqtt_host << ":"
                                                    << mqtt_port);

    int web_port = private_nh_.param("web_port", 8000);
    web_adapter_ = std::make_shared<WebAdapter>(
        shared_from_this(), static_cast<unsigned short>(web_port));
    ROS_INFO_STREAM("[ApiRoutes] Web starting at port " << web_port);

    telemetry_ = std::make_shared<MavlinkTelemetry>(nh_, private_nh_);
    state_diff_tracker_ = std::make_shared<StateDiffTracker>();

    std::string cloud_host =
        private_nh_.param("cloud_host", std::string("localhost"));
    int cloud_port = private_nh_.param("cloud_port", 8001);
    std::string abnormal_topic = private_nh_.param(
        "abnormal_report_topic", std::string("/abnormal/report"));
    std::string abnormal_path = private_nh_.param(
        "abnormal_report_uri", std::string("/abnormal/report"));

    AbnormalReporter::Config abnormal_cfg;
    abnormal_cfg.ros_topic = abnormal_topic;
    abnormal_cfg.cloud_host = cloud_host;
    abnormal_cfg.cloud_port = cloud_port;
    abnormal_cfg.remote_path = abnormal_path;

    abnormal_reporter_ = std::make_shared<AbnormalReporter>(
        nh_, telemetry_, [this]() { return device_code_.value_or(""); },
        abnormal_cfg);
    abnormal_reporter_->start();

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    action_pub_ = nh_.advertise<std_msgs::String>("dank/action", 10);
    routes_status_pub_ =
        nh_.advertise<std_msgs::String>("/api_routes/routes_status", 1);

    setup_mqtt();
    setup_http_service();
    setup_ws_state();
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[ApiRoutes] Exception in on_start: " << e.what());
  } catch (...) {
    ROS_ERROR("[ApiRoutes] Unknown exception in on_start!");
  }
}

void ApiRoutesEngine::on_event(const dk::MqttConnectEvent &event,
                               AppContext &ctx) {
  try {
    mqtt_connected_ = true;
    if (!device_code_.has_value())
      return;
    ROS_INFO("[ApiRoutes] MQTT connected, publishing full state telemetry...");

    nlohmann::json last_state = state_diff_tracker_->get_last_state();
    if (!last_state.empty()) {
      publish_mqtt_msg(last_state, "device/$/state", 0, false);
    }
    std::vector<std::function<void()>> cbs_to_call;
    {
      std::lock_guard<std::mutex> lock(callbacks_mutex_);
      for (const auto &pair : reconnect_callbacks_) {
        if (pair.second) {
          cbs_to_call.push_back(pair.second);
        }
      }
    }
    for (const auto &cb : cbs_to_call) {
      try {
        cb();
      } catch (const std::exception &e) {
        ROS_WARN_STREAM(
            "[ApiRoutes] Reconnect callback exception: " << e.what());
      } catch (...) {
      }
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(
        "[ApiRoutes] Exception in MqttConnectEvent handler: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in MqttConnectEvent handler");
  }
}

void ApiRoutesEngine::on_event(const dk::MqttDisconnectEvent &event,
                               AppContext &ctx) {
  try {
    mqtt_connected_ = false;
    ROS_WARN_STREAM("[ApiRoutes] MQTT disconnected / connection lost: " << event.cause);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(
        "[ApiRoutes] Exception in MqttDisconnectEvent handler: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in MqttDisconnectEvent handler");
  }
}

void ApiRoutesEngine::on_event(const dk::WsOpenEvent &event, AppContext &ctx) {
  try {
    ROS_INFO_STREAM("[ApiRoutes] WebSocket connection on path: " << event.path);
    std::lock_guard<std::mutex> lock(ws_open_callbacks_mutex_);
    auto it = ws_open_callbacks_.find(event.path);
    if (it != ws_open_callbacks_.end() && it->second) {
      it->second(event.conn);
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(
        "[ApiRoutes] Exception in WsOpenEvent handler: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in WsOpenEvent handler");
  }
}

void ApiRoutesEngine::on_tick(double dt, AppContext &ctx) {
  try {
    publish_in_memory_state();

    if (is_hz(1.0)) {
      try {
        setup_all_topic();
      } catch (const std::exception &e) {
        ROS_WARN_STREAM("[ApiRoutes] setup_all_topic exception: " << e.what());
      } catch (...) {
      }

      if (routes_status_pub_ && routes_status_pub_.getNumSubscribers() > 0) {
        publish_routes_status();
      }

      if (telemetry_) {
        auto diag = telemetry_->getOdomDiag();
        std::ostringstream oss;
        oss << "[ApiRoutes Diag] odom_topic=" << diag.topic
            << " | pubs=" << diag.num_publishers
            << " | rx_cnt=" << diag.msg_count << " | age=";
        if (diag.age_sec >= 0) {
          oss << std::fixed << std::setprecision(2) << diag.age_sec << "s";
        } else {
          oss << "NONE";
        }
        oss << " | pos=[" << std::fixed << std::setprecision(3) << diag.pos_x
            << ", " << diag.pos_y << ", " << diag.pos_z << "]"
            << " | dev=" << device_code_.value_or("UNSET");
        ROS_INFO_STREAM_THROTTLE(10.0, oss.str());
      }
    }

    if (is_hz(0.5) && device_code_.has_value()) {
      // 定时全量心跳保底：即使机器人静止未产生差量，也定期上报一次全量状态
      if (state_diff_tracker_) {
        nlohmann::json last_state = state_diff_tracker_->get_last_state();
        if (!last_state.empty()) {
          publish_mqtt_msg(last_state, "device/$/state", 0, false);
        }
      }

      std::vector<std::function<void()>> cbs_to_call;
      {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        for (const auto &pair : state_heartbeat_callbacks_) {
          if (pair.second) {
            cbs_to_call.push_back(pair.second);
          }
        }
      }
      for (const auto &cb : cbs_to_call) {
        try {
          cb();
        } catch (const std::exception &e) {
          ROS_WARN_STREAM(
              "[ApiRoutes] Heartbeat callback exception: " << e.what());
        } catch (...) {
        }
      }
    }
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[ApiRoutes] Exception in on_tick: " << e.what());
  } catch (...) {
    ROS_ERROR("[ApiRoutes] Unknown fatal exception in on_tick!");
  }
}

nlohmann::json ApiRoutesEngine::handle_get_gps() {
  if (!telemetry_) {
    return nlohmann::json{{"error", "telemetry not ready"}};
  }
  return telemetry_->getGpsResponseJson();
}

nlohmann::json ApiRoutesEngine::handle_joystick(const OldJoystickEvent &event) {
  geometry_msgs::Twist twist;
  twist.linear.x = event.right_y;
  twist.linear.y = -event.right_x;
  twist.linear.z = event.left_y;
  twist.angular.z = -event.left_x;

  if (cmd_vel_pub_) {
    cmd_vel_pub_.publish(twist);
  }
  return nlohmann::json{{"status", "success"}};
}

nlohmann::json ApiRoutesEngine::handle_action(ActionType action_type) {

  std_msgs::String action;
  if (action_type == ActionType::STAND_UP) {
    action.data = "stand";
  } else if (action_type == ActionType::LIE_DOWN) {
    action.data = "lie";
  }

  if (action_pub_) {
    action_pub_.publish(action);
  }
  return nlohmann::json{{"status", "success"}};
}

void ApiRoutesEngine::setup_http_service() {
  web_adapter_->register_handler(boost::beast::http::verb::get, "/get_gps",
                                 &ApiRoutesEngine::handle_get_gps, this);
  ROS_INFO("[ApiRoutes] HTTP /get_gps endpoint registered.");

  web_adapter_->register_handler(boost::beast::http::verb::post,
                                 "/set_joystick",
                                 &ApiRoutesEngine::handle_joystick, this);
  ROS_INFO("[ApiRoutes] HTTP /set_joystick endpoints registered.");

  web_adapter_->register_handler(boost::beast::http::verb::post, "/takeoff",
                                 &ApiRoutesEngine::handle_action, this,
                                 ActionType::STAND_UP);
  ROS_INFO("[ApiRoutes] HTTP /takeoff endpoints registered.");

  web_adapter_->register_handler(boost::beast::http::verb::post, "/land",
                                 &ApiRoutesEngine::handle_action, this,
                                 ActionType::LIE_DOWN);
  ROS_INFO("[ApiRoutes] HTTP /land endpoints registered.");

  web_adapter_->register_handler(
      boost::beast::http::verb::get, "/vel_smoother/enable", [this]() -> nlohmann::json {
        bool enable = nh_.param<bool>("/cmd_vel_smoother/enable", false);
        return nlohmann::json{{"status", "success"}, {"msg", enable}};
      });

  web_adapter_->register_handler(
      boost::beast::http::verb::post, "/vel_smoother/enable", [this](const SetSmootherEvent& event) -> nlohmann::json {
        nh_.setParam("/cmd_vel_smoother/enable", event.enable);
        return nlohmann::json{{"status", "success"}};
      });

  ROS_INFO("[ApiRoutes] HTTP /vel_smoother/enable endpoints registered.");
}

static const std::map<std::string, std::string> key_map{
    {"gpsLocation", "gps"},     {"xVel", "x_vel"},
    {"yVel", "y_vel"},          {"xVelBody", "x_vel_body"},
    {"yVelBody", "y_vel_body"}, {"relAlt", "rel_alt"},
    {"gpsNsats", "gps_nsats"},  {"mapLocation", "pos_enu"}};

static nlohmann::json convert_ws_keys(const nlohmann::json &input) {
  if (!input.is_object())
    return input;
  nlohmann::json output = nlohmann::json::object();
  for (auto &el : input.items()) {
    auto it = key_map.find(el.key());
    if (it != key_map.end()) {
      output[it->second] = el.value();
    } else {
      output[el.key()] = el.value();
    }
  }
  return output;
}

void ApiRoutesEngine::setup_ws_state() {
  std::string route_path = "/ws";
  {
    std::lock_guard<std::mutex> lock(ws_open_callbacks_mutex_);
    ws_open_callbacks_[route_path] =
        [this, route_path](std::shared_ptr<dk::WsConnection> conn) {
          nlohmann::json last_state = state_diff_tracker_->get_last_state();
          if (!last_state.empty()) {
            last_state["deviceCode"] = device_code_.value_or("");
            last_state["timestamp"] =
                (uint64_t)(get_time_provider()->now() * 1000);
            last_state["type"] = "state";
            conn->send_state(route_path, convert_ws_keys(last_state));
          }
        };
  }

  web_adapter_->register_managed_ws_route(
      route_path,
      [](std::shared_ptr<dk::WsConnection> conn, std::string msg) {});
  ROS_INFO_STREAM("[ApiRoutes] Managed WebSocket route " << route_path
                                                         << " registered.");
}

void ApiRoutesEngine::publish_in_memory_state() {
  try {
    if (!device_code_.has_value()) {
      ROS_WARN_THROTTLE(5.0, "[ApiRoutes Diag] publish_in_memory_state: "
                             "device_code_ is UNSET, skipping state upload!");
      return;
    }

    if (!telemetry_ || !state_diff_tracker_)
      return;

    nlohmann::json current_json = telemetry_->getMergedState();
    if (current_json.empty())
      return;

    nlohmann::json diff_json =
        state_diff_tracker_->update_and_get_diff(current_json);

    if (diff_json.empty()) {
      ROS_INFO_THROTTLE(10.0, "[Telemetry Diff] Suppressed: all telemetry "
                              "values delta < 0.01 (robot stationary).");
      return;
    }

    if (!diff_json.contains("mapLocation")) {
      ROS_INFO_THROTTLE(10.0, "[Telemetry Diff] Telemetry diff generated "
                              "without mapLocation (pos delta < 0.01m).");
    }

    diff_json["deviceCode"] = device_code_.value_or("");
    diff_json["timestamp"] = (uint64_t)(get_time_provider()->now() * 1000);
    diff_json["type"] = "state";

    std::string mqtt_topic = resolve_topic("device/$/state");
    if (mqtt_adapter_) {
      std::string dump_str = diff_json.dump();
      std::string preview = dump_str.substr(0, 100);
      ROS_INFO_THROTTLE(5.0, "[Telemetry Pub] Publishing to MQTT '%s': %s%s",
                        mqtt_topic.c_str(), preview.c_str(),
                        dump_str.size() > 100 ? "..." : "");
      mqtt_adapter_->publish(mqtt_topic, dump_str, 0, false);
    }
    if (web_adapter_) {
      web_adapter_->publish_state_to_path("/ws", "/ws",
                                          convert_ws_keys(diff_json));
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(
        "[ApiRoutes] Exception in publish_in_memory_state: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in publish_in_memory_state");
  }
}

void ApiRoutesEngine::setup_mqtt() {
  try {
    if (!mqtt_adapter_)
      return;
    if (device_code_.has_value()) {
      ROS_INFO_STREAM("[Mqtt] use existing code: " << device_code_.value());
      try {
        ros::param::set("/device_code", device_code_.value());
      } catch (...) {
      }
      mqtt_adapter_->connect(device_code_.value());
      setup_all_topic();
    } else {
      mqtt_adapter_->register_publish_handler<RegisterEvent>(
          "$exclusive/register", [this](const RegisterEvent &data) -> void {
            try {
              if (!device_code_.has_value()) {
                device_code_ = data.deviceCode;
                try {
                  ros::param::set("/device_code", device_code_.value());
                } catch (...) {
                }
                if (mqtt_adapter_) {
                  mqtt_adapter_->connect(device_code_.value());
                }
                setup_all_topic();

                private_nh_.setParam("device_code", device_code_.value());
                save_param("device_code", device_code_.value());

                ROS_INFO_STREAM(
                    "[Mqtt] register with code: " << device_code_.value());
              }
            } catch (const std::exception &e) {
              ROS_WARN_STREAM(
                  "[ApiRoutes] Exception in register handler: " << e.what());
            } catch (...) {
            }
          });

      mqtt_adapter_->connect();
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM("[ApiRoutes] Exception in setup_mqtt: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in setup_mqtt");
  }
}

bool ApiRoutesEngine::parse_ros_msg(const std::string &ros_topic,
                                    std::string msg_data,
                                    nlohmann::json &current_json) {
  try {
    current_json = nlohmann::json::parse(msg_data);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[ApiRoutes] parse_ros_msg JSON parse failed on topic '"
                                  << ros_topic << "': " << e.what());
    return false;
  } catch (...) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[ApiRoutes] parse_ros_msg unknown parse exception on topic '"
                                  << ros_topic << "'");
    return false;
  }
  return current_json.is_object();
}

void ApiRoutesEngine::publish_mqtt_msg(nlohmann::json &data,
                                       const std::string &mqtt_topic,
                                       const int &qos, const bool &retain) {
  try {
    if (!mqtt_adapter_)
      return;
    data["deviceCode"] = device_code_.value_or("");
    data["timestamp"] = (uint64_t)(get_time_provider()->now() * 1000);
    data["type"] = "state";
    mqtt_adapter_->publish(resolve_topic(mqtt_topic), data.dump(), qos, retain);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM("[ApiRoutes] Exception in publish_mqtt_msg: " << e.what());
  } catch (...) {
    ROS_WARN("[ApiRoutes] Unknown exception in publish_mqtt_msg");
  }
}

void ApiRoutesEngine::load_param() {
  try {
    if (!fs::exists(target_file_))
      return;
    YAML::Node config = YAML::LoadFile(target_file_.string());
    if (config["device_code"] && config["device_code"].IsDefined() &&
        !config["device_code"].IsNull()) {
      std::string code = config["device_code"].as<std::string>();
      if (!device_code_.has_value() || device_code_.value() != code) {
        ROS_INFO_STREAM("[ApiRoutes] Parameter 'device_code' loaded: " << code);
      }
      device_code_ = code;
      try {
        ros::param::set("/device_code", device_code_.value());
      } catch (...) {
      }
    } else {
      if (device_code_.has_value()) {
        ROS_INFO_STREAM("[ApiRoutes] Parameter 'device_code' removed");
      }
      device_code_ = std::nullopt;
    }
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[ApiRoutes] Parameter parse error in " << target_file_ << ": " << e.what());
    device_code_ = std::nullopt;
  }
}

template <typename T>
void ApiRoutesEngine::save_param(const std::string &key, const T &value) {
  try {
    YAML::Node root;
    root[key] = value;
    fs::create_directories(target_file_.parent_path());
    std::ofstream fout(target_file_);
    if (fout.is_open()) {
      fout << root;
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM("[ApiRoutes] save_param failed: " << e.what());
  } catch (...) {
  }
}

std::string ApiRoutesEngine::remove_slash(std::string str) {
  if (!str.empty() && str.front() == '/') {
    str.erase(0, 1);
  }
  return str;
}

std::string ApiRoutesEngine::resolve_topic(const std::string &topic) const {
  if (!device_code_.has_value())
    return topic;
  std::string result = topic;
  size_t pos = 0;
  while ((pos = result.find('$', pos)) != std::string::npos) {
    result.replace(pos, 1, device_code_.value());
    pos += device_code_.value().length();
  }
  return result;
}
