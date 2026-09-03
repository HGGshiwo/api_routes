#include "api_routes/api_routes_engine.hpp"
#include <fstream>

class GetGpsHttpHandler : public dk::IProtocolHandler<ApiRoutesEngine::WebAdapter> {
    std::shared_ptr<MavlinkTelemetry> telemetry_;

   public:
    GetGpsHttpHandler(std::shared_ptr<MavlinkTelemetry> telemetry)
        : telemetry_(telemetry) {}

    void handle(
        std::shared_ptr<dk::HttpSession<ApiRoutesEngine::WebAdapter>> session,
        boost::beast::http::request<boost::beast::http::string_body> req)
        override {
        nlohmann::json res_json = telemetry_->getGpsResponseJson();
        session->send_http_response(boost::beast::http::status::ok,
                                    res_json.dump());
    }
};

void ApiRoutesEngine::on_start() {
    std::string str = ros::package::getPath(ROSNODE_NAME);
    fs::path p(str);
    target_file_ = p / "config" / "device_code.yaml";

    std::string mqtt_host =
        private_nh_.param("mqtt_host", std::string("localhost"));
    unsigned short mqtt_port = private_nh_.param("mqtt_port", 1883);

    load_param();

    mqtt_adapter_ = std::make_shared<MqttAdapter>(shared_from_this(),
                                                   mqtt_host, mqtt_port);
    ROS_INFO_STREAM("[ApiRoutes] MQTT starting at " << mqtt_host << ":"
                                                    << mqtt_port);

    int web_port = private_nh_.param("web_port", 8000);
    web_adapter_ = std::make_shared<WebAdapter>(
        shared_from_this(), static_cast<unsigned short>(web_port));
    ROS_INFO_STREAM("[ApiRoutes] Web starting at port " << web_port);

    telemetry_ = std::make_shared<MavlinkTelemetry>(nh_, private_nh_);
    state_diff_tracker_ = std::make_shared<StateDiffTracker>();

    std::string cloud_host = private_nh_.param(
        "cloud_host", std::string("localhost"));
    int cloud_port = private_nh_.param(
        "cloud_port", 8001);
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

    setup_mqtt();
    setup_http_get_gps();
    setup_ws_state();
}

void ApiRoutesEngine::on_event(const dk::MqttConnectEvent &event, AppContext &ctx) {
    if (!device_code_.has_value()) return;
    ROS_INFO("[ApiRoutes] MQTT connected, publishing full state telemetry...");

    nlohmann::json last_state = state_diff_tracker_->get_last_state();
    if (!last_state.empty()) {
        publish_mqtt_msg(last_state, "/device/$/state", 0, false);
    }
    for (const auto &cb : reconnect_callbacks_) {
        cb();
    }
}

void ApiRoutesEngine::on_event(const dk::WsOpenEvent &event, AppContext &ctx) {
    ROS_INFO_STREAM("[ApiRoutes] WebSocket connection on path: " << event.path);
    std::lock_guard<std::mutex> lock(ws_open_callbacks_mutex_);
    auto it = ws_open_callbacks_.find(event.path);
    if (it != ws_open_callbacks_.end()) {
        it->second(event.conn);
    }
}

void ApiRoutesEngine::on_tick(double dt, AppContext &ctx) {
    publish_in_memory_state();

    if (is_hz(1.0)) {
        setup_all_topic();
    }

    if (is_hz(0.5) && device_code_.has_value()) {
        for (const auto &cb : state_heartbeat_callbacks_) {
            cb();
        }
    }
}

void ApiRoutesEngine::setup_http_get_gps() {
    auto handler = std::make_shared<GetGpsHttpHandler>(telemetry_);
    web_adapter_->register_handler(boost::beast::http::verb::post, "/get_gps",
                                   handler);
    web_adapter_->register_handler(boost::beast::http::verb::get, "/get_gps",
                                   handler);
    ROS_INFO("[ApiRoutes] HTTP /get_gps endpoint registered.");
}

static const std::map<std::string, std::string> key_map{
    {"gpsLocation", "gps"},     {"xVel", "x_vel"},
    {"yVel", "y_vel"},          {"xVelBody", "x_vel_body"},
    {"yVelBody", "y_vel_body"}, {"relAlt", "rel_alt"},
    {"gpsNsats", "gps_nsats"},  {"mapLocation", "pos_enu"}};

static nlohmann::json convert_ws_keys(const nlohmann::json &input) {
    if (!input.is_object()) return input;
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
                    conn->send_state(route_path, convert_ws_keys(last_state));
                }
            };
    }

    web_adapter_->register_managed_ws_route(
        route_path,
        [](std::shared_ptr<dk::WsConnection> conn, std::string msg) {});
    ROS_INFO_STREAM("[ApiRoutes] Managed WebSocket route " << route_path << " registered.");
}

void ApiRoutesEngine::publish_in_memory_state() {
    if (!device_code_.has_value()) return;

    nlohmann::json current_json = telemetry_->getMergedState();
    if (current_json.empty()) return;

    nlohmann::json diff_json =
        state_diff_tracker_->update_and_get_diff(current_json);

    if (diff_json.empty()) return;

    diff_json["deviceCode"] = device_code_.value_or("");
    diff_json["timestamp"] = (uint64_t)(get_time_provider()->now() * 1000);

    mqtt_adapter_->publish(resolve_topic("device/$/state"), diff_json.dump(),
                           0, false);

    web_adapter_->publish_state_to_path("/ws", "/ws", convert_ws_keys(diff_json));
}

void ApiRoutesEngine::setup_mqtt() {
    if (device_code_.has_value()) {
        ROS_INFO_STREAM("[Mqtt] use existing code: " << device_code_.value());
        ros::param::set("/device_code", device_code_.value());
        mqtt_adapter_->connect(device_code_.value());
        setup_all_topic();
    } else {
        mqtt_adapter_->register_publish_handler<RegisterEvent>(
            "$exclusive/register", [this](const RegisterEvent &data) -> void {
                if (!device_code_.has_value()) {
                    device_code_ = data.deviceCode;
                    ros::param::set("/device_code", device_code_.value());
                    mqtt_adapter_->connect(device_code_.value());
                    setup_all_topic();

                    private_nh_.setParam("device_code", device_code_.value());
                    save_param("device_code", device_code_.value());

                    ROS_INFO_STREAM(
                        "[Mqtt] register with code: " << device_code_.value());
                }
            });

        mqtt_adapter_->connect();
    }
}

bool ApiRoutesEngine::parse_ros_msg(const std::string &ros_topic,
                                    std::string msg_data,
                                    nlohmann::json &current_json) {
    try {
        current_json = nlohmann::json::parse(msg_data);
    } catch (...) {
        return false;
    }
    return current_json.is_object();
}

void ApiRoutesEngine::publish_mqtt_msg(nlohmann::json &data,
                                       const std::string &mqtt_topic,
                                       const int &qos, const bool &retain) {
    data["deviceCode"] = device_code_.value();
    data["timestamp"] = (uint64_t)(get_time_provider()->now() * 1000);
    mqtt_adapter_->publish(resolve_topic(mqtt_topic), data.dump(), qos, retain);
}

void ApiRoutesEngine::load_param() {
    if (!fs::exists(target_file_)) return;
    YAML::Node config = YAML::LoadFile(target_file_.string());
    if (config["device_code"] && config["device_code"].IsDefined() &&
        !config["device_code"].IsNull()) {
        device_code_ = config["device_code"].as<std::string>();
        ros::param::set("/device_code", device_code_.value());
    } else {
        device_code_ = std::nullopt;
    }
}

template <typename T>
void ApiRoutesEngine::save_param(const std::string &key, const T &value) {
    YAML::Node root;
    root[key] = value;
    fs::create_directories(target_file_.parent_path());
    std::ofstream fout(target_file_);
    if (fout.is_open()) {
        fout << root;
    }
}

std::string ApiRoutesEngine::remove_slash(std::string str) {
    if (!str.empty() && str.front() == '/') {
        str.erase(0, 1);
    }
    return str;
}

std::string ApiRoutesEngine::resolve_topic(const std::string &topic) const {
    if (!device_code_.has_value()) return topic;
    std::string result = topic;
    size_t pos = 0;
    while ((pos = result.find('$', pos)) != std::string::npos) {
        result.replace(pos, 1, device_code_.value());
        pos += device_code_.value().length();
    }
    return result;
}
