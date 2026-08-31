#pragma once

#include <boost/filesystem.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "dk/RosTimeProvider.hpp"
#include "dk/adapters/mqtt.hpp"
#include "dk/adapters/web.hpp"
#include "dk/engine.hpp"

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <yaml-cpp/yaml.h>

#include "api_routes/abnormal_reporter.hpp"
#include "api_routes/events.hpp"
#include <dk_auto_json.hpp>
#include "api_routes/image_uploader.hpp"
#include "api_routes/mavlink_telemetry.hpp"
#include "api_routes/state_diff_tracker.hpp"

namespace fs = boost::filesystem;

const std::string ROSNODE_NAME = "api_routes";

struct AppContext {};

class ApiRoutesEngine : public dk::BaseEngine<AppContext, ApiRoutesEngine> {
   public:
    using AllowedEvents = std::tuple<dk::MqttConnectEvent, dk::WsOpenEvent>;
    using MqttAdapter = dk::MqttClientAdapter<AppContext, ApiRoutesEngine>;
    using WebAdapter = dk::WebAdapter<AppContext, ApiRoutesEngine>;
    using BaseEngine::BaseEngine;

    std::shared_ptr<MqttAdapter> mqtt_adapter_;
    std::shared_ptr<WebAdapter> web_adapter_;
    std::shared_ptr<MavlinkTelemetry> telemetry_;
    std::shared_ptr<StateDiffTracker> state_diff_tracker_;
    std::shared_ptr<AbnormalReporter> abnormal_reporter_;

    std::map<std::string, ros::Publisher> ros_pub_;
    std::map<std::string, ros::Subscriber> ros_sub_;
    std::vector<std::shared_ptr<ImageUploader>> image_uploaders_;

    std::vector<std::function<void()>> reconnect_callbacks_;
    std::vector<std::function<void()>> state_heartbeat_callbacks_;

    std::map<std::string, std::function<void(std::shared_ptr<dk::WsConnection>)>>
        ws_open_callbacks_;
    std::mutex ws_open_callbacks_mutex_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_{"~"};
    std::optional<std::string> device_code_;
    fs::path target_file_;

    std::set<std::string> task_set_;

    void on_start() override;
    void on_tick(double dt, AppContext &ctx) override;
    void on_event(const dk::MqttConnectEvent &event, AppContext &ctx);
    void on_event(const dk::WsOpenEvent &event, AppContext &ctx);

   private:
    void setup_mqtt();
    void setup_http_get_gps();
    void setup_ws_state();
    void publish_in_memory_state();
    void setup_all_topic();

    void register_mqtt_sub(std::string ros_topic, std::string mqtt_topic, int qos);
    void register_mqtt_pub(std::string ros_topic, std::string mqtt_topic, int qos,
                           bool retain, bool is_state);
    void register_http_service_bridge(std::string ros_service, std::string http_path);
    void register_ws_sub(std::string ros_topic, std::string ws_path);
    void register_ws_pub(std::string ros_topic, std::string ws_path, bool is_state);

    bool parse_ros_msg(const std::string &ros_topic, std::string msg_data,
                       nlohmann::json &current_json);
    void publish_mqtt_msg(nlohmann::json &data, const std::string &mqtt_topic,
                          const int &qos, const bool &retain);

    void load_param();
    template <typename T>
    void save_param(const std::string &key, const T &value);

    template <typename T>
    std::optional<T> extract_param(const std::string &parent_key,
                                   XmlRpc::XmlRpcValue &val,
                                   const std::string &key, bool verbose = true);

    std::string remove_slash(std::string str);
    std::string resolve_topic(const std::string &topic) const;
};
