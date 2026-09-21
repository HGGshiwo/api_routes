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

#include "ros/publisher.h"
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

struct ApiRouteTask {
    std::string key;
    XmlRpc::XmlRpcValue config;
    std::vector<std::string> published_ros_topics;
    std::vector<std::string> subscribed_ros_topics;
    std::vector<std::string> ws_open_routes;
    std::shared_ptr<ImageUploader> image_uploader;
    std::string callback_key;
};

enum ActionType {STAND_UP, LIE_DOWN};

class ApiRoutesEngine : public dk::BaseEngine<AppContext, ApiRoutesEngine> {
   public:
    using AllowedEvents = std::tuple<dk::MqttConnectEvent, dk::MqttDisconnectEvent, dk::WsOpenEvent>;
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
    ros::Publisher cmd_vel_pub_;
    ros::Publisher action_pub_;
    ros::Publisher routes_status_pub_;
    std::atomic<bool> mqtt_connected_{false};

    std::map<std::string, std::function<void()>> reconnect_callbacks_;
    std::map<std::string, std::function<void()>> state_heartbeat_callbacks_;
    std::mutex callbacks_mutex_;

    std::map<std::string, std::function<void(std::shared_ptr<dk::WsConnection>)>>
        ws_open_callbacks_;
    std::mutex ws_open_callbacks_mutex_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_{"~"};
    std::optional<std::string> device_code_;
    fs::path target_file_;

    std::map<std::string, std::shared_ptr<ApiRouteTask>> active_tasks_;
    std::mutex routes_mutex_;

    void on_start() override;
    void on_tick(double dt, AppContext &ctx) override;
    void on_event(const dk::MqttConnectEvent &event, AppContext &ctx);
    void on_event(const dk::MqttDisconnectEvent &event, AppContext &ctx);
    void on_event(const dk::WsOpenEvent &event, AppContext &ctx);

   private:
    void setup_mqtt();
    void setup_http_service();

    nlohmann::json handle_get_gps();
    nlohmann::json handle_joystick(const OldJoystickEvent &event);
    nlohmann::json handle_action(ActionType action_type);

    void setup_ws_state();
    void publish_in_memory_state();
    void setup_all_topic();
    void publish_routes_status();

    void create_task(const std::string &key, XmlRpc::XmlRpcValue &val);
    void destroy_task(const std::string &key);

    void register_mqtt_sub(std::shared_ptr<ApiRouteTask> task, std::string ros_topic, std::string mqtt_topic, int qos);
    void register_mqtt_pub(std::shared_ptr<ApiRouteTask> task, std::string ros_topic, std::string mqtt_topic, int qos,
                           bool retain, bool is_state);
    void register_http_service_bridge(std::shared_ptr<ApiRouteTask> task, std::string ros_service, std::string http_path);
    void register_ws_sub(std::shared_ptr<ApiRouteTask> task, std::string ros_topic, std::string ws_path);
    void register_ws_pub(std::shared_ptr<ApiRouteTask> task, std::string ros_topic, std::string ws_path, bool is_state);

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

    // 动态全量 MQTT 消息桥接
    ros::Subscriber dynamic_mqtt_pub_sub_;
    ros::Publisher dynamic_mqtt_sub_pub_;
    std::string dynamic_mqtt_pub_topic_;
    std::string dynamic_mqtt_sub_topic_;
    std::set<std::string> dynamic_subscribed_mqtt_topics_;
    std::vector<std::string> configured_dynamic_sub_topics_;
    std::mutex dynamic_mqtt_mutex_;

    void setup_dynamic_mqtt_bridge();
    void handle_dynamic_mqtt_send(const std_msgs::String::ConstPtr &msg);
    void subscribe_dynamic_mqtt_topic(const std::string &raw_topic, int qos = 0);
    void forward_dynamic_mqtt_message(const std::string &topic, const std::string &payload, int qos);
};
