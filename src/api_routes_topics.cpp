#include <api_routes/StringSrv.h>
#include "api_routes/api_routes_engine.hpp"

template <typename T>
std::optional<T> ApiRoutesEngine::extract_param(const std::string &parent_key,
                                                 XmlRpc::XmlRpcValue &val,
                                                 const std::string &key,
                                                 bool verbose) {
    if (!val.hasMember(key)) {
        if (verbose)
            ROS_ERROR_STREAM("[ApiRoutes] " << parent_key << " must include "
                                            << key);
        return std::nullopt;
    }

    T ret;
    try {
        ret = (T)val[key];
    } catch (const std::exception &ex) {
        ROS_ERROR_STREAM("[ApiRoutes] " << parent_key << " convert " << key
                                        << " failed: " << ex.what());
        return std::nullopt;
    } catch (...) {
        ROS_ERROR_STREAM("[ApiRoutes] " << parent_key << " convert " << key
                                        << " failed with unknown exception!");
        return std::nullopt;
    }
    return ret;
}

void ApiRoutesEngine::setup_all_topic() {
    if (!device_code_.has_value()) return;

    std::string api_param_ns = ROSNODE_NAME + "/api";
    XmlRpc::XmlRpcValue namespace_params;
    if (ros::param::get(api_param_ns, namespace_params)) {
        if (namespace_params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_STREAM("[ApiRoutes] param in " << api_param_ns
                                                     << " is not struct!");
            return;
        }

        for (auto it = namespace_params.begin(); it != namespace_params.end();
             ++it) {
            std::string key = it->first;

            if (task_set_.find(key) != task_set_.end()) continue;
            task_set_.insert(key);

            XmlRpc::XmlRpcValue &val = it->second;

            auto protocal = extract_param<std::string>(key, val, "protocol");
            if (!protocal.has_value()) continue;

            std::optional<std::string> topic_type;
            if (protocal.value() != "http" && protocal.value() != "upload_image") {
                auto type_opt =
                    extract_param<std::string>(key, val, "topic_type");
                if (!type_opt.has_value()) continue;
                topic_type = type_opt;
            } else if (protocal.value() == "upload_image") {
                topic_type =
                    extract_param<std::string>(key, val, "topic_type", false);
            }

            auto ros_topic = extract_param<std::string>(key, val, "ros_topic");
            if (!ros_topic.has_value() || ros_topic.value().empty()) continue;

            auto remote_uri =
                extract_param<std::string>(key, val, "remote_uri");
            if (!remote_uri.has_value() || remote_uri.value().empty()) continue;

            if (protocal.value() == "mqtt") {
                auto qos = extract_param<int>(key, val, "qos", false);
                auto retain = extract_param<bool>(key, val, "retain", false);
                auto mqtt_topic = remove_slash(remote_uri.value());

                if (topic_type.value() == "pub") {
                    register_mqtt_pub(ros_topic.value(), mqtt_topic,
                                      qos.value_or(0), retain.value_or(false),
                                      false);
                } else if (topic_type.value() == "pub_state") {
                    register_mqtt_pub(ros_topic.value(), mqtt_topic,
                                      qos.value_or(0), retain.value_or(false),
                                      true);
                } else if (topic_type.value() == "sub") {
                    register_mqtt_sub(ros_topic.value(), mqtt_topic,
                                      qos.value_or(0));
                }
            } else if (protocal.value() == "websocket" ||
                       protocal.value() == "ws") {
                auto ws_path = remove_slash(remote_uri.value());
                if (topic_type.value() == "pub") {
                    register_ws_pub(ros_topic.value(), ws_path, false);
                } else if (topic_type.value() == "pub_state") {
                    register_ws_pub(ros_topic.value(), ws_path, true);
                } else if (topic_type.value() == "sub") {
                    register_ws_sub(ros_topic.value(), ws_path);
                }
            } else if (protocal.value() == "http") {
                register_http_service_bridge(ros_topic.value(),
                                             remote_uri.value());
            } else if (protocal.value() == "upload_image") {
                auto form_field_name = extract_param<std::string>(
                    key, val, "form_field_name", false);
                auto interval_ms =
                    extract_param<int>(key, val, "interval_ms", false);
                std::string tt = topic_type.value_or("compressed");

                std::string target_url;
                std::string uri = resolve_topic(remote_uri.value());
                if (uri.rfind("http://", 0) == 0 ||
                    uri.rfind("https://", 0) == 0) {
                    target_url = uri;
                } else {
                    std::string host = private_nh_.param(
                        "cloud_host",
                        private_nh_.param("mqtt_host", std::string("localhost")));
                    int port = private_nh_.param(
                        "cloud_port", private_nh_.param("web_port", 8000));
                    target_url = "http://" + host + ":" + std::to_string(port) +
                                 "/" + remove_slash(uri);
                }

                ImageUploader::Config uploader_cfg;
                uploader_cfg.ros_topic = ros_topic.value();
                uploader_cfg.target_url = target_url;
                uploader_cfg.topic_type = tt;
                uploader_cfg.form_field_name =
                    form_field_name.value_or("file");
                uploader_cfg.interval_ms = interval_ms.value_or(1000);
                if (device_code_.has_value()) {
                    uploader_cfg.extra_fields["device_code"] =
                        device_code_.value();
                }

                auto uploader =
                    std::make_shared<ImageUploader>(nh_, uploader_cfg);
                uploader->start();
                image_uploaders_.push_back(uploader);
            }
        }
    }
}

void ApiRoutesEngine::register_mqtt_sub(std::string ros_topic,
                                        std::string mqtt_topic, int qos) {
    if (!device_code_.has_value()) return;

    if (ros_pub_.find(ros_topic) != ros_pub_.end()) return;

    auto pub = nh_.advertise<std_msgs::String>(ros_topic, 1000);
    ros_pub_[ros_topic] = pub;

    mqtt_topic = resolve_topic(mqtt_topic);
    mqtt_adapter_->register_raw_handler(
        mqtt_topic,
        [pub](const dk::MqttMessage &msg) {
            std_msgs::String ros_msg;
            ros_msg.data = msg.payload;
            pub.publish(ros_msg);
        },
        qos);
}

void ApiRoutesEngine::register_mqtt_pub(std::string ros_topic,
                                        std::string mqtt_topic, int qos,
                                        bool retain, bool is_state) {
    if (!device_code_.has_value()) return;

    if (ros_sub_.find(ros_topic) != ros_sub_.end()) return;

    std::shared_ptr<StateDiffTracker> tracker;
    if (is_state) {
        tracker = std::make_shared<StateDiffTracker>();

        reconnect_callbacks_.push_back(
            [this, mqtt_topic, qos, retain, tracker]() {
                nlohmann::json last_state = tracker->get_last_state();
                if (!last_state.empty()) {
                    publish_mqtt_msg(last_state, mqtt_topic, qos, retain);
                }
            });

        state_heartbeat_callbacks_.push_back(
            [this, mqtt_topic, qos, retain, tracker]() {
                nlohmann::json last_state = tracker->get_last_state();
                publish_mqtt_msg(last_state, mqtt_topic, qos, retain);
            });
    }

    auto sub = nh_.subscribe<std_msgs::String>(
        ros_topic, 1000,
        [this, mqtt_topic, qos, retain, ros_topic, is_state,
         tracker](const std_msgs::String::ConstPtr &msg) -> void {
            if (is_state) {
                nlohmann::json current_json;
                if (!parse_ros_msg(ros_topic, msg->data, current_json)) return;
                nlohmann::json diff_json =
                    tracker->update_and_get_diff(current_json);
                if (diff_json.empty()) return;
                publish_mqtt_msg(diff_json, mqtt_topic, qos, retain);
            } else {
                mqtt_adapter_->publish(resolve_topic(mqtt_topic), msg->data,
                                       qos, retain);
            }
        });

    ros_sub_[ros_topic] = sub;
}

void ApiRoutesEngine::register_http_service_bridge(std::string ros_service,
                                                   std::string http_path) {
    class HttpRosBridgeHandler : public dk::IProtocolHandler<WebAdapter> {
        std::string ros_service_;
        ApiRoutesEngine *engine_;

       public:
        HttpRosBridgeHandler(std::string service, ApiRoutesEngine *engine)
            : ros_service_(std::move(service)), engine_(engine) {}

        void handle(
            std::shared_ptr<dk::HttpSession<WebAdapter>> session,
            boost::beast::http::request<boost::beast::http::string_body> req)
            override {
            try {
                std::string req_str = req.body();
                std::thread([session, service_name = ros_service_, req_str,
                             &ioc = engine_->get_ioc()]() {
                    api_routes::StringSrv srv;
                    srv.request.request = req_str;

                    if (ros::service::call(service_name, srv)) {
                        boost::asio::post(
                            ioc, [session, resp_str = srv.response.response]() {
                                session->send_http_response(
                                    boost::beast::http::status::ok, resp_str);
                            });
                    } else {
                        boost::asio::post(ioc, [session]() {
                            session->send_http_response(
                                boost::beast::http::status::
                                    internal_server_error,
                                "{\"error\":\"ROS Service call failed\"}");
                        });
                    }
                }).detach();
            } catch (const std::exception &e) {
                session->send_http_response(
                    boost::beast::http::status::bad_request,
                    std::string("{\"error\":\"") + e.what() + "\"}");
            }
        }
    };

    std::string route_path = "/" + remove_slash(http_path);
    web_adapter_->register_handler(
        boost::beast::http::verb::post, route_path,
        std::make_shared<HttpRosBridgeHandler>(ros_service, this));
    ROS_INFO_STREAM("[ApiRoutes] HTTP ROS Service Bridge registered: POST "
                    << route_path << " -> " << ros_service);
}

void ApiRoutesEngine::register_ws_sub(std::string ros_topic,
                                      std::string ws_path) {
    if (ros_pub_.find(ros_topic) != ros_pub_.end()) {
        ROS_ERROR_STREAM("[Websocket] rostopic " << ros_topic << " exists!");
        return;
    }

    auto pub = nh_.advertise<std_msgs::String>(ros_topic, 1000);
    ros_pub_[ros_topic] = pub;

    std::string route_path = "/" + remove_slash(ws_path);
    web_adapter_->register_managed_ws_route(
        route_path,
        [pub](std::shared_ptr<dk::WsConnection> conn, std::string msg) {
            std_msgs::String ros_msg;
            ros_msg.data = std::move(msg);
            pub.publish(ros_msg);
        });
    ROS_INFO_STREAM("[Websocket] ws->ros: " << route_path << " -> "
                                            << ros_topic);
}

void ApiRoutesEngine::register_ws_pub(std::string ros_topic,
                                      std::string ws_path, bool is_state) {
    if (ros_sub_.find(ros_topic) != ros_sub_.end()) {
        ROS_ERROR_STREAM("[Websocket] rostopic " << ros_topic << " exists!");
        return;
    }

    std::string route_path = "/" + remove_slash(ws_path);
    std::shared_ptr<StateDiffTracker> tracker;
    if (is_state) {
        tracker = std::make_shared<StateDiffTracker>();

        std::lock_guard<std::mutex> lock(ws_open_callbacks_mutex_);
        ws_open_callbacks_[route_path] =
            [this, route_path, tracker](std::shared_ptr<dk::WsConnection> conn) {
                nlohmann::json last_state = tracker->get_last_state();
                if (!last_state.empty()) {
                    last_state["deviceCode"] = device_code_.value_or("");
                    last_state["timestamp"] =
                        (uint64_t)(get_time_provider()->now() * 1000);
                    conn->send_state(route_path, last_state);
                }
            };
    }

    web_adapter_->register_managed_ws_route(
        route_path,
        [](std::shared_ptr<dk::WsConnection> conn, std::string msg) {});

    auto sub = nh_.subscribe<std_msgs::String>(
        ros_topic, 1000,
        [this, route_path, is_state, tracker,
         ros_topic](const std_msgs::String::ConstPtr &msg) -> void {
            if (is_state) {
                nlohmann::json current_json;
                if (!parse_ros_msg(ros_topic, msg->data, current_json)) return;
                nlohmann::json diff_json =
                    tracker->update_and_get_diff(current_json);
                if (diff_json.empty()) return;
                diff_json["deviceCode"] = device_code_.value_or("");
                diff_json["timestamp"] =
                    (uint64_t)(get_time_provider()->now() * 1000);
                web_adapter_->publish_state_to_path(route_path, route_path,
                                                    diff_json);
            } else {
                try {
                    nlohmann::json j = nlohmann::json::parse(msg->data);
                    j["deviceCode"] = device_code_.value_or("");
                    j["timestamp"] =
                        (uint64_t)(get_time_provider()->now() * 1000);
                    web_adapter_->publish_to_path(route_path, j);
                } catch (...) {
                    nlohmann::json j;
                    j["data"] = msg->data;
                    j["deviceCode"] = device_code_.value_or("");
                    j["timestamp"] =
                        (uint64_t)(get_time_provider()->now() * 1000);
                    web_adapter_->publish_to_path(route_path, j);
                }
            }
        });

    ros_sub_[ros_topic] = sub;
    ROS_INFO_STREAM("[Websocket] ros[state=" << is_state << "] -> ws: "
                                             << ros_topic << " -> "
                                             << route_path);
}
