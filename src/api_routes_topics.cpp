#include <api_routes/StringSrv.h>
#include "api_routes/api_routes_engine.hpp"
#include <xmlrpcpp/XmlRpcException.h>
#include <sstream>

static std::string xmlrpc_to_string(const XmlRpc::XmlRpcValue &val) {
    std::ostringstream oss;
    switch (val.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:
            oss << (static_cast<const bool &>(val) ? "true" : "false");
            break;
        case XmlRpc::XmlRpcValue::TypeInt:
            oss << static_cast<const int &>(val);
            break;
        case XmlRpc::XmlRpcValue::TypeDouble:
            oss << static_cast<const double &>(val);
            break;
        case XmlRpc::XmlRpcValue::TypeString:
            oss << "\"" << static_cast<const std::string &>(val) << "\"";
            break;
        case XmlRpc::XmlRpcValue::TypeArray: {
            oss << "[";
            for (int i = 0; i < val.size(); ++i) {
                if (i > 0) oss << ", ";
                oss << xmlrpc_to_string(val[i]);
            }
            oss << "]";
            break;
        }
        case XmlRpc::XmlRpcValue::TypeStruct: {
            oss << "{";
            bool first = true;
            for (auto it = val.begin(); it != val.end(); ++it) {
                if (!first) oss << ", ";
                first = false;
                oss << it->first << ": " << xmlrpc_to_string(it->second);
            }
            oss << "}";
            break;
        }
        default:
            oss << "<invalid/unknown>";
            break;
    }
    return oss.str();
}

template <typename T>
std::optional<T> ApiRoutesEngine::extract_param(const std::string &parent_key,
                                                 XmlRpc::XmlRpcValue &val,
                                                 const std::string &key,
                                                 bool verbose) {
    if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << parent_key
                                        << "': config is not a struct/map (type="
                                        << val.getType() << ")");
        return std::nullopt;
    }
    if (!val.hasMember(key)) {
        if (verbose)
            ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << parent_key
                                            << "': missing required field '"
                                            << key << "'");
        return std::nullopt;
    }

    T ret;
    try {
        ret = (T)val[key];
    } catch (const XmlRpc::XmlRpcException &ex) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << parent_key
                                        << "': field '" << key
                                        << "' conversion failed: " << ex.getMessage());
        return std::nullopt;
    } catch (const std::exception &ex) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << parent_key
                                        << "': field '" << key
                                        << "' conversion failed: " << ex.what());
        return std::nullopt;
    } catch (...) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << parent_key
                                        << "': field '" << key
                                        << "' conversion failed with unknown exception!");
        return std::nullopt;
    }
    return ret;
}

static bool is_param_equal(XmlRpc::XmlRpcValue &v1, XmlRpc::XmlRpcValue &v2) {
    if (v1.getType() != v2.getType()) return false;
    return v1 == v2;
}

void ApiRoutesEngine::destroy_task(const std::string &key) {
    std::shared_ptr<ApiRouteTask> task;
    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        auto it = active_tasks_.find(key);
        if (it == active_tasks_.end()) return;
        task = it->second;

        for (const auto &topic : task->subscribed_ros_topics) {
            auto sub_it = ros_sub_.find(topic);
            if (sub_it != ros_sub_.end()) {
                sub_it->second.shutdown();
                ros_sub_.erase(sub_it);
                ROS_INFO_STREAM("[ApiRoutes] Shutdown subscriber: " << topic
                                << " (task: " << key << ")");
            }
        }

        for (const auto &topic : task->published_ros_topics) {
            auto pub_it = ros_pub_.find(topic);
            if (pub_it != ros_pub_.end()) {
                pub_it->second.shutdown();
                ros_pub_.erase(pub_it);
                ROS_INFO_STREAM("[ApiRoutes] Shutdown publisher: " << topic
                                << " (task: " << key << ")");
            }
        }

        if (task->image_uploader) {
            task->image_uploader->stop();
            auto img_it = std::find(image_uploaders_.begin(), image_uploaders_.end(),
                                    task->image_uploader);
            if (img_it != image_uploaders_.end()) {
                image_uploaders_.erase(img_it);
            }
            ROS_INFO_STREAM("[ApiRoutes] Stopped image uploader (task: " << key << ")");
        }

        active_tasks_.erase(it);
    }

    if (!task->callback_key.empty()) {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        state_heartbeat_callbacks_.erase(task->callback_key);
        reconnect_callbacks_.erase(task->callback_key);
    }

    {
        std::lock_guard<std::mutex> lock(ws_open_callbacks_mutex_);
        for (const auto &route : task->ws_open_routes) {
            ws_open_callbacks_.erase(route);
        }
    }

    ROS_INFO_STREAM("[ApiRoutes] Route task destroyed: " << key);
}

void ApiRoutesEngine::create_task(const std::string &key, XmlRpc::XmlRpcValue &val) {
    if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                  << "': expected struct/map, got type " << val.getType());
        return;
    }

    auto protocal = extract_param<std::string>(key, val, "protocol", true);
    if (!protocal.has_value()) return;
    if (protocal.value().empty()) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                  << "': 'protocol' cannot be empty");
        return;
    }

    std::string proto = protocal.value();
    std::optional<std::string> topic_type;
    if (proto != "http" && proto != "upload_image") {
        auto type_opt =
            extract_param<std::string>(key, val, "topic_type", true);
        if (!type_opt.has_value()) return;
        if (type_opt.value().empty()) {
            ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                      << "': 'topic_type' cannot be empty for protocol '" << proto << "'");
            return;
        }
        topic_type = type_opt;
    } else if (proto == "upload_image") {
        topic_type =
            extract_param<std::string>(key, val, "topic_type", false);
    }

    auto ros_topic = extract_param<std::string>(key, val, "ros_topic", true);
    if (!ros_topic.has_value()) return;
    if (ros_topic.value().empty()) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                  << "': 'ros_topic' cannot be empty");
        return;
    }

    auto remote_uri =
        extract_param<std::string>(key, val, "remote_uri", true);
    if (!remote_uri.has_value()) return;
    if (remote_uri.value().empty()) {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                  << "': 'remote_uri' cannot be empty");
        return;
    }

    auto task = std::make_shared<ApiRouteTask>();
    task->key = key;
    task->config = val;

    if (proto == "mqtt") {
        auto qos = extract_param<int>(key, val, "qos", false);
        auto retain = extract_param<bool>(key, val, "retain", false);
        auto mqtt_topic = remove_slash(remote_uri.value());
        int qos_val = qos.value_or(0);
        bool retain_val = retain.value_or(false);

        if (topic_type.value() == "pub") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: ROS subscriber on '"
                            << ros_topic.value() << "' -> forward to MQTT topic '"
                            << resolve_topic(mqtt_topic) << "' (qos=" << qos_val
                            << ", retain=" << (retain_val ? "true" : "false") << ")");
            register_mqtt_pub(task, ros_topic.value(), mqtt_topic,
                              qos_val, retain_val,
                              false);
        } else if (topic_type.value() == "pub_state") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: ROS state subscriber on '"
                            << ros_topic.value() << "' -> forward to MQTT topic '"
                            << resolve_topic(mqtt_topic) << "' (qos=" << qos_val
                            << ", retain=" << (retain_val ? "true" : "false") << ")");
            register_mqtt_pub(task, ros_topic.value(), mqtt_topic,
                              qos_val, retain_val,
                              true);
        } else if (topic_type.value() == "sub") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: MQTT subscription on '"
                            << resolve_topic(mqtt_topic) << "' (qos=" << qos_val
                            << ") -> forward to ROS publisher on '" << ros_topic.value() << "'");
            register_mqtt_sub(task, ros_topic.value(), mqtt_topic,
                              qos_val);
        } else {
            ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                      << "': invalid topic_type '" << topic_type.value()
                                      << "' for protocol 'mqtt' (expected 'pub', 'pub_state', or 'sub')");
            return;
        }
    } else if (proto == "websocket" || proto == "ws") {
        auto ws_path = remove_slash(remote_uri.value());
        if (topic_type.value() == "pub") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: ROS subscriber on '"
                            << ros_topic.value() << "' -> forward to WebSocket route '/" << ws_path << "'");
            register_ws_pub(task, ros_topic.value(), ws_path, false);
        } else if (topic_type.value() == "pub_state") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: ROS state subscriber on '"
                            << ros_topic.value() << "' -> forward to WebSocket route '/" << ws_path << "'");
            register_ws_pub(task, ros_topic.value(), ws_path, true);
        } else if (topic_type.value() == "sub") {
            ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: WebSocket subscription on route '/"
                            << ws_path << "' -> forward to ROS publisher on '" << ros_topic.value() << "'");
            register_ws_sub(task, ros_topic.value(), ws_path);
        } else {
            ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                      << "': invalid topic_type '" << topic_type.value()
                                      << "' for protocol '" << proto
                                      << "' (expected 'pub', 'pub_state', or 'sub')");
            return;
        }
    } else if (proto == "http") {
        ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: HTTP service bridge POST '/"
                        << remove_slash(remote_uri.value()) << "' -> ROS service '"
                        << ros_topic.value() << "'");
        register_http_service_bridge(task, ros_topic.value(),
                                     remote_uri.value());
    } else if (proto == "upload_image") {
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

        ROS_INFO_STREAM("[ApiRoutes] Creating route [" << key << "]: Image uploader on ROS topic '"
                        << uploader_cfg.ros_topic << "' -> URL '" << uploader_cfg.target_url
                        << "' (interval=" << uploader_cfg.interval_ms << "ms, type=" << tt << ")");
        auto uploader =
            std::make_shared<ImageUploader>(nh_, uploader_cfg);
        uploader->start();
        image_uploaders_.push_back(uploader);
        task->image_uploader = uploader;
    } else {
        ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                  << "': unsupported protocol '" << proto << "'");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        active_tasks_[key] = task;
    }
    ROS_INFO_STREAM("[ApiRoutes] Route task created: " << key);
}

void ApiRoutesEngine::setup_all_topic() {
    try {
        if (!device_code_.has_value()) return;

        std::string api_param_ns = ROSNODE_NAME + "/api";
        XmlRpc::XmlRpcValue namespace_params;
        if (!ros::param::get(api_param_ns, namespace_params)) {
            std::vector<std::pair<std::string, std::string>> keys_to_destroy;
            {
                std::lock_guard<std::mutex> lock(routes_mutex_);
                for (const auto &p : active_tasks_) {
                    keys_to_destroy.emplace_back(p.first, xmlrpc_to_string(p.second->config));
                }
            }
            if (!keys_to_destroy.empty()) {
                ROS_INFO_STREAM("[ApiRoutes] Parameter namespace '" << api_param_ns
                                << "' not found or cleared. Removing all "
                                << keys_to_destroy.size() << " active tasks.");
            }
            for (const auto &k : keys_to_destroy) {
                ROS_INFO_STREAM("[ApiRoutes] Parameter key removed: " << k.first << " (" << k.second << ")");
                destroy_task(k.first);
            }
            return;
        }

        if (namespace_params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error: '" << api_param_ns
                                      << "' must be a struct/map, got type "
                                      << namespace_params.getType());
            return;
        }

        // 1. Detect removed keys
        std::vector<std::pair<std::string, std::string>> keys_to_destroy;
        {
            std::lock_guard<std::mutex> lock(routes_mutex_);
            for (const auto &p : active_tasks_) {
                if (!namespace_params.hasMember(p.first)) {
                    keys_to_destroy.emplace_back(p.first, xmlrpc_to_string(p.second->config));
                }
            }
        }
        for (const auto &k : keys_to_destroy) {
            ROS_INFO_STREAM("[ApiRoutes] Parameter key removed: " << k.first << " (" << k.second << ")");
            destroy_task(k.first);
        }

        // 2. Detect modified or added keys
        for (auto it = namespace_params.begin(); it != namespace_params.end();
             ++it) {
            std::string key = it->first;
            XmlRpc::XmlRpcValue &val = it->second;

            if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR_STREAM_THROTTLE(5.0, "[ApiRoutes] Parameter parse error for '" << key
                                          << "': expected struct/map, got type "
                                          << val.getType());
                continue;
            }

            bool key_modified = false;
            {
                std::lock_guard<std::mutex> lock(routes_mutex_);
                auto existing_it = active_tasks_.find(key);
                if (existing_it != active_tasks_.end()) {
                    if (is_param_equal(existing_it->second->config, val)) {
                        continue;
                    }
                    ROS_INFO_STREAM("[ApiRoutes] Parameter key modified: " << key
                                    << "\n  old: " << xmlrpc_to_string(existing_it->second->config)
                                    << "\n  new: " << xmlrpc_to_string(val));
                    key_modified = true;
                } else {
                    ROS_INFO_STREAM("[ApiRoutes] Parameter key added: " << key
                                    << " -> " << xmlrpc_to_string(val));
                }
            }

            if (key_modified) {
                destroy_task(key);
            }

            create_task(key, val);
        }
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("[ApiRoutes] Exception in setup_all_topic: " << e.what());
    } catch (...) {
        ROS_WARN("[ApiRoutes] Unknown exception in setup_all_topic");
    }
}

void ApiRoutesEngine::publish_routes_status() {
    try {
        if (!routes_status_pub_) return;

        nlohmann::json status_json;
        status_json["timestamp"] = ros::Time::now().toSec();
        status_json["device_code"] = device_code_.value_or("");
        status_json["ros_master_connected"] = ros::master::check();
        status_json["mqtt_connected"] = mqtt_connected_.load();

        nlohmann::json routes_obj = nlohmann::json::object();

        std::lock_guard<std::mutex> lock(routes_mutex_);
        status_json["active_routes_count"] = active_tasks_.size();

        for (const auto &pair : active_tasks_) {
            const std::string &key = pair.first;
            const auto &task = pair.second;
            if (!task) continue;

            nlohmann::json task_info;
            if (task->config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                for (auto it = task->config.begin(); it != task->config.end(); ++it) {
                    const std::string &field = it->first;
                    XmlRpc::XmlRpcValue &field_val = it->second;
                    if (field_val.getType() == XmlRpc::XmlRpcValue::TypeString) {
                        task_info[field] = static_cast<std::string>(field_val);
                    } else if (field_val.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        task_info[field] = static_cast<int>(field_val);
                    } else if (field_val.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                        task_info[field] = static_cast<bool>(field_val);
                    } else if (field_val.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        task_info[field] = static_cast<double>(field_val);
                    }
                }
            }

            if (task_info.contains("remote_uri") && task_info["remote_uri"].is_string()) {
                task_info["resolved_remote_uri"] = resolve_topic(remove_slash(task_info["remote_uri"].get<std::string>()));
            }

            if (!task->published_ros_topics.empty()) {
                nlohmann::json pub_stats = nlohmann::json::object();
                for (const auto &top : task->published_ros_topics) {
                    auto it = ros_pub_.find(top);
                    if (it != ros_pub_.end()) {
                        pub_stats[top] = {
                            {"subscribers_count", it->second.getNumSubscribers()}
                        };
                    }
                }
                task_info["published_topics"] = pub_stats;
            }

            if (!task->subscribed_ros_topics.empty()) {
                nlohmann::json sub_stats = nlohmann::json::object();
                for (const auto &top : task->subscribed_ros_topics) {
                    auto it = ros_sub_.find(top);
                    if (it != ros_sub_.end()) {
                        sub_stats[top] = {
                            {"publishers_count", it->second.getNumPublishers()}
                        };
                    }
                }
                task_info["subscribed_topics"] = sub_stats;
            }

            routes_obj[key] = task_info;
        }

        status_json["routes"] = routes_obj;

        std_msgs::String msg;
        msg.data = status_json.dump();
        routes_status_pub_.publish(msg);
    } catch (const std::exception &e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "[ApiRoutes] Exception in publish_routes_status: " << e.what());
    } catch (...) {}
}

void ApiRoutesEngine::register_mqtt_sub(std::shared_ptr<ApiRouteTask> task,
                                        std::string ros_topic,
                                        std::string mqtt_topic, int qos) {
    if (!device_code_.has_value()) {
        ROS_WARN("[ApiRoutes] Cannot register MQTT subscription: device_code_ is UNSET");
        return;
    }

    ros::Publisher pub;
    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        if (ros_pub_.find(ros_topic) != ros_pub_.end()) {
            ROS_WARN_STREAM("[ApiRoutes] ROS publisher for topic '" << ros_topic
                            << "' already exists, skipping advertise");
            return;
        }
        pub = nh_.advertise<std_msgs::String>(ros_topic, 1000);
        ros_pub_[ros_topic] = pub;
        if (task) task->published_ros_topics.push_back(ros_topic);
        ROS_INFO_STREAM("[ApiRoutes] Added ROS publisher: " << ros_topic);
    }

    mqtt_topic = resolve_topic(mqtt_topic);
    ROS_INFO_STREAM("[ApiRoutes] Added MQTT subscription: topic '" << mqtt_topic
                    << "' (qos=" << qos << ") -> forward to ROS topic '" << ros_topic << "'");

    std::weak_ptr<ApiRouteTask> weak_task = task;
    bool has_task = (task != nullptr);
    auto last_log_time = std::make_shared<ros::Time>(0);

    mqtt_adapter_->register_raw_handler(
        mqtt_topic,
        [pub, mqtt_topic, weak_task, has_task, last_log_time](const dk::MqttMessage &msg) {
            if (has_task && weak_task.expired()) {
                return;
            }
            ros::Time now = ros::Time::now();
            if ((now - *last_log_time).toSec() >= 1.0) {
                *last_log_time = now;
                std::string preview = msg.payload.substr(0, 100);
                ROS_INFO_STREAM("[ApiRoutes] MQTT received on topic '" << mqtt_topic
                                << "' (size=" << msg.payload.size() << "): "
                                << preview << (msg.payload.size() > 100 ? "..." : ""));
            }
            std_msgs::String ros_msg;
            ros_msg.data = msg.payload;
            pub.publish(ros_msg);
        },
        qos);
}

void ApiRoutesEngine::register_mqtt_pub(std::shared_ptr<ApiRouteTask> task,
                                        std::string ros_topic,
                                        std::string mqtt_topic, int qos,
                                        bool retain, bool is_state) {
    if (!device_code_.has_value()) return;

    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        if (ros_sub_.find(ros_topic) != ros_sub_.end()) return;
    }

    std::shared_ptr<StateDiffTracker> tracker;
    if (is_state) {
        tracker = std::make_shared<StateDiffTracker>();

        std::string cb_key = task ? task->key : ros_topic;
        if (task) task->callback_key = cb_key;

        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        reconnect_callbacks_[cb_key] =
            [this, mqtt_topic, qos, retain, tracker]() {
                try {
                    if (!tracker) return;
                    nlohmann::json last_state = tracker->get_last_state();
                    if (!last_state.empty()) {
                        publish_mqtt_msg(last_state, mqtt_topic, qos, retain);
                    }
                } catch (...) {}
            };

        state_heartbeat_callbacks_[cb_key] =
            [this, mqtt_topic, qos, retain, tracker]() {
                try {
                    if (!tracker) return;
                    nlohmann::json last_state = tracker->get_last_state();
                    if (!last_state.empty()) {
                        publish_mqtt_msg(last_state, mqtt_topic, qos, retain);
                    }
                } catch (...) {}
            };
    }

    auto sub = nh_.subscribe<std_msgs::String>(
        ros_topic, 1000,
        [this, mqtt_topic, qos, retain, ros_topic, is_state,
         tracker](const std_msgs::String::ConstPtr &msg) -> void {
            try {
                if (!msg) return;
                if (is_state) {
                    if (!tracker) return;
                    nlohmann::json current_json;
                    if (!parse_ros_msg(ros_topic, msg->data, current_json)) return;
                    nlohmann::json diff_json =
                        tracker->update_and_get_diff(current_json);
                    if (diff_json.empty()) return;
                    publish_mqtt_msg(diff_json, mqtt_topic, qos, retain);
                } else {
                    if (!mqtt_adapter_) return;
                    try {
                        nlohmann::json j = nlohmann::json::parse(msg->data);
                        if (j.is_object()) {
                            j["deviceCode"] = device_code_.value_or("");
                            mqtt_adapter_->publish(resolve_topic(mqtt_topic),
                                                   j.dump(), qos, retain);
                            return;
                        }
                    } catch (...) {
                        // 非合法 json 格式，跳过加入 deviceCode，直接原始发布
                    }
                    mqtt_adapter_->publish(resolve_topic(mqtt_topic), msg->data,
                                           qos, retain);
                }
            } catch (const std::exception &e) {
                ROS_WARN_STREAM("[ApiRoutes] Exception in topic sub callback: " << e.what());
            } catch (...) {}
        });

    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        ros_sub_[ros_topic] = sub;
        if (task) task->subscribed_ros_topics.push_back(ros_topic);
        ROS_INFO_STREAM("[ApiRoutes] Added ROS subscriber: " << ros_topic
                        << " -> forward to MQTT topic: " << resolve_topic(mqtt_topic));
    }
}

void ApiRoutesEngine::register_http_service_bridge(
    std::shared_ptr<ApiRouteTask> task, std::string ros_service,
    std::string http_path) {
    class HttpRosBridgeHandler : public dk::IProtocolHandler<WebAdapter> {
        std::string ros_service_;
        boost::asio::io_context &ioc_;

       public:
        HttpRosBridgeHandler(std::string service, boost::asio::io_context &ioc)
            : ros_service_(std::move(service)), ioc_(ioc) {}

        void handle(
            std::shared_ptr<dk::HttpSession<WebAdapter>> session,
            boost::beast::http::request<boost::beast::http::string_body> req)
            override {
            try {
                std::string req_str = req.body();
                std::thread([session, service_name = ros_service_, req_str,
                             &ioc = ioc_]() {
                    try {
                        api_routes::StringSrv srv;
                        srv.request.request = req_str;

                        if (ros::service::call(service_name, srv)) {
                            boost::asio::post(
                                ioc, [session, resp_str = srv.response.response]() {
                                    try {
                                        session->send_http_response(
                                            boost::beast::http::status::ok, resp_str);
                                    } catch (...) {}
                                });
                        } else {
                            boost::asio::post(ioc, [session]() {
                                try {
                                    session->send_http_response(
                                        boost::beast::http::status::
                                            internal_server_error,
                                        "{\"error\":\"ROS Service call failed\"}");
                                } catch (...) {}
                            });
                        }
                    } catch (const std::exception &e) {
                        boost::asio::post(ioc, [session, err = std::string(e.what())]() {
                            try {
                                session->send_http_response(
                                    boost::beast::http::status::
                                        internal_server_error,
                                    "{\"error\":\"" + err + "\"}");
                            } catch (...) {}
                        });
                    } catch (...) {}
                }).detach();
            } catch (const std::exception &e) {
                try {
                    session->send_http_response(
                        boost::beast::http::status::bad_request,
                        std::string("{\"error\":\"") + e.what() + "\"}");
                } catch (...) {}
            }
        }
    };

    std::string route_path = "/" + remove_slash(http_path);
    web_adapter_->register_handler(
        boost::beast::http::verb::post, route_path,
        std::make_shared<HttpRosBridgeHandler>(ros_service, get_ioc()));
    ROS_INFO_STREAM("[ApiRoutes] HTTP ROS Service Bridge registered: POST "
                    << route_path << " -> " << ros_service);
}

void ApiRoutesEngine::register_ws_sub(std::shared_ptr<ApiRouteTask> task,
                                      std::string ros_topic,
                                      std::string ws_path) {
    ros::Publisher pub;
    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        if (ros_pub_.find(ros_topic) != ros_pub_.end()) {
            ROS_ERROR_STREAM("[Websocket] rostopic " << ros_topic << " exists!");
            return;
        }
        pub = nh_.advertise<std_msgs::String>(ros_topic, 1000);
        ros_pub_[ros_topic] = pub;
        if (task) task->published_ros_topics.push_back(ros_topic);
    }

    std::string route_path = "/" + remove_slash(ws_path);
    auto last_log_time = std::make_shared<ros::Time>(0);
    web_adapter_->register_managed_ws_route(
        route_path,
        [pub, route_path, last_log_time](std::shared_ptr<dk::WsConnection> conn, std::string msg) {
            ros::Time now = ros::Time::now();
            if ((now - *last_log_time).toSec() >= 1.0) {
                *last_log_time = now;
                std::string preview = msg.substr(0, 100);
                ROS_INFO_STREAM("[ApiRoutes] WebSocket received on '" << route_path
                                << "' (size=" << msg.size() << "): "
                                << preview << (msg.size() > 100 ? "..." : ""));
            }
            std_msgs::String ros_msg;
            ros_msg.data = std::move(msg);
            pub.publish(ros_msg);
        });
    ROS_INFO_STREAM("[Websocket] ws->ros: " << route_path << " -> "
                                            << ros_topic);
}

void ApiRoutesEngine::register_ws_pub(std::shared_ptr<ApiRouteTask> task,
                                      std::string ros_topic,
                                      std::string ws_path, bool is_state) {
    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        if (ros_sub_.find(ros_topic) != ros_sub_.end()) {
            ROS_ERROR_STREAM("[Websocket] rostopic " << ros_topic << " exists!");
            return;
        }
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
        if (task) task->ws_open_routes.push_back(route_path);
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

    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        ros_sub_[ros_topic] = sub;
        if (task) task->subscribed_ros_topics.push_back(ros_topic);
        ROS_INFO_STREAM("[ApiRoutes] Added ROS subscriber: " << ros_topic
                        << " -> forward to WebSocket route: " << route_path);
    }
    ROS_INFO_STREAM("[Websocket] ros[state=" << is_state << "] -> ws: "
                                             << ros_topic << " -> "
                                             << route_path);
}

void ApiRoutesEngine::setup_dynamic_mqtt_bridge() {
    try {
        dynamic_mqtt_pub_topic_ =
            private_nh_.param("dynamic_mqtt_pub_topic", std::string("/api_routes/mqtt/send"));
        dynamic_mqtt_sub_topic_ =
            private_nh_.param("dynamic_mqtt_sub_topic", std::string("/api_routes/mqtt/recv"));

        dynamic_mqtt_sub_pub_ =
            nh_.advertise<std_msgs::String>(dynamic_mqtt_sub_topic_, 1000);
        dynamic_mqtt_pub_sub_ = nh_.subscribe<std_msgs::String>(
            dynamic_mqtt_pub_topic_, 1000, &ApiRoutesEngine::handle_dynamic_mqtt_send, this);

        ROS_INFO_STREAM("[DynamicMQTT] Bridge initialized: Subscribing to ROS '"
                        << dynamic_mqtt_pub_topic_
                        << "' -> forward to MQTT; Publishing MQTT -> ROS '"
                        << dynamic_mqtt_sub_topic_ << "'");

        if (private_nh_.hasParam("dynamic_mqtt_sub_topics")) {
            XmlRpc::XmlRpcValue sub_topics_val;
            private_nh_.getParam("dynamic_mqtt_sub_topics", sub_topics_val);
            if (sub_topics_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < sub_topics_val.size(); ++i) {
                    if (sub_topics_val[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                        configured_dynamic_sub_topics_.push_back(
                            static_cast<std::string>(sub_topics_val[i]));
                    }
                }
            } else if (sub_topics_val.getType() == XmlRpc::XmlRpcValue::TypeString) {
                std::string list_str = static_cast<std::string>(sub_topics_val);
                std::stringstream ss(list_str);
                std::string item;
                while (std::getline(ss, item, ',')) {
                    item.erase(0, item.find_first_not_of(" \t\n\r"));
                    item.erase(item.find_last_not_of(" \t\n\r") + 1);
                    if (!item.empty()) {
                        configured_dynamic_sub_topics_.push_back(item);
                    }
                }
            }
        }

        for (const auto &raw_top : configured_dynamic_sub_topics_) {
            if (raw_top.find('$') != std::string::npos && !device_code_.has_value()) {
                continue;
            }
            subscribe_dynamic_mqtt_topic(raw_top, 0);
        }
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[DynamicMQTT] Exception in setup_dynamic_mqtt_bridge: " << e.what());
    } catch (...) {
        ROS_ERROR("[DynamicMQTT] Unknown exception in setup_dynamic_mqtt_bridge!");
    }
}

void ApiRoutesEngine::handle_dynamic_mqtt_send(const std_msgs::String::ConstPtr &msg) {
    try {
        if (!msg || !mqtt_adapter_) return;

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(msg->data);
        } catch (const std::exception &e) {
            ROS_WARN_STREAM_THROTTLE(5.0, "[DynamicMQTT] Failed to parse JSON on "
                                          << dynamic_mqtt_pub_topic_ << ": " << e.what());
            return;
        }

        if (!j.is_object() || !j.contains("url")) {
            ROS_WARN_STREAM_THROTTLE(5.0, "[DynamicMQTT] Message on "
                                          << dynamic_mqtt_pub_topic_ << " must be a JSON object containing 'url'");
            return;
        }

        std::string raw_url = j["url"].get<std::string>();
        std::string target_topic = resolve_topic(remove_slash(raw_url));

        std::string data_str;
        if (j.contains("data")) {
            if (j["data"].is_string()) {
                data_str = j["data"].get<std::string>();
            } else {
                data_str = j["data"].dump();
            }
        }

        int qos = j.value("qos", 0);
        bool retain = j.value("retain", false);

        mqtt_adapter_->publish(target_topic, data_str, qos, retain);
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("[DynamicMQTT] Exception in handle_dynamic_mqtt_send: " << e.what());
    } catch (...) {}
}

void ApiRoutesEngine::subscribe_dynamic_mqtt_topic(const std::string &raw_topic, int qos) {
    std::string resolved_topic = resolve_topic(remove_slash(raw_topic));
    if (resolved_topic.empty()) return;

    {
        std::lock_guard<std::mutex> lock(dynamic_mqtt_mutex_);
        if (dynamic_subscribed_mqtt_topics_.find(resolved_topic) !=
            dynamic_subscribed_mqtt_topics_.end()) {
            return;
        }
        dynamic_subscribed_mqtt_topics_.insert(resolved_topic);
    }

    if (!mqtt_adapter_) return;

    ROS_INFO_STREAM("[DynamicMQTT] Subscribing to MQTT topic: " << resolved_topic
                    << " (qos=" << qos << ")");
    mqtt_adapter_->register_raw_handler(
        resolved_topic,
        [this](const dk::MqttMessage &msg) {
            forward_dynamic_mqtt_message(msg.topic, msg.payload, msg.qos);
        },
        qos);
}

void ApiRoutesEngine::forward_dynamic_mqtt_message(const std::string &topic,
                                                   const std::string &payload,
                                                   int qos) {
    try {
        nlohmann::json out;
        out["url"] = topic;
        out["data"] = payload;

        std_msgs::String ros_msg;
        ros_msg.data = out.dump();
        dynamic_mqtt_sub_pub_.publish(ros_msg);
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("[DynamicMQTT] Exception in forward_dynamic_mqtt_message: " << e.what());
    } catch (...) {}
}
