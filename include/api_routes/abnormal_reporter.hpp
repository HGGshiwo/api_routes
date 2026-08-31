#pragma once

#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "api_routes/mavlink_telemetry.hpp"

class AbnormalReporter {
   public:
    struct Config {
        std::string ros_topic = "/abnormal/report";
        std::string cloud_host = "localhost";
        int cloud_port = 8000;
        std::string remote_path = "/abnormal/report";
    };

    AbnormalReporter(ros::NodeHandle &nh,
                     std::shared_ptr<MavlinkTelemetry> telemetry,
                     std::function<std::string()> get_device_code_fn,
                     Config config);
    ~AbnormalReporter() = default;

    void start();

   private:
    void onReportCallback(const std_msgs::String::ConstPtr &msg);
    void processAndUploadAsync(std::string json_str);

    bool downloadUrlToBuffer(const std::string &url, std::vector<uint8_t> &buffer);
    bool readAndDeleteLocalFile(const std::string &file_path, std::vector<uint8_t> &buffer);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::shared_ptr<MavlinkTelemetry> telemetry_;
    std::function<std::string()> get_device_code_fn_;
    Config config_;
};
