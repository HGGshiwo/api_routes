#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

class ImageUploader {
public:
  struct Config {
    std::string ros_topic;
    std::string target_url;
    std::string topic_type = "compressed";
    std::string form_field_name = "file";
    int interval_ms = 1000;
    std::map<std::string, std::string> extra_fields;
  };

  ImageUploader(ros::NodeHandle &nh, Config config);
  ~ImageUploader() = default;

  void start();

private:
  void on_compressed_image(const sensor_msgs::CompressedImage::ConstPtr &msg);
  void on_raw_image(const sensor_msgs::Image::ConstPtr &msg);
  void upload_async(std::vector<uint8_t> image_data, std::string filename, std::string mime_type);

  ros::NodeHandle nh_;
  Config config_;
  ros::Subscriber sub_;
  ros::Time last_upload_time_{0};
  std::atomic<bool> is_uploading_{false};
};
