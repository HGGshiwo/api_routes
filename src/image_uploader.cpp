#include "api_routes/image_uploader.hpp"
#include <sensor_msgs/image_encodings.h>

ImageUploader::ImageUploader(ros::NodeHandle &nh, Config config)
    : nh_(nh), config_(std::move(config)) {}

void ImageUploader::start() {
  if (config_.topic_type == "raw") {
    sub_ = nh_.subscribe(config_.ros_topic, 10, &ImageUploader::on_raw_image, this);
  } else {
    sub_ = nh_.subscribe(config_.ros_topic, 10, &ImageUploader::on_compressed_image, this);
  }

  ROS_INFO_STREAM("[ImageUploader] Subscribed to '" << config_.ros_topic
                  << "' -> Target URL: " << config_.target_url
                  << " (type: " << config_.topic_type
                  << ", interval: " << config_.interval_ms << "ms)");
}

void ImageUploader::stop() {
  sub_.shutdown();
  ROS_INFO_STREAM("[ImageUploader] Stopped subscription on '" << config_.ros_topic << "'");
}

void ImageUploader::on_compressed_image(const sensor_msgs::CompressedImage::ConstPtr &msg) {
  ros::Time now = ros::Time::now();
  if (config_.interval_ms > 0 && last_upload_time_ != ros::Time(0) &&
      (now - last_upload_time_).toSec() * 1000.0 < config_.interval_ms) {
    return;
  }

  if (is_uploading_.exchange(true)) {
    return;
  }

  last_upload_time_ = now;

  std::string ext = ".jpg";
  std::string mime = "image/jpeg";
  if (msg->format.find("png") != std::string::npos || msg->format.find("PNG") != std::string::npos) {
    ext = ".png";
    mime = "image/png";
  }

  upload_async(msg->data, "image" + ext, mime);
}

void ImageUploader::on_raw_image(const sensor_msgs::Image::ConstPtr &msg) {
  ros::Time now = ros::Time::now();
  if (config_.interval_ms > 0 && last_upload_time_ != ros::Time(0) &&
      (now - last_upload_time_).toSec() * 1000.0 < config_.interval_ms) {
    return;
  }

  if (is_uploading_.exchange(true)) {
    return;
  }

  last_upload_time_ = now;

  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    std::vector<uint8_t> buf;
    if (cv::imencode(".jpg", cv_ptr->image, buf)) {
      upload_async(std::move(buf), "image.jpg", "image/jpeg");
    } else {
      ROS_ERROR_STREAM("[ImageUploader] Failed to encode raw image to JPEG format");
      is_uploading_ = false;
    }
  } catch (const cv_bridge::Exception &e) {
    ROS_ERROR_STREAM("[ImageUploader] cv_bridge exception: " << e.what());
    is_uploading_ = false;
  }
}

void ImageUploader::upload_async(std::vector<uint8_t> image_data, std::string filename, std::string mime_type) {
  std::thread([this, data = std::move(image_data), filename = std::move(filename), mime_type = std::move(mime_type)]() {
    CURL *curl = curl_easy_init();
    if (curl) {
      curl_mime *mime = curl_mime_init(curl);

      curl_mimepart *file_part = curl_mime_addpart(mime);
      curl_mime_name(file_part, config_.form_field_name.c_str());
      curl_mime_filename(file_part, filename.c_str());
      curl_mime_data(file_part, reinterpret_cast<const char *>(data.data()), data.size());
      curl_mime_type(file_part, mime_type.c_str());

      for (const auto &pair : config_.extra_fields) {
        curl_mimepart *extra_part = curl_mime_addpart(mime);
        curl_mime_name(extra_part, pair.first.c_str());
        curl_mime_data(extra_part, pair.second.c_str(), CURL_ZERO_TERMINATED);
      }

      curl_easy_setopt(curl, CURLOPT_URL, config_.target_url.c_str());
      curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
      curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
      curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);

      CURLcode res = curl_easy_perform(curl);
      if (res != CURLE_OK) {
        ROS_ERROR_STREAM("[ImageUploader] Upload error (" << config_.ros_topic
                         << " -> " << config_.target_url
                         << "): " << curl_easy_strerror(res));
      } else {
        long status_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
        if (status_code >= 200 && status_code < 300) {
          ROS_INFO_STREAM("[ImageUploader] Upload succeeded (" << config_.ros_topic
                          << " -> " << config_.target_url
                          << ") HTTP status: " << status_code);
        } else {
          ROS_WARN_STREAM("[ImageUploader] Upload (" << config_.ros_topic
                          << " -> " << config_.target_url
                          << ") returned HTTP status: " << status_code);
        }
      }

      curl_mime_free(mime);
      curl_easy_cleanup(curl);
    } else {
      ROS_ERROR_STREAM("[ImageUploader] Failed to initialize curl easy handle");
    }
    is_uploading_ = false;
  }).detach();
}
