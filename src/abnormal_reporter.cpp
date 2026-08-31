#include "api_routes/abnormal_reporter.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

namespace fs = boost::filesystem;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                            void *userp) {
    size_t total_size = size * nmemb;
    auto *buf = static_cast<std::vector<uint8_t> *>(userp);
    const uint8_t *bytes = static_cast<const uint8_t *>(contents);
    buf->insert(buf->end(), bytes, bytes + total_size);
    return total_size;
}

AbnormalReporter::AbnormalReporter(
    ros::NodeHandle &nh, std::shared_ptr<MavlinkTelemetry> telemetry,
    std::function<std::string()> get_device_code_fn, Config config)
    : nh_(nh),
      telemetry_(telemetry),
      get_device_code_fn_(std::move(get_device_code_fn)),
      config_(std::move(config)) {}

void AbnormalReporter::start() {
    sub_ = nh_.subscribe(config_.ros_topic, 10,
                         &AbnormalReporter::onReportCallback, this);
    ROS_INFO_STREAM("[AbnormalReporter] Subscribed to "
                    << config_.ros_topic << " -> HTTP Target: "
                    << config_.cloud_host << ":" << config_.cloud_port
                    << config_.remote_path);
}

void AbnormalReporter::onReportCallback(const std_msgs::String::ConstPtr &msg) {
    std::thread(&AbnormalReporter::processAndUploadAsync, this, msg->data)
        .detach();
}

bool AbnormalReporter::downloadUrlToBuffer(const std::string &url,
                                           std::vector<uint8_t> &buffer) {
    CURL *curl = curl_easy_init();
    if (!curl) return false;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    return (res == CURLE_OK && http_code >= 200 && http_code < 300);
}

bool AbnormalReporter::readAndDeleteLocalFile(const std::string &file_path,
                                              std::vector<uint8_t> &buffer) {
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("[AbnormalReporter] Could not open local file: "
                         << file_path);
        return false;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    buffer.resize(size);

    if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
        ROS_ERROR_STREAM("[AbnormalReporter] Error reading file: " << file_path);
        file.close();
        return false;
    }
    file.close();

    boost::system::error_code ec;
    fs::remove(file_path, ec);
    if (ec) {
        ROS_WARN_STREAM("[AbnormalReporter] Failed to remove local file "
                        << file_path << ": " << ec.message());
    } else {
        ROS_INFO_STREAM("[AbnormalReporter] Read and deleted local file: "
                        << file_path);
    }
    return true;
}

void AbnormalReporter::processAndUploadAsync(std::string json_str) {
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(json_str);
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[AbnormalReporter] Failed to parse JSON: " << e.what());
        return;
    }

    if (!root.is_object()) {
        ROS_ERROR_STREAM("[AbnormalReporter] Received JSON is not an object.");
        return;
    }

    // Target HTTP URL
    std::string target_url;
    if (config_.remote_path.rfind("http://", 0) == 0 ||
        config_.remote_path.rfind("https://", 0) == 0) {
        target_url = config_.remote_path;
    } else {
        std::string path = config_.remote_path;
        if (!path.empty() && path.front() == '/') {
            path.erase(0, 1);
        }
        target_url = "http://" + config_.cloud_host + ":" +
                     std::to_string(config_.cloud_port) + "/" + path;
    }

    // Injected fields from telemetry and engine
    nlohmann::json state = telemetry_->getMergedState();
    std::string device_code = get_device_code_fn_();
    std::string map_coord =
        state.contains("mapLocation") ? state["mapLocation"].dump() : "[]";
    std::string gps_loc =
        state.contains("gpsLocation") ? state["gpsLocation"].dump() : "[]";

    CURL *curl = curl_easy_init();
    if (!curl) {
        ROS_ERROR("[AbnormalReporter] Failed to initialize curl handle");
        return;
    }

    curl_mime *mime = curl_mime_init(curl);

    // 1. Inject extra text fields
    auto add_text_field = [&](const std::string &name, const std::string &val) {
        curl_mimepart *part = curl_mime_addpart(mime);
        curl_mime_name(part, name.c_str());
        curl_mime_data(part, val.c_str(), CURL_ZERO_TERMINATED);
    };

    add_text_field("deviceCode", device_code);
    add_text_field("mapCoordinate", map_coord);
    add_text_field("gpsLocation", gps_loc);

    // Struct to hold file buffer memory during curl_easy_perform
    struct FileBufferHolder {
        std::vector<uint8_t> bytes;
        std::string filename;
    };
    std::vector<std::shared_ptr<FileBufferHolder>> file_holders;

    // 2. Iterate JSON keys
    for (auto &el : root.items()) {
        const std::string &key = el.key();
        const auto &val = el.value();

        if (key == "files") {
            std::vector<std::string> file_paths;
            if (val.is_array()) {
                for (const auto &item : val) {
                    if (item.is_string()) file_paths.push_back(item.get<std::string>());
                }
            } else if (val.is_string()) {
                file_paths.push_back(val.get<std::string>());
            }

            for (const auto &fpath : file_paths) {
                if (fpath.empty()) continue;
                auto holder = std::make_shared<FileBufferHolder>();

                bool success = false;
                if (fpath.rfind("http://", 0) == 0 || fpath.rfind("https://", 0) == 0) {
                    holder->filename = fs::path(fpath).filename().string();
                    if (holder->filename.empty()) holder->filename = "remote_file";
                    success = downloadUrlToBuffer(fpath, holder->bytes);
                } else {
                    holder->filename = fs::path(fpath).filename().string();
                    success = readAndDeleteLocalFile(fpath, holder->bytes);
                }

                if (success && !holder->bytes.empty()) {
                    file_holders.push_back(holder);
                    curl_mimepart *file_part = curl_mime_addpart(mime);
                    curl_mime_name(file_part, "files");
                    curl_mime_filename(file_part, holder->filename.c_str());
                    curl_mime_data(
                        file_part,
                        reinterpret_cast<const char *>(holder->bytes.data()),
                        holder->bytes.size());
                } else {
                    ROS_WARN_STREAM("[AbnormalReporter] Failed to fetch file content for: " << fpath);
                }
            }
        } else {
            // All other fields: convert to string and add as form-data field
            std::string text_val = val.is_string() ? val.get<std::string>() : val.dump();
            add_text_field(key, text_val);
        }
    }

    curl_easy_setopt(curl, CURLOPT_URL, target_url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        ROS_ERROR_STREAM("[AbnormalReporter] HTTP POST error ("
                         << target_url << "): " << curl_easy_strerror(res));
    } else {
        long status_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
        ROS_INFO_STREAM("[AbnormalReporter] Uploaded abnormal report to "
                        << target_url << " - HTTP Status: " << status_code);
    }

    curl_mime_free(mime);
    curl_easy_cleanup(curl);
}
