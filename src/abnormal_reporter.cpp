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
    ROS_INFO_STREAM("[AbnormalReporter] Received ROS message on "
                    << config_.ros_topic << " (payload length: "
                    << (msg ? msg->data.size() : 0) << " bytes)");
    std::thread(&AbnormalReporter::processAndUploadAsync, this, msg->data)
        .detach();
}

bool AbnormalReporter::downloadUrlToBuffer(const std::string &url,
                                           std::vector<uint8_t> &buffer) {
    ROS_INFO_STREAM("[AbnormalReporter] Downloading remote file: " << url);
    CURL *curl = curl_easy_init();
    if (!curl) {
        ROS_ERROR_STREAM("[AbnormalReporter] Failed to init curl handle for download: " << url);
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK || http_code < 200 || http_code >= 300) {
        ROS_WARN_STREAM("[AbnormalReporter] Download failed for "
                        << url << " (curl error: " << curl_easy_strerror(res)
                        << ", HTTP status: " << http_code << ")");
        return false;
    }
    ROS_INFO_STREAM("[AbnormalReporter] Successfully downloaded " << url
                    << " (" << buffer.size() << " bytes)");
    return true;
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
    ROS_INFO_STREAM("[AbnormalReporter] Starting async process and upload...");
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(json_str);
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[AbnormalReporter] Failed to parse JSON: " << e.what()
                         << " | Payload: " << json_str);
        return;
    }

    if (!root.is_object()) {
        ROS_ERROR_STREAM("[AbnormalReporter] Received JSON is not an object. Payload: " << json_str);
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

    ROS_INFO_STREAM("[AbnormalReporter] Target HTTP URL: " << target_url);

    // Injected fields from telemetry and engine
    ROS_INFO_STREAM("[AbnormalReporter] Fetching telemetry state & device code...");
    nlohmann::json state = telemetry_->getMergedState();
    std::string device_code = get_device_code_fn_();
    ROS_INFO_STREAM("[AbnormalReporter] Injected deviceCode: " << device_code);

    // Directly inject extra fields into root JSON object (nlohmann::json auto-deduplicates keys)
    root["deviceCode"] = device_code;
    root["mapCoordinate"] = state.contains("mapLocation") ? state["mapLocation"] : nlohmann::json::array();
    root["gpsLocation"] = state.contains("gpsLocation") ? state["gpsLocation"] : nlohmann::json::array();
    if (state.contains("mapId")) {
        root["mapId"] = state["mapId"];
        ROS_INFO_STREAM("[AbnormalReporter] Injected mapId: " << root["mapId"]);
    }

    CURL *curl = curl_easy_init();
    if (!curl) {
        ROS_ERROR("[AbnormalReporter] Failed to initialize curl handle");
        return;
    }

    curl_mime *mime = curl_mime_init(curl);

    // Helper to add form-data text field
    auto add_text_field = [&](const std::string &name, const std::string &val) {
        curl_mimepart *part = curl_mime_addpart(mime);
        curl_mime_name(part, name.c_str());
        curl_mime_data(part, val.c_str(), CURL_ZERO_TERMINATED);
    };

    // Struct to hold file buffer memory during curl_easy_perform
    struct FileBufferHolder {
        std::vector<uint8_t> bytes;
        std::string filename;
    };
    std::vector<std::shared_ptr<FileBufferHolder>> file_holders;

    // Iterate JSON keys (all keys are now unique and properly typed)
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
                ROS_INFO_STREAM("[AbnormalReporter] Processing file attachment: " << fpath);
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
                    ROS_INFO_STREAM("[AbnormalReporter] Attached file: "
                                    << holder->filename << " ("
                                    << holder->bytes.size() << " bytes)");
                } else {
                    ROS_WARN_STREAM("[AbnormalReporter] Failed to fetch file content for: " << fpath);
                }
            }
        } else if (val.is_array()) {
            // For JSON array fields: expand array items into multiple form parts with the same key
            for (const auto &item : val) {
                std::string item_str = item.is_string() ? item.get<std::string>() : item.dump();
                add_text_field(key, item_str);
            }
        } else {
            // For scalar fields: convert to string and add as form part
            std::string text_val = val.is_string() ? val.get<std::string>() : val.dump();
            add_text_field(key, text_val);
        }
    }

    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Expect:");

    curl_easy_setopt(curl, CURLOPT_URL, target_url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "AbnormalReporter/1.0");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    ROS_INFO_STREAM("[AbnormalReporter] Performing HTTP POST to " << target_url << " ...");

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

    if (headers) {
        curl_slist_free_all(headers);
    }
    curl_mime_free(mime);
    curl_easy_cleanup(curl);
    ROS_INFO_STREAM("[AbnormalReporter] Async process and upload completed.");
}
