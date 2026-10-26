#pragma once

#include <ros/ros.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

extern char** environ;

// 通过子进程 rosbag record 实现录制:
// api_routes 只负责子进程的生命周期 (启动/发 SIGINT/收尸), 订阅、缓冲、写盘、
// 收尾全部由 rosbag record 在独立进程内完成。
// 好处: 录制的磁盘 I/O 与线程模型和 api_routes 完全隔离 (不占 spinner 线程,
// 不存在与引擎线程的锁交互), 且天然支持 rosbag record 的全部能力。
class BagRecorder {
   public:
    explicit BagRecorder(ros::NodeHandle nh) : nh_(nh) {
        ros::NodeHandle pnh("~");
        // 可选: 显式指定 rosbag record 可执行文件路径 (默认自动从 ROS_ROOT
        // 解析)
        pnh.param<std::string>("rosbag_record_bin", record_bin_, "");
    }

    ~BagRecorder() { stop(); }

    BagRecorder(const BagRecorder&) = delete;
    BagRecorder& operator=(const BagRecorder&) = delete;

    // 开始录制, prefix 为 bag 文件名前缀 (可为空)。
    // compression: "lz4" / "bz2" / 其他 = 不压缩; split_mb > 0 时按大小分片。
    // 返回空串表示成功, 否则为错误信息。
    std::string start(const std::string& prefix, const std::string& dir,
                      const std::vector<std::string>& topics,
                      const std::string& compression = "", int split_mb = 0) {
        if (rec_pid_ > 0) return "已在录制中!";
        if (topics.empty()) return "record_topics 未配置, 无法开始录制!";

        // 解析录制目录: 绝对路径直接使用; 相对路径锚定在 ROS_HOME
        // (默认 ~/.ros) 下, 不随节点的启动目录漂移
        std::filesystem::path dir_path(dir);
        if (dir_path.is_relative()) {
            const char* ros_home = std::getenv("ROS_HOME");
            std::filesystem::path home =
                ros_home
                    ? std::filesystem::path(ros_home)
                    : (std::getenv("HOME")
                           ? std::filesystem::path(std::getenv("HOME")) / ".ros"
                           : std::filesystem::path("."));
            dir_path = home / dir_path;
        }

        std::error_code ec;
        std::filesystem::create_directories(dir_path, ec);
        if (ec) {
            return "创建录制目录 " + dir_path.string() +
                   " 失败: " + ec.message();
        }

        // rosbag record -O <base> 会生成 <base>.bag, bag 文件名带毫秒,
        // 防止快速停止后立刻重新开始时同名覆盖
        std::string base = (dir_path / ((prefix.empty() ? "record" : prefix) +
                                        "_" + timestamp()))
                               .string();
        std::string bag_file = base + ".bag";

        std::string bin = record_bin_;
        if (bin.empty()) {
            const char* ros_root = std::getenv("ROS_ROOT");
            if (ros_root) {
                std::filesystem::path p =
                    std::filesystem::path(ros_root).parent_path() / "lib" /
                    "rosbag" / "record";
                if (std::filesystem::exists(p)) bin = p.string();
            }
        }
        const bool use_rosrun = bin.empty();  // 兜底: 依赖 PATH 里的 rosrun

        std::vector<std::string> args;
        if (use_rosrun) {
            args = {"rosrun", "rosbag", "record", "-O", base};
        } else {
            args = {bin, "-O", base};
        }
        if (compression == "lz4") {
            args.push_back("--lz4");
        } else if (compression == "bz2") {
            args.push_back("--bz2");
        }
        if (split_mb > 0) {
            args.push_back("--split");
            args.push_back("--size");
            args.push_back(std::to_string(split_mb));
        }
        for (const auto& topic : topics) args.push_back(topic);

        std::vector<char*> argv;
        for (auto& s : args) argv.push_back(const_cast<char*>(s.c_str()));
        argv.push_back(nullptr);

        // 子进程放进独立进程组, 停止时可对整个进程组发 SIGINT
        // (兜底的 rosrun 是脚本包装, 需要 Kill 组才能传到真正的 record)
        posix_spawnattr_t attr;
        posix_spawnattr_init(&attr);
        posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
        posix_spawnattr_setpgroup(&attr, 0);

        pid_t pid = 0;
        int rc = use_rosrun ? posix_spawnp(&pid, "rosrun", nullptr, &attr,
                                           argv.data(), environ)
                            : posix_spawn(&pid, bin.c_str(), nullptr, &attr,
                                          argv.data(), environ);
        posix_spawnattr_destroy(&attr);
        if (rc != 0) {
            return "启动 rosbag record 失败: " + std::string(strerror(rc));
        }

        // 快速失败检测 (可执行文件缺失/参数错误等): 200ms 后若已退出则报错
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        int status = 0;
        if (waitpid(pid, &status, WNOHANG) == pid) {
            return "rosbag record 启动即退出 (status=" +
                   std::to_string(status) + "), 请检查话题列表与磁盘空间";
        }

        rec_pid_ = pid;
        bag_path_ = bag_file;
        ROS_INFO("[BagRecorder] start recording %zu topics -> %s (pid=%d)",
                 topics.size(), bag_file.c_str(), pid);
        return "";
    }

    // 停止录制: 对子进程组发 SIGINT, rosbag record 会自己优雅关闭 bag 退出。
    // 收尸在独立线程完成, 不阻塞引擎线程。返回空串表示成功, 否则为错误信息。
    std::string stop() {
        pid_t pid = rec_pid_.exchange(-1);
        if (pid <= 0) return "当前没有在录制!";

        kill(-pid, SIGINT);
        std::thread([pid, path = bag_path_] {
            int status = 0;
            waitpid(pid, &status, 0);
            ROS_INFO("[BagRecorder] rosbag record (pid=%d) exited, bag saved to %s",
                     pid, path.c_str());
        }).detach();
        return "";
    }

    std::string bag_path() const { return bag_path_; }

   private:
    static std::string timestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch())
                      .count() %
                  1000;
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        return std::string(buf) + "_" + std::to_string(ms);
    }

    ros::NodeHandle nh_;
    std::string record_bin_;
    std::atomic<pid_t> rec_pid_{-1};  // 子进程 pid, -1 表示未在录制
    std::string bag_path_;            // 仅引擎线程读写
};
