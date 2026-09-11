#pragma once

#include <string>

/**
 * @brief CrashHandler 捕获致命信号（SIGBUS, SIGABRT, SIGSEGV, SIGFPE, SIGILL）以及未捕获的 C++ 异常，
 *        将调用栈和故障原因输出到 stderr，并以覆写（修改非追加）模式写入指定的日志文件。
 */
class CrashHandler {
   public:
    /**
     * @brief 安装全局崩溃捕获器
     * @param log_path 崩溃日志文件路径，默认为 /tmp/api_routes_crash.log。
     *                 采用 O_TRUNC 覆写模式，每次崩溃只保留最新一次记录。
     */
    static void install(const std::string &log_path = "/tmp/api_routes_crash.log");

   private:
    static void signalHandler(int sig);
    static void terminateHandler();
    static void writeCallstack(int fd, int sig, const char *extra_msg = nullptr);

    static char log_path_[256];
};
