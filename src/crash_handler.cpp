#include "api_routes/crash_handler.hpp"

#include <csignal>
#include <cstring>
#include <ctime>
#include <exception>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>

char CrashHandler::log_path_[256] = "/tmp/api_routes_crash.log";

void CrashHandler::install(const std::string &log_path) {
    if (!log_path.empty()) {
        std::strncpy(log_path_, log_path.c_str(), sizeof(log_path_) - 1);
        log_path_[sizeof(log_path_) - 1] = '\0';
    }

    // 注册致命系统信号处理器
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = CrashHandler::signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESETHAND; // 触发一次后恢复默认，防止信号处理中死循环

    sigaction(SIGBUS, &sa, nullptr);   // -7: Bus Error (总线错误, 对齐/mmap截断)
    sigaction(SIGABRT, &sa, nullptr);  // -6: Abort (断言失败、堆损坏、std::terminate)
    sigaction(SIGSEGV, &sa, nullptr);  // -11: Segmentation Fault (段错误)
    sigaction(SIGFPE, &sa, nullptr);   // -8: Floating Point Exception (浮点错误)
    sigaction(SIGILL, &sa, nullptr);   // -4: Illegal Instruction (非法指令)

    // 注册 C++ 未捕获异常拦截器 (捕获逃逸异常详细信息，防止直接无声 abort)
    std::set_terminate(CrashHandler::terminateHandler);
}

void CrashHandler::writeCallstack(int fd, int sig, const char *extra_msg) {
    if (fd < 0) return;

    auto safe_write = [fd](const char *str) {
        if (!str) return;
        size_t len = 0;
        while (str[len] != '\0') ++len;
        (void)write(fd, str, len);
    };

    safe_write("\n=======================================================\n");
    safe_write("[CrashHandler] Process Crashed! Details:\n");

    // 获取并写入时间戳
    time_t now = time(nullptr);
    char time_buf[64];
    struct tm tm_info;
    if (localtime_r(&now, &tm_info)) {
        if (asctime_r(&tm_info, time_buf)) {
            safe_write("Timestamp: ");
            safe_write(time_buf);
        }
    }

    // 写入信号描述
    switch (sig) {
        case SIGBUS:
            safe_write("Signal: SIGBUS (exit code -7, Bus Error / Unaligned Memory / mmap failed)\n");
            break;
        case SIGABRT:
            safe_write("Signal: SIGABRT (exit code -6, Abort / Assert / Heap corruption)\n");
            break;
        case SIGSEGV:
            safe_write("Signal: SIGSEGV (exit code -11, Segmentation Fault / Null pointer)\n");
            break;
        case SIGFPE:
            safe_write("Signal: SIGFPE (exit code -8, Floating Point Exception)\n");
            break;
        case SIGILL:
            safe_write("Signal: SIGILL (exit code -4, Illegal Instruction)\n");
            break;
        default:
            safe_write("Signal: Unknown fatal signal\n");
            break;
    }

    if (extra_msg && extra_msg[0] != '\0') {
        safe_write("Exception Info: ");
        safe_write(extra_msg);
        safe_write("\n");
    }

    safe_write("\nBacktrace / Callstack:\n");
    void *callstack[128];
    int frames = backtrace(callstack, 128);
    backtrace_symbols_fd(callstack, frames, fd);
    safe_write("=======================================================\n\n");
}

void CrashHandler::signalHandler(int sig) {
    // 1. 输出到终端标准错误 STDERR
    writeCallstack(STDERR_FILENO, sig);

    // 2. 以覆写方式 (O_TRUNC) 写入专用崩溃日志文件，保留最新一次崩溃信息
    int fd = open(log_path_, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        writeCallstack(fd, sig);
        close(fd);
    }

    // 退出进程，退出码设置为 128 + 信号编号 (POSIX 习惯)
    _exit(128 + sig);
}

void CrashHandler::terminateHandler() {
    char ex_buf[512] = {0};
    try {
        std::exception_ptr e = std::current_exception();
        if (e) {
            std::rethrow_exception(e);
        } else {
            std::strncpy(ex_buf, "std::terminate called without active exception", sizeof(ex_buf) - 1);
        }
    } catch (const std::exception &ex) {
        std::strncpy(ex_buf, ex.what(), sizeof(ex_buf) - 1);
    } catch (...) {
        std::strncpy(ex_buf, "Unknown non-standard C++ exception caught in std::terminate", sizeof(ex_buf) - 1);
    }

    // 输出到 stderr
    writeCallstack(STDERR_FILENO, SIGABRT, ex_buf);

    // 覆写至崩溃日志文件
    int fd = open(log_path_, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        writeCallstack(fd, SIGABRT, ex_buf);
        close(fd);
    }

    _exit(128 + SIGABRT);
}
