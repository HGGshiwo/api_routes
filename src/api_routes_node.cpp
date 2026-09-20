#include <thread>
#include <ros/service.h>
#include "api_routes/api_routes_engine.hpp"
#include "api_routes/crash_handler.hpp"

class ApiRoutesNode {
    ros::NodeHandle nh_;
    boost::asio::io_context ioc_;
    std::shared_ptr<ApiRoutesEngine> engine_;
    std::thread asio_thread_;

   public:
    ApiRoutesNode() {
        auto time_provider = std::make_shared<dk::RosTimeProvider>(nh_, ioc_);
        engine_ = std::make_shared<ApiRoutesEngine>(ioc_, time_provider);

        // Tick engine at 20 Hz (50ms interval) to match MAVLink publish rate
        engine_->start(std::chrono::milliseconds(50));

        asio_thread_ = std::thread([this]() {
            auto work_guard = boost::asio::make_work_guard(ioc_);
            ioc_.run();
        });
        ROS_INFO("[ApiRoutes] ApiRoutesNode initialized and started successfully.");
    }

    ~ApiRoutesNode() {
        stop();
    }

    void stop() {
        // 1. 显式先停掉 Engine（取消所有内部 Ticker/定时器与事件分发），防止后台线程竞争与析构调用纯虚函数
        if (engine_) {
            try {
                engine_->stop();
            } catch (...) {}
        }
        // 2. 停止 Asio 事件循环并安全等待线程退出
        ioc_.stop();
        if (asio_thread_.joinable()) {
            asio_thread_.join();
        }
        // 3. 在所有后台线程安全退出后释放 engine 资源
        engine_.reset();
    }
};

int main(int argc, char **argv) {
    // 注册全局致命崩溃信号及未捕获异常追踪器，覆盖输出至 /tmp/api_routes_crash.log
    CrashHandler::install("/tmp/api_routes_crash.log");

    try {
        ros::init(argc, argv, ROSNODE_NAME);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        // 检查并等待 rosout 服务准备就绪，确保启动极早期的参数与路由日志可靠进入 rosout.log
        ros::WallTime start_wait = ros::WallTime::now();
        while (ros::ok() && !ros::service::exists("/rosout/get_loggers", true)) {
            if ((ros::WallTime::now() - start_wait).toSec() > 3.0) {
                break;
            }
            ros::WallDuration(0.05).sleep();
        }

        try {
            auto node = std::make_unique<ApiRoutesNode>();
            ros::waitForShutdown();
            if (node) {
                node->stop();
                node.reset();
            }
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM("[ApiRoutes] Exception in ApiRoutesNode lifecycle: " << e.what());
        } catch (...) {
            ROS_ERROR("[ApiRoutes] Unknown exception in ApiRoutesNode lifecycle!");
        }

        spinner.stop();
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[ApiRoutes] Fatal exception in main: " << e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("[ApiRoutes] Unknown fatal exception in main!");
        return 1;
    }
    return 0;
}
