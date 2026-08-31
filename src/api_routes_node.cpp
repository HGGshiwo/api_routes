#include <thread>
#include "api_routes/api_routes_engine.hpp"

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
        ioc_.stop();
        if (asio_thread_.joinable()) {
            asio_thread_.join();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, ROSNODE_NAME);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ApiRoutesNode node;

    ros::waitForShutdown();
    return 0;
}
