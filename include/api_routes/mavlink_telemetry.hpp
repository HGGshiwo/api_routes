#pragma once

#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <atomic>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "api_routes/datum_synchronizer.hpp"

class MavlinkTelemetry {
   public:
    MavlinkTelemetry(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~MavlinkTelemetry() = default;

    // Get calculated MAVLink state merged with memory-stored /dank/status JSON
    nlohmann::json getMergedState();

    // Get current interpolated/raw GPS position JSON for HTTP /get_gps response
    nlohmann::json getGpsResponseJson();

    struct OdomDiag {
        std::string topic;
        int num_publishers{0};
        uint64_t msg_count{0};
        double pos_x{0.0};
        double pos_y{0.0};
        double pos_z{0.0};
        double age_sec{-1.0};
    };
    std::optional<nlohmann::json> getMapId();
    OdomDiag getOdomDiag();

    double getPublishRate() const { return publish_rate_; }

   private:
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void globalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void gpsRawCallback(const mavros_msgs::GPSRAW::ConstPtr &msg);
    void relAltCallback(const std_msgs::Float64::ConstPtr &msg);
    void dankStatusCallback(const std_msgs::String::ConstPtr &msg);

    void setupMavrosStreams(double rate);
    void computeGpsOut(double &lon_out, double &lat_out, double &alt_out);

    // ROS Subscribers
    ros::Subscriber sub_state_;
    ros::Subscriber sub_local_odom_;
    ros::Subscriber sub_global_gps_;
    ros::Subscriber sub_gps_raw_;
    ros::Subscriber sub_rel_alt_;
    ros::Subscriber sub_dank_status_;

    std::mutex data_mutex_;

    // Telemetry variables
    double pos_x_{0.0};
    double pos_y_{0.0};
    double pos_z_{0.0};
    double roll_{0.0};
    double pitch_{0.0};
    double yaw_{0.0};
    double lon_{0.0};
    double lat_{0.0};
    double alt_{0.0};
    double x_vel_{0.0};
    double y_vel_{0.0};
    double x_vel_body_{0.0};
    double y_vel_body_{0.0};
    bool connected_{false};
    double rel_alt_{0.0};
    int gps_nsats_{0};
    std::string mode_{"Unknown"};
    int gps_fix_type_{0};

    // Configuration
    bool use_degrees_{false};
    double publish_rate_{20.0};
    bool fcu_connected_{false};
    std::string odom_topic_{"/mavros/local_position/odom"};

    // Buffer for extra /dank/status JSON received from ROS topic
    nlohmann::json dank_status_json_;
    std::optional<nlohmann::json> map_id_;

    // Datum synchronizer
    DatumSynchronizer datum_sync_;
    std::optional<SyncedPair> last_reliable_datum_;

    // Diagnostics
    std::atomic<uint64_t> odom_msg_count_{0};
    ros::Time last_odom_stamp_{0};
};
