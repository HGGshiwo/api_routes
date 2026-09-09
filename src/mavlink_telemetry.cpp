#include "api_routes/mavlink_telemetry.hpp"

#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/StreamRate.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <map>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



MavlinkTelemetry::MavlinkTelemetry(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    pnh.param<double>("publish_rate", publish_rate_, 20.0);
    pnh.param<bool>("use_degrees", use_degrees_, false);
    pnh.param<std::string>("odom_topic", odom_topic_,
                           "/mavros/local_position/odom");

    if (publish_rate_ <= 0.0) {
        ROS_WARN(
            "Invalid publish_rate specified: %f. Resetting to default 20.0 Hz.",
            publish_rate_);
        publish_rate_ = 20.0;
    }

    sub_state_ =
        nh.subscribe("/mavros/state", 10, &MavlinkTelemetry::stateCallback, this);
    sub_local_odom_ = nh.subscribe(odom_topic_, 10,
                                   &MavlinkTelemetry::localOdomCallback, this);
    sub_global_gps_ = nh.subscribe("/mavros/global_position/global", 10,
                                   &MavlinkTelemetry::globalGpsCallback, this);
    sub_gps_raw_ = nh.subscribe("/mavros/gpsstatus/gps1/raw", 10,
                                &MavlinkTelemetry::gpsRawCallback, this);
    sub_rel_alt_ = nh.subscribe("/mavros/global_position/rel_alt", 10,
                                &MavlinkTelemetry::relAltCallback, this);

    sub_dank_status_ = nh.subscribe("/dank/status", 10,
                                    &MavlinkTelemetry::dankStatusCallback, this);

    ROS_INFO("[MavlinkTelemetry] Initialized. Subscribed to MAVROS topics and /dank/status.");
}

void MavlinkTelemetry::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    bool connected = msg->connected;
    bool was_connected = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        was_connected = fcu_connected_;
        fcu_connected_ = connected;
        connected_ = connected;
        mode_ = msg->mode;
    }

    if (connected && !was_connected) {
        ROS_INFO(
            "FCU connected. Requesting stream rates and message intervals...");
        std::thread(&MavlinkTelemetry::setupMavrosStreams, this, publish_rate_)
            .detach();
    }
}

void MavlinkTelemetry::localOdomCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
    odom_msg_count_++;
    last_odom_stamp_ = msg->header.stamp;

    std::lock_guard<std::mutex> lock(data_mutex_);

    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    if (use_degrees_) {
        roll_ = r * 180.0 / M_PI;
        pitch_ = p * 180.0 / M_PI;
        yaw_ = y * 180.0 / M_PI;
    } else {
        roll_ = r;
        pitch_ = p;
        yaw_ = y;
    }

    yaw_ = -yaw_ + M_PI * 0.5;  // ENU -> NED

    x_vel_body_ = msg->twist.twist.linear.x;
    y_vel_body_ = msg->twist.twist.linear.y;

    tf2::Vector3 v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    tf2::Vector3 v_global = tf2::quatRotate(q, v_body);
    x_vel_ = v_global.getX();
    y_vel_ = v_global.getY();

    double time_now = msg->header.stamp.toSec();
    datum_sync_.pushENU(Eigen::Vector3d(pos_x_, pos_y_, pos_z_), time_now);

    auto opt_datum = datum_sync_.getReliableDatum();
    if (opt_datum.has_value()) {
        last_reliable_datum_ = opt_datum;
    }
}

void MavlinkTelemetry::globalGpsCallback(
    const sensor_msgs::NavSatFix::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    lon_ = msg->longitude;
    lat_ = msg->latitude;
    alt_ = msg->altitude;

    if (gps_fix_type_ >= 3) {
        double time_now = msg->header.stamp.toSec();
        datum_sync_.pushGPS(Eigen::Vector3d(lon_, lat_, alt_), time_now);

        auto opt_datum = datum_sync_.getReliableDatum();
        if (opt_datum.has_value()) {
            last_reliable_datum_ = opt_datum;
        }
    }
}

void MavlinkTelemetry::gpsRawCallback(
    const mavros_msgs::GPSRAW::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    gps_fix_type_ = msg->fix_type;
    gps_nsats_ = msg->satellites_visible;
}

void MavlinkTelemetry::relAltCallback(const std_msgs::Float64::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rel_alt_ = msg->data;
}

void MavlinkTelemetry::dankStatusCallback(
    const std_msgs::String::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    try {
        dank_status_json_ = nlohmann::json::parse(msg->data);
        if (dank_status_json_.contains("mapId")) {
            map_id_ = dank_status_json_["mapId"];
        }
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[MavlinkTelemetry] Failed to parse /dank/status JSON: "
                         << e.what());
    }
}

void MavlinkTelemetry::computeGpsOut(double &lon_out, double &lat_out,
                                     double &alt_out) {
    lon_out = lon_;
    lat_out = lat_;
    alt_out = alt_;

    if (gps_fix_type_ < 3 && last_reliable_datum_.has_value()) {
        const double R_e = 6378137.0;
        const double PI = 3.14159265358979323846;
        const auto &datum = last_reliable_datum_.value();

        double lat_ref_rad = datum.gps.y() * PI / 180.0;
        double delta_x = pos_x_ - datum.enu.x();
        double delta_y = pos_y_ - datum.enu.y();
        double delta_z = pos_z_ - datum.enu.z();

        lat_out = datum.gps.y() + (delta_y / R_e) * (180.0 / PI);
        lon_out = datum.gps.x() +
                  (delta_x / (R_e * std::cos(lat_ref_rad))) * (180.0 / PI);
        alt_out = datum.gps.z() + delta_z;
    }
}

nlohmann::json MavlinkTelemetry::getMergedState() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    double lon_out = 0.0;
    double lat_out = 0.0;
    double alt_out = 0.0;
    computeGpsOut(lon_out, lat_out, alt_out);

    nlohmann::json j;

    // 获取到有效值才注入（云端要求）
    if(std::hypot(lon_out, lat_out) > 0.01) {
        j["gpsLocation"] = {lon_out, lat_out, alt_out};
    }
    
    j["mapLocation"] = {pos_x_, pos_y_, pos_z_};
    j["roll"] = roll_;
    j["pitch"] = pitch_;
    j["yaw"] = yaw_;
    
    j["gps_fix_type"] = gps_fix_type_;
    j["xVel"] = x_vel_;
    j["yVel"] = y_vel_;
    j["xVelBody"] = x_vel_body_;
    j["yVelBody"] = y_vel_body_;
    j["connected"] = connected_;
    j["relAlt"] = rel_alt_;
    j["gpsNsats"] = gps_nsats_;
    j["mode"] = mode_;

    // Merge in extra /dank/status JSON unconditionally if available
    if (!dank_status_json_.empty() && dank_status_json_.is_object()) {
        for (auto &el : dank_status_json_.items()) {
            j[el.key()] = el.value();
        }
    }

    if (map_id_.has_value()) {
        j["mapId"] = map_id_.value();
    }

    return j;
}

std::optional<nlohmann::json> MavlinkTelemetry::getMapId() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return map_id_;
}

nlohmann::json MavlinkTelemetry::getGpsResponseJson() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    double lon_out = 0.0;
    double lat_out = 0.0;
    double alt_out = 0.0;
    computeGpsOut(lon_out, lat_out, alt_out);

    nlohmann::json res;
    res["msg"] = {lon_out, lat_out, alt_out};
    res["status"] = "success";
    return res;
}

void MavlinkTelemetry::setupMavrosStreams(double rate) {
    ros::NodeHandle nh;

    ROS_INFO("setupMavrosStreams: Waiting for MAVROS services to start...");

    if (ros::service::waitForService("/mavros/set_stream_rate",
                                     ros::Duration(30.0))) {
        ros::ServiceClient client = nh.serviceClient<mavros_msgs::StreamRate>(
            "/mavros/set_stream_rate");
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = 0;
        srv.request.message_rate = static_cast<uint16_t>(rate);
        srv.request.on_off = true;
        if (client.call(srv)) {
            ROS_INFO(
                "Successfully requested all MAVROS streams at %.1f Hz via "
                "set_stream_rate.",
                rate);
        } else {
            ROS_WARN("Failed to call MAVROS service /mavros/set_stream_rate.");
        }
    } else {
        ROS_WARN(
            "MAVROS service /mavros/set_stream_rate not available (timeout).");
    }

    if (ros::service::waitForService("/mavros/set_message_interval",
                                     ros::Duration(10.0))) {
        ros::ServiceClient client =
            nh.serviceClient<mavros_msgs::MessageInterval>(
                "/mavros/set_message_interval");

        auto set_interval = [&](uint32_t msg_id, float msg_rate) {
            mavros_msgs::MessageInterval srv;
            srv.request.message_id = msg_id;
            srv.request.message_rate = msg_rate;
            if (client.call(srv) && srv.response.success) {
                ROS_INFO(
                    "Successfully set message interval for message ID %u to "
                    "%.1f Hz.",
                    msg_id, msg_rate);
            } else {
                ROS_WARN("Failed to set message interval for message ID %u.",
                         msg_id);
            }
        };

        set_interval(30, static_cast<float>(rate));   // ATTITUDE
        set_interval(32, static_cast<float>(rate));   // LOCAL_POSITION_NED
        set_interval(132, static_cast<float>(rate));  // DISTANCE_SENSOR
    } else {
        ROS_WARN(
            "MAVROS service /mavros/set_message_interval not available "
            "(timeout).");
    }
}

MavlinkTelemetry::OdomDiag MavlinkTelemetry::getOdomDiag() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    OdomDiag diag;
    diag.topic = odom_topic_;
    diag.num_publishers = sub_local_odom_.getNumPublishers();
    diag.msg_count = odom_msg_count_.load();
    diag.pos_x = pos_x_;
    diag.pos_y = pos_y_;
    diag.pos_z = pos_z_;
    if (!last_odom_stamp_.isZero()) {
        diag.age_sec = (ros::Time::now() - last_odom_stamp_).toSec();
    }
    return diag;
}
