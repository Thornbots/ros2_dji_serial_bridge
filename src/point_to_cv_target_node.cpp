// point_to_cv_target_node.cpp
//
// Bridges the vision pipeline's 3-D target estimate to the DJI serial
// bridge's CV_MSG channel. This is the missing link between
// roi_depth_query/roi_depth_node and dji_serial_bridge_node — without it,
// the two packages have no node that knows how to translate one's output
// message type/frame convention into the other's input.
//
// Subscribes:
//   point_topic       (geometry_msgs/PointStamped) — REP-103 camera body
//                      frame (X forward, Y left, Z up). Default "roi_point",
//                      published by roi_depth_query/roi_depth_node.
//   confidence_topic   (vision_msgs/Detection2D, optional) — read only for a
//                      confidence score (max of all hypothesis scores, same
//                      rule detection_picker_node uses). Default "roi".
//
// Publishes:
//   output_topic       (dji_serial_bridge/msg/CVTarget) — the bridge's
//                      camera-frame convention (X right, Y up, Z forward),
//                      see CVTarget.msg. Default "cv_target"; remap this (or
//                      set the parameter) to match dji_serial_bridge_node's
//                      "~/cv_target" subscription, e.g. via dji_bridge.launch.py.
//
// Frame conversion (REP-103 -> CVTarget convention):
//   cv.x =  -p.y   (right    = -left)
//   cv.y =   p.z   (up       =  up)
//   cv.z =   p.x   (forward  =  forward)
//
// Velocity / acceleration:
//   roi_depth_node only publishes position. When estimate_velocity is true
//   (default), v_x/v_y/v_z and a_x/a_y/a_z are estimated by finite-
//   differencing consecutive PointStamped samples (using the message
//   timestamps, not wall-clock arrival time) and smoothed with a simple
//   exponential moving average (velocity_filter_alpha). This is a coarse
//   estimate, not a proper tracker/filter — if you already have a tracked
//   velocity upstream, publish it separately and set
//   estimate_velocity:=false to leave those fields at zero.
//
// Stale-target watchdog:
//   If no new point arrives for target_timeout_s, a single zero-confidence
//   CVTarget is published (so the MCB/gimbal can stop tracking a ghost
//   target) and the velocity filter resets; publishing resumes cleanly on
//   the next fresh point.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include "dji_serial_bridge/msg/cv_target.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace dji_serial_bridge
{

class PointToCvTargetNode : public rclcpp::Node
{
public:
    explicit PointToCvTargetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("point_to_cv_target", options)
    {
        point_topic_      = declare_parameter<std::string>("point_topic",      "roi_point");
        confidence_topic_ = declare_parameter<std::string>("confidence_topic", "roi");
        output_topic_     = declare_parameter<std::string>("output_topic",     "cv_target");
        estimate_velocity_     = declare_parameter<bool>("estimate_velocity", true);
        velocity_filter_alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.4);
        max_gap_s_         = declare_parameter<double>("max_extrapolation_gap_s", 0.5);
        target_timeout_s_  = declare_parameter<double>("target_timeout_s", 0.5);
        default_confidence_ = declare_parameter<double>("default_confidence", 1.0);

        velocity_filter_alpha_ = std::clamp(velocity_filter_alpha_, 0.0, 1.0);

        // Sensor-like, best-effort traffic: a dropped target update is far less
        // harmful than blocking on a slow/disconnected subscriber, and this
        // matches the QoS dji_serial_bridge_node's cv_target subscriber expects.
        pub_ = create_publisher<dji_serial_bridge::msg::CVTarget>(
            output_topic_, rclcpp::SensorDataQoS());

        // Matches roi_depth_node's "/roi_point" publisher (plain depth-10, reliable).
        point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            point_topic_, rclcpp::QoS(10),
            std::bind(&PointToCvTargetNode::onPoint, this, std::placeholders::_1));

        confidence_sub_ = create_subscription<vision_msgs::msg::Detection2D>(
            confidence_topic_, rclcpp::QoS(10),
            std::bind(&PointToCvTargetNode::onDetection, this, std::placeholders::_1));

        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PointToCvTargetNode::checkTimeout, this));

        RCLCPP_INFO(get_logger(),
            "point_to_cv_target ready\n"
            "  %s (PointStamped, REP-103) + %s (Detection2D, confidence only)\n"
            "  -> %s (CVTarget, camera frame X-right Y-up Z-forward)\n"
            "  estimate_velocity=%s  target_timeout_s=%.2f",
            point_topic_.c_str(), confidence_topic_.c_str(), output_topic_.c_str(),
            estimate_velocity_ ? "true" : "false", target_timeout_s_);
    }

private:
    void onDetection(const vision_msgs::msg::Detection2D::ConstSharedPtr & msg)
    {
        double score = 0.0;
        for (const auto & hyp : msg->results) {
            score = std::max(score, hyp.hypothesis.score);
        }
        if (score > 0.0) {
            latest_confidence_ = score;
            have_confidence_ = true;
        }
    }

    void onPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr & msg)
    {
        const double t = rclcpp::Time(msg->header.stamp).seconds();

        // Frame conversion: REP-103 (fwd, left, up) -> CVTarget (right, up, forward)
        const double x = -msg->point.y;
        const double y =  msg->point.z;
        const double z =  msg->point.x;

        double vx = 0.0, vy = 0.0, vz = 0.0;
        double ax = 0.0, ay = 0.0, az = 0.0;

        if (estimate_velocity_) {
            const double dt = t - prev_t_;
            if (have_prev_ && dt > 0.0 && dt <= max_gap_s_) {
                const double rvx = (x - prev_x_) / dt;
                const double rvy = (y - prev_y_) / dt;
                const double rvz = (z - prev_z_) / dt;

                const double a = velocity_filter_alpha_;
                vx = a * rvx + (1.0 - a) * prev_vx_;
                vy = a * rvy + (1.0 - a) * prev_vy_;
                vz = a * rvz + (1.0 - a) * prev_vz_;

                const double rax = (vx - prev_vx_) / dt;
                const double ray = (vy - prev_vy_) / dt;
                const double raz = (vz - prev_vz_) / dt;

                ax = a * rax + (1.0 - a) * prev_ax_;
                ay = a * ray + (1.0 - a) * prev_ay_;
                az = a * raz + (1.0 - a) * prev_az_;
            }
            // else: first sample, or the gap since the last one is too large to
            // trust a finite difference (track loss / reacquisition) — leave
            // velocity & acceleration at zero rather than emit a spike.
        }

        prev_x_ = x;  prev_y_ = y;  prev_z_ = z;
        prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;
        prev_ax_ = ax; prev_ay_ = ay; prev_az_ = az;
        prev_t_ = t;
        have_prev_ = true;

        last_point_wall_time_ = now();
        target_active_ = true;

        dji_serial_bridge::msg::CVTarget out;
        out.header = msg->header;
        out.x = static_cast<float>(x);
        out.y = static_cast<float>(y);
        out.z = static_cast<float>(z);
        out.v_x = static_cast<float>(vx);
        out.v_y = static_cast<float>(vy);
        out.v_z = static_cast<float>(vz);
        out.a_x = static_cast<float>(ax);
        out.a_y = static_cast<float>(ay);
        out.a_z = static_cast<float>(az);
        out.confidence = static_cast<float>(
            have_confidence_ ? latest_confidence_ : default_confidence_);

        pub_->publish(out);
    }

    void checkTimeout()
    {
        if (!target_active_) {
            return;
        }
        const double idle_s = (now() - last_point_wall_time_).seconds();
        if (idle_s <= target_timeout_s_) {
            return;
        }

        target_active_ = false;
        have_prev_ = false;  // force a clean restart of the velocity filter

        dji_serial_bridge::msg::CVTarget lost;
        lost.header.stamp = now();
        lost.confidence = 0.0f;
        pub_->publish(lost);

        RCLCPP_INFO(get_logger(),
            "No message on '%s' for %.2f s — published a zero-confidence "
            "CVTarget and paused until the next point arrives.",
            point_topic_.c_str(), idle_s);
    }

    // Parameters
    std::string point_topic_, confidence_topic_, output_topic_;
    bool estimate_velocity_{true};
    double velocity_filter_alpha_{0.4};
    double max_gap_s_{0.5};
    double target_timeout_s_{0.5};
    double default_confidence_{1.0};

    // Confidence cache (latest score seen on confidence_topic_)
    double latest_confidence_{1.0};
    bool have_confidence_{false};

    // Finite-difference filter state
    bool have_prev_{false};
    double prev_x_{0.0}, prev_y_{0.0}, prev_z_{0.0};
    double prev_vx_{0.0}, prev_vy_{0.0}, prev_vz_{0.0};
    double prev_ax_{0.0}, prev_ay_{0.0}, prev_az_{0.0};
    double prev_t_{0.0};

    // Stale-target watchdog
    bool target_active_{false};
    rclcpp::Time last_point_wall_time_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr confidence_sub_;
    rclcpp::Publisher<dji_serial_bridge::msg::CVTarget>::SharedPtr pub_;
};

}  // namespace dji_serial_bridge

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dji_serial_bridge::PointToCvTargetNode>());
    rclcpp::shutdown();
    return 0;
}
