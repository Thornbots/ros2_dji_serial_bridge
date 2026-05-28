// dji_serial_bridge_node.cpp
//
// ROS 2 node that bridges the Jetson-side serial protocol with ROS topics.
//
// The MCB (main control board) speaks a DJI-framed UART protocol where each
// message payload is a raw packed C struct.  This node handles all five
// message types defined in JetsonSubsystem.hpp:
//
//  ID  Direction       ROS topic          ROS msg type
//  ──  ─────────────── ────────────────── ─────────────────────────────
//   0  Jetson → MCB    ~/nav_goal         geometry_msgs/msg/Point
//   1  Jetson → MCB    ~/cv_target        dji_serial_bridge/msg/CVTarget
//   2  MCB   → Jetson  ~/pose             dji_serial_bridge/msg/RobotPose
//   3  MCB   → Jetson  ~/ref_sys          dji_serial_bridge/msg/RefSysStatus
//   4  Jetson → MCB    ~/relocalize       geometry_msgs/msg/Point
//
// Topics use the node's private namespace so you can remap them in a launch
// file.  For example, ~/nav_goal resolves to /dji_serial_bridge/nav_goal by
// default but can be remapped to /nav_goal.
//
// Parameters (see config/mcb_bridge_params.yaml for defaults):
//   device        (string)  : serial device path, e.g. /dev/ttyTHS1
//   baudrate      (int)     : baud rate in bits-per-second, e.g. 115200
//   read_poll_ms  (int)     : poll() timeout in milliseconds (10 is fine)
//   enforce_crc   (bool)    : drop frames whose CRC does not match (default true)

#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"

#include "dji_serial_bridge/mcb_protocol.hpp"
#include "dji_serial_bridge/crc_dji.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Helper: map an integer baud rate to a POSIX speed_t constant
// ─────────────────────────────────────────────────────────────────────────────
static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 2000000: return B2000000;
        case 4000000: return B4000000;
        default:      return B921600;
    }
}

// ─────────────────────────────────────────────────────────────────────────────

class DjiSerialBridge : public rclcpp::Node
{
public:
    explicit DjiSerialBridge(const rclcpp::NodeOptions & options)
    : rclcpp::Node("dji_serial_bridge", options)
    {
        // ── Parameters ────────────────────────────────────────────────────────
        declare_parameter<std::string>("device",       "/dev/ttyTHS1");
        declare_parameter<int>        ("baudrate",     115200);
        declare_parameter<int>        ("read_poll_ms", 20);
        declare_parameter<bool>       ("enforce_crc",  true);

        const auto device      = get_parameter("device").as_string();
        const auto baudrate    = get_parameter("baudrate").as_int();
        const auto poll_ms     = static_cast<int>(get_parameter("read_poll_ms").as_int());
        enforce_crc_           = get_parameter("enforce_crc").as_bool();

        // ── Serial port ───────────────────────────────────────────────────────
        serial_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            throw std::runtime_error(
                "DjiSerialBridge: failed to open " + device + ": " + strerror(errno));
        }
        if (!configure_serial(serial_fd_, static_cast<int>(baudrate))) {
            ::close(serial_fd_);
            throw std::runtime_error(
                "DjiSerialBridge: failed to configure serial port " + device);
        }

        // ── Publishers  (MCB → Jetson) ────────────────────────────────────────
        pose_pub_    = create_publisher<dji_serial_bridge::msg::RobotPose>(
            "~/pose",    rclcpp::SensorDataQoS());
        ref_sys_pub_ = create_publisher<dji_serial_bridge::msg::RefSysStatus>(
            "~/ref_sys", rclcpp::SensorDataQoS());

        // ── Subscribers (Jetson → MCB) ────────────────────────────────────────
        using std::placeholders::_1;

        // geometry_msgs/Point  x=targetX, y=targetY
        nav_goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "~/nav_goal", 10,
            std::bind(&DjiSerialBridge::nav_goal_callback, this, _1));

        cv_target_sub_ = create_subscription<dji_serial_bridge::msg::CVTarget>(
            "~/cv_target", rclcpp::SensorDataQoS(),
            std::bind(&DjiSerialBridge::cv_target_callback, this, _1));

        // geometry_msgs/Point  x=expectedX, y=expectedY
        relocalize_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "~/relocalize", 10,
            std::bind(&DjiSerialBridge::relocalize_callback, this, _1));

        // ── Read thread ───────────────────────────────────────────────────────
        running_.store(true);
        read_thread_ = std::thread(&DjiSerialBridge::read_loop, this, poll_ms);

        RCLCPP_INFO(get_logger(),
            "MCB serial bridge started on %s @ %ld baud (enforce_crc=%s)",
            device.c_str(), baudrate, enforce_crc_ ? "true" : "false");
    }

    ~DjiSerialBridge() override
    {
        // Signal the read thread first so it stops calling get_logger().
        running_.store(false);
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_fd_ >= 0) {
            ::close(serial_fd_);
        }
    }

private:
    // ═══════════════════════════════════════════════════════════════════════
    // Serial port helpers
    // ═══════════════════════════════════════════════════════════════════════

    bool configure_serial(int fd, int baudrate_hz)
    {
        struct termios tty {};
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
            return false;
        }
        cfmakeraw(&tty);
        speed_t speed = baud_to_speed(baudrate_hz);
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
        tty.c_lflag  = 0;
        tty.c_oflag  = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;   // 0.5 s read timeout as fallback

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Write path — called from subscriber callbacks (executor thread)
    // ═══════════════════════════════════════════════════════════════════════

    // Build a DJI frame around `payload` and write it to the serial port.
    // Thread-safe: protected by write_mutex_.
    bool send_frame(McbMsgType msg_type,
                    const uint8_t * payload,
                    uint16_t        payload_len)
    {
        // Assemble header
        FrameHeader hdr{};
        hdr.head       = FRAME_HEAD;
        hdr.dataLength = payload_len;
        hdr.seq        = tx_seq_++;
        hdr.crc8       = 0;
        hdr.crc8       = calculateCRC8(
            reinterpret_cast<const uint8_t *>(&hdr), CRC8_COVERAGE);
        hdr.msgType    = static_cast<uint16_t>(msg_type);

        // Assemble complete frame: header + payload + CRC16
        std::vector<uint8_t> frame;
        frame.reserve(sizeof(FrameHeader) + payload_len + 2u);
        const uint8_t * hdr_bytes = reinterpret_cast<const uint8_t *>(&hdr);
        frame.insert(frame.end(), hdr_bytes, hdr_bytes + sizeof(FrameHeader));
        frame.insert(frame.end(), payload,   payload   + payload_len);

        uint16_t crc16 = calculateCRC16(frame.data(), frame.size());
        frame.push_back(static_cast<uint8_t>(crc16 & 0xFF));
        frame.push_back(static_cast<uint8_t>((crc16 >> 8) & 0xFF));

        std::lock_guard<std::mutex> lock(write_mutex_);
        ssize_t written = ::write(serial_fd_, frame.data(), frame.size());
        if (written != static_cast<ssize_t>(frame.size())) {
            RCLCPP_ERROR(get_logger(),
                "send_frame: short write for msg_type=%u (%zd/%zu): %s",
                static_cast<unsigned>(msg_type), written, frame.size(), strerror(errno));
            return false;
        }
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Read path — dedicated thread
    // ═══════════════════════════════════════════════════════════════════════

    void read_loop(int poll_ms)
    {
        pollfd pfd{};
        pfd.fd     = serial_fd_;
        pfd.events = POLLIN;

        std::vector<uint8_t> rx_buf;
        rx_buf.reserve(512);
        uint8_t chunk[256];

        while (running_.load()) {
            int ret = poll(&pfd, 1, poll_ms);
            if (ret < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(get_logger(), "poll() error: %s", strerror(errno));
                break;
            }
            if (ret > 0 && (pfd.revents & POLLIN)) {
                ssize_t n = ::read(serial_fd_, chunk, sizeof(chunk));
                if (n > 0) {
                    rx_buf.insert(rx_buf.end(), chunk, chunk + n);
                } else if (n < 0 && errno != EAGAIN) {
                    RCLCPP_ERROR(get_logger(), "read() error: %s", strerror(errno));
                    break;
                }
            }
            // Process whatever is in the buffer (may span multiple read() calls)
            process_rx_buffer(rx_buf);
        }
    }

    // Scan rx_buf for complete, CRC-validated DJI frames and dispatch them.
    // Consumes bytes as frames are extracted; stops when the buffer is
    // exhausted or more bytes are needed to complete the next frame.
    void process_rx_buffer(std::vector<uint8_t> & buf)
    {
        // Minimum viable frame: 7-byte header + 0-byte payload + 2-byte CRC16
        static constexpr size_t MIN_FRAME = sizeof(FrameHeader) + 2u;

        while (buf.size() >= MIN_FRAME) {

            // ── 1. Hunt for frame head ──────────────────────────────────────
            if (buf[0] != FRAME_HEAD) {
                buf.erase(buf.begin());
                continue;
            }

            // ── 2. Parse and CRC8-validate the header ──────────────────────
            FrameHeader hdr;
            std::memcpy(&hdr, buf.data(), sizeof(FrameHeader));

            uint8_t expected_crc8 = calculateCRC8(buf.data(), CRC8_COVERAGE);
            if (expected_crc8 != hdr.crc8 && enforce_crc_) {
                RCLCPP_WARN(get_logger(),
                    "CRC8 mismatch (seq=%u, got=0x%02x expected=0x%02x) — dropping byte",
                    hdr.seq, hdr.crc8, expected_crc8);
                buf.erase(buf.begin());
                continue;
            }

            // ── 3. Wait until the full frame has arrived ────────────────────
            size_t total_len = sizeof(FrameHeader) + hdr.dataLength + 2u;
            if (buf.size() < total_len) {
                break;  // need more bytes
            }

            // ── 4. Validate CRC16 over header + payload ─────────────────────
            size_t crc16_idx    = sizeof(FrameHeader) + hdr.dataLength;
            uint16_t recv_crc16 = static_cast<uint16_t>(buf[crc16_idx]) |
                                   (static_cast<uint16_t>(buf[crc16_idx + 1]) << 8);
            uint16_t calc_crc16 = calculateCRC16(buf.data(), crc16_idx);

            if (recv_crc16 != calc_crc16 && enforce_crc_) {
                RCLCPP_WARN(get_logger(),
                    "CRC16 mismatch (seq=%u, msgType=%u, dataLen=%u, "
                    "got=0x%04x expected=0x%04x) — dropping byte",
                    hdr.seq, hdr.msgType, hdr.dataLength, recv_crc16, calc_crc16);
                buf.erase(buf.begin());
                continue;
            }

            // ── 5. Dispatch to the appropriate handler ──────────────────────
            const uint8_t * payload = buf.data() + sizeof(FrameHeader);
            dispatch_incoming(hdr.msgType, payload, hdr.dataLength);

            // ── 6. Consume the frame ────────────────────────────────────────
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(total_len));
        }
    }

    void dispatch_incoming(uint16_t msg_type, const uint8_t * payload, uint16_t len)
    {
        switch (static_cast<McbMsgType>(msg_type)) {
            case McbMsgType::POSE_MSG:
                handle_pose(payload, len);
                break;
            case McbMsgType::REF_SYS:
                handle_ref_sys(payload, len);
                break;
            // IDs 0, 1, 4 are Jetson→MCB only; the MCB should never send them.
            default:
                RCLCPP_WARN(get_logger(),
                    "Unexpected inbound msgType=%u (len=%u) — ignoring", msg_type, len);
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Incoming message handlers  (MCB → Jetson)
    // ═══════════════════════════════════════════════════════════════════════

    void handle_pose(const uint8_t * payload, uint16_t len)
    {
        if (len != sizeof(PoseDataPayload)) {
            RCLCPP_WARN(get_logger(),
                "POSE_MSG: unexpected payload length %u (expected %zu)",
                len, sizeof(PoseDataPayload));
            return;
        }

        PoseDataPayload raw{};
        std::memcpy(&raw, payload, sizeof(raw));

        auto msg       = dji_serial_bridge::msg::RobotPose{};
        msg.header.stamp = now();
        msg.x          = raw.x;
        msg.y          = raw.y;
        msg.vel_x      = raw.vel_x;
        msg.vel_y      = raw.vel_y;
        msg.head_pitch = raw.head_pitch;
        msg.head_yaw   = raw.head_yaw;

        pose_pub_->publish(msg);
    }

    void handle_ref_sys(const uint8_t * payload, uint16_t len)
    {
        if (len != sizeof(RefSysMsgPayload)) {
            RCLCPP_WARN(get_logger(),
                "REF_SYS: unexpected payload length %u (expected %zu)",
                len, sizeof(RefSysMsgPayload));
            return;
        }

        RefSysMsgPayload raw{};
        std::memcpy(&raw, payload, sizeof(raw));

        auto msg                   = dji_serial_bridge::msg::RefSysStatus{};
        msg.header.stamp           = now();
        msg.game_stage             = raw.gameStage;
        msg.stage_time_remaining   = raw.stageTimeRemaining;
        msg.robot_hp               = raw.robotHp;
        msg.robot_id               = raw.robotID;
        msg.delta_angle_got_hit_in = raw.deltaAngleGotHitIn;

        // Unpack booleans byte — bit layout matches JetsonSubsystem.cpp packing:
        //   bit 7 : isOnBlueTeam           bit 1 : chassisHasPower
        //   bit 6 : isHealing              bit 0 : gimbalHasPower
        //   bit 5 : isInReloadZone
        //   bit 4 : isInCenterZone
        //   bit 3 : teamOccupiesCenter
        //   bit 2 : opponentOccupiesCenter
        const uint8_t b         = raw.booleans;
        msg.is_on_blue_team         = (b >> 7) & 1u;
        msg.is_healing              = (b >> 6) & 1u;
        msg.is_in_reload_zone       = (b >> 5) & 1u;
        msg.is_in_center_zone       = (b >> 4) & 1u;
        msg.team_occupies_center    = (b >> 3) & 1u;
        msg.opponent_occupies_center = (b >> 2) & 1u;
        msg.chassis_has_power       = (b >> 1) & 1u;
        msg.gimbal_has_power        = (b >> 0) & 1u;

        ref_sys_pub_->publish(msg);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Subscriber callbacks  (Jetson → MCB)
    // ═══════════════════════════════════════════════════════════════════════

    // ~/nav_goal  →  ROS_MSG (id=0)
    // Publish the 2-D navigation goal for the chassis auto-drive controller.
    // x = targetX, y = targetY  (Point.z is unused)
    void nav_goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        ROSDataPayload p{};
        p.targetX = static_cast<float>(msg->x);
        p.targetY = static_cast<float>(msg->y);

        if (!send_frame(McbMsgType::ROS_MSG,
                        reinterpret_cast<const uint8_t *>(&p), sizeof(p)))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Failed to send ROS_MSG (nav_goal)");
        }
    }

    // ~/cv_target  →  CV_MSG (id=1)
    // Forward the CV pipeline's target detection to the gimbal ballistics solver.
    void cv_target_callback(const dji_serial_bridge::msg::CVTarget::SharedPtr msg)
    {
        CVDataPayload p{};
        p.x          = msg->x;   p.y    = msg->y;   p.z    = msg->z;
        p.v_x        = msg->v_x; p.v_y  = msg->v_y; p.v_z  = msg->v_z;
        p.a_x        = msg->a_x; p.a_y  = msg->a_y; p.a_z  = msg->a_z;
        p.confidence = msg->confidence;

        if (!send_frame(McbMsgType::CV_MSG,
                        reinterpret_cast<const uint8_t *>(&p), sizeof(p)))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Failed to send CV_MSG (cv_target)");
        }
    }

    // ~/relocalize  →  RELOCALIZE (id=4)
    // Tell the MCB where the lidar thinks the robot currently is so it can
    // correct odometry drift.
    // x = expectedX, y = expectedY  (Point.z is unused)
    void relocalize_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RelocalizePayload p{};
        p.expectedX = static_cast<float>(msg->x);
        p.expectedY = static_cast<float>(msg->y);

        if (!send_frame(McbMsgType::RELOCALIZE,
                        reinterpret_cast<const uint8_t *>(&p), sizeof(p)))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Failed to send RELOCALIZE");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Member variables
    // ═══════════════════════════════════════════════════════════════════════

    // Serial
    int serial_fd_{-1};
    uint8_t tx_seq_{0};
    std::mutex write_mutex_;
    bool enforce_crc_{true};

    // Read thread lifecycle
    std::atomic<bool> running_{false};
    std::thread read_thread_;

    // Publishers
    rclcpp::Publisher<dji_serial_bridge::msg::RobotPose>::SharedPtr    pose_pub_;
    rclcpp::Publisher<dji_serial_bridge::msg::RefSysStatus>::SharedPtr ref_sys_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr           nav_goal_sub_;
    rclcpp::Subscription<dji_serial_bridge::msg::CVTarget>::SharedPtr    cv_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr           relocalize_sub_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiSerialBridge>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
