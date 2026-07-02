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
// Parameters (see config/dji_bridge_params.yaml for defaults):
//   device        (string)  : serial device path, e.g. /dev/ttyTHS1
//   baudrate      (int)     : baud rate in bits-per-second, e.g. 115200
//   read_poll_ms  (int)     : poll() timeout in milliseconds (10 is fine)
//   enforce_crc   (bool)    : drop frames whose CRC does not match (default true)
//   diag_interval_s (int)   : how often to print diagnostic stats (default 5)

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
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"

#include "dji_serial_bridge/dji_protocol.hpp"
#include "dji_serial_bridge/crc_dji.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Helper: map an integer baud rate to a POSIX speed_t constant
// ─────────────────────────────────────────────────────────────────────────────
static speed_t baud_to_speed(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    case 2000000:
        return B2000000;
    case 4000000:
        return B4000000;
    default:
        return B921600;
    }
}

// ─────────────────────────────────────────────────────────────────────────────

class DjiSerialBridge : public rclcpp::Node
{
public:
    explicit DjiSerialBridge(const rclcpp::NodeOptions &options)
        : rclcpp::Node("dji_serial_bridge", options)
    {
        // ── Parameters ────────────────────────────────────────────────────────
        declare_parameter<std::string>("device", "/dev/ttyTHS1");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<int>("read_poll_ms", 20);
        declare_parameter<bool>("enforce_crc", true);
        declare_parameter<int>("diag_interval_s", 5);

        const auto device = get_parameter("device").as_string();
        const auto baudrate = get_parameter("baudrate").as_int();
        const auto poll_ms = static_cast<int>(get_parameter("read_poll_ms").as_int());
        enforce_crc_ = get_parameter("enforce_crc").as_bool();
        const auto diag_s = get_parameter("diag_interval_s").as_int();

        // ── Pre-open device diagnostics ───────────────────────────────────────
        struct stat sb{};
        if (::stat(device.c_str(), &sb) != 0)
        {
            RCLCPP_FATAL(get_logger(),
                         "Device '%s' does not exist or cannot be stat'd: %s",
                         device.c_str(), strerror(errno));
            throw std::runtime_error("Serial device not found: " + device);
        }
        // Check it's a character device
        if (!S_ISCHR(sb.st_mode))
        {
            RCLCPP_FATAL(get_logger(),
                         "Path '%s' exists but is NOT a character device (mode=0%o)",
                         device.c_str(), sb.st_mode);
            throw std::runtime_error("Not a character device: " + device);
        }
        // Warn if we don't have read+write permission
        if (::access(device.c_str(), R_OK | W_OK) != 0)
        {
            RCLCPP_WARN(get_logger(),
                        "Permission check on '%s' failed: %s  "
                        "(try: sudo chmod a+rw %s  OR  sudo usermod -aG dialout $USER)",
                        device.c_str(), strerror(errno), device.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "Device '%s' exists and is readable/writable", device.c_str());
        }

        // ── Serial port ───────────────────────────────────────────────────────
        serial_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0)
        {
            throw std::runtime_error(
                "DjiSerialBridge: failed to open " + device + ": " + strerror(errno));
        }
        RCLCPP_INFO(get_logger(),
                    "Serial port '%s' opened (fd=%d)", device.c_str(), serial_fd_);

        if (!configure_serial(serial_fd_, static_cast<int>(baudrate)))
        {
            ::close(serial_fd_);
            throw std::runtime_error(
                "DjiSerialBridge: failed to configure serial port " + device);
        }
        RCLCPP_INFO(get_logger(),
                    "Serial port configured: %ld baud, 8N1, no flow control, raw mode",
                    baudrate);
        // Clear NONBLOCK so subsequent reads behave normally (poll() handles blocking)
        int fl = fcntl(serial_fd_, F_GETFL);
        fcntl(serial_fd_, F_SETFL, fl & ~O_NONBLOCK);

        // ── Publishers  (MCB → Jetson) ────────────────────────────────────────
        pose_pub_ = create_publisher<dji_serial_bridge::msg::RobotPose>(
            "~/pose", rclcpp::SensorDataQoS());
        ref_sys_pub_ = create_publisher<dji_serial_bridge::msg::RefSysStatus>(
            "~/ref_sys", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(),
                    "Publishers ready:  ~/pose  ~/ref_sys");

        // ── Subscribers (Jetson → MCB) ────────────────────────────────────────
        using std::placeholders::_1;

        nav_goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "~/nav_goal", 10,
            std::bind(&DjiSerialBridge::nav_goal_callback, this, _1));

        cv_target_sub_ = create_subscription<dji_serial_bridge::msg::CVTarget>(
            "~/cv_target", rclcpp::SensorDataQoS(),
            std::bind(&DjiSerialBridge::cv_target_callback, this, _1));

        relocalize_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "~/relocalize", 10,
            std::bind(&DjiSerialBridge::relocalize_callback, this, _1));

        RCLCPP_INFO(get_logger(),
                    "Subscribers ready: ~/nav_goal  ~/cv_target  ~/relocalize");

        // ── Diagnostic timer ──────────────────────────────────────────────────
        // Fires every diag_interval_s seconds and prints a stats summary so you
        // can see at a glance whether bytes / frames are actually flowing.
        diag_timer_ = create_wall_timer(
            std::chrono::seconds(diag_s),
            std::bind(&DjiSerialBridge::print_diagnostics, this));

        // ── Read thread ───────────────────────────────────────────────────────
        running_.store(true);
        read_thread_ = std::thread(&DjiSerialBridge::read_loop, this, poll_ms);

        RCLCPP_INFO(get_logger(),
                    "═══════════════════════════════════════════════════════════\n"
                    "  MCB serial bridge READY\n"
                    "  device=%s  baud=%ld  enforce_crc=%s  poll_ms=%d\n"
                    "  Diagnostic summary every %ld s\n"
                    "═══════════════════════════════════════════════════════════",
                    device.c_str(), baudrate,
                    enforce_crc_ ? "true" : "false",
                    poll_ms, diag_s);
    }

    ~DjiSerialBridge() override
    {
        running_.store(false);
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
        if (serial_fd_ >= 0)
        {
            ::close(serial_fd_);
        }
    }

private:
    // ═══════════════════════════════════════════════════════════════════════
    // Diagnostic stats
    // ═══════════════════════════════════════════════════════════════════════

    // All counters are relaxed-atomic so the read thread can increment them
    // without acquiring the write mutex, and the timer can read them safely.
    std::atomic<uint64_t> bytes_rx_{0};
    std::atomic<uint64_t> frames_rx_{0};
    std::atomic<uint64_t> crc8_errors_{0};
    std::atomic<uint64_t> crc16_errors_{0};
    std::atomic<uint64_t> pose_msgs_pub_{0};
    std::atomic<uint64_t> ref_sys_msgs_pub_{0};
    std::atomic<uint64_t> nav_goal_msgs_tx_{0};
    std::atomic<uint64_t> cv_target_msgs_tx_{0};
    std::atomic<uint64_t> relocalize_msgs_tx_{0};
    // Consecutive poll() calls that returned 0 (no data).
    // Resets to 0 the moment any byte arrives.
    std::atomic<uint64_t> silent_polls_{0};

    void print_diagnostics()
    {
        const uint64_t brx = bytes_rx_.load(std::memory_order_relaxed);
        const uint64_t frx = frames_rx_.load(std::memory_order_relaxed);
        const uint64_t c8 = crc8_errors_.load(std::memory_order_relaxed);
        const uint64_t c16 = crc16_errors_.load(std::memory_order_relaxed);
        const uint64_t pose = pose_msgs_pub_.load(std::memory_order_relaxed);
        const uint64_t ref = ref_sys_msgs_pub_.load(std::memory_order_relaxed);
        const uint64_t ng = nav_goal_msgs_tx_.load(std::memory_order_relaxed);
        const uint64_t cv = cv_target_msgs_tx_.load(std::memory_order_relaxed);
        const uint64_t rl = relocalize_msgs_tx_.load(std::memory_order_relaxed);
        const uint64_t sil = silent_polls_.load(std::memory_order_relaxed);

        // Pick a severity level depending on whether anything is flowing
        if (brx == 0 && frames_rx_.load() == 0)
        {
            RCLCPP_WARN(get_logger(),
                        "── DIAG ─────────────────────────────────────────────────\n"
                        "  ⚠  NO BYTES RECEIVED YET from MCB!\n"
                        "     Check: cable connected? MCB powered? baud rate matches?\n"
                        "     silent_polls=%lu  (each poll_ms timeout = no data arriving)\n"
                        "  RX  bytes=0  frames=0  crc8_err=0  crc16_err=0\n"
                        "  PUB pose=0  ref_sys=0\n"
                        "  TX  nav_goal=%lu  cv_target=%lu  relocalize=%lu\n"
                        "─────────────────────────────────────────────────────────",
                        sil, ng, cv, rl);
        }
        else if (frx == 0 && brx > 0)
        {
            RCLCPP_WARN(get_logger(),
                        "── DIAG ─────────────────────────────────────────────────\n"
                        "  ⚠  BYTES arriving (%lu) but ZERO valid frames decoded!\n"
                        "     Check: baud rate, frame head (0xA5), CRC settings\n"
                        "     crc8_err=%lu  crc16_err=%lu\n"
                        "  RX  bytes=%lu  frames=0\n"
                        "  PUB pose=0  ref_sys=0\n"
                        "  TX  nav_goal=%lu  cv_target=%lu  relocalize=%lu\n"
                        "─────────────────────────────────────────────────────────",
                        brx, c8, c16, brx, ng, cv, rl);
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "── DIAG ─────────────────────────────────────────────────\n"
                        "  ✓ RX  bytes=%lu  frames=%lu  crc8_err=%lu  crc16_err=%lu\n"
                        "  ✓ PUB pose=%lu  ref_sys=%lu\n"
                        "    TX  nav_goal=%lu  cv_target=%lu  relocalize=%lu\n"
                        "─────────────────────────────────────────────────────────",
                        brx, frx, c8, c16, pose, ref, ng, cv, rl);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Serial port helpers
    // ═══════════════════════════════════════════════════════════════════════

    bool configure_serial(int fd, int baudrate_hz)
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
            return false;
        }

        speed_t speed = baud_to_speed(baudrate_hz);
        cfmakeraw(&tty);
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
            return false;
        }

        // Flush Tegra UART hardware FIFO after config — prevents stale DMA
        // bytes from before baud rate was set corrupting the first frame.
        tcflush(fd, TCIOFLUSH);
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Write path — called from subscriber callbacks (executor thread)
    // ═══════════════════════════════════════════════════════════════════════

    bool send_frame(McbMsgType msg_type,
                    const uint8_t *payload,
                    uint16_t payload_len)
    {
        FrameHeader hdr{};
        hdr.head = FRAME_HEAD;
        hdr.dataLength = payload_len;
        hdr.seq = tx_seq_++;
        hdr.crc8 = 0;
        hdr.crc8 = calculateCRC8(
            reinterpret_cast<const uint8_t *>(&hdr), CRC8_COVERAGE);
        hdr.msgType = static_cast<uint16_t>(msg_type);

        std::vector<uint8_t> frame;
        frame.reserve(sizeof(FrameHeader) + payload_len + 2u);
        const uint8_t *hdr_bytes = reinterpret_cast<const uint8_t *>(&hdr);
        frame.insert(frame.end(), hdr_bytes, hdr_bytes + sizeof(FrameHeader));
        frame.insert(frame.end(), payload, payload + payload_len);

        uint16_t crc16 = calculateCRC16(frame.data(), frame.size());
        frame.push_back(static_cast<uint8_t>(crc16 & 0xFF));
        frame.push_back(static_cast<uint8_t>((crc16 >> 8) & 0xFF));

        std::lock_guard<std::mutex> lock(write_mutex_);
        ssize_t written = ::write(serial_fd_, frame.data(), frame.size());
        if (written != static_cast<ssize_t>(frame.size()))
        {
            RCLCPP_ERROR(get_logger(),
                         "send_frame: short write for msg_type=%u (%zd/%zu): %s",
                         static_cast<unsigned>(msg_type), written, frame.size(), strerror(errno));
            return false;
        }
        RCLCPP_DEBUG(get_logger(),
                     "send_frame: msg_type=%u  seq=%u  payload=%u B  frame=%zu B  OK",
                     static_cast<unsigned>(msg_type), hdr.seq, payload_len, frame.size());
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Read path — dedicated thread
    // ═══════════════════════════════════════════════════════════════════════

    void read_loop(int poll_ms)
    {
        RCLCPP_DEBUG(get_logger(), "read_loop: thread started (poll_ms=%d)", poll_ms);

        pollfd pfd{};
        pfd.fd = serial_fd_;
        pfd.events = POLLIN;

        std::vector<uint8_t> rx_buf;
        rx_buf.reserve(512);
        uint8_t chunk[256];

        // We log a "still no data" warning after this many consecutive silent polls.
        // With poll_ms=10 this fires after ~1 second of silence.
        static constexpr uint64_t SILENCE_WARN_POLLS = 100;

        while (running_.load())
        {
            int ret = poll(&pfd, 1, poll_ms);
            if (ret < 0)
            {
                if (errno == EINTR)
                    continue;
                RCLCPP_ERROR(get_logger(), "poll() error: %s", strerror(errno));
                break;
            }
            if (ret == 0)
            {
                // poll timed out — no data this interval
                const uint64_t sil = ++silent_polls_;
                // Emit a one-time notice on the first prolonged silence so the
                // user knows bytes are simply not arriving.
                if (sil == SILENCE_WARN_POLLS)
                {
                    RCLCPP_WARN(get_logger(),
                                "read_loop: NO DATA from MCB for ~%llu ms  "
                                "(poll_ms=%d × %llu polls).  "
                                "Is the MCB powered/connected and transmitting?",
                                static_cast<unsigned long long>(poll_ms) *
                                    static_cast<unsigned long long>(SILENCE_WARN_POLLS),
                                poll_ms,
                                static_cast<unsigned long long>(SILENCE_WARN_POLLS));
                }
                continue;
            }

            if (pfd.revents & POLLIN)
            {
                ssize_t n = ::read(serial_fd_, chunk, sizeof(chunk));
                if (n > 0)
                {
                    const uint64_t prev = bytes_rx_.fetch_add(
                        static_cast<uint64_t>(n), std::memory_order_relaxed);
                    // Log first byte arrival and then periodically every 1 KB
                    if (prev == 0)
                    {
                        RCLCPP_INFO(get_logger(),
                                    "read_loop: first bytes received from MCB! (%zd B)", n);
                    }
                    else if ((prev / 1024) != ((prev + static_cast<uint64_t>(n)) / 1024))
                    {
                        RCLCPP_DEBUG(get_logger(),
                                     "read_loop: %lu bytes total rx (latest chunk %zd B)",
                                     prev + static_cast<uint64_t>(n), n);
                    }
                    silent_polls_.store(0, std::memory_order_relaxed);
                    rx_buf.insert(rx_buf.end(), chunk, chunk + n);
                }
                else if (n < 0 && errno != EAGAIN)
                {
                    RCLCPP_ERROR(get_logger(), "read() error: %s", strerror(errno));
                    break;
                }
            }

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                RCLCPP_ERROR(get_logger(),
                             "read_loop: poll() returned error event 0x%x on fd — "
                             "serial device disconnected?",
                             pfd.revents);
                break;
            }

            process_rx_buffer(rx_buf);
        }

        RCLCPP_INFO(get_logger(),
                    "read_loop: thread exiting  (bytes_rx=%lu  frames_rx=%lu)",
                    bytes_rx_.load(), frames_rx_.load());
    }

    // Scan rx_buf for complete, CRC-validated DJI frames and dispatch them.
    void process_rx_buffer(std::vector<uint8_t> &buf)
    {
        static constexpr size_t MIN_FRAME = sizeof(FrameHeader) + 2u;

        while (buf.size() >= MIN_FRAME)
        {

            // ── 1. Hunt for frame head ──────────────────────────────────────
            if (buf[0] != FRAME_HEAD)
            {
                RCLCPP_DEBUG(get_logger(),
                             "process_rx_buffer: skipping byte 0x%02x (not FRAME_HEAD 0xA5)",
                             buf[0]);
                buf.erase(buf.begin());
                continue;
            }

            // ── 2. Parse and CRC8-validate the header ──────────────────────
            FrameHeader hdr;
            std::memcpy(&hdr, buf.data(), sizeof(FrameHeader));

            uint8_t expected_crc8 = calculateCRC8(buf.data(), CRC8_COVERAGE);
            if (expected_crc8 != hdr.crc8 && enforce_crc_)
            {
                crc8_errors_.fetch_add(1, std::memory_order_relaxed);
                RCLCPP_WARN(get_logger(),
                            "CRC8 mismatch (seq=%u, got=0x%02x expected=0x%02x) — dropping byte",
                            hdr.seq, hdr.crc8, expected_crc8);
                buf.erase(buf.begin());
                continue;
            }

            // ── 3. Wait until the full frame has arrived ────────────────────
            size_t total_len = sizeof(FrameHeader) + hdr.dataLength + 2u;
            if (buf.size() < total_len)
            {
                RCLCPP_DEBUG(get_logger(),
                             "process_rx_buffer: partial frame (have %zu B, need %zu B) — waiting",
                             buf.size(), total_len);
                break;
            }

            // ── 4. Validate CRC16 over header + payload ─────────────────────
            size_t crc16_idx = sizeof(FrameHeader) + hdr.dataLength;
            uint16_t recv_crc16 = static_cast<uint16_t>(buf[crc16_idx]) |
                                  (static_cast<uint16_t>(buf[crc16_idx + 1]) << 8);
            uint16_t calc_crc16 = calculateCRC16(buf.data(), crc16_idx);

            if (recv_crc16 != calc_crc16 && enforce_crc_)
            {
                crc16_errors_.fetch_add(1, std::memory_order_relaxed);
                RCLCPP_WARN(get_logger(),
                            "CRC16 mismatch (seq=%u, msgType=%u, dataLen=%u, "
                            "got=0x%04x expected=0x%04x) — dropping byte",
                            hdr.seq, hdr.msgType, hdr.dataLength, recv_crc16, calc_crc16);
                buf.erase(buf.begin());
                continue;
            }

            // ── 5. Dispatch ─────────────────────────────────────────────────
            const uint8_t *payload = buf.data() + sizeof(FrameHeader);
            RCLCPP_DEBUG(get_logger(),
                         "process_rx_buffer: valid frame  seq=%u  msgType=%u  dataLen=%u",
                         hdr.seq, hdr.msgType, hdr.dataLength);
            frames_rx_.fetch_add(1, std::memory_order_relaxed);
            dispatch_incoming(hdr.msgType, payload, hdr.dataLength);

            // ── 6. Consume the frame ────────────────────────────────────────
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(total_len));
        }
    }

    void dispatch_incoming(uint16_t msg_type, const uint8_t *payload, uint16_t len)
    {
        switch (static_cast<McbMsgType>(msg_type))
        {
        case McbMsgType::POSE_MSG:
            handle_pose(payload, len);
            break;
        case McbMsgType::REF_SYS:
            handle_ref_sys(payload, len);
            break;
        default:
            RCLCPP_WARN(get_logger(),
                        "Unexpected inbound msgType=%u (len=%u) — ignoring", msg_type, len);
            break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Incoming message handlers  (MCB → Jetson)
    // ═══════════════════════════════════════════════════════════════════════

    void handle_pose(const uint8_t *payload, uint16_t len)
    {
        if (len != sizeof(PoseDataPayload))
        {
            RCLCPP_WARN(get_logger(),
                        "POSE_MSG: unexpected payload length %u (expected %zu)",
                        len, sizeof(PoseDataPayload));
            return;
        }

        PoseDataPayload raw{};
        std::memcpy(&raw, payload, sizeof(raw));

        auto msg = dji_serial_bridge::msg::RobotPose{};
        msg.header.stamp = now();
        msg.x = raw.x;
        msg.y = raw.y;
        msg.vel_x = raw.vel_x;
        msg.vel_y = raw.vel_y;
        msg.head_pitch = raw.head_pitch;
        msg.head_yaw = raw.head_yaw;

        const uint64_t count =
            pose_msgs_pub_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (count == 1)
        {
            RCLCPP_INFO(get_logger(),
                        "handle_pose: FIRST pose message published! "
                        "x=%.3f y=%.3f vel_x=%.3f vel_y=%.3f pitch=%.3f yaw=%.3f",
                        raw.x, raw.y, raw.vel_x, raw.vel_y, raw.head_pitch, raw.head_yaw);
        }
        else
        {
            RCLCPP_DEBUG(get_logger(),
                         "handle_pose #%lu: x=%.3f y=%.3f vel_x=%.3f vel_y=%.3f",
                         count, raw.x, raw.y, raw.vel_x, raw.vel_y);
        }
        pose_pub_->publish(msg);
    }

    void handle_ref_sys(const uint8_t *payload, uint16_t len)
    {
        if (len != sizeof(RefSysMsgPayload))
        {
            RCLCPP_WARN(get_logger(),
                        "REF_SYS: unexpected payload length %u (expected %zu)",
                        len, sizeof(RefSysMsgPayload));
            return;
        }

        RefSysMsgPayload raw{};
        std::memcpy(&raw, payload, sizeof(raw));

        auto msg = dji_serial_bridge::msg::RefSysStatus{};
        msg.header.stamp = now();
        msg.game_stage = raw.gameStage;
        msg.stage_time_remaining = raw.stageTimeRemaining;
        msg.robot_hp = raw.robotHp;
        msg.robot_id = raw.robotID;
        msg.delta_angle_got_hit_in = raw.deltaAngleGotHitIn;

        const uint8_t b = raw.booleans;
        msg.is_on_blue_team = (b >> 7) & 1u;
        msg.is_healing = (b >> 6) & 1u;
        msg.is_in_reload_zone = (b >> 5) & 1u;
        msg.is_in_center_zone = (b >> 4) & 1u;
        msg.team_occupies_center = (b >> 3) & 1u;
        msg.opponent_occupies_center = (b >> 2) & 1u;
        msg.chassis_has_power = (b >> 1) & 1u;
        msg.gimbal_has_power = (b >> 0) & 1u;

        const uint64_t count =
            ref_sys_msgs_pub_.fetch_add(1, std::memory_order_relaxed) + 1;
        
        // Log every ref_sys message received
        RCLCPP_INFO(get_logger(),
                    "[ref_sys RX #%lu] stage=%u time_rem=%u hp=%u robot_id=%u "
                    "blue=%u healing=%u reload=%u center=%u "
                    "chassis_pwr=%u gimbal_pwr=%u delta_angle=%.1f",
                    count, raw.gameStage, raw.stageTimeRemaining, raw.robotHp, raw.robotID,
                    (b >> 7) & 1u, (b >> 6) & 1u, (b >> 5) & 1u, (b >> 4) & 1u,
                    (b >> 1) & 1u, b & 1u, raw.deltaAngleGotHitIn);
        
        ref_sys_pub_->publish(msg);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Subscriber callbacks  (Jetson → MCB)
    // ═══════════════════════════════════════════════════════════════════════

    void nav_goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        ROSDataPayload p{};
        p.targetX = static_cast<float>(msg->x);
        p.targetY = static_cast<float>(msg->y);

        const bool ok = send_frame(McbMsgType::ROS_MSG,
                                   reinterpret_cast<const uint8_t *>(&p), sizeof(p));
        if (ok)
        {
            nav_goal_msgs_tx_.fetch_add(1, std::memory_order_relaxed);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Failed to send ROS_MSG (nav_goal)");
        }
    }

    void cv_target_callback(const dji_serial_bridge::msg::CVTarget::SharedPtr msg)
    {
        CVDataPayload p{};
        p.x = msg->x;
        p.y = msg->y;
        p.z = msg->z;
        p.v_x = msg->v_x;
        p.v_y = msg->v_y;
        p.v_z = msg->v_z;
        p.a_x = msg->a_x;
        p.a_y = msg->a_y;
        p.a_z = msg->a_z;
        p.confidence = msg->confidence;

        const bool ok = send_frame(McbMsgType::CV_MSG,
                                   reinterpret_cast<const uint8_t *>(&p), sizeof(p));
        if (ok)
        {
            cv_target_msgs_tx_.fetch_add(1, std::memory_order_relaxed);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Failed to send CV_MSG (cv_target)");
        }
    }

    void relocalize_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RelocalizePayload p{};
        p.expectedX = static_cast<float>(msg->x);
        p.expectedY = static_cast<float>(msg->y);

        const bool ok = send_frame(McbMsgType::RELOCALIZE,
                                   reinterpret_cast<const uint8_t *>(&p), sizeof(p));
        if (ok)
        {
            const uint64_t count = relocalize_msgs_tx_.fetch_add(1, std::memory_order_relaxed) + 1;
            // Log every relocalize message sent
            RCLCPP_INFO(get_logger(),
                        "[relocalize TX #%lu] x=%.3f y=%.3f",
                        count, msg->x, msg->y);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Failed to send RELOCALIZE");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Member variables
    // ═══════════════════════════════════════════════════════════════════════

    int serial_fd_{-1};
    uint8_t tx_seq_{0};
    std::mutex write_mutex_;
    bool enforce_crc_{true};

    std::atomic<bool> running_{false};
    std::thread read_thread_;

    rclcpp::TimerBase::SharedPtr diag_timer_;

    rclcpp::Publisher<dji_serial_bridge::msg::RobotPose>::SharedPtr pose_pub_;
    rclcpp::Publisher<dji_serial_bridge::msg::RefSysStatus>::SharedPtr ref_sys_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<dji_serial_bridge::msg::CVTarget>::SharedPtr cv_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr relocalize_sub_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiSerialBridge>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
