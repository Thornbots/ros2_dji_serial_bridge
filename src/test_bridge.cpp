// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "dji_serial_bridge/diagnostic_serial.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"

namespace
{
using Results = std::vector<std::pair<bool, std::string>>;
constexpr const char *kGreen = "\033[32m", *kYellow = "\033[33m",
  *kRed = "\033[31m";
constexpr const char *kCyan = "\033[36m", *kBold = "\033[1m",
  *kReset = "\033[0m";
void ok(const std::string & s)
{
  std::cout << "  " << kGreen << "✓  " << kReset << s << '\n';
}
void warn(const std::string & s)
{
  std::cout << "  " << kYellow << "⚠  " << kReset << s << '\n';
}
void fail(const std::string & s)
{
  std::cout << "  " << kRed << "✗  " << kReset << s << '\n';
}
void info(const std::string & s) {std::cout << "     " << s << '\n';}
void hdr(const std::string & s)
{
  std::cout << '\n' << kBold << kCyan << s << kReset << '\n';
}
std::string mode_string(mode_t mode)
{
  std::ostringstream out;
  out << "0o" << std::oct << mode;
  return out.str();
}
std::string fixed(double value, int precision = 0)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}
std::string boolean(bool value) {return value ? "True" : "False";}
std::string yaml_value(const YAML::Node & params, const char * key)
{
  const auto value = params[key];
  if (!value || !value.IsScalar()) {
    return "(not set)";
  }
  auto text = value.as<std::string>();
  if (text == "true") {
    text = "True";
  } else if (text == "false") {
    text = "False";
  }
  return text;
}
Results check_config(
  const std::string & device, int baud,
  const std::string & params_file)
{
  hdr("═══  CONFIG CHECK  ═══════════════════════════════════════");
  Results results;
  if (!params_file.empty()) {
    if (std::filesystem::is_regular_file(params_file)) {
      ok("Params file found: " + params_file);
      results.emplace_back(true, "params file exists");
      try {
        const auto data = YAML::LoadFile(params_file);
        YAML::Node params(YAML::NodeType::Map);
        if (data.IsMap()) {
          for (const auto & entry : data) {
            const auto value = entry.second;
            if (value.IsMap()) {
              const auto values =
                value["ros__parameters"] ? value["ros__parameters"] : value;
              for (const auto & parameter : values) {
                params[parameter.first.as<std::string>()] = parameter.second;
              }
            } else {
              params = data;
              break;
            }
          }
        }
        const auto yaml_device = yaml_value(params, "device");
        const auto yaml_baud = yaml_value(params, "baudrate");
        info("  device       = " + yaml_device);
        info("  baudrate     = " + yaml_baud);
        info("  read_poll_ms = " + yaml_value(params, "read_poll_ms"));
        info("  enforce_crc  = " + yaml_value(params, "enforce_crc"));
        if (yaml_device != device) {
          warn("CLI --device (" + device + ") differs from YAML (" +
               yaml_device + ")");
        }
        if (yaml_baud != std::to_string(baud)) {
          warn("CLI --baudrate (" + std::to_string(baud) +
               ") differs from YAML (" + yaml_baud + ")");
        }
      } catch (const std::exception & error) {
        warn(std::string("Could not parse YAML: ") + error.what());
      }
    } else {
      warn("Params file not found: " + params_file);
      results.emplace_back(false, "params file missing");
    }
  } else {
    info("No --params-file given; skipping YAML check");
  }
  struct stat status{};
  if (stat(device.c_str(), &status) != 0) {
    fail("Device NOT found: " + device);
    results.emplace_back(false, "device " + device + " missing");
    std::cout
        << "\n  Possible fixes:\n    • Check USB/UART cable is connected\n    "
      "• List available serial ports:\n"
      "        ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null\n"
      "    • If using USB adapter: lsusb | grep -i cp210 (or ch340, "
      "ftdi)\n";
    return results;
  }
  ok("Device path exists: " + device);
  results.emplace_back(true, "device exists");
  if (S_ISCHR(status.st_mode)) {
    ok("Is a character device (mode " + mode_string(status.st_mode) + ")");
    results.emplace_back(true, "is char device");
  } else {
    fail("Path exists but is NOT a character device (mode " +
         mode_string(status.st_mode) + ")");
    results.emplace_back(false, "not a char device");
  }
  for (const auto & permission :
    {std::make_pair(R_OK, "read"), std::make_pair(W_OK, "write")})
  {
    const bool readable = permission.first == R_OK;
    const std::string title = readable ? "Read" : "Write",
      name = permission.second;
    if (access(device.c_str(), permission.first) == 0) {
      ok(title + " permission OK");
      results.emplace_back(true, readable ? "readable" : "writable");
    } else {
      fail("No " + name + " permission on " + device);
      results.emplace_back(false, "no " + name + " permission");
      info("Fix:  sudo chmod a+" + std::string(readable ? "r " : "w ") +
           device);
      info("  OR: sudo usermod -aG dialout $USER  (then log out/in)");
    }
  }
  try {
    dji_diagnostic::SerialPort port(device, baud);
    ok("Opened " + device + " @ " + std::to_string(baud) +
       " baud with POSIX termios");
    results.emplace_back(true, "opened @ " + std::to_string(baud));
    tcflush(port.fd(), TCIFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int waiting = 0;
    ioctl(port.fd(), FIONREAD, &waiting);
    if (waiting > 0) {
      unsigned char sample[64];
      const auto count = read(port.fd(), sample, std::min(waiting, 64));
      if (count < 0) {
        throw std::runtime_error(std::strerror(errno));
      }
      ok("  Bytes already in RX buffer: " + std::to_string(waiting) +
         " (first " + std::to_string(count) +
         " B = " + dji_diagnostic::hex(sample, count, false) + ")");
      results.emplace_back(true, "bytes in RX buffer");
      if (std::find(sample, sample + count, 0xa5) != sample + count) {
        ok("  0xA5 (DJI frame head) found in sample — MCB is probably "
           "transmitting!");
        results.emplace_back(true, "0xA5 seen");
      } else {
        warn("  0xA5 NOT seen in " + std::to_string(count) +
             "-byte sample — might be noise, wrong baud, or MCB not sending "
             "yet");
      }
    } else {
      warn(
          "  No bytes waiting in RX buffer immediately after open  (MCB might "
          "not be sending yet, or wrong baud rate)");
      results.emplace_back(false, "no bytes in RX buffer");
    }
  } catch (const std::exception & error) {
    fail("POSIX termios could not open " + device + ": " + error.what());
    results.emplace_back(false, "open failed: " + std::string(error.what()));
  }
  return results;
}
Results check_ros_topics(std::string ns, double timeout)
{
  hdr("═══  LIVE TOPIC CHECK  ═══════════════════════════════════");
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }
  auto node = std::make_shared<rclcpp::Node>("dji_bridge_test_listener");
  dji_serial_bridge::msg::RobotPose::ConstSharedPtr pose;
  dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr referee;
  info("Subscribing to:  " + ns + "/pose");
  info("                 " + ns + "/ref_sys");
  info("Waiting up to " + fixed(timeout) + " s for messages...\n");
  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  auto pose_sub = node->create_subscription<dji_serial_bridge::msg::RobotPose>(
      ns + "/pose", qos,
    [&](dji_serial_bridge::msg::RobotPose::ConstSharedPtr msg) {
      if (!pose) {
        pose = msg;
      }
      });
  auto ref_sub =
    node->create_subscription<dji_serial_bridge::msg::RefSysStatus>(
          ns + "/ref_sys", qos,
    [&](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr msg) {
      if (!referee) {
        referee = msg;
      }
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
    !(pose && referee))
  {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  Results results;
  if (pose) {
    ok("Received ~/pose");
    info("  x=" + fixed(pose->x, 3) + "  y=" + fixed(pose->y, 3) + "  vel_x=" +
         fixed(pose->vel_x, 3) + "  vel_y=" + fixed(pose->vel_y, 3) +
         "  pitch=" + fixed(pose->head_pitch, 3) +
         "  yaw=" + fixed(pose->head_yaw, 3));
    results.emplace_back(true, "~/pose received");
  } else {
    fail("~/pose — NO message received within " + fixed(timeout) + " s");
    results.emplace_back(false, "~/pose timeout");
    info("Possible causes:");
    info("  • MCB not powered / not connected");
    info("  • Wrong baud rate (check dji_bridge_params.yaml vs MCB firmware)");
    info(
        "  • CRC mismatches causing all frames to be dropped  (try "
        "enforce_crc: false temporarily)");
    info("  • Bridge node not running  (ros2 node list | grep dji)");
  }
  if (referee) {
    ok("Received ~/ref_sys");
    info("  stage=" + std::to_string(referee->game_stage) +
         "  hp=" + std::to_string(referee->robot_hp) +
         "  robot_id=" + std::to_string(referee->robot_id) +
         "  blue=" + boolean(referee->is_on_blue_team) +
         "  chassis_power=" + boolean(referee->chassis_has_power) +
         "  gimbal_power=" + boolean(referee->gimbal_has_power));
    results.emplace_back(true, "~/ref_sys received");
  } else {
    fail("~/ref_sys — NO message received within " + fixed(timeout) + " s");
    results.emplace_back(false, "~/ref_sys timeout");
  }
  return results;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string device = "/dev/ttyTHS1", params, ns = "/dji_serial_bridge";
  int baud = 115200;
  double timeout = 10;
  bool config_only = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        std::cout << "DJI Serial Bridge config + live-traffic smoke test\n"
          "Usage: test_bridge [--device /dev/ttyTHS1] [--baudrate "
          "115200] [--params-file YAML]\n"
          "                   [--node-ns /dji_serial_bridge] "
          "[--timeout 10] [--config-only]\n";
        return 0;
      }
      if (arg == "--config-only") {
        config_only = true;
        continue;
      }
      if (++i >= argc) {
        throw std::runtime_error("incomplete argument " + arg);
      }
      if (arg == "--device") {
        device = argv[i];
      } else if (arg == "--baudrate") {
        baud = std::stoi(argv[i]);
      } else if (arg == "--params-file") {
        params = argv[i];
      } else if (arg == "--node-ns") {
        ns = argv[i];
      } else if (arg == "--timeout") {
        timeout = std::stod(argv[i]);
      } else {
        throw std::runtime_error("unknown argument " + arg);
      }
    }
    if (params.empty()) {
      std::error_code error;
      const auto program = std::filesystem::canonical("/proc/self/exe", error);
      const auto installed =
        program.parent_path().parent_path().parent_path() /
        "share/dji_serial_bridge/config/dji_bridge_params.yaml";
      for (const auto & candidate :
        {installed,
          std::filesystem::path("config/dji_bridge_params.yaml")})
      {
        if (std::filesystem::is_regular_file(candidate)) {
          params = std::filesystem::canonical(candidate).string();
          break;
        }
      }
    }
    std::cout << '\n'
              << kBold << "DJI Serial Bridge — Diagnostic Test" << kReset
              << "\n  device=" << device << "  baudrate=" << baud
              << "  timeout=" << timeout
              << "s  config_only=" << boolean(config_only) << "\n\n";
    auto results = check_config(device, baud, params);
    if (!config_only) {
      rclcpp::init(argc, argv);
      const auto live = check_ros_topics(ns, timeout);
      results.insert(results.end(), live.begin(), live.end());
      rclcpp::shutdown();
    } else {
      info("(skipping live topic check — --config-only)");
    }
    hdr("═══  SUMMARY  ════════════════════════════════════════════");
    std::size_t passed = 0;
    for (const auto & item : results) {
      if (item.first) {
        ok(item.second);
        ++passed;
      } else {
        fail(item.second);
      }
    }
    std::cout << '\n';
    if (passed == results.size()) {
      std::cout << "  " << kGreen << kBold << "ALL " << passed
                << " checks PASSED ✓" << kReset;
    } else {
      std::cout << "  " << kRed << kBold << results.size() - passed << '/'
                << results.size() << " checks FAILED" << kReset;
    }
    std::cout << "\n\n";
    return passed == results.size() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
