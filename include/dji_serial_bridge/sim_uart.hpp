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

#ifndef DJI_SERIAL_BRIDGE__SIM_UART_HPP_
#define DJI_SERIAL_BRIDGE__SIM_UART_HPP_

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dji_serial_bridge::sim_uart
{

struct Packet
{
  uint64_t sequence;
  int64_t stamp_ns;
  std::vector<uint8_t> bytes;
};

inline constexpr std::size_t max_payload = 4096;

namespace detail
{
inline constexpr std::size_t header_size = 28;

inline void put_le(uint8_t * out, uint64_t value, std::size_t size)
{
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

inline uint64_t get_le(const uint8_t * in, std::size_t size)
{
  uint64_t value = 0;
  for (std::size_t i = 0; i < size; ++i) {
    value |= static_cast<uint64_t>(in[i]) << (8 * i);
  }
  return value;
}

inline int no_signal_flag()
{
#ifdef MSG_NOSIGNAL
  return MSG_NOSIGNAL;
#else
  return 0;
#endif
}
}  // namespace detail

inline void send_packet(int fd, const Packet & packet)
{
  if (packet.bytes.size() > max_payload) {
    throw std::length_error("sim UART payload exceeds 4096 bytes");
  }
  std::array<uint8_t, detail::header_size> header{};
  std::memcpy(header.data(), "SUAR", 4);
  detail::put_le(header.data() + 4, 1, 4);
  detail::put_le(header.data() + 8, packet.sequence, 8);
  uint64_t stamp_bits = 0;
  static_assert(sizeof(stamp_bits) == sizeof(packet.stamp_ns));
  std::memcpy(&stamp_bits, &packet.stamp_ns, sizeof(stamp_bits));
  detail::put_le(header.data() + 16, stamp_bits, 8);
  detail::put_le(header.data() + 24, packet.bytes.size(), 4);

  iovec parts[2]{};
  parts[0].iov_base = header.data();
  parts[0].iov_len = header.size();
  parts[1].iov_base = const_cast<uint8_t *>(packet.bytes.data());
  parts[1].iov_len = packet.bytes.size();
  msghdr message{};
  message.msg_iov = parts;
  message.msg_iovlen = packet.bytes.empty() ? 1 : 2;
  ssize_t sent;
  do {
    sent = ::sendmsg(fd, &message, detail::no_signal_flag());
  } while (sent < 0 && errno == EINTR);
  const auto expected = static_cast<ssize_t>(header.size() + packet.bytes.size());
  if (sent < 0) {
    throw std::runtime_error("sim UART sendmsg: " + std::string(std::strerror(errno)));
  }
  if (sent != expected) {
    throw std::runtime_error("sim UART sendmsg returned a short packet");
  }
}

inline std::optional<Packet> receive_packet(int fd, bool nonblocking = false)
{
  std::array<uint8_t, detail::header_size + max_payload> buffer{};
  iovec part{};
  part.iov_base = buffer.data();
  part.iov_len = buffer.size();
  msghdr message{};
  message.msg_iov = &part;
  message.msg_iovlen = 1;
  const int flags = nonblocking ? MSG_DONTWAIT : 0;
  ssize_t received;
  do {
    received = ::recvmsg(fd, &message, flags);
  } while (received < 0 && errno == EINTR);
  if (received < 0) {
    if (nonblocking && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return std::nullopt;
    }
    throw std::runtime_error("sim UART recvmsg: " + std::string(std::strerror(errno)));
  }
  if (received == 0) {
    throw std::runtime_error("sim UART peer closed the socket");
  }
  if ((message.msg_flags & MSG_TRUNC) != 0 ||
    static_cast<std::size_t>(received) > buffer.size())
  {
    throw std::length_error("sim UART packet exceeds 4096-byte payload limit");
  }
  if (static_cast<std::size_t>(received) < detail::header_size) {
    throw std::runtime_error("truncated sim UART envelope");
  }
  if (std::memcmp(buffer.data(), "SUAR", 4) != 0) {
    throw std::runtime_error("invalid sim UART magic");
  }
  if (detail::get_le(buffer.data() + 4, 4) != 1) {
    throw std::runtime_error("unsupported sim UART envelope version");
  }
  const uint64_t length = detail::get_le(buffer.data() + 24, 4);
  if (length > max_payload) {
    throw std::length_error("sim UART payload exceeds 4096 bytes");
  }
  if (static_cast<std::size_t>(received) != detail::header_size + length) {
    throw std::runtime_error("sim UART envelope length does not match packet size");
  }

  const uint64_t stamp_bits = detail::get_le(buffer.data() + 16, 8);
  int64_t stamp_ns = 0;
  std::memcpy(&stamp_ns, &stamp_bits, sizeof(stamp_ns));
  Packet packet{
    detail::get_le(buffer.data() + 8, 8), stamp_ns,
    std::vector<uint8_t>(buffer.begin() + detail::header_size,
      buffer.begin() + detail::header_size + static_cast<std::ptrdiff_t>(length))};
  return packet;
}

inline int connect_socket(const std::string & path)
{
  sockaddr_un address{};
  if (path.empty() || path.find('\0') != std::string::npos ||
    path.size() >= sizeof(address.sun_path))
  {
    throw std::invalid_argument("invalid sim UART socket path");
  }
  const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (fd < 0) {
    throw std::runtime_error("sim UART socket: " + std::string(std::strerror(errno)));
  }
  auto close_on_error = [&fd]() {::close(fd);};
  const int descriptor_flags = ::fcntl(fd, F_GETFD);
  if (descriptor_flags < 0 || ::fcntl(fd, F_SETFD, descriptor_flags | FD_CLOEXEC) < 0) {
    const std::string error = std::strerror(errno);
    close_on_error();
    throw std::runtime_error("sim UART FD_CLOEXEC: " + error);
  }
#ifdef SO_NOSIGPIPE
  const int enabled = 1;
  if (::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled)) < 0) {
    const std::string error = std::strerror(errno);
    close_on_error();
    throw std::runtime_error("sim UART SO_NOSIGPIPE: " + error);
  }
#endif
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  const auto address_length = static_cast<socklen_t>(
    offsetof(sockaddr_un, sun_path) + path.size() + 1);
  if (::connect(fd, reinterpret_cast<const sockaddr *>(&address), address_length) < 0) {
    const std::string error = std::strerror(errno);
    close_on_error();
    throw std::runtime_error("sim UART connect: " + error);
  }
  return fd;
}

}  // namespace dji_serial_bridge::sim_uart

#endif  // DJI_SERIAL_BRIDGE__SIM_UART_HPP_
