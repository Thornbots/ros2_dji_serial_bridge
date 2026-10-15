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

#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dji_serial_bridge/sim_uart.hpp"

namespace
{
using dji_serial_bridge::sim_uart::Packet;
using dji_serial_bridge::sim_uart::receive_packet;
using dji_serial_bridge::sim_uart::send_packet;

class SocketPair
{
public:
  SocketPair()
  {
    int fds[2];
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, fds) != 0) {
      throw std::runtime_error("socketpair failed");
    }
    a = fds[0];
    b = fds[1];
  }
  ~SocketPair()
  {
    if (a >= 0) {::close(a);}
    if (b >= 0) {::close(b);}
  }
  int a{-1};
  int b{-1};
};

void put_le(
  std::vector<uint8_t> & bytes, std::size_t offset, uint64_t value,
  std::size_t width)
{
  for (std::size_t i = 0; i < width; ++i) {
    bytes[offset + i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

std::vector<uint8_t> envelope(
  uint64_t declared_length,
  const std::vector<uint8_t> & payload = {})
{
  std::vector<uint8_t> bytes(28 + payload.size(), 0);
  std::memcpy(bytes.data(), "SUAR", 4);
  put_le(bytes, 4, 1, 4);
  put_le(bytes, 8, 15, 8);
  put_le(bytes, 16, 123, 8);
  put_le(bytes, 24, declared_length, 4);
  std::copy(payload.begin(), payload.end(), bytes.begin() + 28);
  return bytes;
}

void send_raw(int fd, const std::vector<uint8_t> & bytes)
{
  ASSERT_EQ(::send(fd, bytes.data(), bytes.size(), 0),
            static_cast<ssize_t>(bytes.size()));
}

TEST(SimUart, PreservesSequenceBytesAndSignedStamp)
{
  SocketPair sockets;
  const Packet sent{0xfedcba9876543210ULL,
    std::numeric_limits<int64_t>::min() + 12345,
    {0xa5, 0x00, 0xff, 0x17}};
  send_packet(sockets.a, sent);
  const auto received = receive_packet(sockets.b);
  ASSERT_TRUE(received);
  EXPECT_EQ(received->sequence, sent.sequence);
  EXPECT_EQ(received->stamp_ns, sent.stamp_ns);
  EXPECT_EQ(received->bytes, sent.bytes);
}

TEST(SimUart, EmptyPacketIsDeliveryAckAndWouldBlockIsEmpty)
{
  SocketPair sockets;
  EXPECT_FALSE(receive_packet(sockets.b, true));
  const Packet ack{42, -987654321, {}};
  send_packet(sockets.a, ack);
  const auto received = receive_packet(sockets.b, true);
  ASSERT_TRUE(received);
  EXPECT_EQ(received->sequence, ack.sequence);
  EXPECT_EQ(received->stamp_ns, ack.stamp_ns);
  EXPECT_TRUE(received->bytes.empty());
}

TEST(SimUart, RejectsBadMagicAndVersion)
{
  for (const auto & invalid : {std::make_pair(std::size_t{0}, uint8_t{0}),
      std::make_pair(std::size_t{4}, uint8_t{2})})
  {
    SocketPair sockets;
    auto bytes = envelope(0);
    bytes[invalid.first] = invalid.second;
    send_raw(sockets.a, bytes);
    EXPECT_THROW(receive_packet(sockets.b), std::runtime_error);
  }
}

TEST(SimUart, RejectsTruncatedEnvelopeAndLengthMismatch)
{
  {
    SocketPair sockets;
    send_raw(sockets.a, {0x53, 0x55, 0x41});
    EXPECT_THROW(receive_packet(sockets.b), std::runtime_error);
  }
  {
    SocketPair sockets;
    send_raw(sockets.a, envelope(2, {0xaa}));
    EXPECT_THROW(receive_packet(sockets.b), std::runtime_error);
  }
}

TEST(SimUart, RejectsOversizedPacketAndLocalPayload)
{
  {
    SocketPair sockets;
    std::vector<uint8_t> payload(dji_serial_bridge::sim_uart::max_payload + 1, 0x5a);
    send_raw(sockets.a, envelope(payload.size(), payload));
    EXPECT_THROW(receive_packet(sockets.b), std::length_error);
  }
  {
    SocketPair sockets;
    Packet too_large{1, 0,
      std::vector<uint8_t>(dji_serial_bridge::sim_uart::max_payload + 1)};
    EXPECT_THROW(send_packet(sockets.a, too_large), std::length_error);
  }
}

TEST(SimUart, RejectsEof)
{
  SocketPair sockets;
  ::close(sockets.a);
  sockets.a = -1;
  EXPECT_THROW(receive_packet(sockets.b), std::runtime_error);
}

TEST(SimUart, ConnectSocketSetsCloseOnExec)
{
  const std::string path = "/tmp/sim_uart_test_" + std::to_string(getpid());
  const int server = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  ASSERT_GE(server, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  ASSERT_LT(path.size(), sizeof(address.sun_path));
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  const auto address_length = static_cast<socklen_t>(
    offsetof(sockaddr_un, sun_path) + path.size() + 1);
  ASSERT_EQ(::bind(server, reinterpret_cast<const sockaddr *>(&address), address_length), 0);
  ASSERT_EQ(::listen(server, 1), 0);
  const int client = dji_serial_bridge::sim_uart::connect_socket(path);
  ASSERT_GE(client, 0);
  const int accepted = ::accept(server, nullptr, nullptr);
  ASSERT_GE(accepted, 0);
  const int flags = ::fcntl(client, F_GETFD);
  ASSERT_GE(flags, 0);
  EXPECT_NE(flags & FD_CLOEXEC, 0);
  ::close(accepted);
  ::close(client);
  ::close(server);
  ::unlink(path.c_str());
}
}  // namespace
