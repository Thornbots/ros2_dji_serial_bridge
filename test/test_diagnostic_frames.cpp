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
#include <pty.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "dji_serial_bridge/diagnostic_frames.hpp"

namespace
{
using dji_diagnostic::FrameParser;
const char * kValid =
  "[FRAME OK] seq= 42 msg=0x0004 len=  8 payload=00 00 a0 40 00 00 40 40\n";
std::string parse(FrameParser & parser, const std::vector<uint8_t> & bytes)
{
  testing::internal::CaptureStdout();
  parser.append(bytes.data(), bytes.size());
  return testing::internal::GetCapturedStdout();
}
TEST(DiagnosticFrames, MatchesPythonBytesAndFragmentedFrames) {
  const auto frame = dji_diagnostic::test_frame(42);
  EXPECT_EQ(dji_diagnostic::hex(frame.data(), frame.size()),
            "a5 08 00 2a bb 04 00 00 00 a0 40 00 00 40 40 10 74");
  FrameParser parser;
  EXPECT_EQ(parse(parser, {frame.begin(), frame.begin() + 6}), "");
  EXPECT_EQ(parse(parser, {frame.begin() + 6, frame.end()}), kValid);
  EXPECT_EQ(parser.frames, 1u);
  EXPECT_EQ(parser.bad_crc8 + parser.bad_crc16 + parser.desyncs, 0u);
}
TEST(DiagnosticFrames, MatchesPythonGarbageAndDesyncOutput) {
  auto frame = dji_diagnostic::test_frame(42);
  frame.insert(frame.begin(), {'a', 'b', 'c'});
  FrameParser parser;
  EXPECT_EQ(parse(parser, frame),
            std::string("[GARBAGE] skipped 3 bytes: 61 62 63\n") + kValid);
  EXPECT_EQ(parse(parser, {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'}),
            "[DESYNC] discarded 8 bytes\n");
  EXPECT_EQ(parser.frames, 1u);
  EXPECT_EQ(parser.desyncs, 2u);
}
TEST(DiagnosticFrames, MatchesPythonCorruptHeaderOutput) {
  auto frame = dji_diagnostic::test_frame(42);
  frame[4] ^= 1;
  FrameParser parser;
  EXPECT_EQ(parse(parser, frame),
            "[BAD CRC8] recv=0xBA calc=0xBB header=a5 08 00 2a ba 04 00\n"
            "[DESYNC] discarded 16 bytes\n");
  EXPECT_EQ(parser.bad_crc8, 1u);
  EXPECT_EQ(parser.frames, 0u);
}
TEST(DiagnosticFrames, MatchesPythonCorruptTrailerOutput) {
  auto frame = dji_diagnostic::test_frame(42);
  frame.back() ^= 1;
  FrameParser parser;
  EXPECT_EQ(parse(parser, frame),
            "[BAD CRC16] recv=0x7510 calc=0x7410\n"
            "frame: a5 08 00 2a bb 04 00 00 00 a0 40 00 00 40 40 10 "
            "75\n[DESYNC] discarded 16 bytes\n");
  EXPECT_EQ(parser.bad_crc16, 1u);
  EXPECT_EQ(parser.frames, 0u);
}
TEST(DiagnosticFrames, SerialPortConfiguresRaw8N1AndMovesBytes) {
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  {
    dji_diagnostic::SerialPort port(path, 115200);
    termios options{};
    ASSERT_EQ(tcgetattr(port.fd(), &options), 0);
    EXPECT_EQ(cfgetispeed(&options), B115200);
    EXPECT_EQ(options.c_cflag & CSIZE, static_cast<tcflag_t>(CS8));
    EXPECT_EQ(options.c_cflag & (PARENB | CSTOPB | CRTSCTS), 0u);
    EXPECT_EQ(options.c_lflag & (ICANON | ECHO), 0u);
    const char bytes[] = {'\x00', '\xa5', '\r', '\n'};
    EXPECT_EQ(write(port.fd(), bytes, sizeof(bytes)), 4);
    char received[4];
    EXPECT_EQ(read(master, received, sizeof(received)), 4);
    EXPECT_EQ(std::string(received, 4), std::string(bytes, 4));
  }
  close(master);
  close(slave);
}
}  // namespace
