// Copyright (c) 2022 Alberto J. Tudela Roldán
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "sicks300_ros2/common/ISerialIO.hpp"
#include "sicks300_ros2/common/ScannerSickS300.hpp"
#include "telegram_test_helpers.hpp"

using test_helpers::buildDistanceTelegram;

namespace
{

// Test double for ISerialIO that replays a queue of pre-loaded byte chunks instead of talking
// to a real serial port, letting ScannerSickS300::getScan() be exercised deterministically.
class FakeSerialIO : public ISerialIO
{
public:
  void setDeviceName(const char *) override {}
  void setBaudRate(int) override {}
  void setMultiplier(double) override {}
  void SetFormat(int, ParityFlags, int) override {}
  void setHandshake(HandshakeFlags) override {}
  void setBufferSize(int, int) override {}
  void setTimeout(double) override {}
  int openIO() override {return 0;}
  void closeIO() override {}
  void purge() override {}

  int readBlocking(char * buffer, int length) override
  {
    if (chunks_.empty()) {
      // No more data queued: mimic a serial read timeout with nothing received.
      return 0;
    }
    std::vector<uint8_t> & chunk = chunks_.front();
    const int n = std::min<int>(length, static_cast<int>(chunk.size()));
    std::memcpy(buffer, chunk.data(), n);
    chunk.erase(chunk.begin(), chunk.begin() + n);
    if (chunk.empty()) {
      chunks_.pop_front();
    }
    return n;
  }

  void pushChunk(std::vector<uint8_t> chunk) {chunks_.push_back(std::move(chunk));}

private:
  std::deque<std::vector<uint8_t>> chunks_;
};

ScannerSickS300::ParamType makeField1Param(double scale = 0.01)
{
  ScannerSickS300::ParamType param;
  param.range_field = 1;
  param.dScale = scale;
  param.dStartAngle = -M_PI_2;
  param.dStopAngle = M_PI_2;
  return param;
}

}  // namespace

TEST(ScannerSickS300Test, GetScanParsesValidTelegramIntoRangesAndAngles)
{
  auto fake_serial = std::make_unique<FakeSerialIO>();
  FakeSerialIO * fake_serial_ptr = fake_serial.get();
  fake_serial_ptr->pushChunk(buildDistanceTelegram({100, 200, 300}));

  ScannerSickS300 scanner(std::move(fake_serial));
  scanner.setRangeField(1, makeField1Param(0.01));
  ASSERT_TRUE(scanner.open("fake", 500000, 7));

  std::vector<double> ranges, angles, intensities;
  ASSERT_TRUE(scanner.getScan(ranges, angles, intensities, false));
  EXPECT_FALSE(scanner.isInStandby());

  ASSERT_EQ(ranges.size(), 3u);
  EXPECT_NEAR(ranges[0], 1.00, 1e-9);       // 100 * 0.01
  EXPECT_NEAR(ranges[1], 2.00, 1e-9);       // 200 * 0.01
  EXPECT_NEAR(ranges[2], 3.00, 1e-9);       // 300 * 0.01

  ASSERT_EQ(angles.size(), 3u);
  EXPECT_NEAR(angles.front(), -M_PI_2, 1e-9);
  EXPECT_NEAR(angles.back(), M_PI_2, 1e-9);
  // Angles must be monotonically increasing across the field.
  EXPECT_LT(angles[0], angles[1]);
  EXPECT_LT(angles[1], angles[2]);
}

TEST(ScannerSickS300Test, DetectsStandbyWhenAllPointsAreTheStandbyMarker)
{
  auto fake_serial = std::make_unique<FakeSerialIO>();
  fake_serial->pushChunk(buildDistanceTelegram({0x4004, 0x4004, 0x4004}));

  ScannerSickS300 scanner(std::move(fake_serial));
  scanner.setRangeField(1, makeField1Param());
  ASSERT_TRUE(scanner.open("fake", 500000, 7));

  std::vector<double> ranges, angles, intensities;
  ASSERT_TRUE(scanner.getScan(ranges, angles, intensities, false));
  EXPECT_TRUE(scanner.isInStandby());
}

TEST(ScannerSickS300Test, NotInStandbyWhenAtLeastOnePointHasData)
{
  auto fake_serial = std::make_unique<FakeSerialIO>();
  fake_serial->pushChunk(buildDistanceTelegram({0x4004, 150, 0x4004}));

  ScannerSickS300 scanner(std::move(fake_serial));
  scanner.setRangeField(1, makeField1Param());
  ASSERT_TRUE(scanner.open("fake", 500000, 7));

  std::vector<double> ranges, angles, intensities;
  ASSERT_TRUE(scanner.getScan(ranges, angles, intensities, false));
  EXPECT_FALSE(scanner.isInStandby());
}

TEST(ScannerSickS300Test, GetScanFailsWhenNoDataAvailable)
{
  // No chunk pushed at all: FakeSerialIO::readBlocking() returns 0, as a real read timeout
  // would when the scanner has gone silent.
  auto scanner_serial = std::make_unique<FakeSerialIO>();
  ScannerSickS300 scanner(std::move(scanner_serial));
  scanner.setRangeField(1, makeField1Param());
  ASSERT_TRUE(scanner.open("fake", 500000, 7));

  std::vector<double> ranges, angles, intensities;
  EXPECT_FALSE(scanner.getScan(ranges, angles, intensities, false));
}

TEST(ScannerSickS300Test, GetScanCompactsBufferAcrossAPartialTelegram)
{
  // A complete telegram (A) followed by the first half of a second one (B) arrives in a
  // single read. getScan() must return A and correctly keep B's partial bytes at the front
  // of the receive buffer (a `memmove` of the *remaining*, not *consumed*, byte count - see
  // the H04 fix) so that once the rest of B arrives on a later read, it can still be parsed.
  std::vector<uint8_t> telegram_a = buildDistanceTelegram({10, 20});
  telegram_a.resize(test_helpers::telegramLength(2));       // drop the safety padding
  std::vector<uint8_t> telegram_b = buildDistanceTelegram({30, 40});
  telegram_b.resize(test_helpers::telegramLength(2));       // drop the safety padding

  const size_t split = telegram_b.size() / 2;
  std::vector<uint8_t> first_chunk = telegram_a;
  first_chunk.insert(first_chunk.end(), telegram_b.begin(), telegram_b.begin() + split);
  std::vector<uint8_t> second_chunk(telegram_b.begin() + split, telegram_b.end());

  auto fake_serial = std::make_unique<FakeSerialIO>();
  fake_serial->pushChunk(first_chunk);
  fake_serial->pushChunk(second_chunk);

  ScannerSickS300 scanner(std::move(fake_serial));
  scanner.setRangeField(1, makeField1Param(1.0));
  ASSERT_TRUE(scanner.open("fake", 500000, 7));

  std::vector<double> ranges, angles, intensities;
  ASSERT_TRUE(scanner.getScan(ranges, angles, intensities, false));
  ASSERT_EQ(ranges.size(), 2u);
  EXPECT_NEAR(ranges[0], 10.0, 1e-9);
  EXPECT_NEAR(ranges[1], 20.0, 1e-9);

  ASSERT_TRUE(scanner.getScan(ranges, angles, intensities, false));
  ASSERT_EQ(ranges.size(), 2u);
  EXPECT_NEAR(ranges[0], 30.0, 1e-9);
  EXPECT_NEAR(ranges[1], 40.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
