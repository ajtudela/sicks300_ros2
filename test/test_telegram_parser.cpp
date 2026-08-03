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

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "sicks300_ros2/common/TelegramS300.hpp"
#include "telegram_test_helpers.hpp"

using test_helpers::buildDistanceTelegram;
using test_helpers::telegramLength;

TEST(TelegramParserTest, ParsesValidDistanceTelegram)
{
  const std::vector<uint16_t> points = {100, 250, 4004};
  std::vector<uint8_t> buffer = buildDistanceTelegram(points);

  TelegramParser parser;
  ASSERT_TRUE(parser.parseHeader(buffer.data(), buffer.size(), 7, false));
  EXPECT_TRUE(parser.isDist());
  EXPECT_EQ(parser.getField(), 1);
  EXPECT_EQ(static_cast<size_t>(parser.getCompletePacketSize()), telegramLength(points.size()));

  std::vector<int> raw;
  parser.readDistRaw(buffer.data(), raw, false);
  ASSERT_EQ(raw.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(raw[i], points[i]);
  }
}

TEST(TelegramParserTest, RejectsCorruptedCrc)
{
  std::vector<uint8_t> buffer = buildDistanceTelegram({100, 200});
  // Flip a bit in the middle of the payload without recomputing the CRC.
  buffer[25] ^= 0xFF;

  TelegramParser parser;
  EXPECT_FALSE(parser.parseHeader(buffer.data(), buffer.size(), 7, false));
}

TEST(TelegramParserTest, RejectsBufferSmallerThanFixedHeader)
{
  std::vector<uint8_t> buffer = buildDistanceTelegram({100});
  TelegramParser parser;
  // Fixed header (COMMON1) alone is 10 bytes; anything smaller must fail immediately.
  EXPECT_FALSE(parser.parseHeader(buffer.data(), 9, 7, false));
}

TEST(TelegramParserTest, RejectsMissingCoordinationFlag)
{
  std::vector<uint8_t> buffer = buildDistanceTelegram({100, 200});
  // byte 8 (coordination_flag) must be exactly 0xFF for the basic header check to pass.
  buffer[8] = 0x00;

  TelegramParser parser;
  EXPECT_FALSE(parser.parseHeader(buffer.data(), buffer.size(), 7, false));
}

TEST(TelegramParserTest, ReadsMultiplePointsAcrossFields)
{
  const std::vector<uint16_t> points = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<uint8_t> buffer = buildDistanceTelegram(points);

  TelegramParser parser;
  ASSERT_TRUE(parser.parseHeader(buffer.data(), buffer.size(), 7, false));

  std::vector<int> raw;
  parser.readDistRaw(buffer.data(), raw, false);
  ASSERT_EQ(raw.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(raw[i], points[i]);
  }
}

TEST(TelegramParserTest, CreateCrcIsDeterministicAndSensitiveToInput)
{
  uint8_t data_a[] = {0x01, 0x02, 0x03, 0x04};
  uint8_t data_b[] = {0x01, 0x02, 0x03, 0x05};

  unsigned int crc_a1 = TelegramParser::createCRC(data_a, sizeof(data_a));
  unsigned int crc_a2 = TelegramParser::createCRC(data_a, sizeof(data_a));
  unsigned int crc_b = TelegramParser::createCRC(data_b, sizeof(data_b));

  EXPECT_EQ(crc_a1, crc_a2);
  EXPECT_NE(crc_a1, crc_b);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
