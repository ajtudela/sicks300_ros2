/*
 * Copyright 2026 omniLink
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "sicks300_ros2/common/TelegramS300.hpp"

namespace
{

std::array<unsigned char, 22> basic_header()
{
  std::array<unsigned char, 22> telegram{};
  telegram[8] = 0xFF;
  telegram[9] = 0x07;
  return telegram;
}

TEST(TelegramParser, RejectsNullBuffer)
{
  TelegramParser parser;
  EXPECT_FALSE(parser.parseHeader(nullptr, 0, 7, false));
}

TEST(TelegramParser, RejectsTruncatedCommonHeader)
{
  auto telegram = basic_header();
  TelegramParser parser;

  EXPECT_FALSE(parser.parseHeader(telegram.data(), 10, 7, false));
}

TEST(TelegramParser, RejectsPacketSizeBeyondAvailableBytes)
{
  auto telegram = basic_header();
  telegram[6] = 0x00;
  telegram[7] = 0x20;
  telegram[10] = 0x01;
  telegram[11] = 0x02;
  TelegramParser parser;

  EXPECT_FALSE(parser.parseHeader(telegram.data(), telegram.size(), 7, false));
}

}  // namespace
