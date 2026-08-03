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

#ifndef TELEGRAM_TEST_HELPERS_HPP_
#define TELEGRAM_TEST_HELPERS_HPP_

#include <arpa/inet.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "sicks300_ros2/common/TelegramS300.hpp"

namespace test_helpers
{

// TelegramParser::parseHeader(), when the CRC of its first size-guess doesn't match, retries
// with a second, larger size guess *before* checking that guess against `max_size` - so a
// mismatching CRC can make it read a few bytes past a tightly-sized buffer. Pad every buffer
// with this many extra (unused) bytes so tests that exercise the CRC-mismatch path don't turn
// into a real out-of-bounds heap read; it does not affect what a correctly-sized parse sees,
// since the parser derives lengths from the telegram's own `size` field, not from how much
// memory happens to be allocated after it.
constexpr size_t kOverreadSafetyPadding = 16;

// Returns the exact byte length of the telegram built by buildDistanceTelegram() for the
// given number of points (i.e. excluding the safety padding described above).
inline size_t telegramLength(size_t num_points)
{
  // Common1 (10) + Common2 (10) + Common3 (2) + field marker (2) + points (2*N) + CRC (2)
  return 10 + 10 + 2 + 2 + 2 * num_points + 2;
}

// Builds a synthetic, protocol-2.10 ("new protocol"), single measurement-field (field 1),
// distance-only S300 telegram wrapping the given raw 16-bit points (already scaled/packed the
// way the scanner would send them, e.g. 100 for 1.00m with a 0.01 scale, or 0x4004 for a
// standby/no-data point), following the layout documented in TelegramS300.hpp. The CRC is
// computed with the production TelegramParser::createCRC(), so the result decodes correctly
// through the real parser. The returned buffer is telegramLength(raw_points.size()) bytes of
// telegram followed by kOverreadSafetyPadding zero bytes; pass telegramLength(...) as the
// logical size, not buffer.size().
inline std::vector<uint8_t> buildDistanceTelegram(
  const std::vector<uint16_t> & raw_points, uint8_t device_addr = 7)
{
  const size_t num_points = raw_points.size();
  std::vector<uint8_t> buf(telegramLength(num_points) + kOverreadSafetyPadding, 0);

  // COMMON1: reply_telegram(4)=0, trigger_result(2)=0, size(2), coordination_flag(1)=0xFF,
  // device_addresss(1)
  const uint16_t size_words = static_cast<uint16_t>(9 + num_points);
  uint16_t size_net = htons(size_words);
  std::memcpy(buf.data() + 6, &size_net, 2);
  buf[8] = 0xFF;
  buf[9] = device_addr;

  // COMMON2: protocol_version(2), status(2)=0, scan_number(4), telegram_number(2)
  uint16_t protocol_version_net = htons(0x0301);
  std::memcpy(buf.data() + 10, &protocol_version_net, 2);
  uint32_t scan_number_net = htonl(42);
  std::memcpy(buf.data() + 14, &scan_number_net, 4);
  uint16_t telegram_number_net = htons(1);
  std::memcpy(buf.data() + 18, &telegram_number_net, 2);

  // COMMON3: type(2) = DISTANCE (0xBBBB)
  uint16_t type_net = htons(0xBBBB);
  std::memcpy(buf.data() + 20, &type_net, 2);

  // TELEGRAM_DISTANCE: field marker(2) = field 1 (0x1111)
  uint16_t field_net = htons(0x1111);
  std::memcpy(buf.data() + 22, &field_net, 2);

  // Distance points: read back with no ntoh() applied by the parser, so write them in the
  // host's native byte order, matching how readDistRaw() will interpret them.
  for (size_t i = 0; i < num_points; ++i) {
    std::memcpy(buf.data() + 24 + 2 * i, &raw_points[i], 2);
  }

  // CRC (TAIL): also read back with no byte-swap (TelegramParser::ntoh(TELEGRAM_TAIL&) is a
  // no-op), so write the raw numeric value directly.
  const size_t crc_offset = 24 + 2 * num_points;
  uint16_t crc = static_cast<uint16_t>(
    TelegramParser::createCRC(buf.data() + 4, static_cast<int>(crc_offset - 4)));
  std::memcpy(buf.data() + crc_offset, &crc, 2);

  return buf;
}

}  // namespace test_helpers

#endif  // TELEGRAM_TEST_HELPERS_HPP_
