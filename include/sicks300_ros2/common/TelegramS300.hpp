/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <arpa/inet.h>
#include <iostream>
#include <vector>

/*
* S300 header format in continuous mode:
*
*      | 00 00 00 00 |   4 byte reply header
*      | 00 00 |         data block number (fixed for continuous output)
*      | xx xx |         size of data telegram in 16-bit data words
*      | FF xx |         coordination flag and device address (07 in most cases, but 08 for slave configured scanners)
*      | xx xx |         protocol version (02 01 for old protocol,.otherwise 03 01)
*      | 0x 00 |         status: 00 00 = normal, 01 00 = lockout
*      | xx xx xx xx |   scan number (time stamp)
*      | xx xx |         telegram number
*      | BB BB |         ID of output (AAAA=I/O, BBBB=range measruements, CCCC=reflector measurements)
*      | 11 11 |         number of configures measurement field (1111, 2222, 3333, 4444 or 5555)
*         ...            data
*      | xx xx |         CRC
*
* in this parser, user_data_ denotes all but the first 20 bytes (up to and including telegram number above)
* and the last two bytes (CRC)
*
*/


class TelegramParser
{
  #pragma pack(push, 1)
  union TELEGRAM_COMMON1 {
    struct COMMON1
    {
      uint32_t reply_telegram;
      uint16_t trigger_result;
      uint16_t size;                   // in 16bit=2byte words
      uint8_t coordination_flag;
      uint8_t device_addresss;
    } common1;
    uint8_t bytes[10];
  };
  union TELEGRAM_COMMON2 {
    struct COMMON2
    {
      uint16_t protocol_version;
      uint16_t status;
      uint32_t scan_number;
      uint16_t telegram_number;
    } common2;
    uint8_t bytes[10];
  };
  union TELEGRAM_COMMON3 {
    struct TYPE
    {
      uint16_t type;
    } type;
    uint8_t bytes[2];
  };

  union TELEGRAM_DISTANCE {
    struct TYPE
    {
      uint16_t type;
    } type;
    uint8_t bytes[2];
  };

  union TELEGRAM_TAIL {
    struct CRC
    {
      uint16_t crc;
    } crc_struct;
    uint8_t bytes[2];
  };

  union TELEGRAM_S300_DIST_2B {
    struct DISTANCE
    {
      unsigned distance : 13;                // cm
      unsigned bit13 : 1;                    // reflector or scanner distorted
      unsigned protective : 1;
      unsigned warn_field : 1;
    } distance_struct;
    uint16_t val16;
    uint8_t bytes[2];
  };

  #pragma pack(pop)

  enum TELEGRAM_COMMON_HS {JUNK_SIZE = 4};
  enum TELEGRAM_COMMON_TYPES {IO = 0xAAAA, DISTANCE = 0xBBBB, REFLEXION = 0xCCCC};
  enum TELEGRAM_DIST_SECTOR {_1 = 0x1111, _2 = 0x2222, _3 = 0x3333, _4 = 0x4444, _5 = 0x5555};

  static void ntoh(TELEGRAM_COMMON1 & tc)
  {
    tc.common1.reply_telegram = ntohl(tc.common1.reply_telegram);
    tc.common1.trigger_result = ntohs(tc.common1.trigger_result);
    tc.common1.size = ntohs(tc.common1.size);
  }

  static void ntoh(TELEGRAM_COMMON2 & tc)
  {
    tc.common2.protocol_version = ntohs(tc.common2.protocol_version);
    tc.common2.status = ntohs(tc.common2.status);
    tc.common2.scan_number = ntohl(tc.common2.scan_number);
    tc.common2.telegram_number = ntohs(tc.common2.telegram_number);
  }

  static void ntoh(TELEGRAM_COMMON3 & tc)
  {
    tc.type.type = ntohs(tc.type.type);
  }

  static void ntoh(TELEGRAM_DISTANCE & tc)
  {
    tc.type.type = ntohs(tc.type.type);
  }

  static void ntoh(TELEGRAM_TAIL & /* tc */)
  {
    // crc calc. is also in network order
    // tc.crc = ntohs(tc.crc);
  }

  static void print(const TELEGRAM_COMMON1 & tc)
  {
    std::cout << "HEADER" << std::dec << std::endl;
    std::cout << "reply_telegram" << ":" << tc.common1.reply_telegram << std::endl;
    std::cout << "trigger_result" << ":" << tc.common1.trigger_result << std::endl;
    std::cout << "size" << ":" << 2 * tc.common1.size << std::endl;
    std::cout << "coordination_flag" << ":" << std::hex <<
      tc.common1.coordination_flag << std::endl;
    std::cout << "device_addresss" << ":" << std::hex <<
      static_cast<int>(tc.common1.device_addresss) << std::endl;
  }

  static void print(const TELEGRAM_COMMON2 & tc)
  {
    std::cout << "protocol_version" << ":" << std::hex << tc.common2.protocol_version << std::endl;
    std::cout << "status" << ":" << tc.common2.status << std::endl;
    std::cout << "scan_number" << ":" << std::hex << tc.common2.scan_number << std::endl;
    std::cout << "telegram_number" << ":" << std::hex << tc.common2.telegram_number << std::endl;
  }

  static void print(const TELEGRAM_COMMON3 & tc)
  {
    std::cout << "type" << ":" << std::hex << tc.type.type << std::endl;
    switch (tc.type.type) {
      case IO: std::cout << "type" << ": " << "IO" << std::endl; break;
      case DISTANCE: std::cout << "type" << ": " << "DISTANCE" << std::endl; break;
      case REFLEXION: std::cout << "type" << ": " << "REFLEXION" << std::endl; break;
      default: std::cout << "type" << ": " << "unknown " << tc.type.type << std::endl; break;
    }
    std::cout << std::dec << std::endl;
  }

  static void print(const TELEGRAM_DISTANCE & tc)
  {
    std::cout << "DISTANCE" << std::endl;
    std::cout << "type" << ":" << std::hex << tc.type.type << std::endl;
    switch (tc.type.type) {
      case _1: std::cout << "field 1" << std::endl; break;
      case _2: std::cout << "field 2" << std::endl; break;
      case _3: std::cout << "field 3" << std::endl; break;
      case _4: std::cout << "field 4" << std::endl; break;
      case _5: std::cout << "field 5" << std::endl; break;
      default: std::cout << "unknown " << tc.type.type << std::endl; break;
    }
    std::cout << std::dec << std::endl;
  }

  static void print(const TELEGRAM_TAIL & tc)
  {
    std::cout << "TAIL" << std::endl;
    std::cout << "crc" << ":" << std::hex << tc.crc_struct.crc << std::endl;
    std::cout << std::dec << std::endl;
  }

  //-------------------------------------------
  static unsigned int createCRC(uint8_t * ptrData, int Size);

  // Supports versions: 0301, 0201
  static bool check(const TELEGRAM_COMMON1 & tc, const uint8_t DEVICE_ADDR)
  {
    uint8_t TELEGRAM_COMMON_PATTERN_EQ[] =
    {0, 0, 0, 0, 0, 0, 0, 0, 0xFF, static_cast<uint8_t>(0 & DEVICE_ADDR) /*version, 2, 1*/};
    uint8_t TELEGRAM_COMMON_PATTERN_OR[] =
    {0, 0, 0, 0, 0, 0, 0xff, 0xff, 0, 0xff /*version, 1, 0*/};

    for (size_t i = 0; i < sizeof(TELEGRAM_COMMON_PATTERN_EQ); i++) {
      if (TELEGRAM_COMMON_PATTERN_EQ[i] != (tc.bytes[i] & (~TELEGRAM_COMMON_PATTERN_OR[i])) ) {
        // std::cout<<"invalid at byte "<<i<<std::endl;
        return false;
      }
    }

    return true;
  }

  TELEGRAM_COMMON1 tc1_;
  TELEGRAM_COMMON2 tc2_;
  TELEGRAM_COMMON3 tc3_;
  TELEGRAM_DISTANCE td_;
  int size_field_start_byte_, crc_bytes_in_size_, user_data_size_;

public:
  TelegramParser()
  : size_field_start_byte_(0),
    crc_bytes_in_size_(0),
    user_data_size_(0)
  {
  }

  bool parseHeader(
    const unsigned char * buffer, const size_t max_size, const uint8_t DEVICE_ADDR,
    const bool debug)
  {
    if (sizeof(tc1_) > max_size) {return false;}
    tc1_ = *reinterpret_cast<const TELEGRAM_COMMON1 *>(buffer);

    if (!check(tc1_, DEVICE_ADDR)) {
      // if(debug) std::cout<<"basic check failed"<<std::endl;
      return false;
    }

    ntoh(tc1_);
    if (debug) {print(tc1_);}

    tc2_ = *(reinterpret_cast<const TELEGRAM_COMMON2 *>(buffer + sizeof(TELEGRAM_COMMON1)));
    tc3_ =
      *(reinterpret_cast<const TELEGRAM_COMMON3 *>(buffer +
      (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2))));

    TELEGRAM_TAIL tt;
    uint16_t crc;
    int full_data_size = 0;

    // The size reported by the protocol varies depending on the calculation
    // which is different depending on several factors.
    // The calculation is described on pp. 70-73 in:
    // https://www.sick.com/media/dox/1/91/891/Telegram_listing_S3000_Expert_Anti_Collision_S300_Expert_de_en_IM0022891.PDF // NOLINT
    //
    // Also, the size is reported as 16bit-words = 2 bytes...
    //
    // For the old protocol/compatability mode:
    // "The telegram size is calculated starting with the 5 byte up to and including the CRC."
    if (tc2_.common2.protocol_version == 0x102) {
      size_field_start_byte_ = 4;               // start at 5th byte (started numbering at 0)
      crc_bytes_in_size_ = 2;                   // include 2 bytes CRC

      // the user_data_size is the size of the actual payload data in bytes,
      // i.e. all data except of the CRC and the first two common telegrams
      user_data_size_ =
        2 * tc1_.common1.size -
        (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) - size_field_start_byte_ +
        crc_bytes_in_size_);
      full_data_size = sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_ +
        sizeof(TELEGRAM_TAIL);

      tt =
        *(reinterpret_cast<const TELEGRAM_TAIL *>(buffer +
        (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_)) );
      ntoh(tt);
      crc =
        createCRC(
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(buffer)) + JUNK_SIZE,
        full_data_size - JUNK_SIZE - sizeof(TELEGRAM_TAIL));
    } else {
      // Special handling for the new protocol, as the settings cannot be fully deduced
      // from the protocol itself
      // Thus, we have to try both possibilities and check against the CRC...
      // If NO I/O or measuring fields are configured:
      // "The telegram size is calculated starting with the 9 byte up to and including the CRC."
      size_field_start_byte_ = 8;               // start at 9th byte (started numbering at 0)
      crc_bytes_in_size_ = 2;                   // include 2 bytes CRC
      user_data_size_ =
        2 * tc1_.common1.size -
        (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) - size_field_start_byte_ +
        crc_bytes_in_size_);
      full_data_size = sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_ +
        sizeof(TELEGRAM_TAIL);

      tt =
        *(reinterpret_cast<const TELEGRAM_TAIL *>(buffer +
        (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_)) );
      ntoh(tt);
      crc =
        createCRC(
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(buffer)) + JUNK_SIZE,
        full_data_size - JUNK_SIZE - sizeof(TELEGRAM_TAIL));

      if (tt.crc_struct.crc != crc) {
        // If any I/O or measuring field is configured:
        // "The telegram size is calculated starting with the 13 byte up to and including the
        // last byte bevfre (sic!) the CRC."
        size_field_start_byte_ = 12;                // start at 13th byte (started numbering at 0)
        crc_bytes_in_size_ = 0;                     // do NOT include 2 bytes CRC
        user_data_size_ =
          2 * tc1_.common1.size -
          (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) - size_field_start_byte_ +
          crc_bytes_in_size_);
        full_data_size =
          sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_ +
          sizeof(TELEGRAM_TAIL);

        tt =
          *(reinterpret_cast<const TELEGRAM_TAIL *>(buffer +
          (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_)) );
        ntoh(tt);
        crc =
          createCRC(
          const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(buffer)) + JUNK_SIZE,
          full_data_size - JUNK_SIZE - sizeof(TELEGRAM_TAIL));
      }
    }

    if ( (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_ +
      sizeof(TELEGRAM_TAIL)) > static_cast<size_t>(max_size))
    {
      if (debug) {std::cout << "invalid header size" << std::endl;}
      return false;
    }


    if (tt.crc_struct.crc != crc) {
      if (debug) {
        print(tc2_);
        print(tc3_);
        print(tt);
        std::cout << "at " << std::dec <<
          (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_) << std::hex <<
          std::endl;
        std::cout << "invalid CRC: " << crc << " (" << tt.crc_struct.crc << ")" << std::endl;
      }
      return false;
    }

    memset(&td_, 0, sizeof(td_));
    switch (tc3_.type.type) {
      case IO: break;

      case DISTANCE:
        if (debug) {std::cout << "got distance" << std::endl;}

        td_ =
          *(reinterpret_cast<const TELEGRAM_DISTANCE *>(buffer + sizeof(TELEGRAM_COMMON1) +
          sizeof(TELEGRAM_COMMON2) + sizeof(TELEGRAM_COMMON3)));
        ntoh(td_);
        // print(td_);
        break;

      case REFLEXION: break;
      default: return false;
    }

    return true;
  }

  bool isDist() const {return tc3_.type.type == DISTANCE;}
  int getField() const
  {
    switch (td_.type.type) {
      case _1: return 1;
      case _2: return 2;
      case _3: return 3;
      case _4: return 4;
      case _5: return 5;
      default: return -1;
    }
  }

  int getCompletePacketSize() const
  {
    return sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) + user_data_size_ +
           sizeof(TELEGRAM_TAIL);
  }

  void readDistRaw(const unsigned char * buffer, std::vector<int> & res, bool debug) const
  {
    res.clear();
    if (!isDist()) {return;}

    size_t num_points =
      (user_data_size_ - sizeof(TELEGRAM_COMMON3) - sizeof(TELEGRAM_DISTANCE)) /
      sizeof(TELEGRAM_S300_DIST_2B);
    if (debug) {std::cout << "Number of points: " << std::dec << num_points << std::endl;}
    for (size_t i = 0; i < num_points; ++i) {
      TELEGRAM_S300_DIST_2B dist =
        *(reinterpret_cast<const TELEGRAM_S300_DIST_2B *>(buffer +
        (sizeof(TELEGRAM_COMMON1) + sizeof(TELEGRAM_COMMON2) +
        sizeof(TELEGRAM_COMMON3) + sizeof(TELEGRAM_DISTANCE) +
        i * sizeof(TELEGRAM_S300_DIST_2B))) );
      // for distance only: res.push_back((int)dist.distance);
      res.push_back(static_cast<int>(dist.val16));
    }
  }
};
