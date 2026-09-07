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

#ifndef SICKS300_ROS2__COMMON__ISERIALIO_HPP_
#define SICKS300_ROS2__COMMON__ISERIALIO_HPP_

/**
 * Minimal transport interface used by ScannerSickS300 to talk to the scanner.
 *
 * Extracting this interface out of the concrete `SerialIO` (a thin wrapper around POSIX
 * termios) allows injecting a test double that replays captured telegrams without any
 * real hardware, and would allow adding alternative transports (e.g. TCP/IP) later
 * without touching the telegram parsing logic.
 */
class ISerialIO
{
public:
  /// Constants for defining the handshake.
  enum HandshakeFlags
  {
    HS_NONE,
    HS_HARDWARE,
    HS_XONXOFF
  };

  /// Constants for defining the parity bits.
  enum ParityFlags
  {
    PA_NONE,
    PA_EVEN,
    PA_ODD,
    PA_MARK,
    PA_SPACE
  };

  /// Constants for defining the stop bits.
  enum StopBits
  {
    SB_ONE,
    SB_ONE_5,
    SB_TWO
  };

  virtual ~ISerialIO() = default;

  /**
   * Sets the device name.
   * @param name 'COM1', 'COM2', '/dev/ttyUSB0', ...
   */
  virtual void setDeviceName(const char * name) = 0;

  /**
   * Sets the baudrate.
   * @param baud_rate baudrate.
   */
  virtual void setBaudRate(int baud_rate) = 0;

  /**
   * Sets a multiplier for the baudrate.
   * Some serial cards need a specific multiplier for the baudrate.
   * @param multiplier default is one.
   */
  virtual void setMultiplier(double multiplier = 1) = 0;

  /**
   * Sets the message format.
   */
  virtual void SetFormat(int byte_size, ParityFlags parity, int stop_bits) = 0;

  /**
   * Defines the handshake type.
   */
  virtual void setHandshake(HandshakeFlags handshake) = 0;

  /**
   * Sets the buffer sizes.
   * @param read_buf_size number of bytes of the read buffer.
   * @param write_buf_size number of bytes of the write buffer.
   */
  virtual void setBufferSize(int read_buf_size, int write_buf_size) = 0;

  /**
   * Sets the read timeout.
   * @param timeout in seconds
   */
  virtual void setTimeout(double timeout) = 0;

  /**
   * Opens the transport. It has to be configured before.
   * @return 0 on success.
   */
  virtual int openIO() = 0;

  /**
   * Closes the transport.
   */
  virtual void closeIO() = 0;

  /**
   * Reads blocking. Blocks until at least one byte has been read or an error occurs.
   * @param buffer pointer to the buffer.
   * @param length maximum number of bytes to read
   * @return number of bytes read, or a negative value on error.
   */
  virtual int readBlocking(char * buffer, int length) = 0;

  /**
   * Clears the read and transmit buffers.
   */
  virtual void purge() = 0;
};

#endif  // SICKS300_ROS2__COMMON__ISERIALIO_HPP_
