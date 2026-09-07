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

#ifndef SICKS300_ROS2__COMMON__SERIALIO_HPP_
#define SICKS300_ROS2__COMMON__SERIALIO_HPP_

#include <termios.h>
#include <sys/select.h>
#include <string.h>

#include <string>

#include "sicks300_ros2/common/ISerialIO.hpp"

/**
 * Wrapper class for serial communication, implementing the ISerialIO transport interface.
 */
class SerialIO : public ISerialIO
{
public:
  /// Default constructor
  SerialIO();

  /// Destructor
  ~SerialIO() override;

  void setDeviceName(const char * Name) override {m_DeviceName = Name;}

  void setBaudRate(int BaudRate) override {m_BaudRate = BaudRate;}

  void setMultiplier(double Multiplier = 1) override {m_Multiplier = Multiplier;}

  void SetFormat(int ByteSize, ParityFlags Parity, int stopBits) override
  {m_ByteSize = ByteSize; m_Parity = Parity; m_StopBits = stopBits;}

  void setHandshake(HandshakeFlags Handshake) override {m_Handshake = Handshake;}

  void setBufferSize(int ReadBufSize, int WriteBufSize) override
  {m_ReadBufSize = ReadBufSize; m_WriteBufSize = WriteBufSize;}

  void setTimeout(double Timeout) override;

  /**
   * Sets the byte period for transmitting bytes.
   * If the period is not equal to 0, the transmit will be repeated with the given
   * period until all bytes are transmitted.
   * @param default is 0.
   */
  void setBytePeriod(double Period);

  int openIO() override;

  void closeIO() override;

  int readBlocking(char * Buffer, int Length) override;

  /**
   * Reads the serial port non blocking.
   * The function returns all avaiable bytes but not more than requested.
   * @param Buffer pointer to the buffer.
   * @param Length number of bytes to read
   */
  int readNonBlocking(char * Buffer, int Length);

  /**
   * Writes bytes to the serial port.
   * @param Buffer buffer of the message
   * @param Length number of bytes to send
   */
  int writeIO(const char * Buffer, int Length);

  /**
   * Returns the number of bytes available in the read buffer.
   */
  int getSizeRXQueue();


  void purge() override
  {
    ::tcflush(m_Device, TCIOFLUSH);
  }

  /** Clears the read buffer.
   */
  void purgeRx()
  {
    tcflush(m_Device, TCIFLUSH);
  }

  /**
   * Clears the transmit buffer.
   * The content of the buffer will not be transmitted.
   */
  void purgeTx()
  {
    tcflush(m_Device, TCOFLUSH);
  }

  /**
   * Sends the transmit buffer.
   * All bytes of the transmit buffer will be sent.
   */
  void flushTx()
  {
    tcdrain(m_Device);
  }

protected:
  ::termios m_tio;
  std::string m_DeviceName;
  int m_Device;
  int m_BaudRate;
  double m_Multiplier;
  int m_ByteSize, m_StopBits;
  ParityFlags m_Parity;
  HandshakeFlags m_Handshake;
  int m_ReadBufSize, m_WriteBufSize;
  double m_Timeout;
  ::timeval m_BytePeriod;
  bool m_ShortBytePeriod;
};


#endif  // SICKS300_ROS2__COMMON__SERIALIO_HPP_
