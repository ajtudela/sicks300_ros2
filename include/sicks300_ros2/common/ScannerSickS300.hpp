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

#ifndef SICKS300_ROS2__COMMON__SCANNERSICKS300_HPP_
#define SICKS300_ROS2__COMMON__SCANNERSICKS300_HPP_

// base classes
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sicks300_ros2/common/ISerialIO.hpp"
#include "sicks300_ros2/common/SerialIO.hpp"
#include "sicks300_ros2/common/TelegramS300.hpp"

/**
 * Driver class for the laser scanner SICK S300 Professional.
 * This driver only supports use with 500KBaud in cont. mode
 *
 * if the scanner is in standby, the measurements are 0x4004 according to the Sick Support
 */

class ScannerSickS300
{
public:
  // set of parameters which are specific to the SickS300
  struct ParamType
  {
    int range_field;             // measurement range (1 to 5) --> usually 1 (default)
    double dScale;               // scaling of the scan (multiply with to get scan in meters)
    double dStartAngle;          // scan start angle
    double dStopAngle;           // scan stop angle
  };

  // storage container for received scanner data
  struct ScanPolarType
  {
    double dr;             // distance //r;
    double da;             // angle //a;
    double di;             // intensity; //bool bGlare;
  };

  enum
  {
    SCANNER_S300_READ_BUF_SIZE = 10000,
    READ_BUF_SIZE = 10000,
    WRITE_BUF_SIZE = 10000
  };

  /**
   * @brief Construct a new Scanner Sick S300 object.
   * @param serial_io Transport used to talk to the scanner. Defaults to the real serial
   * port implementation; tests can inject a fake to replay captured telegrams.
   */
  explicit ScannerSickS300(std::unique_ptr<ISerialIO> serial_io = std::make_unique<SerialIO>());

  // Destructor
  ~ScannerSickS300();

  /**
   * Opens serial port.
   * @param pcPort used "COMx" or "/dev/tty1"
   * @param iBaudRate baud rate
   * @param iScanId the scanner id in the data header (7 by default)
   */
  bool open(const char * pcPort, int iBaudRate, int iScanId = 7);

  // whether the scanner is currently in Standby or not
  bool isInStandby() {return m_bInStandby;}

  void purgeScanBuf();

  bool getScan(
    std::vector<double> & vdDistanceM, std::vector<double> & vdAngleRAD,
    std::vector<double> & vdIntensityAU, const bool debug);

  void setRangeField(const int field, const ParamType & param) {m_Params[field] = param;}

private:
  // Constants
  static const double c_dPi;

  // Parameters
  typedef std::map<int, ParamType> PARAM_MAP;
  PARAM_MAP m_Params;
  double m_dBaudMult;

  // Variables
  unsigned char m_ReadBuf[READ_BUF_SIZE + 10];
  unsigned int m_uiSumReadBytes;
  std::vector<int> m_viScanRaw;
  unsigned char m_iScanId;
  int m_actualBufferSize;
  bool m_bInStandby;

  // Components
  std::unique_ptr<ISerialIO> m_SerialIO;
  TelegramParser tp_;

  // Functions
  void convertScanToPolar(
    const PARAM_MAP::const_iterator param, std::vector<int> viScanRaw,
    std::vector<ScanPolarType> & vecScanPolar);
};

#endif  // SICKS300_ROS2__COMMON__SCANNERSICKS300_HPP_
