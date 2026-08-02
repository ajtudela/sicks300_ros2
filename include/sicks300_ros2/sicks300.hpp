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

#ifndef SICKS300_ROS2__SICKS300_HPP_
#define SICKS300_ROS2__SICKS300_HPP_

// C++
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

// Common
#include "sicks300_ros2/common/ScannerSickS300.hpp"
#include "sicks300_ros2/common/parameter_utils.hpp"

namespace sicks300_ros2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class sicks300_ros2::SickS300
 * @brief ROS2 driver for the SICK S300 Professional laser scanner
 */
class SickS300 : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new Sick S300 object
   * @param options Node options
   */
  explicit SickS300(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Sick S300 object
   */
  ~SickS300();

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Open the scanner
   *
   * @return true if the scanner is opened
   */
  bool open();

  /**
   * @brief Receive the scan
   *
   * Runs on the node's executor thread (via the wall timer). Picks up the latest scan
   * produced by the acquisition thread (if any) and publishes it, then checks whether the
   * scanner has been silent for longer than `communication_timeout_`; if so, reports it
   * through an ERROR diagnostic instead of silently doing nothing.
   */
  void receiveScan();

  /**
   * @brief Body of the dedicated acquisition thread
   *
   * Continuously blocks on `scanner_.getScan()` (a serial read with a bounded but
   * potentially non-trivial timeout) and hands off the latest successfully parsed scan to
   * `receiveScan()` through `pending_scan_`. Running this on its own thread instead of the
   * timer callback keeps the executor responsive (lifecycle services, other timers) even if
   * the scanner stops sending data.
   */
  void acquisitionLoop();

  /**
   * @brief Publish the standby status
   *
   * @param in_standby Standby status
   */
  void publishStandby(bool in_standby);

  /**
   * @brief Publish the laser scan
   *
   * @param ranges_m Vector of distances in meters
   * @param angles_rad Vector of angles in radians
   * @param intensities_au Vector of intensities in arbitrary units
   */
  void publishLaserScan(
    const std::vector<double> & ranges_m, const std::vector<double> & angles_rad,
    const std::vector<double> & intensities_au);

  /**
   * @brief Fill out the scanner's DiagnosticStatus for diagnostic_updater
   *
   * Called by `diagnostic_updater_` on its own schedule (~1Hz by default), acting as the
   * single point of truth for `/diagnostics`: it just reports the latest `scanner_status_`
   * set by `receiveScan()`, instead of every caller building and publishing its own
   * DiagnosticArray at the (much higher) scan rate.
   *
   * @param stat Diagnostic status to fill out
   */
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr in_standby_pub_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_, scan_topic_, port_;
  int baud_, scan_id_;
  bool inverted_, debug_;
  double scan_duration_, scan_cycle_time_, scan_delay_, communication_timeout_;
  std_msgs::msg::Bool in_standby_;
  ScannerSickS300 scanner_;

  // Scan handed off from the acquisition thread to receiveScan(), guarded by scan_mutex_.
  struct PendingScan
  {
    bool valid = false;
    bool in_standby = false;
    std::vector<double> ranges, angles, intensities;
  };

  std::mutex scan_mutex_;
  PendingScan pending_scan_;
  rclcpp::Time point_time_communication_ok_;

  std::thread acquisition_thread_;
  std::atomic_bool acquisition_running_{false};

  // Latest status reported by receiveScan(), read back by produceDiagnostics(). Both run on
  // the executor thread (wall timer / diagnostic_updater's own timer), so no locking needed.
  enum class ScannerStatus {kOk, kStandby, kCommunicationError};
  ScannerStatus scanner_status_ = ScannerStatus::kOk;
  std::string scanner_status_message_;
};

}  // namespace sicks300_ros2

#endif  // SICKS300_ROS2__SICKS300_HPP_
