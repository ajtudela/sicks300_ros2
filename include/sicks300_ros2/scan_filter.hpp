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

#ifndef SICKS300_ROS2__SCAN_FILTER_HPP_
#define SICKS300_ROS2__SCAN_FILTER_HPP_

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Common
#include "sicks300_ros2/common/parameter_utils.hpp"

/**
 * @class ScanFilter
 * @brief Truncates a LaserScan to an angular range [lower_angle, upper_angle]
 *
 * @deprecated this is a lightweight, unmaintained reimplementation of what
 * `laser_filters`' `LaserScanAngularBoundsFilter` already does; prefer that plugin for new
 * setups (see the README). Kept for backwards compatibility.
 */
class ScanFilter : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Scan Filter object
   * @param options Node options
   */
  explicit ScanFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("scan_filter", options)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "The 'scan_filter' node is a lightweight, unmaintained reimplementation of angular "
      "bounds filtering. Prefer the 'laser_filters/LaserScanAngularBoundsFilter' plugin "
      "(package 'laser_filters', already an exec_depend of this package) for new setups; "
      "see the README for a ready-to-use configuration.");

    sicks300_ros2::declare_parameter_if_not_declared(
      this, "lower_angle",
      rclcpp::ParameterValue(0.0), rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The angle of the scan to begin filtering at"));
    this->get_parameter("lower_angle", lower_angle_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter lower_angle is set to: %f", lower_angle_);

    sicks300_ros2::declare_parameter_if_not_declared(
      this, "upper_angle",
      rclcpp::ParameterValue(0.0), rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The angle of the scan to end filtering at"));
    this->get_parameter("upper_angle", upper_angle_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter upper_angle is set to: %f", upper_angle_);


    // Create publisher and subscriber
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanFilter::scan_callback, this, std::placeholders::_1));
    laser_scan_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SystemDefaultsQoS());
  }

private:
  /**
   * @brief Truncate an incoming scan to [lower_angle_, upper_angle_] and publish it
   *
   * @param msg Unfiltered laser scan
   */
  void scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    // Create new message
    auto msg_filtered = sensor_msgs::msg::LaserScan();
    msg_filtered.ranges.resize(msg.ranges.size());
    msg_filtered.intensities.resize(msg.intensities.size());

    double start_angle = msg.angle_min;
    double current_angle = msg.angle_min;
    builtin_interfaces::msg::Time start_time = msg.header.stamp;
    unsigned int count = 0;

    // Loop through the scan and truncate the beginning and the end of the scan as necessary
    for (unsigned int i = 0; i < msg.ranges.size(); ++i) {
      // Wait until we get to our desired starting angle
      if (start_angle < lower_angle_) {
        start_angle += msg.angle_increment;
        current_angle += msg.angle_increment;
        // start_time.set__sec(start_time.sec + msg.time_increment);
      } else {
        msg_filtered.ranges[count] = msg.ranges[i];

        // Make sure  that we don't update intensity data if its not available
        if (msg.intensities.size() > i) {
          msg_filtered.intensities[count] = msg.intensities[i];
        }
        count++;

        // Check if we need to break out of the loop,
        // basically if the next increment will put us over the threshold
        if (current_angle + msg.angle_increment > upper_angle_) {
          break;
        }

        current_angle += msg.angle_increment;
      }
    }

    // Make sure to set all the needed fields on the filtered scan
    msg_filtered.header.frame_id = msg.header.frame_id;
    msg_filtered.header.stamp = start_time;
    msg_filtered.angle_min = start_angle;
    msg_filtered.angle_max = current_angle;
    msg_filtered.angle_increment = msg.angle_increment;
    msg_filtered.time_increment = msg.time_increment;
    msg_filtered.scan_time = msg.scan_time;
    msg_filtered.range_min = msg.range_min;
    msg_filtered.range_max = msg.range_max;

    msg_filtered.ranges.resize(count);

    if (msg.intensities.size() >= count) {
      msg_filtered.intensities.resize(count);
    }

    // Publish message
    laser_scan_filtered_pub_->publish(msg_filtered);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_filtered_pub_;

  double lower_angle_, upper_angle_;
};

#endif  // SICKS300_ROS2__SCAN_FILTER_HPP_
