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

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sicks300_ros2/scan_filter.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::LaserScan makeScan(size_t num_points)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_laser_link";
  scan.angle_min = -1.0;
  scan.angle_increment = 0.5;
  scan.angle_max = scan.angle_min + scan.angle_increment * static_cast<double>(num_points - 1);
  scan.range_min = 0.0;
  scan.range_max = 30.0;
  scan.time_increment = 0.001;
  scan.scan_time = 0.04;
  scan.ranges.resize(num_points);
  scan.intensities.resize(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    scan.ranges[i] = static_cast<double>(i);
    scan.intensities[i] = static_cast<double>(i) * 10.0;
  }
  return scan;
}

// Publishes `scan` on the filter's input topic and spins until a filtered scan is received on
// its output topic or `timeout` elapses.
std::optional<sensor_msgs::msg::LaserScan> filterOnce(
  const sensor_msgs::msg::LaserScan & scan,
  double lower_angle, double upper_angle,
  std::chrono::milliseconds timeout = 2s)
{
  auto filter_node = std::make_shared<ScanFilter>(
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("lower_angle", lower_angle),
        rclcpp::Parameter("upper_angle", upper_angle)}));

  auto driver_node = std::make_shared<rclcpp::Node>("test_scan_filter_driver");
  auto pub = driver_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

  std::optional<sensor_msgs::msg::LaserScan> received;
  auto sub = driver_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SystemDefaultsQoS(),
    [&received](const sensor_msgs::msg::LaserScan & msg) {received = msg;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter_node);
  executor.add_node(driver_node);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!received.has_value() && std::chrono::steady_clock::now() < deadline) {
    pub->publish(scan);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  return received;
}

}  // namespace

TEST(ScanFilterTest, TruncatesScanToConfiguredAngularRange)
{
  // 9 points from -1.0 to 3.0 rad, step 0.5. With lower=-0.3/upper=1.3, tracing
  // scan_callback()'s logic: original indices [2, 3, 4] (angles 0.0, 0.5, 1.0) survive.
  auto filtered = filterOnce(makeScan(9), -0.3, 1.3);

  ASSERT_TRUE(filtered.has_value());
  EXPECT_EQ(filtered->header.frame_id, "base_laser_link");
  EXPECT_DOUBLE_EQ(filtered->angle_min, 0.0);
  EXPECT_DOUBLE_EQ(filtered->angle_max, 1.0);
  EXPECT_DOUBLE_EQ(filtered->angle_increment, 0.5);

  ASSERT_EQ(filtered->ranges.size(), 3u);
  EXPECT_DOUBLE_EQ(filtered->ranges[0], 2.0);
  EXPECT_DOUBLE_EQ(filtered->ranges[1], 3.0);
  EXPECT_DOUBLE_EQ(filtered->ranges[2], 4.0);

  ASSERT_EQ(filtered->intensities.size(), 3u);
  EXPECT_DOUBLE_EQ(filtered->intensities[0], 20.0);
  EXPECT_DOUBLE_EQ(filtered->intensities[1], 30.0);
  EXPECT_DOUBLE_EQ(filtered->intensities[2], 40.0);
}

TEST(ScanFilterTest, KeepsWholeScanWhenBoundsCoverIt)
{
  auto filtered = filterOnce(makeScan(5), -10.0, 10.0);

  ASSERT_TRUE(filtered.has_value());
  ASSERT_EQ(filtered->ranges.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(filtered->ranges[i], static_cast<double>(i));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
