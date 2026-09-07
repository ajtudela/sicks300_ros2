# sicks300_ros2

![ROS2](https://img.shields.io/badge/ros2-rolling-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/ajtudela/sicks300_ros2)
[![Build](https://github.com/ajtudela/sicks300_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/ajtudela/sicks300_ros2/actions/workflows/build.yml)

## Overview

This package implements a driver for the Sick S300 Safety laser scanners with an interface for ROS 2 using a lifecycle node.
It provides an implementation for both, the old (1.40) and the new (2.10) protocol.
Thus, the old Sick S300 Professional CMS as well as the new Sick S300 Expert are supported.

However, it does not cover the full functionality of the protocol:
- It only handles distance measurements properly
- It only handles no or only one configured measurement range field properly
- It does not handle I/O-data or reflector data
(though it reads the reflector marker field in the distance measurements)

**Keywords:** ROS2, laser, driver, sick s300, lifecycle

The sicks300_ros2 package has been tested under [ROS2] Rolling on [Ubuntu] 24.04. but ported to ROS2. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## S300 Configuration
Here are a few notes about how to best configure the S300:
- Configure the RS422 output to 500kBaud (otherwise, the scanner only provides a lower frequency)
- Configure the scanner to Continuous Data Output
- Send data via one telegram
- Only configure distances, no I/O or reflector data (otherwise, the scanner only provides a lower frequency).
- Configuration of the measurement ranges
    - For protocol 1.40: only configure one measurement range field with the full range (-45° to 225°) with all values.
    - For protocol 2.10: do not configure a measurement range field
      (otherwise, the scanner only provides a lower frequency).
- If you want to only use certain measurement ranges, do this on the ROS side using e.g. the `laser_filters` package.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using
```bash
cd colcon_workspace/src
git clone https://github.com/ajtudela/sicks300_ros2.git
cd ../
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build
```

## Usage

Add the user to the dialout group to access the USB port:

```bash
sudo usermod -a -G dialout $USER
```

Run the sicks300_ros2 node with:
```bash
ros2 run sicks300_ros2 sicks300_ros2
```

Optionally, you can launch this node with an angulor bound filter:
```bash
ros2 launch sicks300_ros2 scan_with_filter.launch.py
```
By default this launches the package's own `scan_filter` node (see below). To use
`laser_filters` instead, uncomment the `laser_filter_node` block in
[scan_with_filter.launch.py](launch/scan_with_filter.launch.py) (and use it instead of
`filter_node` in the event handler) together with the matching commented-out `scan_filter`
configuration in [params/default.yaml](params/default.yaml).

## Nodes

### sicks300_ros2

Driver for the Sick S300 Safety laser scanners.

#### Published Topics

* **`scan`** ([sensor_msgs/LaserScan])

	The laserscan data.

* **`scan/standby`** ([std_msgs/Bool])

	True if the scanner is in standby mode, false otherwise.

* **`/diagnostics`** ([diagnostic_msgs/DiagnosticArray])

	Diagnostic about the laser scan.

#### Parameters

* **`port`** (string, default: "/dev/ttyUSB0")

	USB port of the scanner.

* **`baud`** (int, default: 500000)

	Baudrate to communicate with the laser scanner.

* **`scan_id`** (int, default: 7)

	Identifier of the scanner.

* **`inverted`** (bool, default: false)

	Option to invert the direction of the measurements.

* **`scan_topic`** (string, default: "scan")

	The topic where the laser scan will be published.

* **`frame_id`** (string, default: "base_laser_link")

	The frame of the scanner.

* **`scan_duration`** (double, default: 0.025)

	Time between laser scans in seconds.

* **`scan_cycle_time`** (double, default: 0.040)

	Cycle time of the scan in seconds. Documentation says S300 scans every 40ms.

* **`scan_delay`** (double, default: 0.075)

	Delay between the start of the scan and the first measurement in seconds.

* **`debug`** (bool, default: false)

	Option to toggle scanner debugging information.

* **`communication_timeout`** (double, default: 0.2)

	Time without a valid scan before reporting a communication error diagnostic, in seconds.

* **`fields`**

	Range configuration of the field. Set 1 by default.

### scan_filter

Lightweight node that truncates a `LaserScan` to an angular range `[lower_angle, upper_angle]`.

**Deprecated:** this node is an unmaintained, more fragile reimplementation of what
[`laser_filters`]'s `LaserScanAngularBoundsFilter` already does; this package already
declares `laser_filters` as a dependency. Prefer configuring a `laser_filters` filter chain
(see the commented-out example in [params/default.yaml](params/default.yaml)) for new setups.
`scan_filter` is kept for backwards compatibility.

#### Subscribed Topics

* **`scan`** ([sensor_msgs/LaserScan])

	The unfiltered laser scan.

#### Published Topics

* **`scan_filtered`** ([sensor_msgs/LaserScan])

	The laser scan truncated to `[lower_angle, upper_angle]`.

#### Parameters

* **`lower_angle`** (double, default: 0.0)

	The angle of the scan to begin filtering at.

* **`upper_angle`** (double, default: 0.0)

	The angle of the scan to end filtering at.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/jazzy/
[sensor_msgs/LaserScan]: https://docs.ros2.org/jazzy/api/sensor_msgs/msg/LaserScan.html
[std_msgs/Bool]: https://docs.ros2.org/jazzy/api/std_msgs/msg/Bool.html
[diagnostic_msgs/DiagnosticArray]: https://docs.ros2.org/jazzy/api/diagnostic_msgs/msg/DiagnosticArray.html
[`laser_filters`]: https://github.com/ros-perception/laser_filters