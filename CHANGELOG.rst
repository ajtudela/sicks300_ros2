^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sicks300_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (07-09-2026)
------------------
* Add a unit test suite for the parser, scanner and scan filter.
* Move serial acquisition to a dedicated thread behind an injectable ``ISerialIO`` transport interface.
* Give the serial read a real wall-clock timeout and make the communication timeout report an error.
* Consolidate diagnostics behind ``diagnostic_updater``.
* Guard against degenerate scans and keep ``angle_min``/``angle_max``, ``angle_increment`` and ``scan_time`` consistent.
* Fix the receive buffer compaction in ``getScan``.
* Replace unaligned ``reinterpret_cast`` reads in the parser with ``memcpy``.
* Remove static state shared between scanner instances.
* Remove the inert scan timestamp synchronization, dead members, stubs and commented-out code.
* Extract ``declare_parameter_if_not_declared`` to a shared header.
* Rename Hungarian-notation identifiers to ``snake_case`` and name magic numbers.
* Pass vectors and strings by const reference.
* Deprecate the ``scan_filter`` node in favor of the ``laser_filters`` package and remove the old laser filter.
* Declare ``lower_angle_``/``upper_angle_`` as ``double`` in ``scan_filter``.
* Rename package to ``sicks300_ros2`` and update publisher QoS.
* Update the CI workflow.

1.3.3 (06-02-2025)
------------------
* First jazzy release.

1.3.2 (20-12-2024)
------------------
* Improve formating and linting.
* Remove nav2_util dependency.
* CMakelists.txt use modern idioms.
* Add more restricted compiled options.

1.3.0 (20-06-2023)
------------------
* Added scan_filter node.
* Added logging info in launch file.
* Added scan delay parameter.

1.2.3 (02-05-2023)
------------------
* Update on_activate and on_deactivate methods of the publishers: https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html#rclcpp-lifecycle

1.2.2 (28-11-2022)
------------------
* Add LICENSE file.

1.2.1 (24-11-2022)
------------------
* Replace diagnostic messages with enums.
* Replace declare_parameter_if_not_declared with the one inside nav2_util.

1.2.0 (27-10-2022)
------------------
* Update find_minimums script with QoS.
* Remove boost dependencies.

1.1.1 (26-10-2022)
------------------
* Update launch file with arguments.
* Rename folder from config to params.
* Update publishers QoS.

1.1.0 (11-10-2022)
------------------
* Update parameters declarations.
* Check if parameters have been declared.
* Fix timer when transition from states.
* Rename dummy_launch.py to scan_with_filter.py.
* Remove undeclare the parameters.

1.0.0 (25-07-2022)
-------------------
* From cob_driver version 0.7.12.
* Convert to ROS2.
* Add github workflow.
* Added QoS.
* Convert node into a lifecycle node.
* Added laser_filters.
* Added script to find minimum. Thanks to Manolo Fernandez Carmona.
* Contributors: Alberto Tudela.