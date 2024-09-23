## ARS548 ROS2 Driver

## Overview

The ARS548 ROS2 driver is a software component developed specifically for the ARS548 radar. It enables the communication between the sensor and the ROS2 framework, allowing users to easily integrate the sensor into their ROS2-based robotic systems.

The driver is built on top of the [ROS2 Iron](https://docs.ros.org/en/iron/index.html) distribution, which provides a robust and reliable foundation for developing robotic applications. It utilizes the capabilities of ROS2, such as the pub-sub messaging system, to receive data from the ARS548 radar and publish it for further processing and analysis.

With the ARS548 ROS2 driver, users can access various sensor data such as distance measurements, velocity, and angular position. This information can be utilized by other ROS2 components to perform tasks such as obstacle avoidance, localization, mapping, and more.

## Installation

Unzip the ars548-iron-v0.2.3.zip file to the desired installation directory, assuming it is ~/workcopy/ars548-iron-v0.2.3:

```shell
mkdir ~/workcopy
cd ~/workcopy
unzip ars548-iron-v0.2.3.zip
```

### Directory Structure

| Path                                         | Description                          |
| -------------------------------------------- | ------------------------------------ |
| ars548-iron-v0.2.3                           | setup files                          |
| ars548-iron-v0.2.3/ars548                    | root directory of the ars548 package |
| ars548-iron-v0.2.3/ars548/include            | header files                         |
| ars548-iron-v0.2.3/ars548/lib/ars548         | node executable files                |
| ars548-iron-v0.2.3/ars548/share/ars548/cmake | CMake-related files                  |
| ars548-iron-v0.2.3/ars548/share/ars548/msg   | message definition files             |
| ars548-iron-v0.2.3/ars548/share/ars548/srv   | service definition files             |
| ars548-iron-v0.2.3/ars548_ros2_demo          | C++ console demo                     |
| ars548-iron-v0.2.3/ars548_ros2_qt_demo       | C++ Qt demo                          |
| ars548-iron-v0.2.3/ars548_ros2_py_demo       | Python Qt demo                       |

### Install ars548 package

Go to the installation directory and run the installation script:

```shell
cd ars548-iron-v0.2.3
./setup
```

The installation script will automatically modify several initialization scripts related to environment variables in the current directory.

If the execution is successful, it will prompt:

```
Setup path: /home/dev/workcopy/ars548-iron-v0.2.3
OK
```

If the authorization file ars548.lic is distributed with the product, make sure it is in the same directory as libars548.so, which should be `ars548/lib/`.

### Install USB Camera Module (optional)

The USB Camera Module is not part of the ars548 package. However, during testing, a USB camera was used to display the images of the areas detected by the radar. Therefore, it is included in the `default_launch.py` file to start together, and an image component is added in rviz for visualization. you can decide whether to install this module based on your own needs.

```bash
sudo apt install ros-iron-camera-calibration-parsers
sudo apt install ros-iron-camera-info-manager
sudo apt install ros-iron-launch-testing-ament-cmake
sudo apt install ros-iron-usb-cam
```

After connecting the USB camera, you can start the usb_cam node and use rviz2 to view it by adding the Image type display component and selecting `/image_raw` as the topic.

## Network configuration

To successfully communicate with the radar, you need to configure your computer's network settings correctly:
- Identify a wired network card that communicates with the radar.
- Set a vlan for the network card with an id of 19 (this step is the key to successfully sending data to the radar, such as sending configuration to the radar).
- Set a static IP address for the network card, which needs to be in the same subnet as the radar's IP address (10.13.1.0/24).
- The radar's default IP address is usually 10.13.1.113, so don't conflict with that.

This is an example of configuring a network card using netplan in Ubuntu 22.04:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp0sX:
      dhcp4: no
  vlans:
    vlan19:
      id: 19
      link: enp0sX
      addresses:
        - 10.13.1.200/24
```

## Environment Setup

Before launching any files or running nodes in a new terminal, ensure you source both the ROS2 Iron and ARS548 package environments as outlined below.

### Step 1: Initialize ROS 2 Iron Environment

```shell
source /opt/ros/iron/setup.bash
```

This command prepares your terminal to use the ROS 2 Iron distribution by setting up the appropriate environment variables.

### Step 2: Initialize ARS548 Package Environment

```shell
source ~/workcopy/ars548-iron-v0.2.3/setup.sh
```

This ensures that your current terminal session is configured with the correct environment variables for the ARS548 package.

## Launch

In ROS2, a launch file is used to start up and manage multiple nodes and configurations at once, simplifying the process of running complex systems. Along these lines, the ars548 package also provides some launch files, which are designed for two scenarios: radar data real-time reception and recording playback. 

### Real-time Reception

Specify the IP address of the network card connected to the radar and the IP address of the radar:

```bash
export ARS548_BIND_IP=10.13.1.200
export ARS548_RADAR_IP=10.13.1.113
```

These two IP addresses are used by default, so there is no need to set them if they match your situation.

Start all nodes at once:

```bash
ros2 launch ars548 default_launch.py
```

### Recording Playback

Playback of recordings in .rec format generated by our RT-Viewer software is supported.

Specify the recording file or directory to be played back:

```shell
export ARS548_REPLAY_FILE=/home/dev/workcopy/records/2023-10-28_17.13.00
```

launch replay:

```bash
ros2 launch ars548 replay_launch.py
```

## Nodes

The ARS548 ROS2 driver consists of several nodes, each designated for specific tasks within the ROS2 framework.

### Radar Communication Node

The `radar_node` establishes a two-way communication channel with the ARS548 radar, facilitating not only the retrieval of raw data but also enabling the transmission of configuration commands back to the radar.

Since running the `radar_node` requires administrator privileges, to execute it as a regular user, please perform the following two commands:

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=<install dir>/ars548/share/ars548/fastdds_root_workaround.xml
sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c "<install dir>/ars548/lib/ars548/radar_node <bind_ip> <radar_ip>"
```

Make sure to replace  `<install dir>` with the actual installation directory of the ars548 driver, `<bind_ip>` with the IP address of the network card connected to the radar, and `<radar_ip>` with the IP address of the radar.

Taking the installation directory `/home/dev/workcopy/ars548-iron-v0.2.3` as an example, it would be:

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/dev/workcopy/ars548-iron-v0.2.3/ars548/share/ars548/fastdds_root_workaround.xml
sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER"  bash -c "/home/dev/workcopy/ars548-iron-v0.2.3/ars548/lib/ars548/radar_node 10.13.1.200 10.13.1.113"
```

### Additional Nodes

Beyond the `radar_node`, the ARS548 package further incorporates several nodes for additional functionalities, as listed below.

| Node Name   | Command                                               | Description                      |
| :---------- | :---------------------------------------------------- | :------------------------------- |
| axis        | `ros2 run ars548 axis`                                | Coordinate axis drawing node     |
| point_cloud | `ros2 run ars548 point_cloud`                         | Point cloud data publishing node |
| rec_replay  | `ros2 run ars548 rec_replay <.rec-file-or-directory>` | Recording playback node          |

Once again, ensure that both your ROS2 setup file and ars548 setup file have been sourced to update the environment variables. This step is crucial before attempting to run any nodes in a new terminal.

##  Development Instructions

The `radar_node` provided by the ars548 package communicates with the radar and republishes the data as ROS2 messages. As a developer, you can perform further processing and application of the radar data by subscribing to the topics that interest you. 

The following demo program demonstrates how to subscribe to topics and call the associated services.

### Demo Programs

Before running demo programs, please make sure `radar_node` is already running and communicating with the radar properly. Usually, the `radar_node` is run in a separate console window. Therefore, when running the demo programs, you need to open a new console window and source the setup files of both ROS2 and ars548.

```shell
source /opt/ros/iron/setup.bash
source ars548-iron-v0.2.3/setup.sh
```

#### ars548_ros2_demo

This is a simplest C++ demo that demonstrates how to subscribe to the messages published by the radar_node and output the number of targets detected by the radar.

Build:

```shell
cd ars548-iron-v0.2.3/ars548_ros2_demo
colcon build
```

Execution:

```shell
source install/local_setup.sh
ros2 run ars548_ros2_demo ars548_ros2_demo
```

#### ars548_ros2_qt_demo

This is a C++ demo that demonstrates how to subscribe to messages to receive radar data and modify the configuration of the radar by calling the services provided by the `radar_node`, using a Qt window interface.

Build:

```shell
cd ars548-iron-v0.2.3/ars548_ros2_qt_demo
colcon build
```

Execution:

```shell
source install/local_setup.sh
ros2 run ars548_ros2_qt_demo ars548_ros2_qt_demo
```

After running the program, you will see the figure below.

![ars548_ros2_qt_demo](.images/ars548_ros2_qt_demo.png)

The table on the main window displays the targets data received from the radar. You can toggle between target displays using the Object/Detection radio buttons and sort your desired data by clicking the corresponding column header.

Clicking View in the upper left corner will expand the menu, and clicking on the corresponding menu item will open the corresponding window, providing options to view and set sensor and filter data for the radar.

#### ars548_ros2_py_demo

This is a Python demo that has the same functionality as the ars548_ros2_qt_demo, but it is implemented using the Python language.

Build:

```shell
cd ars548-iron-v0.2.3/ars548_ros2_py_demo
colcon build
```

Execution:

```shell
source install/local_setup.sh
python3 ars548_ros2_py_demo/main.py
```

## Appendix

### Topics and Messages

The topics published by the ars548 package are listed in the following table. You can subscribe to the topics that interest you for further processing or display.

| Node Name   | Published Topic               | Message Type                | Description                                                  |
| ----------- | ----------------------------- | --------------------------- | ------------------------------------------------------------ |
| radar_node  | /ars548/detection_list        | Ars548DetectionList         | Raw radar detection data                                     |
|             | /ars548/object_list           | Ars548ObjectList            | Recognized radar object data                                 |
|             | /ars548/sensor_status         | Ars548SensorStatus          | Radar sensor status data                                     |
|             | /ars548/filter_status         | Ars548FilterStatus          | Radar filter status data                                     |
|             | /ars548/target_list/detection | Ars548TargetList            | The target's horizontal/vertical distance and velocity relative to the radar are provided to facilitate further processing by the user. |
|             | /ars548/target_list/object    | Ars548TargetList            | The target's horizontal/vertical distance and velocity relative to the radar are provided to facilitate further processing by the user. |
| point_cloud | /ars548/viz/detection         | sensor_msgs/msg/PointCloud2 | Point cloud data for rviz2 display                           |
|             | /ars548/viz/object            | sensor_msgs/msg/PointCloud2 | Point cloud data for rviz2 display                           |

Please refer to the message type definitions for the specific fields and descriptions.

### Message Structures

#### Ars548Detection

|Field Type | Field Name  | Description|
|----|----|----|
|float32|f_azimuth_angle|Detection Azimuth Angle|
|float32|f_azimuth_angle_std|Azimuth Angle Std|
|uint8|u_invalid_flags|Detection Invalid Flags|
|float32|f_elevation_angle|Detection Elevation Angle|
|float32|f_elevation_angle_std|Elevation Angle Std|
|float32|f_range|Detection Radial Distance|
|float32|f_range_std|Radial Distance Std|
|float32|f_range_rate|Detection Radial Velocity|
|float32|f_range_rate_std|Radial Velocity Std|
|int8|s_rcs|Detecion RCS|
|uint16|u_measurement_id|Detection ID|
|uint8|u_positive_predictive_value|Existence Probability|
|uint8|u_classification|Detection Classification|
|uint8|u_multi_target_probability|Multi-Target Probability|
|uint16|u_object_id|Associated Object|
|uint8|u_ambiguity_flag|tbd|
|uint16|u_sort_index|tbd|


#### Ars548DetectionList
|Field Type | Field Name  | Description|
|----|----|----|
|uint64|crc|checksum (E2E Profile 7)|
|uint32|length|len (E2E Profile 7)|
|uint32|sqc|sqc (E2E Profile 7)|
|uint32|data_id|data id (E2E Profile 7)|
|uint32|timestamp_nanoseconds|timestamp nanoseconds|
|uint32|timestamp_seconds|timestamp seconds|
|uint8|timestamp_sync_status|timestamp sync status|
|uint32|event_data_qualifier|event data qualifier|
|uint8|extended_qualifier|extended qualifier|
|uint16|origin_invalid_flags|sensor position invalid flags|
|float32|origin_xpos|sensor x position|
|float32|origin_xstd|sensor x position std|
|float32|origin_ypos|sensor y position|
|float32|origin_ystd|sensor y position std|
|float32|origin_zpos|sensor z position|
|float32|origin_zstd|sensor z position std|
|float32|origin_roll|sensor roll angle|
|float32|origin_rollstd|sensor roll angle std|
|float32|origin_pitch|sensor pitch angle|
|float32|origin_pitchstd|sensor pitch angle std|
|float32|origin_yaw|sensor yaw angle|
|float32|origin_yawstd|sensor yaw angle std|
|uint8|list_invalid_flags|invalid flags|
|Ars548Detection[]|detections|array of ars548detection messages|
|float32|list_radvel_domain_min|radial velocity domain min|
|float32|list_radvel_domain_max|radial velocity domain max|
|uint32|list_num_of_detections|number of detections|
|float32|aln_azimuth_correction|azimuth correction|
|float32|aln_elevation_correction|elevation correction|
|uint8|aln_status|status of alignment|


#### Ars548FilterStatus
|Field Type | Field Name  | Description|
|----|----|----|
|uint32|timestamp_nanoseconds|Timestamp Nanoseconds|
|uint32|timestamp_seconds|Timestamp Seconds|
|uint8|timestamp_sync_status|Timestamp Sync Status|
|uint8|filter_configuration_counter|Counter that counts up if new filter configuration has been received and accepted|
|uint8|detection_sort_index|Detection list sorting index|
|uint8|object_sort_index|Object list sorting index|
|Ars548FilterStatusEntry[]|detection_filter|Detection filter entries (index 1 to 7)|
|Ars548FilterStatusEntry[]|object_filter|Object filter entries (index 1 to 24)|


#### Ars548FilterStatusEntry
|Field Type | Field Name  | Description|
|----|----|----|
|uint8|active|Flag to activate/deactivate filter|
|uint8|filter_id|Filter data index|
|float32|minimum_value|Minimum data value to pass the filter|
|float32|maximum_value|Maximum data value to pass the filter|


#### Ars548Object
|Field Type | Field Name  | Description|
|----|----|----|
|uint16|u_status_sensor|tbd|
|uint32|u_id|ID of object|
|uint16|u_age|Age of object|
|uint8|u_status_measurement|Object Status|
|uint8|u_status_movement|Object Movement Status|
|uint16|u_position_invalidflags|tbd|
|uint8|u_position_reference|Reference point position|
|float32|f_position_x|X Position|
|float32|f_position_x_std|X Position Std|
|float32|f_position_y|Y Position|
|float32|f_position_y_std|Y Position Std|
|float32|f_position_z|Z Position|
|float32|f_position_z_std|Z Position Std|
|float32|f_position_covariancexy|Covariance X Y|
|float32|f_position_orientation|Object Orientation|
|float32|f_position_orientation_std|Orientation Std|
|uint8|u_existence_invalidflags|tbd|
|float32|f_existence_probability|Probability of Existence|
|float32|f_rcs|RCS|
|uint8|u_classification_car|Car Classification|
|uint8|u_classification_truck|Truck Classification|
|uint8|u_classification_motorcycle|Motorcycle Classification|
|uint8|u_classification_bicycle|Bicycle Classification|
|uint8|u_classification_pedestrian|Pedestrian Classification|
|uint8|u_classification_animal|Animal Classification|
|uint8|u_classification_hazard|Hazard Classification|
|uint8|u_classification_unknown|Unknown Classification|
|uint8|u_classification_overdrivable|Overdrivable Classification|
|uint8|u_classification_underdrivable|Underdrivable Classification|
|uint8|u_dynamics_absvel_invalidflags|Invalid Flags AbsVel|
|float32|f_dynamics_absvel_x|X Abs Vel|
|float32|f_dynamics_absvel_x_std|X Abs Vel Std|
|float32|f_dynamics_absvel_y|Y Abs Vel|
|float32|f_dynamics_absvel_y_std|Y Abs Vel Std|
|float32|f_dynamics_absvel_covariancexy|Covariance Abs Vel X Y|
|uint8|u_dynamics_relvel_invalidflags|Invalid Flags RelVel|
|float32|f_dynamics_relvel_x|X Rel Vel|
|float32|f_dynamics_relvel_x_std|X Rel Vel Std|
|float32|f_dynamics_relvel_y|Y Rel Vel|
|float32|f_dynamics_relvel_y_std|Y Rel Vel Std|
|float32|f_dynamics_relvel_covariancexy|Covariance Rel Vel X Y|
|uint8|u_dynamics_absaccel_invalidflags|Invalid Flags AbsAccel|
|float32|f_dynamics_absaccel_x|X Abs Accel|
|float32|f_dynamics_absaccel_x_std|X Abs Accel Std|
|float32|f_dynamics_absaccel_y|Y Abs Accel|
|float32|f_dynamics_absaccel_y_std|Y Abs Accel Std|
|float32|f_dynamics_absaccel_covariancexy|Covariance Abs Accel X Y|
|uint8|u_dynamics_relaccel_invalidflags|Invalid Flags RelAccel|
|float32|f_dynamics_relaccel_x|X Rel Accel|
|float32|f_dynamics_relaccel_x_std|X Rel Accel Std|
|float32|f_dynamics_relaccel_y|Y Rel Accel|
|float32|f_dynamics_relaccel_y_std|Y Rel Accel Std|
|float32|f_dynamics_relaccel_covariancexy|Covariance Rel Accel X Y|
|uint8|u_dynamics_orientation_invalidflags|Invalid Flags Orientation|
|float32|u_dynamics_orientation_rate_mean|Object Orientation Rate|
|float32|u_dynamics_orientation_rate_std|Orientation Rate Std|
|uint32|u_shape_length_status|Shape Length Status|
|uint8|u_shape_length_edge_invalidflags|Invalid Flags Shape Length|
|float32|u_shape_length_edge_mean|Mean Shape Length|
|float32|u_shape_length_edge_std|Shape Length Std|
|uint32|u_shape_width_status|Shape Width Status|
|uint8|u_shape_width_edge_invalidflags|Invalid Flags Shape Width|
|float32|u_shape_width_edge_mean|Mean Shape Width|
|float32|u_shape_width_edge_std|Shape Width Std|


#### Ars548ObjectList
|Field Type | Field Name  | Description|
|----|----|----|
|uint64|crc|Checksum (E2E Profile 7)|
|uint32|length|Len (E2E Profile 7)|
|uint32|sqc|SQC (E2E Profile 7)|
|uint32|dataid|Data ID (E2E Profile 7)|
|uint32|timestamp_nanoseconds|Timestamp Nanoseconds|
|uint32|timestamp_seconds|Timestamp Seconds|
|uint8|timestamp_syncstatus|Timestamp Sync Status|
|uint32|eventdataqualifier|Event Data Qualifier|
|uint8|extendedqualifier|Extended Qualifier|
|uint8|objectlist_numofobjects|Number of Objects|
|Ars548Object[]|objects|Array of Objects|


#### Ars548SensorStatus
|Field Type | Field Name  | Description|
|----|----|----|
|uint32|timestamp_nanoseconds|Timestamp Nanoseconds|
|uint32|timestamp_seconds|Timestamp Seconds|
|uint8|timestamp_sync_status|Timestamp Sync Status|
|uint8|swversion_major|Software version (major)|
|uint8|swversion_minor|Software version (minor)|
|uint8|swversion_patch|Software version (patch)|
|float32|longitudinal|Longitudinal sensor position (AUTOSAR)|
|float32|lateral|Lateral sensor position (AUTOSAR)|
|float32|vertical|Vertical sensor position (AUTOSAR)|
|float32|yaw|Sensor yaw angle (AUTOSAR)|
|float32|pitch|Sensor pitch angle (AUTOSAR)|
|uint8|plug_orientation|Orientation of plug|
|float32|length|Vehicle length|
|float32|width|Vehicle width|
|float32|height|Vehicle height|
|float32|wheelbase|Vehicle wheelbase|
|uint16|maximum_distance|Maximum detection distance|
|uint8|frequency_slot|Center frequency|
|uint8|cycle_time|Cycle time|
|uint8|time_slot|Cycle offset|
|uint8|hcc|Country code|
|uint8|powersave_standstill|Power saving in standstill|
|uint32|sensor_ip_address_0|Sensor IP address|
|uint32|sensor_ip_address_1|Reserved|
|uint8|configuration_counter|Counter that counts up if new configuration has been received and accepted|
|uint8|status_longitudinalvelocity|Signals if current VDY is OK or timed out|
|uint8|status_longitudinalacceleration|Signals if current VDY is OK or timed out|
|uint8|status_lateralacceleration|Signals if current VDY is OK or timed out|
|uint8|status_yawrate|Signals if current VDY is OK or timed out|
|uint8|status_steeringangle|Signals if current VDY is OK or timed out|
|uint8|status_drivingdirection|Signals if current VDY is OK or timed out|
|uint8|status_characteristicspeed|Signals if current VDY is OK or timed out|
|uint8|status_radar|Signals if Radar Status is OK|
|uint8|status_voltagestatus|Bitfield to report under- and overvoltage errors|
|uint8|status_temperaturestatus|Bitfield to report under- and overtemperature errors|
|uint8|status_blockagestatus|Current blockage state and blockage self test state|


#### Ars548Target
|Field Type | Field Name  | Description|
|----|----|----|
|uint32|id|unique id|
|float32|distance|vertical distance (m)|
|float32|position|horizontal distance (m)|
|float32|height|height (m)|
|float32|degrees|angle, reserved, currently 0|
|float32|kmph|speed (km/h)|
|float32|rcs|radar cross section (dbm²)|


#### Ars548TargetList
|Field Type | Field Name  | Description|
|----|----|----|
|Ars548Target[]|targets|Array of targets|





