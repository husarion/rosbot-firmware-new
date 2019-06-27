# ROSbot firmware CHANGELOG

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) and this project adheres to [Semantic Versioning](http://semver.org/).

## [0.1.0] - 2019-06-19

Initial log. Introducing new ROSbot firmware written using Mbed OS framework.

### Added 
  - VL53L0X sensors support
  - backward compatible ROS interface
  - battery voltage measurement
  - support for 4 wheel and 2 wheel ROSbot

## [0.2.0] - 2019-06-25

### Fixed
  - `rosserial-mbed.lib` v.1.1.1 fixes Buffer bug that would prevent usage larger number of publishers/subscribers

### Added
  - MPU9250 support. ROS interface consists of `/imu` topic with `geometry_msgs/QuaternionStamped` message.
  - Buttons support. ROS interface consists of `/buttons` topic with `std_msgs/UInt8`
    - `1` - BUTTON1 was pressed
    - `2` - BUTTON2 was pressed
  - Multipurpose service server `/config`. See `README.md` to learn more.

### Changed
  - VL53LX0 and MPU9250 are disabled on default. They can be enabled using `/config` service server.

## [0.3.0] - 2019-06-27

### Added
 - new commands:
   * `EJSM` - ENABLE JOINT STATES MESSAGES
   * `RODOM` - RESET ODOMETRY

### Fixed
 - fixed odometry for 4-wheeled robot
  
### Changed
- VL53L0X i2c interface frequency set at 400kHz
- BATTERY LOW indication set at 10.8V