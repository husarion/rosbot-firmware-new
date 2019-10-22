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

## [0.4.0] - 2019-07-19

### Added
  - new command:
    * `RIMU` - RESET IMU
    * `EWCH` - ENABLE SPEED WATCHDOG 
  - speed watchdog (by default enabled), that sets speed to 0 if there is no speed update of `/cmd_vel` for 1s
  - support for EKF

### Changed
  - `imu` topic changed to `mpu9250` with message type `rosbot/Imu`

### Fixed
  - more frequent odometry update
  - odometry coefficient are slightly better adjusted
  - overall firmware's stability is improved
  - max target speed limited to 1.5m/s

## [0.4.1] - 2019-07-30

### Changed
- LPF set to 42 Hz and Gyro FSR set to 500dps for IMU.
- Mbed version changed from `5.12.4` to `5.13.1`

## [0.5.0] - 2019-08-01

### Changed
  - Improved DDR8848 driver implementation for CORE2. Motors' pwm frequency is now set to 21kHz with 1000 steps resolution. 
  - changed symbols C and C++ versions (following C11 and C++14 support in Mbed 5.13.x)

## [0.6.0] - 2019-08-14

### Added
  - support for WS2812B LEDs signalization
  - new command:
    * `SANI` - SET WS2812B LEDS ANIMATION

### Changes
  - some code refactoring and updates
  - `DIAMETER_MODIFICATOR` factor update 

## [0.7.0] - 2019-08-23

### Added 
  - `AnimationManager` class to control ws2812b's signalization

### Changed
  - Better rosserial interface for ws2812b signalization, now you can instantly change the animation and the color. 

## [0.7.1] - 2019-08-27

### Changes
- the support package name changed from `rosbot` to `rosbot_ekf`

## [0.7.2] - 2019-09-26

### Fixed
  - critical changes in CORE2 target definitions

## [0.8.0] - 2019-10-22
    //TODO
    
## TODO
  - forward regulator controlling acceleration and deacceleration
  - virtual bumper using VL53L0X proximity sensors