# ROSbot firmware
The ROSbot mobile platform's microcontroller firmware. Written in C++ using [arm's Mbed OS framework](https://os.mbed.com/) ( `mbed-os-5.14.1`).

```
______  _____  _____  _             _           __           
| ___ \|  _  |/  ___|| |           | |         / _|          
| |_/ /| | | |\ `--. | |__    ___  | |_       | |_ __      __
|    / | | | | `--. \| '_ \  / _ \ | __|      |  _|\ \ /\ / /
| |\ \ \ \_/ //\__/ /| |_) || (_) || |_       | |   \ V  V / 
\_| \_| \___/ \____/ |_.__/  \___/  \__|      |_|    \_/\_/  
```                                                    
**Firmware version:** `0.13.1`

## Prerequisites
You need to install following tools:
* [Visual Studio Code IDE](https://code.visualstudio.com/)
* [GNU Arm Embedded version 6 toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* [STM32 ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html) (Windows)
* [stlink flasher](https://github.com/texane/stlink/blob/master/README.md) (Mac/Linux)
* [Mbed CLI](https://os.mbed.com/docs/mbed-os/v5.12/tools/installation-and-setup.html) 

### Required Visual Studio Code extensions:
* [Microsoft C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (`ms-vscode.cpptools`)
* [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) (`marus25.cortex-debug`)

## Mbed CLI installation

To install the tool follow the official documentation:
* [Linux](https://os.mbed.com/docs/mbed-os/v5.14/tools/manual-installation.html)
* [Mac/Windows](https://os.mbed.com/docs/mbed-os/v5.14/tools/installation-and-setup.html)


After installation set the path to the binary directory of your **GCC Arm Embedded Compiler** installation:

Example for Windows:
```
$ mbed config -G GCC_ARM_PATH "C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin" 
```

Example for Linux:
```
$ mbed config -G GCC_ARM_PATH ~\opt\gcc-arm-none-eabi-6-2017-q2-update\bin
```

> Make sure you have the **GNU Arm Embedded version 6 toolchain** installed on your system. Check the *Prerequisites* section.

To check current configuration run:

```
$ mbed config --list
```

## Preparing a workspace
Create a new folder `core2-mbed-workspace`. It will serve as workspace for your mbed projects.  Run:

```bash
$ mkdir core2-mbed-workspace && cd core2-mbed-workspace
```
Next step is to import `mbed-os` library. It will be used by all your projects. In your workspace folder run:

```bash
$ mbed import mbed-os
```

Set Mbed OS version to supported by this template:

```bash
$ cd mbed-os
$ mbed update mbed-os-5.14.1
```

During Mbed OS installation you can be asked to install additional python libraries. Switch to `mbed-os` dir and run:

```bash
$ pip install -r requirements.txt --user
```

Set path to `mbed-os` directory in Mbed CLI. These way all your projects can use one instance of the library (default configuration is to have separate instance of library for each project). Run:

```bash
$ mbed config -G MBED_OS_DIR <path to mbed-os>
```

Example:

```bash
$ mbed config -G MBED_OS_DIR "E:\mbed_projects\core2-mbed-workspace\mbed-os"
```

### Adding `.mbedignore` file

In `mbed-os` directory create `.mbedignore` (filename starts with dot) file with following content:

```
features/cellular/*
features/cryptocell/*
features/deprecated_warnings/*
features/lorawan/*
features/lwipstack/*
features/nanostack/*
features/netsocket/*
features/nfc/*
features/unsupported/*
components/wifi/*
components/cellular/*
components/802.15.4_RF/*
components/TARGET_PSA/*
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/STM32F407XG.ld
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/startup_stm32f407xx.S
```

## Using firmware

Open Visual Studio Code, press `CTRL + SHIFT + P` and type `Git: Clone` in Command Pallet. Copy and paste `https://github.com/husarion/rosbot-firmware-new.git` URL.

You will be prompted to select your r\epo location. Choose `core2-mbed-workspace` directory.

### Updating project files
Open `rosbot-firmware-new` in Visual Studio Code IDE. In `.vscode` directory find `settings.json` file and change the value of `C_cpp.default.compilerPath` with path to `arm-none-eabi-g++` location on your system:

Example (Windows):
```json
{
    "C_Cpp.default.compilerPath": "C:\\Program Files (x86)\\GNU Tools ARM Embedded\\6 2017-q2-update\\bin\\arm-none-eabi-g++"
}
```

After that update all repository dependencies. In `rosbot-firmware-new/lib` directory run:

```bash
$ mbed update master
```

### Compilation tasks

To build and flash your firmware press `CTRL + SHIFT + P` and type `Tasks: Run Task` in Command Pallete. Here is the list of available tasks: 
* `BUILD (RELEASE)`
* `BUILD (DEBUG)`
* `FLASH FIRMWARE (RELEASE)`*
* `FLASH FIRMWARE (DEBUG)`  *
* `CREATE STATIC MBED-OS LIB (RELEASE)`
* `CREATE STATIC MBED-OS LIB (DEBUG)`
* `BUILD FROM STATIC LIB (RELEASE)`
* `BUILD FROM STATIC LIB (DEBUG)`
* `CLEAN DEBUG`
* `CLEAN RELEASE`

`*` *require ST-LINK programmer*

You can add new tasks and customize existing ones by editing `task.json` file. 

#### Building firmware

To build firmware use `BUILD (RELEASE)` or `BUILD (DEBUG)` tasks.

The debug version is intended to be used with ST-LINK probe. You can launch debugger in VSC by pressing `CTRL + SHIFT + D`. We use `Cortex-Debug` extension and `ST-Util GDB`.

#### Uploading firmware using ST-Link
> Before proceeding with the following steps make sure you conducted mass erase of the memory and made all flash memory sectors write unprotected.

To flash firmware connect ST-LINK to debug connector of CORE2 and use `FLASH FIRMWARE (RELEASE)` or `FLASH FIRMWARE (DEBUG)` task.

#### Uploading firmware using `core2-flasher`

```bash
$ arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex 
$ ./core2-flasher firmware.hex
```

You will find `firmware.elf` in `./BUILD/RELEASE` or `./BUILD/DEBUG`.

Here you can learn where to find `core2-flasher` for your system:
https://husarion.com/manuals/core2/#updating-core2-bootloader

#### Uploading firmware using `stm32loader`
https://github.com/husarion/stm32loader

This tool allows you to upload firmware using RPi connector.

If you have the bootloader the first two sectors are write protected. Before uploading new firmware you must unlock them (this will erase the bootloader):

```bash
$ sudo stm32loader -c <your_sbc> -u -W
```

To upload new firmware run:
```bash
$ sudo stm32loader -c <your_sbc> -e -v -w firmware.bin
```

where `<your_sbc>` :
* `tinker` for Asus Tinker Board
* `upboard` for Upboard
* `rpi` for Raspberry Pi

You will find `firmware.bin` in `./BUILD/RELEASE` or `./BUILD/DEBUG`.

### Debug

To debug:
* make sure you have stlink from texane installed on your system: https://github.com/texane/stlink/blob/master/README.md
* install extension: https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug
* compile and flash DEBUG firmware
* `CTRL + SHIFT + D` and click on `start debug` button

## rosserial interface

To use this firmware you have to disable communication with Husarion Cloud. On your SBC run:

```bash
$ sudo systemctl disable husarnet-configurator
$ sudo reboot
```

To start rosserial communication run:

```bash
$ rosrun rosserial_node serial_node.py.py _port:=<SBC_port_name> _baud:=<port_baudrate>
```

`<SBC_port_name>`:
- `/dev/ttyS1` for Asus Tinker Board,
- `/dev/serial0` for Raspberry Pi
- `/dev/ttyS4` for UpBoard

`<port_baudrate>`:
- `460800` for UpBoard
- `500000` for Asus Tinker Board
- `230400` for Raspberry Pi

The baudrate should be adjusted for SBC you use. The default value for this firmware is `500000` (ROSbot 2.0).

You can build firmware for the another baudrate changing only one line in `mbed_app.json`:

```json
"rosserial-mbed.baudrate": 460800,
```

The following `rosserial.launch` file can be used to start `roscore` and `rosserial_python` communication:

```xml
<launch>
  <arg name="serial_port" default="/dev/ttyUSB0"/>
  <arg name="serial_baudrate" default="500000"/>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen">
    <param name="port" value="$(arg serial_port)"/>
    <param name="baud" value="$(arg serial_baudrate)"/>
  </node>
</launch>
```

Usage for Asus Tinker Board:

```bash
$ roslaunch rosserial.launch serial_port:=/dev/ttyS1 serial_baudrate:=500000
```

## ROS communication

ROSbot subscribes to:

* `/cmd_vel` with message type `geometry_msgs/Twist`
* `/cmd_ser` with message type `std_msgs/UInt32` - control configured servo output. See `CSER` service command to learn how to configure servo outputs. Message format:

    ```plain
    MSB [ duty_cycle_us | output_id] LSB
            28bits           4bits
    ```
    Servos are numbered from 1 to 6, where 1 means hServo 1 output etc. To set SERVO 1 duty cycle to 1000us (0x3E8) run:
    ```plain
    $ rostopic pub /cmd_ser std_msgs/UInt32 "data: 0x3E81" --once 
    ``` 

ROSbot publishes to:

* `/velocity` with message type `geometry_msgs/Twist`
* `/battery` with message type `sensor_msgs/BatteryState`
* `/pose` with message type `geometry_msgs/Pose`
* `/range/fl` with message type `sensor_msgs/Range`
* `/range/fr` with message type `sensor_msgs/Range`
* `/range/rl` with message type `sensor_msgs/Range`
* `/range/rr` with message type `sensor_msgs/Range`
* `/joint_states` with message type `sensor_msgs/JointState`
* `/mpu9250` with custom message type `rosbot_ekf/Imu`
* `/buttons` with message type `std_msgs/UInt8`

ROSbot provides service server:
* `/config` with custom message type `rosbot_ekf/Configuration` 

```bash
$ rossrv show rosbot_ekf/Configuration 
string command
string data
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 COMMAND_NOT_FOUND=2
string data
uint8 result
```

At the moment following commands are available:
* `CSER` - CONFIGURE SERVO

    Change a configuration of servo outputs. Can be repeated as many times as required to change several configuration parameter at once. The parameter name should be separated from the value with a full column `:` character. Available parameters:
    * `S` - select servo output, required with `P` and `W` options [`1`:`6`]
    * `V` - select voltage mode:
        * `0` - about 5V
        * `1` - about 6V
        * `2` - about 7.4V
        * `3` - about 8.6V
    * `E` - enable servo output [`1`,`0`]
    * `P` - set period in us
    * `W` - set duty cycle in us

    To set servo voltages to 5V and enable `SERVO 1` output with period 20ms and width 1ms run:
    ```bash
    $ rosservice call /config "command: 'CSER'
    >data: 'V:0 S:1 E:1 P:20000 W:1000 '"
    ```

    ![](.img/servo_voltage_analog_discovery.png) 

* `CPID` - CONFIGURE PID

    Change the motor's pid configuration. This command is similar to CSER command. You can change multiple parameters at the same time.
    Available parameters:
    * `kp` - proportional gain (default: 0.8)
    * `ki` - integral gain (default: 0.2)
    * `kd` - derivative gain (default: 0.015)
    * `out_max` - upper limit of the pid output, represents pwm duty cycle (default: 0.80, max: 0.80)
    * `out_min` - lower limit of the pid output, represents pwm duty cycle when motor spins in opposite direction (default: -0.80, min: -0.80)
    * `a_max` - acceleration limit (default: 1.5e-4 m/s2)
    * `speed_max` - max motor speed (default: 1.0 m/s, max: 1.25 m/s)

    To limit pid outputs to 75% run: 
    ```bash
    $ rosservice call /config "command: 'CPID'
    >data: 'out_max:0.75 out_min:-0.75'"
    ```
    
* `GPID` - GET PID CONFIGURATION

    To get current PID configuration run:
    ```bash
    $ rosservice call /config "command: 'GPID'
    data: ''" 
    ```
    Response:
    ```bash
    data: "kp:0.800 ki:0.200 kd:0.015 out_max:1.000 out_min:-1.000 a_max:1. 500e-04 speed_max:\
      \ 1.500"
    result: 0

    ```

* `SLED` - SET LED:

    To set LED2 on run:
    ```bash
    $ rosservice call /config "command: 'SLED'
    >data: '2 1'" 
    ```
*  `EIMU` - ENABLE/DISABLE IMU:

    To enable IMU MPU9250 run:
    ```bash
    $ rosservice call /config "command: 'EIMU'
    >data: '1'" 
    ```
    * `data: '1'` - enable
    * `data: '0'` - disable

* `RIMU` - RESET IMU (for Kalman related odometry)

    To reset IMU MPU9250 run:
    ```bash
    $ rosservice call /config "command: 'RIMU'
    >data: ''"
    ``` 

<!-- * `EDSE` - ENABLE/DISABLE DISTANCE SENSORS:
    
    To enable VL53LX0 distance sensors run:
    ```bash
    $ rosservice call /config "command: 'EDSE'
    >data: '1'" 
    ```
    * `data: '1'` - enable
    * `data: '0'` - disable -->
    
* `EJSM` -  ENABLE/DISABLE JOINT STATES MESSAGES

    To enable JointStates messages run:
    ```bash
    $ rosservice call /config "command: `EJSM`
    >data: '1'"
    ```
    * `data: '1'` - enable
    * `data: '0'` - disable

* `RODOM` - RESET ODOMETRY

    To reset odometry run:
    ```bash
    $ rosservice call /config "command: `RODOM`
    >data: ''"
    
* `CALI` - ODOMETRY CALIBRATION (update coefficients)

    To update `diameter_modificator` and `tyre_deflation` run:
    ```bash
    $ rosservice call /config "command: `CALI`
    >data: 'X Y'"
    ```
    * `X` - `diameter_modificator` value
    * `Y` - `tyre_deflation` value

* `EMOT` - ENABLE/DISABLE MOTORS

    To disable motors run:
    ```bash
    $ rosservice call /config "command 'EMOT'
    >data '0'
    ```
    * `0` - disconnect motors
    * `1` - connect motors

* `SANI` - SET WS2812B LEDS ANIMATION

    To enable the ws2812b interface open the `mbed_app.json` file and change the line:

    ```json
    "enable-ws2812b-signalization": 0
    ```
    to
    ```json
    "enable-ws2812b-signalization": 1
    ```

    To set fading blue animation run:
    ```bash
    $ rosservice call /config "command: `SANI`
    >data: 'F #0000aa'"
    ```
    
    Available commands:
    * `O` - OFF
    * `S <hex color code>` - SOLID COLOR 
    * `F <hex color code>` - FADE IN FADE OUT ANIMATION
    * `B <hex color code>` - BLINK FRONT/REAR ANIMATION
    * `R` - RAINBOW ANIMATION

### ROS requirements - `rosbot_ekf` package

In order to use the service you have to download the package `rosbot_ekf` that can be found [HERE](https://github.com/husarion/rosbot_ekf). For installation details check the [README](https://github.com/husarion/rosbot_ekf/blob/master/README.md). 

The package incorporate a ready to use **Extended Kalman Filter** that combines both the imu and encoders measurements to better approximate the ROSbot position and orientation. The package also contains custom messages that are required by the new firmware.

To launch the rosserial communication and Kalman filter run:
```bash
$ roslaunch rosbot_ekf all.launch
```

For PRO version add parameter:

```bash
$ roslaunch rosbot_ekf all.launch rosbot_pro:=true
```

## Versioning

The project uses [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/husarion/rosbot-firmware-new/tags). 

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## Starting with Mbed OS

Documentation:
* [MBED OS Documentation](https://os.mbed.com/docs/v5.14/)
* [MBED OS API Doxygen](https://os.mbed.com/docs/v5.14/mbed-os-api-doxy/modules.html)
