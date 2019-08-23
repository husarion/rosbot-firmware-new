# ROSbot firmware
The ROSbot mobile platform's microcontroller firmware. Written in C++ using [arm's Mbed OS framework](https://os.mbed.com/). 

<!-- <p align="center">
<img width="800px" src=".img/pro.jpg"/>
</p> -->

```
        _____    ______ _____ 
       |  _  |  |___  /|  _  |
__   __| |/' |     / / | |/' |
\ \ / /|  /| |    / /  |  /| |
 \ V / \ |_/ /_ ./ /_  \ |_/ /
  \_/   \___/(_)\_/(_)  \___/ 
```                                                    


## Prerequisites
You need following tools:
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
* [Linux](https://os.mbed.com/docs/mbed-os/v5.12/tools/linux.html)
* [Mac/Windows](https://os.mbed.com/docs/mbed-os/v5.12/tools/installation-and-setup.html)


After installation set the path to the binary directory of your GCC Arm Embedded Compiler installation:

```
$ mbed config -G GCC_ARM_PATH "C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin" 
```
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
$ mbed update mbed-os-5.13.3
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
components/802.15.4_RF/*
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/STM32F407XG.ld
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/startup_stm32f407xx.S
usb/*
```

## Using firmware

Open Visual Studio Code, press `CTRL + SHIFT + P` and type `Git: Clone` in Command Pallet. Copy and paste `https://github.com/husarion/rosbot-firmware-new.git` URL.

You will be prompted to select your repo location. Choose `core2-mbed-workspace` directory.

### Updating project files
Open `rosbot-firmware-new` in Visual Studio Code IDE. In `.vscode` directory find `settings.json` file and change the value of `C_cpp.default.compilerPath` with path to `arm-none-eabi-g++` location on your system:

Example (Windows):
```json
{
    "C_Cpp.default.compilerPath": "C:/Program Files (x86)/GNU Tools ARM Embedded/6 2017-q2-update/bin/arm-none-eabi-g++"
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
* `FLASH FIRMWARE WHEN BOOTLOADER (RELEASE)`*
* `FLASH FIRMWARE WHEN BOOTLOADER (DEBUG)`  *
* `FLASH FIRMWARE NO BOOTLOADER (RELEASE)`  *
* `FLASH FIRMWARE NO BOOTLOADER (DEBUG)`    *
* `CREATE STATIC MBED-OS LIB (RELEASE)`
* `CREATE STATIC MBED-OS LIB (DEBUG)`
* `BUILD FROM STATIC LIB (RELEASE)`
* `BUILD FROM STATIC LIB (DEBUG)`
* `CLEAN DEBUG`
* `CLEAN RELEASE`

`*` *require ST-LINK programmer*

You can add new tasks and customize existing ones by editing `task.json` file. 

#### Building and uploading firmware (BOOTLOADER)

> This method allows you to easily switch between both Mbed-os and hFramework.

The software bootloader allows the use of Husarion Cloud and hFramework. You can find it in `TARGET_CORE2/bootloader_1_0_0_cortex.hex`. The instruction how to flash it can be found [here](https://husarion.com/manuals/core2/#updating-core2-bootloader).

If you want to use the bootloader just add this lines to `mbed_app.json`:
```json
    "target.mbed_app_start":"0x08010000",
    "target.mbed_rom_start":"0x08000000",
    "target.mbed_rom_size":"0x100000"
```

To build firmware use `BUILD (RELEASE)` or `BUILD (DEBUG)` tasks.

To flash firmware connect ST-LINK to debug connector of CORE2 and use `FLASH FIRMWARE WHEN BOOTLOADER (RELEASE)` or `FLASH FIRMWARE WHEN BOOTLOADER (DEBUG)` task.

#### Building and uploading firmware (NO BOOTLOADER)
> Before proceeding with the following steps make sure you conducted mass erase of the memory and made all flash memory sectors write unprotected.

To build firmware use `BUILD (RELEASE)` or `BUILD (DEBUG)` tasks.

To flash firmware connect ST-LINK to debug connector of CORE2 and use `FLASH FIRMWARE NO BOOTLOADER (RELEASE)` or `FLASH FIRMWARE NO BOOTLOADER (DEBUG)` task.

#### Flashing firmware using `core2-flasher`

```bash
$ arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex 
$ ./core2-flasher firmware.hex
```

You will find `firmware.elf` in `./BUILD/RELEASE` or `./BUILD/DEBUG`.

Here you can learn where to find `core2-flasher` for your system:
https://husarion.com/manuals/core2/#updating-core2-bootloader

#### Flashing firmware using `stm32loader`
https://github.com/byq77/stm32loader

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
- `230400` for UpBoard and Raspberry Pi
- `500000` for Asus Tinker Board

The baudrate can be adjusted for particular SBC, however it should be above `115200` to achieve smooth communication. The default value for this firmware is `500000` (ROSbot 2.0).

You can build firmware for the another baudrate changing only one line in `mbed_app.json`:

```json
"rosserial-mbed.baudrate": 230400,
```

The following `rosserial.launch` file can be used to start `roscore` and `rosserial_python` communication:

```xml
<launch>
  <arg name="serial_port" default="/dev/ttyUSB0"/>
  <arg name="serial_baudrate" default="230400"/>
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

ROSbot publishes to:

* `/velocity` with message type `geometry_msgs/Twist`
* `/battery` with message type `sensor_msgs/BatteryState`
* `/pose` with message type `geometry_msgs/Pose`
* `/range/fl` with message type `sensor_msgs/Range`
* `/range/fr` with message type `sensor_msgs/Range`
* `/range/rl` with message type `sensor_msgs/Range`
* `/range/rr` with message type `sensor_msgs/Range`
* `/joint_states` with message type `sensor_msgs/JointState`
* `/mpu9250` with custom message type `rosbot/Imu`
* `/buttons` with message type `std_msgs/UInt8`

ROSbot provides service server:
* `/config` with custom message type `rosbot/Configuration` 

```bash
$ rossrv show rosbot/Configuration 
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
* `SLED` - SET LED:

    To set LED2 on run:
    ```bash
    $ rosservice call /config "command: 'SLED'
    >data: '2 1'" 
    ```
*  `EIMU` - ENABLE IMU:

    To enable IMU MPU9250 run:
    ```bash
    $ rosservice call /config "command: 'EIMU'
    >data: '1'" 
    ```
* `RIMU` - RESET IMU (for Kalman related odometry)

    To reset IMU MPU9250 run:
    ```bash
    $ rosservice call /config "command: 'RIMU'
    >data: ''"
    ``` 
* `EDSE` - ENABLE DISTANCE SENSORS:
    
    To enable VL53LX0 distance sensors run:
    ```bash
    $ rosservice call /config "command: 'EDSE'
    >data: '1'" 
    ```
* `EJSM` -  ENABLE JOINT STATES MESSAGES

    To enable JointStates messages run:
    ```bash
    $ rosservice call /config "command: `EJSM`
    >data: '1'"
    ```
* `RODOM` - RESET ODOMETRY

    To reset odometry run:
    ```bash
    $ rosservice call /config "command: `RODOM`
    >data: ''"
    ```
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

### ROS requirements - `rosbot` package

In order to use the service you have to download the package `rosbot` that can be found [HERE](https://github.com/byq77/rosbot). For installation details check the [README](https://github.com/byq77/rosbot/blob/master/README.md). 

The package contains the ready to use **Extended Kalman Filter** that combines both the imu and encoders measurements to better approximate the ROSbot position and orientation. The package also has the custom messages required by the new firmware.

To launch the rosserial communication and Kalman filter run:
```bash
$ roslaunch rosbot all.launch
```

For PRO version add parameter:

```bash
$ roslaunch rosbot all.launch rosbot_pro:=true
```

## Versioning

The project uses [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/husarion/rosbot-firmware-new/tags). 

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## Starting with Mbed OS

Documentation:
* [MBED OS Documentation](https://os.mbed.com/docs/v5.12/)
* [MBED OS API Doxygen](https://os.mbed.com/docs/v5.12/mbed-os-api-doxy/modules.html)
