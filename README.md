# esp_now_ros

This package enables to send and receive packet between multiple machines via ESP-NOW.

## How to use

### Make esp_now_ros device

First, instal `platformio` package to your environment.

it can be installed with pip. (Python>=3.6 is strongly recommended.)

```bash
pip3 install platformio
```

Now you can execute `pio` command.

```bash
~ $ pio
Usage: pio [OPTIONS] COMMAND [ARGS]...

Options:
  --version          Show the version and exit.
  -c, --caller TEXT  Caller ID (service)
  --no-ansi          Do not print ANSI control characters
  -h, --help         Show this message and exit.

Commands:
  access    Manage resource access
  account   Manage PlatformIO account
  boards    Board Explorer
  check     Static Code Analysis
  ci        Continuous Integration
  debug     Unified Debugger
  device    Device manager & Serial/Socket monitor
  home      GUI to manage PlatformIO
  org       Manage organizations
  pkg       Unified Package Manager
  project   Project Manager
  remote    Remote Development
  run       Run project targets (build, upload, clean, etc.)
  settings  Manage system settings
  system    Miscellaneous system commands
  team      Manage organization teams
  test      Unit Testing
  upgrade   Upgrade PlatformIO Core to the latest version
```

Then, connect M5Stack-Core2 to your PC. You can check which port is connected to it by

```bash
~ $ pio device list
/dev/ttyACM0
------------
Hardware ID: USB VID:PID=1A86:55D4 SER=54BB013663 LOCATION=7-1:1.0
Description: USB Single Serial
```

So let's build firmware and burn it to M5Stack-Core2

```bash
roscd esp_now_ros/sketchbooks/esp_now_ros/
rosrun rosserial_arduino make_libraries.py ./lib/
pio run -e m5stack-core2 --target upload --upload-port /dev/ttyACM0
```

### Run interface program

After burned, let's start with 

```bash
roslaunch esp_now_ros demo.launch port:=/dev/ttyACM0
```

with this, you can see topics below.

```bash
$ rostopic list
/diagnostics
/esp_now_ros/recv
/esp_now_ros/send
/rosout
/rosout_agg

```

You can send packet by sending ROS a message to `/esp_now_ros/send` topic.

```bash
rostopic pub -1 /esp_now_ros/send esp_now_ros/Packet "mac_address: [255, 255, 255, 255, 255, 255]
data: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]"
```

### Run demo

TODO

## Packet Specification

### System packet

#### Test Packet

| PACKET_TYPE (2 byte unsigned int) | Integer number = -120 (4 byte signed int) | Float number -1.0 (4 byte float) | String (Hello, world!) (64 bytes String) |
|-|-|-|-|

### Named value packet

#### Named String Packet

| PACKET_TYPE (2 byte unsigned int) | Name (64 byte string) | Value (64 bytes String) |
|-|-|-|

#### Named Int Packet

| PACKET_TYPE (2 byte unsigned int) | Name (64 byte string) | Value (4 bytes signed int) |
|-|-|-|

#### Named Float Packet

| PACKET_TYPE (2 byte unsigned int) | Name (64 byte string) | Value (4 bytes float) |
|-|-|-|

### Sensor modules

#### ENV III Packet

| PACKET_TYPE (2 byte unsigned int) | MODULE_NAME (64 byte String) | PRESSURE (4 byte signed int) |
|-|-|-|

#### UNITV2 Person Counter Packet

| PACKET_TYPE (2 byte unsigned int) | NUMBER OF PERSON (4 byte unsigned int) | PLACE_NAME (64 byte String) |
|-|-|-|

#### IMU Packet

| PACKET_TYPE (2 byte unsigned int) | MODULE_NAME (64 byte String) | Accel_X, Y, Z (4 byte float x 3) |
|-|-|-|

### Task packet

#### Emergency Packet

| PACKET_TYPE (2 byte unsigned int) | MAP_FRAME (64 byte String) | POS_X, Y, Z (4 byte float x 3) | ROT_X, Y, Z, W (4byte float x 4) |
|-|-|-|-|

#### Task Dispatcher Packet

| PACKET_TYPE (2 byte unsigned int) | Caller name (16 byte String) | Target name (16 byte String) | Task name (16 byte String) | Task Args (String) |
|-|-|-|-|-|

### Task Received Packet

| PACKET_TYPE (2 byte unsigned int) | Worker name (16 byte String) | Caller name (16 byte String) | Task name (16 byte String) |
|-|-|-|-|

### Task Result Packet

| PACKET_TYPE (2 byte unsigned int) | Worker name (16 byte String) | Caller name (16 byte String) | Task name (16 byte String) | Result (String) |
|-|-|-|-|-|
