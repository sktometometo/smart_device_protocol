# esp_now_ros

## Packet Specification

### System packet

#### Test Packet

| PACKET_TYPE (2 byte unsigned int) | Integer number = -120 (4 byte signed int) | Float number -1.0 (4 byte float) | String (Hello, world!) (64 bytes String) |
|-|-|-|-|

### ROS Bridge packets

### 1st

```
+----------------------+----------------+------------------+---------------------+
| PACKET_TYPE (2 byte) | Stamp (4 byte) | DATA_ID (4 byte) | TOPIC_TYPE (String) |
+----------------------+----------------+------------------+---------------------+
```

- 2nd

```
+----------------------+----------------+------------------+---------------------+
| PACKET_TYPE (2 byte) | Stamp (4 byte) | DATA_ID (4 byte) | TOPIC_NAME (String) |
+----------------------+----------------+------------------+---------------------+
```

- 3rd

```
+----------------------+----------------+------------------+--------------------------+
| PACKET_TYPE (2 byte) | Stamp (4 byte) | DATA_ID (4 byte) | SERIALIZED_DATA (String) |
+----------------------+----------------+------------------+--------------------------+
```

### Sensor modules

#### ENV III Packet

```
+----------------------------- -----+---------------------------------+------------------------------+
| PACKET_TYPE (2 byte unsigned int) | MODULE_ID (4 byte unsigned int) | PRESSURE (4 byte signed int) |
+-----------------------------------+---------------------------------+------------------------------+
```

#### IMU Packet

TODO

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
