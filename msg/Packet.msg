#
# DATA Packet Specification
#
# Emergency Packet
#
#   +----------------------+----------------------------+--------------------------+----------------------------+
#   | PACKET_TYPE (1 byte) | MAP_FRAME (64 byte String) | POS_X, Y, Z (4 byte x 3) | ROT_X, Y, Z, W (4byte x 4) |
#   +----------------------+----------------------------+--------------------------+----------------------------+
#
# ROS Bridge packets

#   1st
#   +----------------------+----------------+------------------+---------------------+
#   | PACKET_TYPE (1 byte) | Stamp (4 byte) | DATA_ID (4 byte) | TOPIC_TYPE (String) |
#   +----------------------+----------------+------------------+---------------------+

#   2nd
#   +----------------------+----------------+------------------+---------------------+
#   | PACKET_TYPE (1 byte) | Stamp (4 byte) | DATA_ID (4 byte) | TOPIC_NAME (String) |
#   +----------------------+----------------+------------------+---------------------+

#   3rd
#   +----------------------+----------------+------------------+--------------------------+
#   | PACKET_TYPE (1 byte) | Stamp (4 byte) | DATA_ID (4 byte) | SERIALIZED_DATA (String) |
#   +----------------------+----------------+------------------+--------------------------+
#

uint8 PACKET_TYPE_NONE = 0
uint8 PACKET_TYPE_EMERGENCY = 1
uint8 PACKET_TYPE_ROSBRIDGE_TOPIC_TYPE = 11
uint8 PACKET_TYPE_ROSBRIDGE_TOPIC_NAME = 12
uint8 PACKET_TYPE_ROSBRIDGE_TOPIC_DATA = 13

uint8[] mac_address # Must be 6
uint8[] data
