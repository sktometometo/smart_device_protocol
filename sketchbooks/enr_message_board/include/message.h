#include <packet_creator.h>
#include <packet_parser.h>

class Message
{
public:
  char message[64];
  char source_name[64];
  unsigned long deadline;

  Message(const uint8_t* data)
  {
    uint16_t packet_type;
    uint64_t timeout_duration;
    parse_packet_as_message_board_data_packet(data, packet_type, source_name, timeout_duration, message);
    this->deadline = millis() + timeout_duration;
  }

  void to_packet(uint8_t* data)
  {
    create_device_message_board_data_packet(data, source_name, deadline - millis(), message);
  }
};
