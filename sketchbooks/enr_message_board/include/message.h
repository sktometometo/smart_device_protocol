#include <packet_creator.h>
#include <packet_parser.h>

class Message
{
public:
  char message[64];
  char source_name[64];
  unsigned long deadline;

  Message(const uint8_t* data, unsigned long deadline)
  {
    uint16_t packet_type;
    parse_packet_as_message_board_data_packet(data, packet_type, source_name, message);
    this->deadline = deadline;
  }

  void to_packet(uint8_t* data)
  {
    create_device_message_board_data_packet(data, source_name, message);
  }
};
