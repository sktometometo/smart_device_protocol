#include <packet_creator.h>
#include <packet_parser.h>

class Message
{
private:
  std::string message;
  std::string source_name;

public:
  Message(const uint8_t* data)
  {
    uint16_t packet_type;
    char* bytes_source_name;
    char* bytes_message;
    parse_packet_as_message_board_data_packet(data, packet_type, bytes_source_name, bytes_message);
    message = std::string(bytes_message);
    source_name = std::string(bytes_source_name);
  }

  void to_packet(uint8_t* data)
  {
    return;
  }
};
