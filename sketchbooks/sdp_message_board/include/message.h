#include <smart_device_protocol/Packet.h>

#include <variant>
#include <vector>

#include "sdp/packet_creator.h"
#include "sdp/packet_parser.h"

class Message {
 public:
  inline static std::string packet_description_write = "Message Board to write";
  inline static std::string serialization_format_write = "siS";
  inline static std::string packet_description_message = "Message Board message";

  char message[64];
  char source_name[64];
  unsigned long deadline;

  Message(char *source_name, char *message, int32_t timeout_duration) {
    strncpy(this->source_name, source_name, 16);
    strncpy(this->message, message, 64);
    this->deadline = (int32_t)millis() + timeout_duration;
  }

  void to_v2_packet(uint8_t *data) {
    std::vector<std::variant<int32_t, float, std::string, bool>> data_vector;
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(source_name)));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>((int32_t)(this->deadline - millis())));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(message)));
    generate_data_frame(data, packet_description_message.c_str(), data_vector);
  }

  static SDPInterfaceDescription get_interface_description() {
    return std::make_tuple(packet_description_message, serialization_format_write);
  }
};
