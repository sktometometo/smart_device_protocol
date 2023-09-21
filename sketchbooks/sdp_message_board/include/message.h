#include <packet_creator.h>
#include <packet_parser.h>

#include <variant>
#include <vector>

const std::string packet_description = std::string("Message Board to write");
const std::string serialization_format = std::string("siS");

class Message
{
public:
  char message[64];
  char source_name[16];
  int32_t deadline;

  Message(char* source_name, char* message, int32_t timeout_duration)
  {
    strncpy(this->source_name, source_name, 16);
    strncpy(this->message, message, 64);
    this->deadline = (int32_t)millis() + timeout_duration;
  }

  void to_packet(uint8_t* data)
  {
    std::vector<std::variant<int32_t, float, std::string, bool>> data_vector;
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(source_name)));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>((int32_t)(this->deadline - millis())));
    data_vector.push_back(std::variant<int32_t, float, std::string, bool>(std::string(message)));
    generate_data_frame(data, packet_description.c_str(), data_vector);
  }
};
