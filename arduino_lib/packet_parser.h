#ifndef ESP_NOW_ROS_PACKET_PARSER_H__
#define ESP_NOW_ROS_PACKET_PARSER_H__

#include <esp_now_ros/Packet.h>
#include <string.h>
#include <tuple>
#include <variant>

uint16_t get_packet_type(const uint8_t* packet)
{
  return *(uint16_t*)packet;
}

std::tuple<std::string, std::vector<std::tuple<std::string, std::string>>> parse_packet_as_meta_packet(const uint8_t* packet)
{
  std::string device_name;
  std::vector<std::tuple<std::string, std::string>> packet_description_and_serialization_format;
  device_name = std::string((char*)(packet + 2), 20);
  for (int i = 0; i < 3; ++i) {
    std::string packet_description = std::string((char*)(packet + 2 + 20 + 74 * i), 64);
    std::string serialization_format = std::string((char*)(packet + 2 + 20 + 74 * i + 10), 10);
    packet_description_and_serialization_format.push_back(std::make_tuple(packet_description, serialization_format));
  }
  return std::make_tuple(device_name, packet_description_and_serialization_format);
}

std::tuple<std::string, std::string, std::vector<std::variant<int32_t, float, std::string, bool>>> parse_packet_as_data_packet(const uint8_t* packet)
{
  std::string packet_description = std::string((char*)(packet + 2), 64);
  std::string serialization_format = std::string((char*)(packet + 2 + 64), 10);
  std::vector<std::variant<int32_t, float, std::string, bool>> data;
  auto packet_data_p = packet + 2 + 64 + 10;
  for (int i = 0; i < serialization_format.size(); ++i) {
    if (serialization_format[i] == 'i') {
      data.push_back(std::variant<int32_t, float, std::string, bool>(*(int32_t*)packet_data_p));
      packet_data_p += sizeof(int32_t);
    } else if (serialization_format[i] == 'f') {
      data.push_back(std::variant<int32_t, float, std::string, bool>(*(float*)packet_data_p));
      packet_data_p += sizeof(float);
    } else if (serialization_format[i] == 's') {
      std::variant<int32_t, float, std::string, bool> str = std::string((char*)packet_data_p, 64);
      data.push_back(str);
      packet_data_p += 64;
    } else if (serialization_format[i] == '?') {
      data.push_back(std::variant<int32_t, float, std::string, bool>(*(bool*)packet_data_p));
      packet_data_p += sizeof(bool);
    }
  }
}

#endif
