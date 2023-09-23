#ifndef ESP_NOW_ROS_PACKET_PARSER_H__
#define ESP_NOW_ROS_PACKET_PARSER_H__

#include <string.h>
#include <tuple>
#include <variant>
#include <vector>

#include <esp_now_ros/Packet.h>

#include "packet_util.h"

uint16_t get_packet_type(const uint8_t *packet)
{
  return *(uint16_t *)packet;
}

std::tuple<std::string, std::vector<std::tuple<std::string, std::string>>> parse_packet_as_meta_packet(const uint8_t *packet)
{
  std::string device_name;
  std::vector<std::tuple<std::string, std::string>> packet_description_and_serialization_format;
  device_name = std::string((char *)(packet + 2), 20);
  for (int i = 0; i < 3; ++i)
  {
    std::string packet_description = std::string((char *)(packet + 2 + 20 + 74 * i), 64);
    std::string serialization_format = std::string((char *)(packet + 2 + 20 + 74 * i + 10), 10);
    packet_description_and_serialization_format.push_back(std::make_tuple(packet_description, serialization_format));
  }
  return std::make_tuple(device_name, packet_description_and_serialization_format);
}

std::tuple<std::string, std::string, std::vector<SDPData>> parse_packet_as_data_packet(const uint8_t *packet)
{
  std::string packet_description = std::string((char *)(packet + 2), 64);
  std::string serialization_format = std::string((char *)(packet + 2 + 64), 10);
  packet_description.erase(std::find(packet_description.begin(), packet_description.end(), '\0'), packet_description.end());
  serialization_format.erase(std::find(serialization_format.begin(), serialization_format.end(), '\0'), serialization_format.end());
  std::vector<SDPData> data;
  auto packet_data_p = packet + 2 + 64 + 10;
  for (int i = 0; i < serialization_format.size(); ++i)
  {
    if (serialization_format[i] == 'i')
    {
      data.push_back(SDPData(*(int32_t *)packet_data_p));
      packet_data_p += sizeof(int32_t);
    }
    else if (serialization_format[i] == 'f')
    {
      data.push_back(SDPData(*(float *)packet_data_p));
      packet_data_p += sizeof(float);
    }
    else if (serialization_format[i] == 'S')
    {
      SDPData str = std::string((char *)packet_data_p, 64);
      data.push_back(str);
      packet_data_p += 64;
    }
    else if (serialization_format[i] == 's')
    {
      SDPData str = std::string((char *)packet_data_p, 16);
      data.push_back(str);
      packet_data_p += 16;
    }
    else if (serialization_format[i] == '?')
    {
      data.push_back(SDPData(*(bool *)packet_data_p));
      packet_data_p += sizeof(bool);
    }
  }
  return std::make_tuple(packet_description, serialization_format, data);
}

/* Version 1 functions */
void parse_packet_as_test_packet(const uint8_t *packet, uint16_t &packet_type, int32_t &num_int, float num_float,
                                 char *str)
{
  packet_type = *(uint16_t *)packet;
  num_int = *(int32_t *)(packet + 2);
  num_float = *(float *)(packet + 2 + 4);
  strncpy(str, (char *)(packet + 2 + 4 + 4), 64);
}

void parse_packet_as_named_string_packet(const uint8_t *packet, uint16_t &packet_type, char *name, char *value)
{
  packet_type = *(uint16_t *)packet;
  strncpy(name, (char *)(packet + 2), 64);
  strncpy(value, (char *)(packet + 2 + 64), 64);
}

void parse_packet_as_message_board_meta_packet(const uint8_t *packet, uint16_t &packet_type, char *module_name)
{
  packet_type = *(uint16_t *)packet;
  strncpy(module_name, (char *)(packet + 2), 64);
}

void parse_packet_as_message_board_data_packet(const uint8_t *packet, uint16_t &packet_type, char *source_name,
                                               uint64_t &timeout_duration, char *message)
{
  packet_type = *(uint16_t *)packet;
  strncpy(source_name, (char *)(packet + 2), 64);
  timeout_duration = *(uint64_t *)(packet + 2 + 64);
  strncpy(message, (char *)(packet + 2 + 64 + 8), 64);
}

#endif
