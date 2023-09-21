#ifndef ESP_NOW_ROS_PACKET_CREATOR_H__
#define ESP_NOW_ROS_PACKET_CREATOR_H__

#include <variant>
#include <vector>
#include <string.h>

#include <esp_now_ros/Packet.h>

std::string get_serialization_format(std::vector<std::variant<int32_t, float, std::string, bool>>& data)
{
  std::string serialization_format;
  for (auto itr = data.begin(); itr != data.end(); ++itr)
  {
    if (std::holds_alternative<int32_t>(*itr))
    {
      serialization_format += "i";
    }
    else if (std::holds_alternative<float>(*itr))
    {
      serialization_format += "f";
    }
    else if (std::holds_alternative<std::string>(*itr))
    {
      if (std::get<std::string>(*itr).size() > 16)
      {
        serialization_format += "S";
      }
      else
      { 
        serialization_format += "s";
      }
    }
    else if (std::holds_alternative<bool>(*itr))
    {
      serialization_format += "b";
    }
  }
  return serialization_format;
}



void generate_meta_frame(uint8_t* packet, const char* device_name, const char* packet_description_01,
                         const char* serialization_format_01, const char* packet_description_02,
                         const char* serialization_format_02, const char* packet_description_03,
                         const char* serialization_format_03)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_META;
  strncpy((char*)(packet + 2), device_name, 20);
  strncpy((char*)(packet + 2 + 20), packet_description_01, 64);
  strncpy((char*)(packet + 2 + 20 + 64), serialization_format_01, 10);
  strncpy((char*)(packet + 2 + 20 + 64 + 10), packet_description_02, 64);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64), serialization_format_02, 10);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64 + 10), packet_description_03, 64);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64 + 10 + 64), serialization_format_03, 10);
}

void generate_data_frame(uint8_t* packet, const char* packet_description, const char* serialization_format,
                         std::vector<std::variant<int32_t, float, std::string, bool>> data)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_DATA;
  strncpy((char*)(packet + 2), packet_description, 64);
  strncpy((char*)(packet + 2 + 64), serialization_format, 10);
  auto packet_data_p = packet + 2 + 64 + 10;
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    if (std::holds_alternative<int32_t>(*it))
    {
      *(int32_t*)packet_data_p = std::get<int32_t>(*it);
      packet_data_p += sizeof(int32_t);
    }
    else if (std::holds_alternative<float>(*it))
    {
      *(float*)packet_data_p = std::get<float>(*it);
      packet_data_p += sizeof(float);
    }
    else if (std::holds_alternative<std::string>(*it))
    {
      std::string str = std::get<std::string>(*it);
      if (str.size() > 64)
      {
        str.resize(64);
        strncpy((char*)packet_data_p, str.c_str(), 64);
        packet_data_p += 64;
      }
      else if (str.size() <= 64 and str.size() > 16)
      {
        strncpy((char*)packet_data_p, str.c_str(), str.size());
        for (int i = str.size(); i < 64; ++i)
        {
          *(char*)(packet + 2 + 64 + 10 + i) = '\0';
        }
        packet_data_p += 64;
      }
      else if (str.size() <= 16)
      {
        strncpy((char*)packet_data_p, str.c_str(), str.size());
        for (int i = str.size(); i < 16; ++i)
        {
          *(char*)(packet + 2 + 64 + 10 + i) = '\0';
        }
        packet_data_p += 16;
      }
    }
    else if (std::holds_alternative<bool>(*it))
    {
      *(bool*)packet_data_p = std::get<bool>(*it);
      packet_data_p += sizeof(bool);
    }
  }
}

void generate_data_frame(uint8_t* packet, const char* packet_description,
                         std::vector<std::variant<int32_t, float, std::string, bool>>& data)
{
  std::string serialization_format = get_serialization_format(data);
  generate_data_frame(packet, packet_description, serialization_format.c_str(), data);
}

#endif
