#ifndef ESP_NOW_ROS_PACKET_CREATOR_H__
#define ESP_NOW_ROS_PACKET_CREATOR_H__

#include <esp_now_ros/Packet.h>

void create_sensor_enviii_packet(uint8_t* packet, const char* module_name, int32_t pressure)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_SENSOR_ENV_III;
  strncpy((char*)(packet + 2), module_name, 64);
  *(int32_t*)(packet + 2 + 64) = pressure;
}

void create_sensor_stickv2_packet(uint8_t* packet, uint32_t number_of_person, const char* place_name)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER;
  *(uint32_t*)(packet + 2) = number_of_person;
  strncpy((char*)(packet + 2 + 4), place_name, 64);
}

void create_device_message_board_meta_packet(uint8_t* packet, const char* module_name)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_META;
  strncpy((char*)(packet + 2), module_name, 64);
}

void create_device_message_board_data_packet(uint8_t* packet, const char* source_name, uint64_t timeout_duration,
                                             const char* message)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA;
  strncpy((char*)(packet + 2), source_name, 64);
  *(uint64_t*)(packet + 2 + 64) = timeout_duration;
  strncpy((char*)(packet + 2 + 64 + 8), message, 64);
}

void generate_meta_frame(uint8_t* packet, const char* device_name, const char* packet_description_01, const char* serialization_format_01, const char* packet_description_02, const char* serialization_format_02, const char* packet_description_03, const char serialization_format_03)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_META_FRAME;
  strncpy((char*)(packet + 2), device_name, 20);
  strncpy((char*)(packet + 2 + 20), packet_description_01, 64);
  strncpy((char*)(packet + 2 + 20 + 64), serialization_format_01, 10);
  strncpy((char*)(packet + 2 + 20 + 64 + 10), packet_description_02, 64);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64), serialization_format_02, 10);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64 + 10), packet_description_03, 64);
  strncpy((char*)(packet + 2 + 20 + 64 + 10 + 64 + 10 + 64), serialization_format_03, 10);
}

void generate_data_frame(uint8_t* packet, const char* packet_description, const char* serialization_format, std::vector<std::variant<int, float, std::string, bool>> data)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_DATA_FRAME;
  strncpy((char*)(packet + 2), packet_description, 64);
  strncpy((char*)(packet + 2 + 64), serialization_format, 10);
  for (auto = data.begin(); it != data.end(); ++it) {
    
  }
  // TODO
}

#endif
