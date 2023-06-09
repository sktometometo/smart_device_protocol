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

#endif
