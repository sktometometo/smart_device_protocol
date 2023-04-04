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
