#include <esp_now_ros/Packet.h>

uint16_t get_packet_type(const uint8_t* packet)
{
  return *(uint16_t*)packet;
}

void parse_packet_as_test_packet(const uint8_t* packet, uint16_t& packet_type, int32_t& num_int, float num_float,
                                 char* str)
{
  packet_type = *(uint16_t*)packet;
  num_int = *(int32_t*)(packet + 2);
  num_float = *(float*)(packet + 2 + 4);
  size_t str_len = strncpy(str, (char*)(packet + 2 + 4 + 4), 64);
  str[str_len] = 0;
}

void parse_packet_as_named_string_packet(const uint8_t* packet, uint16_t& packet_type, char* name, char* value)
{
  packet_type = *(uint16_t*)packet;
  size_t str_len;
  str_len = strncpy(name, (char*)(packet + 2), 64);
  name[str_len] = 0;
  str_len = strncpy(value, (char*)(packet + 2 + 64), 64);
  name[str_len] = 0;
}
