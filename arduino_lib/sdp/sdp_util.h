#ifndef SMART_DEVICE_PROTOCOL_SDP_UTIL_H
#define SMART_DEVICE_PROTOCOL_SDP_UTIL_H

#include <Arduino.h>

#include <array>
#include <string>

std::string _convert_mac_address(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return "";
  } else {
    String addr = String(mac_addr[0], 16) + ":" + String(mac_addr[1], 16) +
                  ":" + String(mac_addr[2], 16) + ":" +
                  String(mac_addr[3], 16) + ":" + String(mac_addr[4], 16) +
                  ":" + String(mac_addr[5], 16);
    return addr.c_str();
  }
}

void _convert_mac_address(const std::string &mac_addr, uint8_t *mac_addr_array) {
  sscanf(mac_addr.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
         &mac_addr_array[0], &mac_addr_array[1], &mac_addr_array[2],
         &mac_addr_array[3], &mac_addr_array[4], &mac_addr_array[5]);
}

#endif  // SMART_DEVICE_PROTOCOL_SDP_UTIL_H
