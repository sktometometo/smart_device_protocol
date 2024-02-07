#pragma once

/*
 * SDP (Smart Device Protocol) Library
 *
 * To use these functions, you need to include the following libraries:
 * - smart_device_protocol/Packet.h
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <smart_device_protocol/Packet.h>
#include <smart_device_protocol/PacketType.h>

#include <map>

#include "sdp/esp_now.h"
#include "sdp/packet_creator.h"
#include "sdp/packet_parser.h"
#include "sdp/sdp_util.h"

/*
 * Type Definitions
 */
typedef void (*sdp_data_if_recv_cb_t)(const uint8_t *mac_addr,
                                      const std::vector<SDPData> &body);
typedef void (*sdp_data_recv_cb_t)(
    const uint8_t *mac_addr,
    const SDPInterfaceDescription &interface_description,
    const std::vector<SDPData> &body);
typedef void (*sdp_meta_recv_cb_t)(
    const uint8_t *mac_addr, const std::string &device_name,
    const std::vector<SDPInterfaceDescription> &interfaces);
typedef void (*sdp_rpc_meta_recv_cb_t)(
    const uint8_t *mac_addr, const std::string &device_name,
    const SDPInterfaceDescription &interface_request,
    const SDPInterfaceDescription &interface_response);
typedef std::tuple<SDPInterfaceDescription, sdp_data_if_recv_cb_t>
    SDPInterfaceCallbackEntry;

/*
 * Internal variables
 */
inline String _sdp_device_name;
inline std::vector<SDPInterfaceCallbackEntry> _sdp_interface_data_callbacks;
inline std::vector<sdp_data_recv_cb_t> _sdp_data_callbacks;
inline std::vector<sdp_meta_recv_cb_t> _sdp_meta_callbacks;
inline std::vector<sdp_rpc_meta_recv_cb_t> _sdp_rpc_meta_callbacks;
inline std::vector<esp_now_recv_cb_t> _esp_now_recv_callbacks;
// Internal variables for get_sdp_interfaces()
// Each element stands for mac_addr, device_name, interfaces
inline std::vector<
    std::tuple<std::string, std::string, std::vector<SDPInterfaceDescription>>>
    _vector_device_interfaces;
// Address <-> Device name dictionary
inline std::map<std::string, std::string> _device_name_dictionary;

/*
 *　Declaration of functions
 */

/**
 * @brief Initialize SDP (Smart Device Protocol) library
 *
 * @param mac_address The MAC address of the device will be output to this parameter
 * @param device_name The name of the device.
 * @param meta_task_stack_size The stack size of the meta frame broadcast task
 * @return true if the initialization is successful
 * @return false if the initialization is failed
 */
bool init_sdp(uint8_t *mac_address, const String &device_name, int meta_task_stack_size = 16384);

/**
 * @brief Register a callback function for a DataFrame for a specific interface
 *
 * @param packet_description_and_serialization_format The description of the interface
 * @param callback The callback function
 * @return true if the registration is successful
 * @return false if the registration is failed
 */
bool register_sdp_interface_callback(
    SDPInterfaceDescription packet_description_and_serialization_format,
    sdp_data_if_recv_cb_t callback);

/**
 * @brief Unregister a callback function for a DataFrame for a specific interface
 *
 * @param packet_description_and_serialization_format The description of the interface
 * @return true
 * @return false
 */
bool unregister_sdp_interface_callback(
    SDPInterfaceDescription packet_description_and_serialization_format);

/**
 * @brief Register a callback function for a DataFrame
 *
 * @param callback The callback function
 * @return true
 * @return false
 */
bool register_sdp_data_callback(sdp_data_recv_cb_t callback);

/**
 * @brief Unregister a callback function for a DataFrame
 *
 * @param callback The callback function
 * @return true
 * @return false
 */
bool unregister_sdp_data_callback(sdp_data_recv_cb_t callback);

/**
 * @brief Register a callback function for a MetaFrame
 *
 * @param callback
 * @return true
 * @return false
 */
bool register_sdp_meta_callback(sdp_meta_recv_cb_t callback);

/**
 * @brief Unregister a callback function for a MetaFrame
 *
 * @param callback
 * @return true
 * @return false
 */
bool unregister_sdp_meta_callback(sdp_meta_recv_cb_t callback);

/**
 * @brief Register a callback function for a RPCMetaFrame
 *
 * @param callback
 * @return true
 * @return false
 */
bool register_sdp_rpc_meta_callback(sdp_rpc_meta_recv_cb_t callback);

/**
 * @brief Unregister a callback function for a RPCMetaFrame
 *
 * @param callback
 * @return true
 * @return false
 */
bool unregister_sdp_rpc_meta_callback(sdp_rpc_meta_recv_cb_t callback);

/**
 * @brief Register a callback function for a received ESP-NOW packet
 *
 * @param callback
 * @return true
 * @return false
 */
bool register_sdp_esp_now_recv_callback(esp_now_recv_cb_t callback);

/**
 * @brief Unregister a callback function for a received ESP-NOW packet
 *
 * @param callback
 * @return true
 * @return false
 */
bool unregister_sdp_esp_now_recv_callback(esp_now_recv_cb_t callback);

/**
 * @brief Send a DataFrame to a specific device.
 *
 * @param peer_addr The MAC address of the device
 * @param interface_description The description of the interface
 * @param body The body of the DataFrame
 * @return true
 * @return false
 */
bool send_sdp_data_packet(const uint8_t *peer_addr,
                          const SDPInterfaceDescription &interface_description,
                          const std::vector<SDPData> &body);

/**
 * @brief Broadcast a DataFrame
 *
 * @param packet_description The description of the interface
 * @param body The body of the DataFrame
 * @return true
 * @return false
 */
bool send_sdp_data_packet(const std::string &packet_description,
                          const std::vector<SDPData> &body);

/**
 * @brief Broadcast a DataFrame. Serialization format is automatically determined from body
 *
 * @param interface_description The description of the interface
 * @param body The body of the DataFrame
 * @return true
 * @return false
 */
bool send_sdp_data_packet(const SDPInterfaceDescription &interface_description,
                          const std::vector<SDPData> &body);

/**
 * @brief Send an ESP-NOW packet
 *
 * @param peer_addr
 * @param data
 * @param data_len
 * @return esp_err_t
 */
esp_err_t send_sdp_esp_now_packet(const uint8_t *peer_addr, const uint8_t *data,
                                  const int data_len);

/**
 * @brief Broadcast an ESP-NOW packet
 *
 * @param data
 * @param data_len
 * @return esp_err_t
 */
esp_err_t broadcast_sdp_esp_now_packet(const uint8_t *data, const int data_len);

/**
 * @brief Get the sdp interfaces of other devices. This function will return the list of SDP interfaces of other devices. The scanning process will take a few seconds.
 *
 * @param duration timeout duration in miliseconds
 * @return const std::vector<std::tuple<std::string, std::string,
 * std::vector<SDPInterfaceDescription>>>&
 */
const std::vector<std::tuple<std::string, std::string,
                             std::vector<SDPInterfaceDescription>>> &
get_sdp_interfaces(unsigned long duration = 3000);

/*
 *  Function declarations
 */
void _OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
bool _broadcast_sdp_meta_packet(
    const SDPInterfaceDescription &packet_description_and_serialization_format);
void _meta_frame_broadcast_task(void *parameter);
void _get_device_interfaces_callback(
    const uint8_t *mac_addr, const std::string &device_name,
    const std::vector<SDPInterfaceDescription> &interfaces);

/*
 * Definitions
 */
void _OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  uint8_t packet_type = get_packet_type(data);
  for (auto &entry : _esp_now_recv_callbacks) {
    entry(mac_addr, data, data_len);
  }
  if (packet_type == smart_device_protocol::Packet::PACKET_TYPE_DATA) {
    auto packet = parse_packet_as_data_packet(data);
    SDPInterfaceDescription packet_description_and_serialization_format =
        std::get<0>(packet);
    std::string packet_description =
        std::get<0>(packet_description_and_serialization_format);
    std::string serialization_format =
        std::get<1>(packet_description_and_serialization_format);
    std::vector<SDPData> body = std::get<1>(packet);

    for (auto &entry : _sdp_data_callbacks) {
      entry(mac_addr, packet_description_and_serialization_format, body);
    }

    for (auto &entry : _sdp_interface_data_callbacks) {
      if (packet_description == std::get<0>(std::get<0>(entry)) and
          serialization_format == std::get<1>(std::get<0>(entry))) {
        std::get<1>(entry)(mac_addr, body);
      }
    }
  } else if (packet_type == smart_device_protocol::Packet::PACKET_TYPE_META) {
    auto packet = parse_packet_as_meta_packet(data);
    std::string device_name = std::get<0>(packet);
    std::vector<SDPInterfaceDescription> interfaces = std::get<1>(packet);

    for (auto &entry : _sdp_meta_callbacks) {
      entry(mac_addr, device_name, interfaces);
    }
  } else if (packet_type == smart_device_protocol::PacketType::PACKET_TYPE_RPC_META) {
    auto packet = parse_packet_as_rpc_meta_packet(data);
    std::string device_name = std::get<0>(packet);
    SDPInterfaceDescription interface_request = std::get<1>(packet);
    SDPInterfaceDescription interface_response = std::get<2>(packet);
    for (auto &entry : _sdp_rpc_meta_callbacks) {
      entry(mac_addr, device_name, interface_request, interface_response);
    }
  }
}

bool _broadcast_sdp_meta_packet(
    const SDPInterfaceDescription
        &packet_description_and_serialization_format) {
  uint8_t buf[245];
  const std::string &packet_description =
      std::get<0>(packet_description_and_serialization_format);
  const std::string &serialization_format =
      std::get<1>(packet_description_and_serialization_format);
  generate_meta_frame(buf, _sdp_device_name.c_str(), packet_description.c_str(),
                      serialization_format.c_str(), "", "", "", "");
  bool success = broadcast_sdp_esp_now_packet(buf, sizeof(buf)) == ESP_OK;
  return success;
}

void _meta_frame_broadcast_task(void *parameter) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (auto &entry : _sdp_interface_data_callbacks) {
      const SDPInterfaceDescription
          &packet_description_and_serialization_format = std::get<0>(entry);
      _broadcast_sdp_meta_packet(packet_description_and_serialization_format);
    }
    if (_sdp_interface_data_callbacks.size() == 0) {
      _broadcast_sdp_meta_packet(std::make_tuple("", ""));
    }
  }
}

bool init_sdp(uint8_t *mac_address, const String &device_name, int meta_task_stack_size) {
  if (mac_address != NULL) {
    esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
  }
  _sdp_device_name = device_name;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (not esp_now_init() == ESP_OK) {
    return false;
  }
  auto result = xTaskCreate(_meta_frame_broadcast_task, "meta_br_task", meta_task_stack_size, NULL, 1, NULL);
  if (result != pdPASS) {
    return false;
  }
  esp_now_register_recv_cb(_OnDataRecv);
  return true;
}

bool register_sdp_interface_callback(
    SDPInterfaceDescription packet_description_and_serialization_format,
    sdp_data_if_recv_cb_t callback) {
  // Check if the callback is already registered
  for (auto &entry : _sdp_interface_data_callbacks) {
    if (std::get<0>(std::get<0>(entry)) ==
            std::get<0>(packet_description_and_serialization_format) and
        std::get<1>(std::get<0>(entry)) ==
            std::get<1>(packet_description_and_serialization_format)) {
      return false;
    }
  }
  _sdp_interface_data_callbacks.push_back(
      std::make_tuple(packet_description_and_serialization_format, callback));
  return true;
}

bool unregister_sdp_interface_callback(
    SDPInterfaceDescription packet_description_and_serialization_format) {
  for (auto it = _sdp_interface_data_callbacks.begin();
       it != _sdp_interface_data_callbacks.end(); ++it) {
    if (std::get<0>(std::get<0>(*it)) ==
            std::get<0>(packet_description_and_serialization_format) and
        std::get<1>(std::get<0>(*it)) ==
            std::get<1>(packet_description_and_serialization_format)) {
      _sdp_interface_data_callbacks.erase(it);
      return true;
    }
  }
  return false;
}

bool register_sdp_data_callback(sdp_data_recv_cb_t callback) {
  _sdp_data_callbacks.push_back(callback);
  return true;
}

bool unregister_sdp_data_callback(sdp_data_recv_cb_t callback) {
  for (auto it = _sdp_data_callbacks.begin(); it != _sdp_data_callbacks.end();
       ++it) {
    if (*it == callback) {
      _sdp_data_callbacks.erase(it);
      return true;
    }
  }
  return false;
}

bool register_sdp_meta_callback(sdp_meta_recv_cb_t callback) {
  _sdp_meta_callbacks.push_back(callback);
  return true;
}

bool unregister_sdp_meta_callback(sdp_meta_recv_cb_t callback) {
  for (auto it = _sdp_meta_callbacks.begin(); it != _sdp_meta_callbacks.end();
       ++it) {
    if (*it == callback) {
      _sdp_meta_callbacks.erase(it);
      return true;
    }
  }
  return false;
}

bool register_sdp_rpc_meta_callback(sdp_rpc_meta_recv_cb_t callback) {
  _sdp_rpc_meta_callbacks.push_back(callback);
  return true;
}

bool unregister_sdp_rpc_meta_callback(sdp_rpc_meta_recv_cb_t callback) {
  for (auto it = _sdp_rpc_meta_callbacks.begin();
       it != _sdp_rpc_meta_callbacks.end(); ++it) {
    if (*it == callback) {
      _sdp_rpc_meta_callbacks.erase(it);
      return true;
    }
  }
  return false;
}

bool register_sdp_esp_now_recv_callback(esp_now_recv_cb_t callback) {
  _esp_now_recv_callbacks.push_back(callback);
  return true;
}

bool unregister_sdp_esp_now_recv_callback(esp_now_recv_cb_t callback) {
  for (auto it = _esp_now_recv_callbacks.begin();
       it != _esp_now_recv_callbacks.end(); ++it) {
    if (*it == callback) {
      _esp_now_recv_callbacks.erase(it);
      return true;
    }
  }
  return false;
}

bool send_sdp_data_packet(const uint8_t *peer_addr,
                          const SDPInterfaceDescription &interface_description,
                          const std::vector<SDPData> &body) {
  uint8_t buf[240];
  const std::string &packet_description = std::get<0>(interface_description);
  const std::string &serialization_format = std::get<1>(interface_description);
  bool ret = generate_data_frame(buf, packet_description.c_str(),
                                 serialization_format.c_str(), body);
  if (not ret) {
    return false;
  } else {
    esp_err_t result = send_sdp_esp_now_packet(peer_addr, buf, sizeof(buf));
    return result == ESP_OK;
  }
}

bool send_sdp_data_packet(const std::string &packet_description,
                          const std::vector<SDPData> &body) {
  uint8_t buf[240];
  bool ret = generate_data_frame(buf, packet_description.c_str(), body);
  if (not ret) {
    return false;
  } else {
    esp_err_t result = broadcast_sdp_esp_now_packet(buf, sizeof(buf));
    return result == ESP_OK;
  }
}

bool send_sdp_data_packet(const SDPInterfaceDescription &interface_description,
                          const std::vector<SDPData> &body) {
  uint8_t buf[240];
  const std::string &packet_description = std::get<0>(interface_description);
  const std::string &serialization_format = std::get<1>(interface_description);
  bool ret = generate_data_frame(buf, packet_description.c_str(),
                                 serialization_format.c_str(), body);
  if (not ret) {
    return false;
  } else {
    esp_err_t result = broadcast_sdp_esp_now_packet(buf, sizeof(buf));
    return result == ESP_OK;
  }
}

esp_err_t send_sdp_esp_now_packet(const uint8_t *peer_addr, const uint8_t *data,
                                  const int data_len) {
  esp_now_peer_info_t peer_info;
  memset(&peer_info, 0, sizeof(peer_info));
  for (int i = 0; i < 6; i++) {
    peer_info.peer_addr[i] = peer_addr[i];
  }
  esp_err_t addStatus = esp_now_add_peer(&peer_info);
  if (addStatus != ESP_OK) {
    return addStatus;
  }
  esp_err_t result = esp_now_send(peer_addr, data, data_len);
  esp_now_del_peer(peer_addr);
  return result;
}

esp_err_t broadcast_sdp_esp_now_packet(const uint8_t *data, const int data_len) {
  const uint8_t broadcast_address[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  return send_sdp_esp_now_packet(broadcast_address, data, data_len);
}

// Retrive device interfaces from meta package for given duration
// Return value is vector of tuple of address, device name, interfaces
const std::vector<std::tuple<std::string, std::string,
                             std::vector<SDPInterfaceDescription>>> &
get_sdp_interfaces(unsigned long duration) {
  _vector_device_interfaces.clear();
  unsigned long start_time = millis();
  register_sdp_meta_callback(_get_device_interfaces_callback);
  while (millis() - start_time < duration) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  unregister_sdp_meta_callback(_get_device_interfaces_callback);
  return _vector_device_interfaces;
}

// Call back function for get_device_interfaces() to store MetaFrame
void _get_device_interfaces_callback(
    const uint8_t *mac_addr, const std::string &device_name,
    const std::vector<SDPInterfaceDescription> &interfaces) {
  std::string address = _convert_mac_address(mac_addr);

  // if _vector_device_interfaces has not entry for address, add interfaces to
  // it
  for (auto &entry : _vector_device_interfaces) {
    if (std::get<0>(entry) == address) {
      for (auto &interface : interfaces) {
        // Add only if interface is not empty
        if (std::get<0>(interface).size() > 0 and
            std::get<1>(interface).size() > 0) {
          // Add only there is no same interface
          bool ok = true;
          for (auto &original_interface : std::get<2>(entry)) {
            if (std::get<0>(original_interface) == std::get<0>(interface) and
                std::get<1>(original_interface) == std::get<1>(interface)) {
              ok = false;
              break;
            }
          }
          if (ok) {
            std::get<2>(entry).push_back(interface);
          }
        }
      }
      return;
    }
  }
  // add interface which is not empty
  std::vector<SDPInterfaceDescription> valid_interfaces;
  for (auto &interface : interfaces) {
    if (std::get<0>(interface).size() > 0 and
        std::get<1>(interface).size() > 0) {
      valid_interfaces.push_back(
          std::make_tuple(std::get<0>(interface), std::get<1>(interface)));
    }
  }
  _vector_device_interfaces.push_back(
      std::make_tuple(address, device_name, valid_interfaces));
  return;
}
