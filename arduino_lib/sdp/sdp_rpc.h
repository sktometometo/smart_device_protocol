#pragma once

#include <functional>

#include "sdp/sdp.h"

/*
 * RPC (Remote Procedure Call) Library with SDP communication
 *
 * NOTICE:
 *   - The first content of request and response bodies must be request id (integer).
 *   - The request id is used to match the request and response.
 *   - The request id must be unique for each request.
 */

extern inline String _sdp_device_name;

/**
 * Typedefs
 **/
typedef void (*sdp_rpc_cb_t)(const uint8_t *mac_addr, const std::vector<SDPData> &request_body, std::vector<SDPData> &response_body);
typedef std::tuple<SDPInterfaceDescription, SDPInterfaceDescription, sdp_rpc_cb_t> SDPRPCCallbackEntry;

/**
 * Declarations
 **/

/**
 * @brief Initialize SDP RPC library. Before using this library, this function must be called. And before calling this function, SDP library must be initialized.
 *
 * @return true if the initialization is successful
 * @return false if the initialization is failed
 */
bool init_sdp_rpc();

/**
 * @brief Register RPC service to the server. The server will call the callback function when the request is received.
 *
 * @param request_description The description of the request interface
 * @param response_description The description of the response interface
 * @param callback The callback function
 * @return true if the registration is successful
 * @return false if the registration is failed
 */
bool register_rpc_service(const SDPInterfaceDescription &request_description, const SDPInterfaceDescription &response_description, sdp_rpc_cb_t callback);

/**
 * @brief Unregister RPC service from the server.
 *
 * @param request_description The description of the request interface
 */
void unregister_rpc_service(const SDPInterfaceDescription &request_description);

/**
 * @brief Get RPC interfaces of other devices. This function will return the list of RPC interfaces of other devices. The scanning process will take a few seconds.
 *
 * @param duration The duration of the scanning process (in milliseconds)
 * @return std::vector<std::tuple<std::string, SDPInterfaceDescription, SDPInterfaceDescription>> The list of RPC interfaces. Each element of the list is a tuple of device name, request interface description, and response interface description.
 */
std::vector<std::tuple<std::string, SDPInterfaceDescription, SDPInterfaceDescription>> get_rpc_interfaces(unsigned long duration = 5000);

/**
 * @brief Call RPC function of other devices. This function will send a request to the device and wait for the response. The response will be stored in the response_body parameter.
 *
 * @param mac_addr The MAC address of the device
 * @param request_interface The description of the request interface
 * @param request_body The request body
 * @param response_interface The description of the response interface
 * @param response_body The response body (output)
 * @param duration The duration of the waiting process (in milliseconds)
 * @return true if the request is successful
 * @return false if the request is failed
 */
bool call_sdp_rpc(const uint8_t *mac_addr, const SDPInterfaceDescription &request_interface, const std::vector<SDPData> &request_body, const SDPInterfaceDescription &response_interface, std::vector<SDPData> &response_body, unsigned long duration = 3000);

//
void _OnDataRecvRPC(const uint8_t *mac_addr, const uint8_t *data, int data_len);
bool _broadcast_sdp_rpc_meta_packet(const SDPInterfaceDescription &interface_request, const SDPInterfaceDescription &interface_response);
void _rpc_meta_frame_broadcast_task(void *parameter);

/**
 * Global variables
 **/
// RPC server callback list
std::vector<SDPRPCCallbackEntry> _rpc_server_callbacks;
// RPC response callback dictionary
std::map<int32_t, std::tuple<std::string, SDPInterfaceDescription, std::vector<SDPData>>> _rpc_response_callback_dictionary;

/**
 * RPC Library
 **/
bool init_sdp_rpc() {
  auto result = xTaskCreate(_rpc_meta_frame_broadcast_task, "rpc_meta_frame_broadcast_task", 4096, NULL, 5, NULL);
  return result == pdPASS;
  register_sdp_esp_now_recv_callback(_OnDataRecvRPC);
}

bool _broadcast_sdp_rpc_meta_packet(
    const SDPInterfaceDescription
        &interface_request,
    const SDPInterfaceDescription &interface_response) {
  uint8_t buf[240];
  std::string request_packet_description = std::get<0>(interface_request);
  std::string request_serialization_format = std::get<1>(interface_request);
  std::string response_packet_description = std::get<0>(interface_response);
  std::string response_serialization_format = std::get<1>(interface_response);
  generate_rpc_meta_frame(buf, _sdp_device_name.c_str(), request_packet_description.c_str(), request_serialization_format.c_str(), response_packet_description.c_str(), response_serialization_format.c_str());
  bool success = broadcast_sdp_esp_now_packet(buf, sizeof(buf)) == ESP_OK;
  return success;
}

void _rpc_meta_frame_broadcast_task(void *parameter) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (auto &entry : _rpc_server_callbacks) {
      SDPInterfaceDescription request_interface = std::get<0>(entry);
      SDPInterfaceDescription response_interface = std::get<1>(entry);
      _broadcast_sdp_rpc_meta_packet(request_interface, response_interface);
    }
  }
}

void _OnDataRecvRPC(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  int packet_type = get_packet_type(data);
  if (packet_type == smart_device_protocol::Packet::PACKET_TYPE_DATA) {
    auto packet = parse_packet_as_data_packet(data);
    SDPInterfaceDescription interface_description = std::get<0>(packet);
    std::string packet_description = std::get<0>(interface_description);
    std::string serialization_format = std::get<1>(interface_description);
    std::vector<SDPData> body = std::get<1>(packet);
    std::vector<SDPData> response_body;
    for (auto &entry : _rpc_server_callbacks) {
      SDPInterfaceDescription request_interface = std::get<0>(entry);
      SDPInterfaceDescription response_interface = std::get<1>(entry);
      if (std::get<0>(request_interface) == packet_description and
          std::get<1>(request_interface) == serialization_format) {
        std::get<2>(entry)(mac_addr, body, response_body);
        send_sdp_data_packet(mac_addr, response_interface, response_body);
      }
    }
  }
}

bool register_rpc_service(const SDPInterfaceDescription &request_description,
                          const SDPInterfaceDescription &response_description,
                          sdp_rpc_cb_t callback) {
  _rpc_server_callbacks.push_back(std::make_tuple(request_description, response_description, callback));
  return true;
}

void unregister_rpc_service(const SDPInterfaceDescription &request_description) {
  for (int i = 0; i < _rpc_server_callbacks.size(); i++) {
    if (std::get<0>(_rpc_server_callbacks[i]) == request_description) {
      _rpc_server_callbacks.erase(_rpc_server_callbacks.begin() + i);
      break;
    }
  }
}

// callback for response waiting
void _request_response_waiting_callback(const uint8_t *mac_addr,
                                        const SDPInterfaceDescription
                                            &interface_description,
                                        const std::vector<SDPData> &body) {
  // Check if there is a callback matches with the response interface description
  bool ok = false;
  for (int i = 0; i < _rpc_response_callback_dictionary.size(); i++) {
    SDPInterfaceDescription response_interface = std::get<1>(_rpc_response_callback_dictionary[i]);
    if (std::get<0>(response_interface) == std::get<0>(interface_description) and
        std::get<1>(response_interface) == std::get<1>(interface_description)) {
      ok = true;
      break;
    }
  }
  if (not ok) {
    return;
  }

  int32_t request_id = std::get<int32_t>(body[0]);
  if (_rpc_response_callback_dictionary.find(request_id) ==
      _rpc_response_callback_dictionary.end()) {
    return;
  }

  // Get request and response
  std::string server_address = std::get<0>(
      _rpc_response_callback_dictionary[request_id]);
  SDPInterfaceDescription response_interface = std::get<1>(
      _rpc_response_callback_dictionary[request_id]);

  // Check if the request is correct
  if (std::get<0>(response_interface) != std::get<0>(interface_description) or
      std::get<1>(response_interface) != std::get<1>(interface_description)) {
    return;
  }

  // Get response body
  std::vector<SDPData> response_body;
  for (int i = 1; i < body.size(); i++) {
    response_body.push_back(body[i]);
  }

  // Store response body
  _rpc_response_callback_dictionary[request_id] = std::make_tuple(server_address, response_interface, response_body);
}

bool call_sdp_rpc(
    const uint8_t *mac_addr, const SDPInterfaceDescription &request_interface, const std::vector<SDPData> &request_body,
    const SDPInterfaceDescription &response_interface, std::vector<SDPData> &response_body,
    unsigned long duration) {
  // Generate request id
  int32_t request_id = random(0, 1000000000);

  // Store callback
  std::string mac_addr_str = _convert_mac_address(mac_addr);
  _rpc_response_callback_dictionary[request_id] = std::make_tuple(mac_addr_str, response_interface, std::vector<SDPData>());

  // Send request
  if (not send_sdp_data_packet(mac_addr, request_interface, request_body)) {
    return false;
  }

  // Wait for response
  unsigned long start_time = millis();
  while (millis() - start_time < duration) {
    // Check if response is received
    if (std::get<2>(_rpc_response_callback_dictionary[request_id]).size() > 0) {
      // Get response body
      response_body = std::get<2>(_rpc_response_callback_dictionary[request_id]);
      // Remove callback
      _rpc_response_callback_dictionary.erase(request_id);
      return true;
    }

    // Wait for 1ms
    delay(1);
  }

  return false;
}