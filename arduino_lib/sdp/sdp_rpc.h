#pragma once

#include "sdp/sdp.h"

/*
 * RPC (Remote Procedure Call) Library with SDP communication
 *
 * NOTICE:
 *   - The first content of request and response bodies must be request id (integer).
 *
 */

bool init_sdp_rpc() {
  // Register callback for response waiting
  register_sdp_data_callback(_request_response_waiting_callback);
  register_sdp_data_callback(_rcp_server_base_callback);
  return true;
}

//
// RPC Server Library
//

typedef void (*sdp_rpc_cb_t)(const uint8_t *mac_addr, const std::vector<SDPData> &request_body, std::vector<SDPData> &response_body);

// RPC Server callbacks
// This dictionary is used to store the callback for each request and used in _request_response_callback
// Each element is a tuple of (request interface, response interface, callback)
std::vector<std::tuple<SDPInterfaceDescription, SDPInterfaceDescription, sdp_rpc_cb_t>> _rpc_server_callbacks;

void _rcp_server_base_callback(const uint8_t *mac_addr, const SDPInterfaceDescription &interface_description, const std::vector<SDPData> &request_body) {
  // Check if there is a callback for the request
  bool ok = false;
  SDPInterfaceDescription response_interface;
  sdp_rpc_cb_t callback;
  for (int i = 0; i < _rpc_server_callbacks.size(); i++) {
    if (std::get<0>(_rpc_server_callbacks[i]) == interface_description) {
      ok = true;
      response_interface = std::get<1>(_rpc_server_callbacks[i]);
      callback = std::get<2>(_rpc_server_callbacks[i]);
      break;
    }
  }
  if (not ok) {
    return;
  }

  // Get request body
  std::vector<SDPData> response_body;

  // Call callback
  callback(mac_addr, request_body, response_body);

  // Send response
  send_sdp_data_packet(mac_addr, response_interface, response_body);
}

bool register_rpc_service(const SDPInterfaceDescription &request_description, const SDPInterfaceDescription &response_description, sdp_rpc_cb_t callback) {
  // Check if request and response serialization formats are valid as RPC
  std::string request_serialization_format = std::get<1>(request_description);
  std::string response_serialization_format = std::get<1>(response_description);
  if (request_serialization_format[0] != 'i' or response_serialization_format != 'i') {
    return false;
  }
  _rpc_server_callbacks.push_back(std::make_tuple(request_description, response_description, callback));
  return true;
}

void unregister_rpc_service(const SDPInterfaceDescription &request_description, const SDPInterfaceDescription &response_description) {
  for (int i = 0; i < _rpc_server_callbacks.size(); i++) {
    if (std::get<0>(_rpc_server_callbacks[i]) == request_description and
        std::get<1>(_rpc_server_callbacks[i]) == response_description) {
      _rpc_server_callbacks.erase(_rpc_server_callbacks.begin() + i);
      return;
    }
  }
}

//
// RPC Client
//

// RPC response callback dictionary
// This dictionary is used to store the response from RPC response for each request and used in _request_response_callback
std::map<int32_t, std::tuple<std::string, SDPInterfaceDescription, std::vector<SDPData>>> _rpc_response_callback_dictionary;  // request id -> (server_address, response, response body)

// callback for response waiting
void _request_response_waiting_callback(const uint8_t *mac_addr,
                                        const SDPInterfaceDescription
                                            &interface_description,
                                        const std::vector<SDPData> &body) {
  // Check if the request is registered
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

// Send request as SDP Data packet and wait for response from the address
bool call_sdp_rpc(
    const uint8_t *mac_addr, const SDPInterfaceDescription &request_interface, const std::vector<SDPData> &request_body,
    const SDPInterfaceDescription &response_interface, std::vector<SDPData> &response_body,
    unsigned long duration = 3000) {
  // Generate request id
  int32_t request_id = random(0, 1000000000);

  // Store callback
  _rpc_response_callback_dictionary[request_id] = std::make_tuple(std::string(mac_addr), response_interface, std::vector<SDPData>());

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