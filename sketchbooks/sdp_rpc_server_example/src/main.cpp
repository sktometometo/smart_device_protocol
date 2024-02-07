#if defined(M5STACK_FIRE)
#include <M5Stack.h>
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>
#endif
#include <smart_device_protocol/Packet.h>

#include "sdp/sdp.h"
#include "sdp/sdp_rpc.h"

// Device name
String device_name = "SDP RPC EXAMPLE";

// ESP-NOW
uint8_t mac_address[6] = {0};

// SDP RPC Interface
std::string packet_description_request = "Echo req";
std::string serialization_format_request = "isifS";
SDPInterfaceDescription interface_description_request(packet_description_request, serialization_format_request);
std::string packet_description_response = "Echo res";
std::string serialization_format_response = "isifS";
SDPInterfaceDescription interface_description_response(packet_description_response, serialization_format_response);

// Other
std::vector<SDPData> data;

void rpc_handler(const uint8_t *mac_addr,
                 const std::vector<SDPData> &request_body,
                 std::vector<SDPData> &response_body) {
  std::string mac_str = _convert_mac_address(mac_addr);
  Serial.printf("RPC Handler called from %s\n", mac_str.c_str());
  int request_id = std::get<int32_t>(request_body[0]);
  response_body.clear();
  response_body.push_back(SDPData(request_id));
  response_body.push_back(SDPData(std::get<std::string>(request_body[1])));
  response_body.push_back(SDPData(std::get<int32_t>(request_body[2])));
  response_body.push_back(SDPData(std::get<float>(request_body[3])));
  response_body.push_back(SDPData(std::get<std::string>(request_body[4])));
}

void setup() {
  M5.begin(true, true, true, false);
  Serial.begin(115200);

  // LCD Print
  M5.Lcd.printf("SDP RPC EXAMPLE\n");

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name)) {
    Serial.println("Failed to initialize SDP");
    M5.lcd.printf("Failed to initialize SDP\n");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SDP Initialized!");

  // Initialization of SDP RPC
  if (not init_sdp_rpc()) {
    Serial.println("Failed to initialize SDP RPC");
    M5.lcd.printf("Failed to initialize SDP RPC\n");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SDP RPC Initialized!");

  // Register RPC handler
  register_rpc_service(interface_description_request, interface_description_response, rpc_handler);

  // Display MAC address
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  Serial.printf("Name: %s\n", device_name.c_str());
  Serial.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  Serial.println("Initialization completed!");
}

void loop() {
  delay(5000);
}
