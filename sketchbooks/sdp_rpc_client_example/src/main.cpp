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

// Other
std::vector<SDPData> data;

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

  // Display MAC address
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  Serial.printf("Name: %s\n", device_name.c_str());
  Serial.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  Serial.println("Initialization completed!");

  Serial.print("Input >");
}

std::vector<std::tuple<std::__cxx11::string, SDPInterfaceDescription, SDPInterfaceDescription>> rpc_interfaces;

void loop() {
  delay(1000);

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.replace("\n", "");
    command.replace("\r", "");
    Serial.println(command);

    if (command == "help") {
      Serial.printf("Commands:\n");
      Serial.printf("  help: Show this help\n");
      Serial.printf("  list: List RPC Interfaces\n");
      Serial.printf("  call: Call RPC Interface\n");
    } else if (command == "list") {
      Serial.printf("Getting RPC Interfaces...\n");
      rpc_interfaces = get_rpc_interfaces();
      Serial.printf("Done! %d interfaces found\n", rpc_interfaces.size());

      for (const auto& rpc_interface : rpc_interfaces) {
        const std::string& name = std::get<0>(rpc_interface);
        const SDPInterfaceDescription& request_description = std::get<1>(rpc_interface);
        const SDPInterfaceDescription& response_description = std::get<2>(rpc_interface);
        const std::string& request_packet_description = std::get<0>(request_description);
        const std::string& request_serialization_format = std::get<1>(request_description);
        const std::string& response_packet_description = std::get<0>(response_description);
        const std::string& response_serialization_format = std::get<1>(response_description);

        Serial.printf("=============================\n");
        Serial.printf("Name: \"%s\"\n", name.c_str());
        Serial.printf("Request Packet Description: \"%s\"\n", request_packet_description.c_str());
        Serial.printf("Request Serialization: \"%s\"\n", request_serialization_format.c_str());
        Serial.printf("Response Packet Description: \"%s\"\n", response_packet_description.c_str());
        Serial.printf("Response Serialization: \"%s\"\n", response_serialization_format.c_str());
        Serial.printf("=============================\n");
      }
    } else if (command == "call") {
      Serial.print("Select interface index > ");
      while (not Serial.available()) {
        delay(100);
      }
      String index_str = Serial.readStringUntil('\n');
      index_str.replace("\n", "");
      index_str.replace("\r", "");
      Serial.println(index_str);
      int index = index_str.toInt();
      if (index < 0 or index >= rpc_interfaces.size()) {
        Serial.printf("Invalid index: %d\n", index);
        return;
      }
      auto rpc_interface = rpc_interfaces[index];
      const std::string& name = std::get<0>(rpc_interface);
      const SDPInterfaceDescription& request_description = std::get<1>(rpc_interface);
      const SDPInterfaceDescription& response_description = std::get<2>(rpc_interface);
      std::string request_serialization_format = std::get<1>(request_description);
      std::string response_serialization_format = std::get<1>(response_description);
      std::vector<SDPData> request_data;
      request_data.push_back(SDPData(1));
      std::vector<SDPData> response_data;
      for (int i = 1; i < request_serialization_format.size(); i++) {
        char type = request_serialization_format[i];
        Serial.printf("Input %d/%d (type: %c) > ", i + 1, request_serialization_format.size(), type);
        while (not Serial.available()) {
          delay(100);
        }
        String input = Serial.readStringUntil('\n');
        input.replace("\n", "");
        input.replace("\r", "");
        Serial.println(input);
        int32_t val;
        float fval;
        bool bval;
        std::string sval;
        switch (type) {
          case 'i':
            val = input.toInt();
            Serial.printf("val: %d\n", val);
            request_data.push_back(SDPData(val));
            break;
          case 'f':
            fval = input.toFloat();
            Serial.printf("val: %f\n", fval);
            request_data.push_back(SDPData(fval));
            break;
          case '?':
          case 'b':
            bval;
            if (input == "true") {
              bval = true;
            } else if (input == "false") {
              bval = false;
            } else {
              Serial.printf("Invalid input: %s\n", input.c_str());
            }
            Serial.printf("val: %s\n", bval ? "true" : "false");
            request_data.push_back(SDPData(bval));
          case 's':
          case 'S':
            sval = input.c_str();
            Serial.printf("val: %s\n", sval.c_str());
            request_data.push_back(SDPData(sval));
            break;
          default:
            Serial.printf("Unknown type: %c\n", type);
            return;
        }
      }
      uint8_t mac_address_list[6] = {0};
      _convert_mac_address(name, mac_address_list);
      if (call_sdp_rpc(mac_address_list, request_description, request_data, response_description, response_data)) {
        Serial.printf("Response: ");
        for (int i = 0; i < response_serialization_format.size(); i++) {
          char type = response_serialization_format[i];
          switch (type) {
            case 'i':
              Serial.printf("%d ", std::get<int32_t>(response_data[i]));
              break;
            case 'f':
              Serial.printf("%f ", std::get<float>(response_data[i]));
              break;
            case '?':
            case 'b':
              Serial.printf("%s ", std::get<bool>(response_data[i]) ? "true" : "false");
              break;
            case 's':
            case 'S':
              Serial.printf("%s ", std::get<std::string>(response_data[i]).c_str());
              break;
            default:
              Serial.printf("Unknown type: %c\n", type);
              return;
          }
        }
        Serial.printf("\n");
      } else {
        Serial.printf("Failed to call RPC\n");
      }
    } else {
      Serial.printf("Unknown command: \"%s\"\n", command.c_str());
    }

    Serial.print("Input >");
  }
}
