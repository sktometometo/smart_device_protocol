#include <ArduinoJson.h>
#include <FS.h>
#include <M5Core2.h>
#include <SPIFFS.h>
#include <smart_device_protocol/Packet.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>
#include <optional>
#include <variant>
#include <vector>

#include "devices/uwb_module_util.h"
#include "lcd.h"
#include "m5stack_utils/m5core2.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// LGFX
LGFX lcd;
LGFX_Sprite sprite_header(&lcd);
LGFX_Sprite sprite_status(&lcd);
LGFX_Sprite sprite_lock_image(&lcd);

// Device name
String device_name = "";

// ESP-NOW
uint8_t mac_address[6] = {0};

// Interface
std::string packet_description_operation = "Key control: arg is \"lock\" or \"unlock\"";
std::string serialization_format_operation = "s";
SDPInterfaceDescription interface_description_operation = std::make_tuple(packet_description_operation, serialization_format_operation);

// Key Status
std::string packet_description_key_status = "Key status: true if locked, false if unlocked";
std::string serialization_format_key_status = "b";
std::vector<SDPData> data_for_key_status_data_packet;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> data_for_uwb_data_packet;

// Status
bool lock_status = true;

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;

bool load_config_from_FS(fs::FS &fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }
  if (not doc.containsKey("device_name") and
      not doc.containsKey("uwb_id")) {
    return false;
  }
  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  return true;
}

void callback_lock_operation(const uint8_t *mac_address, const std::vector<SDPData> &body) {
  std::string operation_key = std::get<std::string>(body[0]);
  Serial.printf("operation_key: %s\n", operation_key.c_str());
  Serial.printf("operation_key length: %d\n", operation_key.length());
  if (operation_key == "lock") {
    Serial.printf("Lock the key\n");
    lock_status = true;
  } else if (operation_key == "unlock") {
    Serial.printf("Unlock the key\n");
    lock_status = false;
  } else {
    Serial.printf("Unknown operation key\n");
  }
  Serial.printf("Key control command done\n");
}

void setup() {
  M5.begin(true, true, true, false);
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PORT_C_SERIAL_RX, PORT_C_SERIAL_TX);

  init_lcd();

  sprite_header.printf("SDP SESAMI HOST DEVICE\n");
  sprite_header.pushSprite(0, 0);

  // Load config from FS
  if (not load_config_from_FS(SD, "/config.json")) {
    Serial.println("Failed to load config file");
    while (true) {
      delay(1000);
    }
  }

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name)) {
    Serial.println("Failed to initialize SDP");
    while (true) {
      delay(1000);
    }
  }
  register_sdp_interface_callback(interface_description_operation, callback_lock_operation);
  Serial.println("SDP Initialized!");

  // UWB module
  bool result = false;
  if (uwb_id >= 0) {
    result = initUWB(false, uwb_id, Serial1);
    if (not result) {
      uwb_id = -1;
    } else {
      data_for_uwb_data_packet.clear();
      data_for_uwb_data_packet.push_back(SDPData(uwb_id));
    }
  } else {
    result = resetUWB(Serial1);
  }

  String header_message = "Name: " + device_name + "\n";
  header_message += "ADDR: " + String(mac_address[0], HEX) + ":" + String(mac_address[1], HEX) + ":" + String(mac_address[2], HEX) + ":" + String(mac_address[3], HEX) + ":" + String(mac_address[4], HEX) + ":" + String(mac_address[5], HEX) + "\n";
  header_message += "UWB ID: " + String(uwb_id) + "\n";
  print_header(header_message);
}

void loop() {
  delay(500);

  if (lock_status) {
    show_lock_image();
  } else {
    show_unlock_image();
  }

  // Send SDP data packet
  data_for_key_status_data_packet.clear();
  data_for_key_status_data_packet.push_back(SDPData(lock_status));
  if (not send_sdp_data_packet(packet_description_key_status, data_for_key_status_data_packet)) {
    Serial.println("Failed to send SDP data packet");
  }
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, data_for_uwb_data_packet)) {
      Serial.println("Failed to send SDP data packet");
    }
  }
}
