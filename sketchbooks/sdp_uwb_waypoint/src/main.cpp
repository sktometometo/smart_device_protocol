#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

#include <vector>

#include "M5Stack.h"

#define LGFX_M5STACK
#define LGFX_USE_V1
#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "devices/uwb_module_util.h"
#include "lcd.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// Device name
String device_name;

// ESP-NOW
uint8_t mac_address[6] = {0};

// LovyanGFX
static LGFX lcd;
static LGFX_Sprite sprite_device_header(&lcd);
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_device_status(&lcd);

// SDPInterface Example
std::string waypoint_packet_description = "Waypoint for nav";
std::string waypoint_serialization_format = "sS";
std::vector<SDPData> body_waypoint;

std::string uwb_toggle_packet_description = "Turn On/Off UWB";
std::string uwb_toggle_serialization_format = "?i";
SDPInterfaceDescription uwb_toggle_interface_description = std::make_tuple(
    uwb_toggle_packet_description, uwb_toggle_serialization_format);

// UWB
int32_t uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> body_uwb;

// Waypoint interface
std::string waypoint_name;
std::string waypoint_description;

bool load_config_from_FS(fs::FS &fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or
      not doc.containsKey("waypoint_name") or
      not doc.containsKey("waypoint_description") or
      not doc.containsKey("uwb_id")) {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  waypoint_name = doc["waypoint_name"].as<std::string>();
  waypoint_description = doc["waypoint_description"].as<std::string>();
  uwb_id = doc["uwb_id"].as<int>();
  return true;
}

void callback_uwb_toggle(const uint8_t *mac_addr,
                         const std::vector<SDPData> &body) {
  bool uwb_on = std::get<bool>(body[0]);
  if (uwb_on) {
    uwb_id = std::get<int32_t>(body[1]);
  } else {
    uwb_id = -1;
  }

  if (uwb_on) {
    Serial.println("Turn On UWB");
    initUWB(false, uwb_id, Serial2);
    body_uwb.clear();
    body_uwb.push_back(SDPData(uwb_id));
  } else {
    Serial.println("Turn Off UWB");
    resetUWB(Serial2);
    body_uwb.clear();
  }
}

void setup() {
  M5.begin(false, true, true, false);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize LCD
  init_lcd(lcd, sprite_device_header, sprite_device_info, sprite_device_status);
  sprite_device_status.println("Initializing...");
  update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json")) {
    if (not load_config_from_FS(SPIFFS, "/config.json")) {
      Serial.println("Failed to load config file");
      sprite_device_info.println("Failed to load config file");
      update_lcd(sprite_device_header, sprite_device_info,
                 sprite_device_status);
      while (true) {
        delay(1000);
      }
    }
  }

  // Initialize SDP
  if (not init_sdp(mac_address, device_name, 8192)) {
    Serial.println("Failed to initialize SDP");
    sprite_device_info.println("Failed to initialize SDP");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
    while (true) {
      delay(1000);
    }
  } else {
    Serial.println("Initialized SDP");
    sprite_device_info.println("Initialized SDP");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
  }

  // subscribe SDP
  if (register_sdp_interface_callback(uwb_toggle_interface_description,
                                      callback_uwb_toggle)) {
    Serial.println("Registered UWB toggle callback");
    sprite_device_info.println("Registered UWB toggle callback");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
  } else {
    Serial.println("Failed to register UWB toggle callback");
    sprite_device_info.println("Failed to register UWB toggle callback");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
  }

  // Initialize UWB
  if (uwb_id >= 0) {
    initUWB(false, uwb_id, Serial2);
    body_uwb.clear();
    body_uwb.push_back(SDPData(uwb_id));
  } else {
    resetUWB(Serial2);
    body_uwb.clear();
  }

  // Update Info Screen
  sprite_device_header.printf("Device Name: %s\n", device_name.c_str());
  sprite_device_header.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                              mac_address[0], mac_address[1], mac_address[2],
                              mac_address[3], mac_address[4], mac_address[5]);
  sprite_device_header.printf("Waypoint Name: %s\n", waypoint_name.c_str());
  sprite_device_header.printf("Waypoint Description: %s\n",
                              waypoint_description.c_str());
  update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);

  // Prepare waypoint data
  body_waypoint.clear();
  body_waypoint.push_back(SDPData(std::string(waypoint_name.c_str())));
  body_waypoint.push_back(SDPData(std::string(waypoint_description.c_str())));
  Serial.printf("Waypoint Name: \"%s\"\n", waypoint_name.c_str());
  Serial.printf("Waypoint Description: \"%s\"\n", waypoint_description.c_str());
  sprite_device_info.printf("Waypoint Name: \"%s\"\n", waypoint_name.c_str());
  sprite_device_info.printf("Waypoint Description: \"%s\"\n",
                            waypoint_description.c_str());

  sprite_device_status.println("Initialized");
  sprite_device_info.println("Initialized");
  update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
}

void loop() {
  delay(1000);

  if (uwb_id >= 0) {
    if (send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Sent SDP UWB data packet\n");
      sprite_device_status.printf("Sent SDP UWB data packet\n");
      update_lcd(sprite_device_header, sprite_device_info,
                 sprite_device_status);
    } else {
      Serial.printf("Failed to send SDP UWB data packet\n");
      sprite_device_status.printf("Failed to send SDP UWB data packet\n");
      update_lcd(sprite_device_header, sprite_device_info,
                 sprite_device_status);
    }
  }

  delay(1000);

  if (send_sdp_data_packet(waypoint_packet_description, body_waypoint)) {
    Serial.printf("Sent SDP waypoint interface packet\n");
    sprite_device_status.printf("Sent SDP waypoint interface packet\n");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
  } else {
    Serial.printf("Failed to send SDP waypoint interface packet\n");
    sprite_device_status.printf(
        "Failed to send SDP waypoint interface packet\n");
    update_lcd(sprite_device_header, sprite_device_info, sprite_device_status);
  }

  delay(1000);
}
