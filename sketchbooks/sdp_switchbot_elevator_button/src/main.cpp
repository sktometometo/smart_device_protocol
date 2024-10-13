#include <variant>
#include <vector>

#if defined(M5STACK_FIRE)
#include <M5Stack.h>

#include "m5stack_utils/m5stack.h"
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>

#include "m5stack_utils/m5core2.h"
#endif
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <smart_device_protocol/Packet.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "devices/m5stack_tof_unit.h"
#include "devices/uwb_module_util.h"
#include "iot_com_util/iot_host_util.h"
#include "lcd.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// Display
LGFX lcd;
LGFX_Sprite sprite_header(&lcd);
LGFX_Sprite sprite_status(&lcd);
LGFX_Sprite sprite_info(&lcd);
LGFX_Sprite sprite_plot(&lcd);

// Device Name
String device_name;

// ESP-NOW
uint8_t mac_address[6] = {0};

// Interface
// Elevator Panel Control
std::string packet_description_control = "Elevator call";
std::string serialization_format_control = "s";
SDPInterfaceDescription interface_description_control =
    std::make_tuple(packet_description_control, serialization_format_control);
// Door Open
std::string packet_description_door_open = "Door opening state";
std::string serialization_format_door_open = "?";
SDPInterfaceDescription interface_description_door_open =
    std::make_tuple(packet_description_door_open, serialization_format_door_open);
std::vector<SDPData> body_door_open;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> body_uwb;

// Switchbot Client Configuration
String wifi_ssid = "";
String wifi_password = "";
String switchbot_token = "";
String switchbot_secret = "";
String switchbot_upward_device_id = "";
String switchbot_downward_device_id = "";

// Elevator Door Detection Parameters
uint16_t door_closed_distance = 300;
uint16_t door_closed_clearance = 100;

// Port configuration
// Port A is used for Wire
M5StackSerialPortInfo port_info_m5atoms3 = M5StackSerialPortInfoList[PORT_B];
M5StackSerialPortInfo port_info_uwb = M5StackSerialPortInfoList[PORT_C];

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;

bool load_config_from_FS(fs::FS& fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or
      not doc.containsKey("wifi_ssid") or
      not doc.containsKey("wifi_password") or
      not doc.containsKey("switchbot_token") or
      not doc.containsKey("switchbot_secret") or
      not doc.containsKey("switchbot_upward_device_id") or
      not doc.containsKey("switchbot_downward_device_id") or
      not doc.containsKey("uwb_id")) {
    return false;
  }

  if (doc.containsKey("door_closed_clearance")) {
    door_closed_clearance = doc["door_closed_clearance"].as<int>();
  }

  device_name = doc["device_name"].as<String>();
  wifi_ssid = doc["wifi_ssid"].as<String>();
  wifi_password = doc["wifi_password"].as<String>();
  switchbot_token = doc["switchbot_token"].as<String>();
  switchbot_secret = doc["switchbot_secret"].as<String>();
  switchbot_upward_device_id = doc["switchbot_upward_device_id"].as<String>();
  switchbot_downward_device_id = doc["switchbot_downward_device_id"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  return true;
}

void callback_for_elevator_panel_control(const uint8_t* mac_address, const std::vector<SDPData>& body) {
  Serial.printf("Length of body: %d\n", body.size());
  std::string target = std::get<std::string>(body[0]);
  if (target == "up") {
    Serial.printf("Press upward button\n");
    String ret = send_serial_command(String("") + "{\"command\":\"send_device_command\"," + "\"device_id\":\"" +
                                         switchbot_upward_device_id + "\"," + "\"sb_command_type\":\"command\"," +
                                         "\"sb_command\":\"press\"}\n",
                                     10000);
    Serial.printf("Response: %s\n", ret.c_str());
  } else if (target == "down") {
    Serial.printf("Press downward button\n");
    String ret = send_serial_command(String("") + "{\"command\":\"send_device_command\"," + "\"device_id\":\"" +
                                         switchbot_downward_device_id + "\"," + "\"sb_command_type\":\"command\"," +
                                         "\"sb_command\":\"press\"}\n",
                                     10000);
    Serial.printf("Response: %s\n", ret.c_str());
  } else {
    Serial.printf("Unknown command\n");
  }
}

void setup() {
  M5.begin(true, true, true, false);
  Serial.begin(115200);
  init_lcd();

  sprite_header.printf("SDP SWITCHBOT ELEVATOR PANEL\n");
  sprite_header.pushSprite(0, 0);

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json")) {
    if (not load_config_from_FS(SPIFFS, "/config.json")) {
      Serial.println("Failed to load config file");
      sprite_status.printf("Failed to load config file\n");
      sprite_status.pushSprite(0, lcd.height() / 3);
      while (true) {
        delay(1000);
      }
    }
  }
  Serial.println("Config loaded!");

  Serial1.begin(115200, SERIAL_8N1, port_info_uwb.rx, port_info_uwb.tx);
  Serial2.begin(115200, SERIAL_8N1, port_info_m5atoms3.rx, port_info_m5atoms3.tx);

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name.c_str())) {
    Serial.println("Failed to initialize SDP");
    sprite_status.printf("Failed to initialize SDP\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
    while (true) {
      delay(1000);
    }
  }
  if (not register_sdp_interface_callback(interface_description_control, callback_for_elevator_panel_control)) {
    Serial.println("Failed to register callback for elevator panel control");
    sprite_status.printf("Failed to register callback for elevator panel control\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SDP Initialized!");

  // Show device info
  sprite_header.printf("Name: %s\n", device_name.c_str());
  sprite_header.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                       mac_address[4], mac_address[5]);
  sprite_header.pushSprite(0, 0);

  // UWB module
  if (uwb_id >= 0) {
    bool result = initUWB(false, uwb_id, Serial1);
    body_uwb.clear();
    body_uwb.push_back(SDPData(uwb_id));
    if (result) {
      sprite_header.printf("UWB ID: %d\n", uwb_id);
    } else {
      uwb_id = -1;
      sprite_header.printf("UWB ID: Failed to initialize\n");
    }
  } else {
    bool result = resetUWB(Serial1);
    sprite_header.printf("UWB ID: Not initialized\n");
  }
  sprite_header.pushSprite(0, 0);

  // Wifi Configuration
  Serial.printf("Wifi Configuration\n");
  String ret = send_serial_command(String("") + "{\"command\":\"config_wifi\"," + "\"ssid\":\"" + wifi_ssid + "\"," +
                                       "\"password\":\"" + wifi_password + "\"}\n",
                                   20000);
  Serial.printf("Response for wifi config: %s\n", ret.c_str());

  delay(3000);

  // Switchbot Client Configuration
  Serial.printf("Switchbot Client Configuration\n");
  ret = send_serial_command(String("") + "{\"command\":\"config_switchbot\"," + "\"token\":\"" + switchbot_token +
                                "\"," + "\"secret\":\"" + switchbot_secret + "\"}\n",
                            5000);
  Serial.printf("Response for switchbot config: %s\n", ret.c_str());

  delay(3000);

  // Get device status
  ret = send_serial_command(String("") + "{\"command\":\"get_device_config\"}\n", 5000);
  Serial.printf("Response for get_device_status: %s\n", ret.c_str());

  // ToF ranging unit
  if (not init_m5stack_tof_unit()) {
    Serial.println("Failed to initialize ToF unit");
    sprite_status.printf("Failed to initialize ToF unit\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
    while (true) {
      delay(1000);
    }
  }
  Serial.println("ToF unit initialized!");

  // Door detection parameter initialization
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("Initialization of Door Detection Parameters will start in 3 seconds\n");
  sprite_status.pushSprite(0, lcd.height() / 3);
  delay(3000);
  auto result = get_m5stack_tof_unit_data();
  if (not result.has_value()) {
    Serial.println("Failed to get ToF data");
    sprite_status.fillScreen(0xFFFFFF);
    sprite_status.setCursor(0, 0);
    sprite_status.printf("Failed to get ToF data\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
    while (true) {
      delay(1000);
    }
  } else {
    auto [acnt, scnt, dist, status] = result.value();
    door_closed_distance = dist;
    sprite_status.fillScreen(0xFFFFFF);
    sprite_status.setCursor(0, 0);
    sprite_status.printf("Door closed distance: %d\n", door_closed_distance);
    sprite_status.pushSprite(0, lcd.height() / 3);
  }
  delay(3000);
}

void loop() {
  delay(100);

  Serial.println("Loop");

  auto result = get_m5stack_tof_unit_data();
  if (not result.has_value()) {
    Serial.println("Failed to get ToF data");
    sprite_status.fillScreen(0xFFFFFF);
    sprite_status.setCursor(0, 0);
    sprite_status.printf("Failed to get ToF data\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
  } else {
    auto [acnt, scnt, dist, status] = result.value();
    Serial.printf("acnt: %d, scnt: %d, dist: %d, status: %d\n", acnt, scnt, dist, status);
    sprite_status.fillScreen(0xFFFFFF);
    sprite_status.setCursor(0, 0);
    sprite_status.printf("ambient count: %d\n", acnt);
    sprite_status.printf("signal count: %d\n", scnt);
    sprite_status.printf("distance: %d\n", dist);
    sprite_status.printf("status: %d\n", status);
    if (dist > door_closed_distance + door_closed_clearance) {
      Serial.println("Door is open");
      sprite_status.printf("Door is open\n");
    } else if (status == 11) {
      Serial.println("Door is closed");
      sprite_status.printf("Door is closed\n");
    } else {
      Serial.println("Door status is unknown");
      sprite_status.printf("Door status is unknown\n");
    }
    body_door_open.clear();
    body_door_open.push_back(SDPData(dist > door_closed_distance + door_closed_clearance));
    if (not send_sdp_data_packet(packet_description_door_open, body_door_open)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_door_open.c_str());
      sprite_status.printf("Failed to send SDP data packet\n");
    }
    sprite_status.pushSprite(0, lcd.height() / 3);
  }
  // Send SDP Data
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_uwb.c_str());
    }
  }
}
