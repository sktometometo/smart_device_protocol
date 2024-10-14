#include <Arduino.h>

#if 1
#include <M5Stack.h>

#include "m5stack_utils/m5stack.h"
#endif

#include <variant>
#include <vector>
#define LGFX_USE_V1
#define LGFX_AUTODETECT
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <smart_device_protocol/Packet.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "devices/m5stack_servo_module.h"
#include "devices/uwb_module_util.h"
#include "esp32-hal-ledc.h"
#include "lcd.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

M5StackSerialPortInfo port_info_uwb = M5StackSerialPortInfoList[PORT_C];

// LovyanGFX
LGFX lcd;
LGFX_Sprite sprite_header(&lcd);
LGFX_Sprite sprite_status(&lcd);

// Device Name
String device_name;

// ESP-NOW
uint8_t mac_address[6] = {0};

// Interface
std::string packet_description_control = "Target floor";
std::string serialization_format_control = "i";
SDPInterfaceDescription interface_description_control =
    std::make_tuple(packet_description_control, serialization_format_control);

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> body_uwb;

// Config floor num -> (servo id, default angle, pressed angle)
std::map<int, std::tuple<int, float, float>> floor_to_servo;

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;

bool load_config_from_FS(fs::FS& fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or
      not doc.containsKey("uwb_id") or
      not doc.containsKey("panel_config")) {
    Serial.println("Invalid config file");
    return false;
  }

  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  JsonArray panel_config = doc["panel_config"].as<JsonArray>();
  for (auto panel : panel_config) {
    if (not panel.containsKey("floor") or
        not panel.containsKey("servo_id") or
        not panel.containsKey("default_angle") or
        not panel.containsKey("pressed_angle")) {
      Serial.println("Invalid panel config");
      return false;
    }
    int floor = panel["floor"].as<int>();
    int servo_id = panel["servo_id"].as<int>();
    float default_angle = panel["default_angle"].as<float>();
    float pressed_angle = panel["pressed_angle"].as<float>();
    floor_to_servo[floor] = std::make_tuple(servo_id, default_angle, pressed_angle);
  }
  return true;
}

void callback_for_servo_control(const uint8_t* mac_address, const std::vector<SDPData>& body) {
  int32_t target_floor = std::get<int32_t>(body[0]);
  Serial.printf("Target floor: %d\n", target_floor);
  sprite_status.fillScreen(TFT_WHITE);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("Target floor: %d\n", target_floor);
  sprite_status.pushSprite(0, lcd.height() / 3);
  if (floor_to_servo.find(target_floor) != floor_to_servo.end()) {
    int servo_id = std::get<0>(floor_to_servo[target_floor]);
    float default_angle = std::get<1>(floor_to_servo[target_floor]);
    float pressed_angle = std::get<2>(floor_to_servo[target_floor]);
    sprite_status.printf("Servo ID: %d\n", servo_id);
    sprite_status.printf("Default angle: %f\n", default_angle);
    sprite_status.printf("Pressed angle: %f\n", pressed_angle);
    sprite_status.pushSprite(0, lcd.height() / 3);
    Servo_write_angle(servo_id, pressed_angle);
    sprite_status.println("Pressed");
    sprite_status.pushSprite(0, lcd.height() / 3);
    delay(500);
    Servo_write_angle(servo_id, default_angle);
    sprite_status.println("Done");
    sprite_status.pushSprite(0, lcd.height() / 3);
  } else {
    Serial.printf("Invalid target floor: %d\n", target_floor);
    sprite_status.printf("Invalid target floor: %d\n", target_floor);
    sprite_status.pushSprite(0, lcd.height() / 3);
  }
}

void setup() {
  // put your setup code here, to run once:
  M5.begin(true, true, true, true);
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, port_info_uwb.rx, port_info_uwb.tx);
  init_servo_module();
  init_lcd();

  sprite_header.println("SDP Elevator Internal Panel");
  sprite_header.pushSprite(0, 0);

  // Load config
  SD.begin();
  SPIFFS.begin();
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

  // Display device name
  sprite_header.printf("Device Name: %s\n", device_name.c_str());
  sprite_header.pushSprite(0, 0);

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name.c_str())) {
    Serial.println("Failed to initialize SDP");
    sprite_status.printf("Failed to initialize SDP\n");
    sprite_status.pushSprite(0, lcd.height() / 3);
    while (true) {
      delay(1000);
    }
  }
  register_sdp_interface_callback(interface_description_control, callback_for_servo_control);
  Serial.println("SDP Initialized!");

  sprite_header.printf("ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       mac_address[0], mac_address[1], mac_address[2],
                       mac_address[3], mac_address[4], mac_address[5]);
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

  // Move servo to default position
  for (auto [floor, servo_info] : floor_to_servo) {
    sprite_status.fillScreen(TFT_WHITE);
    sprite_status.setCursor(0, 0);
    int servo_id = std::get<0>(servo_info);
    float default_angle = std::get<1>(servo_info);
    float pressed_angle = std::get<2>(servo_info);
    sprite_status.printf("Testing servo %d\n", servo_id);
    sprite_status.printf("Move to %.2f deg\n", default_angle);
    sprite_status.pushSprite(0, lcd.height() / 3);
    Servo_write_angle(servo_id, default_angle);
    delay(500);
    sprite_status.printf("Move to %.2f deg\n", pressed_angle);
    sprite_status.pushSprite(0, lcd.height() / 3);
    Servo_write_angle(servo_id, pressed_angle);
    delay(500);
  }
  sprite_status.fillScreen(TFT_WHITE);
  sprite_status.setCursor(0, 0);
  sprite_status.printf("All servos are ready\n");
  sprite_status.pushSprite(0, lcd.height() / 3);
}

void loop() {
  delay(100);
  // Send SDP Data
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_uwb.c_str());
    }
  }
}