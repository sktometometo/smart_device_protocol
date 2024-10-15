#include <Arduino.h>
#include <M5StickCPlus.h>

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

#include "EspEasyServo.h"
#include "devices/uwb_module_util.h"
#include "esp32-hal-ledc.h"
#include "m5stack_utils/m5stickcplus.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

const int COUNT_LOW = 1500;
const int COUNT_HIGH = 8500;
const int TIMER_WIDTH = 16;
const int LEDC_CHANNEL = 1;
const int LEDC_FREQ = 50;
const int LEDC_PIN = 26;

const int SERVO_MAX_ANGLE = 180;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAXIMUM_STEP = 5;

M5StackSerialPortInfo port_info_uwb = M5StackSerialPortInfoList[PORT_A];

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

//
int current_target = 0;
int current_angle = 0;
bool press = false;
unsigned long last_pressed_sec = 0;

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;

bool load_config_from_FS(fs::FS& fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }

  if (not doc.containsKey("device_name") or not doc.containsKey("uwb_id")) {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  return true;
}

void move_servo(int angle) {
  int contained_angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  uint32_t count = (uint32_t)(COUNT_LOW + (COUNT_HIGH - COUNT_LOW) * (contained_angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE));
  ledcWrite(LEDC_CHANNEL, count);
}

void init_servo() {
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, TIMER_WIDTH);
  ledcAttachPin(LEDC_PIN, LEDC_CHANNEL);
}

void callback_for_servo_control(const uint8_t* mac_address, const std::vector<SDPData>& body) {
  int32_t target_floor = std::get<int32_t>(body[0]);
  if (target_floor == 2) {
    press = true;
    last_pressed_sec = millis() / 1000;
  }
}

void setup() {
  // put your setup code here, to run once:
  M5.begin(true, true, true);
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, port_info_uwb.rx, port_info_uwb.tx);

  SPIFFS.begin();
  if (not load_config_from_FS(SPIFFS, "/config.json")) {
    Serial.println("Failed to load config file");
    M5.lcd.printf("Failed to load config file\n");
    while (true) {
      delay(1000);
    }
  }

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name.c_str())) {
    Serial.println("Failed to initialize SDP");
    M5.Lcd.printf("Failed to initialize SDP\n");
    while (true) {
      delay(1000);
    }
  }
  register_sdp_interface_callback(interface_description_control, callback_for_servo_control);
  Serial.println("SDP Initialized!");

  // UWB module
  if (uwb_id >= 0) {
    bool result = initUWB(false, uwb_id, Serial1);
    body_uwb.clear();
    body_uwb.push_back(SDPData(uwb_id));
    if (result) {
      M5.Lcd.printf("UWB ID: %d\n", uwb_id);
    } else {
      uwb_id = -1;
      M5.Lcd.printf("UWB ID: Failed to initialize\n");
    }
  } else {
    bool result = resetUWB(Serial1);
    M5.Lcd.printf("UWB ID: Not initialized\n");
  }

  // Init Servo
  init_servo();
  move_servo(current_target);
  M5.Lcd.printf("Servo initialized\n");
}

void loop() {
  delay(100);
  if (press) {
    current_target = 180;
    if (millis() / 1000 - last_pressed_sec > 5) {
      press = false;
      current_target = 0;
    }
  }

  // current angle update
  if (current_target < SERVO_MIN_ANGLE) {
    current_target = SERVO_MIN_ANGLE;
  } else if (current_target > SERVO_MAX_ANGLE) {
    current_target = SERVO_MAX_ANGLE;
  }
  if (current_angle < current_target) {
    if (current_target - current_angle > SERVO_MAXIMUM_STEP) {
      current_angle = current_angle + SERVO_MAXIMUM_STEP;
    } else {
      current_angle = current_target;
    }
  } else if (current_angle > current_target) {
    if (current_angle - current_target > SERVO_MAXIMUM_STEP) {
      current_angle = current_angle - SERVO_MAXIMUM_STEP;
    } else {
      current_angle = current_target;
    }
  }
  move_servo(current_angle);
  Serial.printf("Current target: %d, angle: %d\n", current_target, current_angle);
  // Send SDP Data
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_uwb.c_str());
    }
  }
}