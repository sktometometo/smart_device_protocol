#include <ArduinoJson.h>
#include <M5Core2.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "devices/stickv2_util.h"
#include "devices/uwb_module_util.h"
#include "lcd.h"
#include "m5stack_utils/m5core2.h"
#include "sdp/sdp.h"
#include "sdp/sdp_rpc.h"
#include "utils/config_loader.h"

// LovyanGFX
static LGFX lcd;
static LGFX_Sprite sprite_title(&lcd);
static LGFX_Sprite sprite_status(&lcd);
static LGFX_Sprite sprite_info(&lcd);

/* SDP Interface */
std::string packet_description_mode_change = "Target mode";
std::string serialization_format_mode_change = "s";

std::string packet_description_mode_change_response = "Mode change response";
std::string serialization_format_mode_change_response = "?s";

std::string packet_description_object_detection = "Number of object";
std::string serialization_format_object_detection = "si";

std::string packet_description_face_detection = "Detected face";
std::string serialization_format_face_detection = "s";

/* device information */
uint8_t mac_address[6];
String device_name;
bool auto_start = true;

// String target_mode = "object_detection";
String target_mode = "face_recognition";

// Object Detection threshold
float threshold = 0.5;
String target_class = "person";

/* UWB */
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> data_for_uwb_data_packet;

bool set_mode();

bool load_config_from_FS(fs::FS &fs, const String &filename) {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }
  if (not doc.containsKey("device_name") or
      not doc.containsKey("uwb_id") or
      not doc.containsKey("threshold") or
      not doc.containsKey("target_class")) {
    return false;
  }
  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  threshold = doc["threshold"].as<float>();
  target_class = doc["target_class"].as<String>();

  if (doc.containsKey("auto_start")) {
    auto_start = doc["auto_start"].as<bool>();
  }
  if (doc.containsKey("target_mode")) {
    target_mode = doc["target_mode"].as<String>();
  }
  return true;
}

void mode_change_cb(const uint8_t *addr, const std::vector<SDPData> &data) {
  target_mode = String(std::get<std::string>(data[0]).c_str());
  clear_sprite(sprite_info);
  sprite_info.printf("Change mode to %s", target_mode.c_str());
  set_mode();
}

void mode_change_handler(const uint8_t *mac_addr, const std::vector<SDPData> &request_body, std::vector<SDPData> &response_body) {
  target_mode = String(std::get<std::string>(request_body[0]).c_str());
  clear_sprite(sprite_info);
  sprite_info.printf("Change mode to %s", target_mode.c_str());
  set_mode();
  response_body.clear();
  response_body.push_back(SDPData(true));
  response_body.push_back(SDPData("Success"));
}

void setup() {
  // Initialize
  M5.begin(true, true, true, false);
  Serial.begin(115200, SERIAL_8N1);

  // Display Initialization
  init_lcd(lcd, sprite_title, sprite_status, sprite_info);

  // Display Title
  sprite_title.println("SDP StickV2 Interface");
  update_lcd(lcd, sprite_title, sprite_status, sprite_info);

  // Load config
  SD.begin();
  SPIFFS.begin();
  if (not load_config_from_FS(SD, String("/config.json"))) {
    if (not load_config_from_FS(SPIFFS, String("/config.json"))) {
      Serial.println("Failed to load config.");
      sprite_status.println("Failed to load config.");
      update_lcd(lcd, sprite_title, sprite_status, sprite_info);
      while (true) {
        delay(1000);
      }
    }
  }
  SPIFFS.end();
  SD.end();

  Serial1.begin(115200, SERIAL_8N1, PORT_A_SERIAL_RX, PORT_A_SERIAL_TX);
  Serial2.begin(115200, SERIAL_8N1, PORT_C_SERIAL_RX, PORT_C_SERIAL_TX);

  // UWB Initialization
  if (uwb_id >= 0) {
    data_for_uwb_data_packet.push_back(SDPData(uwb_id));
    if (not initUWB(false, uwb_id, Serial2)) {
      Serial.println("UWB Initialization Failed.");
      sprite_status.println("UWB Initialization Failed.");
      update_lcd(lcd, sprite_title, sprite_status, sprite_info);
    }
  } else {
    resetUWB(Serial2);
  }

  // SDP Initialization
  if (not init_sdp(mac_address, device_name.c_str()) or not init_sdp_rpc()) {
    Serial.println("SDP Initialization Failed.");
    sprite_status.println("SDP Initialization Failed.");
    update_lcd(lcd, sprite_title, sprite_status, sprite_info);
    while (true) {
      delay(1000);
    }
  }

  register_sdp_interface_callback(std::make_tuple<>(packet_description_mode_change, serialization_format_mode_change),
                                  mode_change_cb);
  register_rpc_service(std::make_tuple<>(packet_description_mode_change, serialization_format_mode_change),
                       std::make_tuple<>(packet_description_mode_change_response, serialization_format_mode_change_response),
                       mode_change_handler);

  // Display Device Info
  sprite_title.printf("Device Name: %s\n", device_name.c_str());
  sprite_title.printf("MfAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac_address[0], mac_address[1], mac_address[2],
                      mac_address[3], mac_address[4], mac_address[5]);
  sprite_title.printf("UWB ID: %d\n", uwb_id);
  sprite_title.printf("Target Class: %s\n", target_class.c_str());
  sprite_title.printf("threshold: %f\n", threshold);
  sprite_title.printf("initial mode: %s\n", target_mode.c_str());
  sprite_status.println("Initialization Completed.");
  update_lcd(lcd, sprite_title, sprite_status, sprite_info);
}

bool set_mode() {
  if (target_mode == "object_detection") {
    set_object_recognition_model(Serial1, String("./uploads/models/nanodet_80class"));
  } else if (target_mode == "face_recognition") {
    set_face_recognition(Serial1);
  } else {
    clear_sprite(sprite_info);
    sprite_info.printf("Invalid mode: %s", target_mode.c_str());
    Serial.printf("Invalid mode: %s", target_mode.c_str());
    return false;
  }
  clear_sprite(sprite_info);
  sprite_info.printf("Set %s mode.", target_mode.c_str());
  Serial.printf("Set %s mode.", target_mode.c_str());
  return true;
}

void loop() {
  auto last_read_stamp = millis();
  StaticJsonDocument<2048> doc;
  while (true) {
    delay(100);
    Serial.printf("last_read_stamp: %ld\n", last_read_stamp);
    Serial.printf("millis(): %ld\n", millis());
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      if (cmd == "start") {
        auto_start = true;
      } else {
        Serial1.print(cmd);
        Serial.printf("cmd: %s\n", cmd.c_str());
        String response = Serial1.readStringUntil('\n');
        Serial.printf("Response: %s\n", response.c_str());
      }
    }
    if (not Serial1.available() and (millis() - last_read_stamp > 10000) and auto_start) {
      Serial.println("Timeout for autostart.");
      last_read_stamp = millis();
      if (not set_mode()) {
        continue;
      }
    } else if (Serial1.available()) {
      doc.clear();
      bool success = read_data_from_serial(Serial1, doc);
      std::vector<String> names;
      String doc_str;
      serializeJson(doc, doc_str);
      clear_sprite(sprite_info);
      Serial.printf("Read doc data: %s\n", doc_str.c_str());
      sprite_info.printf("Read doc data: %s\n", doc_str.c_str());
      if (success and target_mode == "object_detection") {
        if (parse_object_recognition_response(doc, target_class, threshold) >= 0) {
          int num_of_target = parse_object_recognition_response(doc, target_class, threshold);
          std::vector<SDPData> data;
          data.push_back(SDPData(std::string(target_class.c_str())));
          data.push_back(SDPData(num_of_target));
          send_sdp_data_packet(packet_description_object_detection, data);
          clear_sprite(sprite_status);
          sprite_status.printf("Send SDP packet: %d\n", num_of_target);
          Serial.printf("Send SDP packet: %d\n", num_of_target);
        }
      } else if (success and target_mode == "face_recognition") {
        if (parse_face_recognition_response(doc, 0.5, 0.5, names)) {
          std::vector<SDPData> data;
          for (auto &name : names) {
            data.clear();
            data.push_back(SDPData(std::string(name.c_str())));
            send_sdp_data_packet(packet_description_face_detection, data);
          }
          clear_sprite(sprite_status);
          sprite_status.println("Send SDP packet: 1");
          Serial.println("Send SDP packet: 1");
        } else {
          clear_sprite(sprite_status);
          sprite_status.println("Failed to read data.");
        }
      }
      last_read_stamp = millis();
    }
    update_lcd(lcd, sprite_title, sprite_status, sprite_info);

    if (uwb_id >= 0) {
      if (not send_sdp_data_packet(packet_description_uwb, data_for_uwb_data_packet)) {
        Serial.println("Failed to send UWB SDP data packet");
      } else {
        Serial.println("Success to send UWB SDP data packet");
      }
    }
  }
}
