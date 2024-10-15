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

#include "devices/uwb_module_util.h"
#include "iot_com_util/iot_host_util.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// Device Name
String device_name;

// ESP-NOW
uint8_t mac_address[6] = {0};

// Interface
std::string packet_description_control = "Lock control. True: Lock, False: Unlock";
std::string serialization_format_control = "?";
SDPInterfaceDescription interface_description_control =
    std::make_tuple(packet_description_control, serialization_format_control);

// Lock Status
std::string packet_description_status = "Lock status: True: Locked, False: Unlocked";
std::string serialization_format_status = "?";
std::vector<SDPData> body_status;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> body_uwb;

// Switchbot Client Configuration
String wifi_ssid = "";
String wifi_password = "";
String switchbot_device_id = "";
String switchbot_token = "";
String switchbot_secret = "";

// Status retrieve iteration
int status_retrieve_iteration = 100;

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;
int loop_counter = 0;

void get_bot_status_and_update_buf() {
  Serial.printf("Get switchbot status\n");

  clear_recv_buf(500);
  String result = send_serial_command(
      String("") + "{\"command\":\"get_device_status\"," + "\"device_id\":\"" + switchbot_device_id + "\"}\n", 5000);
  Serial.printf("Response for get_device_status: %s\n", result.c_str());
  DeserializationError error = deserializeJson(result_json, result);
  if (error or (result_json.containsKey("success") and not result_json["success"].as<bool>())) {
    Serial.printf("deserializeJson() failed or get_device_status failed: %s, result: %s\n", error.c_str(),
                  result.c_str());
    return;
  } else {
    String lock_status = result_json["result"]["body"]["lockState"];
    body_status.clear();
    body_status.push_back(SDPData(std::string(lock_status.c_str())));
    return;
  }
}

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
      not doc.containsKey("switchbot_device_id") or
      not doc.containsKey("uwb_id")) {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  wifi_ssid = doc["wifi_ssid"].as<String>();
  wifi_password = doc["wifi_password"].as<String>();
  switchbot_token = doc["switchbot_token"].as<String>();
  switchbot_secret = doc["switchbot_secret"].as<String>();
  switchbot_device_id = doc["switchbot_device_id"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();

  if (not doc.containsKey("status_retrieve_iteration")) {
    status_retrieve_iteration = 100;
  } else {
    status_retrieve_iteration = doc["status_retrieve_iteration"].as<int>();
  }

  return true;
}

void callback_for_switch_control(const uint8_t* mac_address, const std::vector<SDPData>& body) {
  Serial.printf("Length of body: %d\n", body.size());
  bool control = std::get<bool>(body[0]);
  Serial.printf("Lock Control Command: %s\n", control ? "Lock" : "Unlock");
  if (control) {
    Serial.printf("Lock the key.\n");
    clear_recv_buf(500);
    String command = String("") +
                     "{\"command\":\"send_device_command\"," +
                     "\"device_id\":\"" + switchbot_device_id + "\"," +
                     "\"sb_command_type\":\"command\"," +
                     "\"sb_command\":\"lock\"}\n";
    String ret = send_serial_command(command, 10000);
    Serial.printf("Response: %s\n", ret.c_str());
  } else {
    Serial.printf("Unlock the key.\n");
    clear_recv_buf(500);
    String command = String("") +
                     "{\"command\":\"send_device_command\"," +
                     "\"device_id\":\"" + switchbot_device_id + "\"," +
                     "\"sb_command_type\":\"command\"," +
                     "\"sb_command\":\"unlock\"}\n";
    String ret = send_serial_command(command, 10000);
    Serial.printf("Response: %s\n", ret.c_str());
  }
  Serial.printf("Lock Control Command Done\n");
  delay(5000);
  get_bot_status_and_update_buf();
}

void setup() {
  M5.begin(true, true, true, false);
  Serial.begin(115200);
#if defined(M5STACK_FIRE)
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
#elif defined(M5STACK_CORE2)
  Serial1.begin(115200, SERIAL_8N1, 33, 32);
#endif
#if defined(M5STACK_FIRE)
  Serial2.begin(115200, SERIAL_8N1, 22, 21);
#elif defined(M5STACK_CORE2)
  Serial2.begin(115200, SERIAL_8N1, 13, 14);
#endif

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json")) {
    if (not load_config_from_FS(SPIFFS, "/config.json")) {
      Serial.println("Failed to load config file");
      M5.lcd.printf("Failed to load config file\n");
      while (true) {
        delay(1000);
      }
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
  register_sdp_interface_callback(interface_description_control, callback_for_switch_control);
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

  // Show device info
  M5.lcd.println("=== Device Configuration ===");
  M5.Lcd.printf("SDP SWITCHBOT LOCK HOST\n");
  M5.lcd.printf("device_name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  M5.lcd.printf("wifi_ssid: %s\n", wifi_ssid.c_str());
  M5.lcd.printf("switchbot_device_id: %s\n", switchbot_device_id.c_str());
  M5.lcd.printf("uwb_id: %d\n", uwb_id);
  M5.lcd.println("============================");

  // Wifi Configuration
  Serial.printf("Wifi Configuration\n");
  clear_recv_buf(500);
  String ret = send_serial_command(
      String("") +
          "{\"command\":\"config_wifi\"," +
          "\"ssid\":\"" + wifi_ssid + "\"," +
          "\"password\":\"" + wifi_password + "\"}\n",
      30000);
  Serial.printf("Response for wifi config: %s\n", ret.c_str());

  // Switchbot Client Configuration
  Serial.printf("Switchbot Client Configuration\n");
  clear_recv_buf(500);
  ret = send_serial_command(
      String("") +
          "{\"command\":\"config_switchbot\"," +
          "\"token\":\"" + switchbot_token + "\"," +
          "\"secret\":\"" + switchbot_secret + "\"}\n",
      10000);
  Serial.printf("Response for switchbot config: %s\n", ret.c_str());

  // Get device status
  clear_recv_buf(500);
  ret = send_serial_command(String("") + "{\"command\":\"get_device_config\"}\n", 5000);
  Serial.printf("Response for get_device_status: %s\n", ret.c_str());
}

void loop() {
  // Get switchbot status
  if (loop_counter % status_retrieve_iteration == 0) {
    get_bot_status_and_update_buf();
  }

  // Send SDP Data
  if (not send_sdp_data_packet(packet_description_status, body_status)) {
    Serial.printf("Failed to send SDP data packet for status\n");
  }
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet for uwb\n");
    }
  }
  loop_counter++;
}
