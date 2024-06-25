#include <ArduinoJson.h>
#include <FS.h>
#if defined(M5STACK_FIRE)
#include <M5Stack.h>
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>
#endif
#include <SPIFFS.h>
#include <smart_device_protocol/Packet.h>

#include <optional>
#include <variant>
#include <vector>

#include "devices/uwb_module_util.h"
#include "iot_com_util/iot_host_util.h"
#include "sdp/sdp.h"
#include "utils/config_loader.h"

// Device name
String device_name = "";

// ESP-NOW
uint8_t mac_address[6] = {0};

// Interface
std::string packet_description_operation = "Key control";
std::string serialization_format_operation = "s";
SDPInterfaceDescription interface_description_operation = std::make_tuple(packet_description_operation, serialization_format_operation);

// Key Status
std::string packet_description_key_status = "Key status";
std::vector<SDPData> data_for_key_status_data_packet;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> data_for_uwb_data_packet;

// Switchbot Client Configuration
String wifi_ssid = "";
String wifi_password = "";
String sesami_device_uuid = "";
String sesami_secret_key = "";
String sesami_api_key = "";

// Automatic status update duration
float automatic_status_update_duration = 3600.0;  // sec

// Other
std::vector<SDPData> data;
StaticJsonDocument<1024> result_json;
int loop_counter = 0;

std::optional<bool> get_key_status_and_update_buf() {
  Serial.printf("Get key status\n");

  String result = send_serial_command("{\"command\":\"status\"}\n");
  if (result.length() == 0) {
    Serial.println("Failed to get key status");
    return std::nullopt;
  }
  DeserializationError error = deserializeJson(result_json, result);
  if (error or (result_json.containsKey("success") and not result_json["success"].as<bool>())) {
    Serial.printf("deserializeJson() failed or Failed to get key status: %s\n", error.c_str());
    return std::nullopt;
  } else {
    String status = result_json["result"]["CHSesame2Status"].as<String>();
    Serial.printf("Key result: %s\n", result_json["result"].as<String>().c_str());
    Serial.printf("Key status: %s\n", status.c_str());
    bool locked = true ? status == "locked" : false;
    data_for_key_status_data_packet.clear();
    data_for_key_status_data_packet.push_back(SDPData(locked));
    return locked;
  }
}

void show_device_config() {
  String result = send_serial_command("{\"command\":\"get_device_config\"}\n");
  Serial.printf("Device config: %s\n", result.c_str());
}

bool load_config_from_FS(fs::FS &fs, String filename = "/config.json") {
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc)) {
    return false;
  }
  if (not doc.containsKey("device_name") and
      not doc.containsKey("wifi_ssid") and
      not doc.containsKey("wifi_password") and
      not doc.containsKey("sesami_device_uuid") and
      not doc.containsKey("sesami_secret_key") and
      not doc.containsKey("sesami_api_key") and
      not doc.containsKey("uwb_id")) {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  wifi_ssid = doc["wifi_ssid"].as<String>();
  wifi_password = doc["wifi_password"].as<String>();
  sesami_device_uuid = doc["sesami_device_uuid"].as<String>();
  sesami_secret_key = doc["sesami_secret_key"].as<String>();
  sesami_api_key = doc["sesami_api_key"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();

  if (doc.containsKey("automatic_status_update_duration")) {
    automatic_status_update_duration = doc["automatic_status_update_duration"].as<float>();
  }

  return true;
}

void callback_sesami_operation(const uint8_t *mac_address, const std::vector<SDPData> &body) {
  std::string operation_key = std::get<std::string>(body[0]);
  Serial.printf("operation_key: %s\n", operation_key.c_str());
  Serial.printf("operation_key length: %d\n", operation_key.length());
  if (operation_key == "lock") {
    Serial.printf("Lock the key\n");
    String ret = send_serial_command("{\"command\":\"lock\"}\n");
    Serial.printf("Response: %s\n", ret.c_str());
    auto deadline = millis() + 10000;
    while (millis() < deadline) {
      delay(500);
      std::optional<bool> locked = get_key_status_and_update_buf();
      if (locked.has_value() and locked.value()) {
        break;
      }
    }
  } else if (operation_key == "unlock") {
    Serial.printf("Unlock the key\n");
    String ret = send_serial_command("{\"command\":\"unlock\"}\n");
    Serial.printf("Response: %s\n", ret.c_str());
    auto deadline = millis() + 10000;
    while (millis() < deadline) {
      delay(500);
      std::optional<bool> locked = get_key_status_and_update_buf();
      if (locked.has_value() and not locked.value()) {
        break;
      }
    }
  } else {
    Serial.printf("Unknown operation key\n");
  }
  Serial.printf("Key control command done\n");
}

void setup() {
  esp_read_mac(mac_address, ESP_MAC_WIFI_STA);

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

  M5.Lcd.printf("SDP SESAMI HOST DEVICE\n");

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json")) {
    if (not load_config_from_FS(SPIFFS, "/config.json")) {
      Serial.println("Failed to load config file");
      M5.Lcd.printf("Failed to load config file\n");
      while (true) {
        delay(1000);
      }
    }
  }

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name)) {
    Serial.println("Failed to initialize SDP");
    M5.lcd.printf("Failed to initialize SDP\n");
    while (true) {
      delay(1000);
    }
  }
  register_sdp_interface_callback(interface_description_operation, callback_sesami_operation);
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

  // Print info
  M5.Lcd.printf("====================================\n");
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  M5.Lcd.printf("WiFi SSID: %s\n", wifi_ssid.c_str());
  M5.Lcd.printf("WiFi Password: %s\n", wifi_password.c_str());
  M5.Lcd.printf("SESAMI Device UUID: %s\n", sesami_device_uuid.c_str());
  M5.Lcd.printf("SESAMI Secret Key: %s\n", sesami_secret_key.c_str());
  M5.Lcd.printf("SESAMI API Key: %s\n", sesami_api_key.c_str());
  M5.Lcd.printf("UWB ID: %d\n", uwb_id);
  M5.Lcd.printf("Automatic status update duration: %f\n", automatic_status_update_duration);
  M5.Lcd.printf("====================================\n");

  // WiFi Configuration
  String ret = send_serial_command(
      String("") +
          "{\"command\":\"config_wifi\"," +
          "\"ssid\":\"" + wifi_ssid + "\"," +
          "\"password\":\"" + wifi_password + "\"}\n",
      20000);
  Serial.printf("Response for wifi config: %s\n", ret.c_str());

  // Sesami Client Configuration
  ret = send_serial_command(
      String("") +
          "{\"command\":\"config_sesami\"," +
          "\"device_uuid\":\"" + sesami_device_uuid + "\"," +
          "\"secret_key\":\"" + sesami_secret_key + "\"," +
          "\"api_key\":\"" + sesami_api_key + "\"}\n",
      5000);
  Serial.printf("Response for sesami config: %s\n", ret.c_str());

  // Show device config
  show_device_config();
}

void loop() {
  delay(5000);

  // Run dummy callback if Serial available
  if (Serial.available()) {
    uint8_t buf_dummy[240];
    std::vector<SDPData> data_dummy;
    data_dummy.clear();
    String str = Serial.readStringUntil('\n');
    Serial.printf("Input: %s\n", str.c_str());
    if (str.indexOf("unlock") != -1) {
      data_dummy.push_back(SDPData(std::string("unlock")));
    } else if (str.indexOf("lock") != -1) {
      data_dummy.push_back(SDPData(std::string("lock")));
    } else {
      Serial.println("Unknown command");
      return;
    }
    Serial.println("Generate data frame");
    std::string serialization_format = get_serialization_format(data_dummy);
    Serial.printf("serialization_format: %s\n", serialization_format.c_str());
    bool ret = generate_data_frame(
        buf_dummy,
        packet_description_operation.c_str(),
        data_dummy);
    if (not ret) {
      Serial.println("Failed to generate data frame");
      return;
    }
    Serial.printf("Dummy callback calling\n");
    _OnDataRecv(NULL, buf_dummy, sizeof(buf_dummy));
    Serial.printf("Dummy callback done\n");
    return;
  }

  // Send SDP data packet
  if (not send_sdp_data_packet(packet_description_key_status, data_for_key_status_data_packet)) {
    Serial.println("Failed to send SDP data packet");
  }
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, data_for_uwb_data_packet)) {
      Serial.println("Failed to send SDP data packet");
    }
  }

  if (loop_counter % (int)(automatic_status_update_duration / 5.0) == 0) {
    get_key_status_and_update_buf();
  }

  loop_counter++;
}
