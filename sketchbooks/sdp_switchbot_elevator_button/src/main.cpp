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
std::string packet_description_control = "Elevator call";
std::string serialization_format_control = "s";
SDPInterfaceDescription interface_description_control =
    std::make_tuple(packet_description_control, serialization_format_control);

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

// Port configuration
M5StackSerialPortInfo port_info_m5atoms3 = M5StackSerialPortInfoList[PORT_A];
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

  M5.Lcd.printf("SDP SWITCHBOT ELEVATOR PANEL\n");

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

  Serial1.begin(115200, SERIAL_8N1, port_info_uwb.rx, port_info_uwb.tx);
  Serial2.begin(115200, SERIAL_8N1, port_info_m5atoms3.rx, port_info_m5atoms3.tx);

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name.c_str())) {
    Serial.println("Failed to initialize SDP");
    M5.Lcd.printf("Failed to initialize SDP\n");
    while (true) {
      delay(1000);
    }
  }
  if (not register_sdp_interface_callback(interface_description_control, callback_for_elevator_panel_control)) {
    Serial.println("Failed to register callback for elevator panel control");
    M5.Lcd.printf("Failed to register callback for elevator panel control\n");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SDP Initialized!");

  // Show device info
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3],
                mac_address[4], mac_address[5]);
  M5.Lcd.printf("SSID: %s\n", wifi_ssid.c_str());
  M5.Lcd.printf("PASS: %s\n", wifi_password.c_str());

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
}

void loop() {
  delay(1000);

  Serial.println("Loop");

  // Send SDP Data
  if (uwb_id >= 0) {
    if (not send_sdp_data_packet(packet_description_uwb, body_uwb)) {
      Serial.printf("Failed to send SDP data packet\n");
      Serial.printf("packet description is %s\n", packet_description_uwb.c_str());
    }
  }
}
