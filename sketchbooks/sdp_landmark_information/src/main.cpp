#include <vector>
#include <variant>

#include <M5Stack.h>
#include <esp_system.h>
#include <esp_now.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <esp_now_ros/Packet.h>
#include <packet_creator.h>
#include <packet_parser.h>
#include "uwb_module_util.h"
#include "iot_host_util.h"
#include "sdp/sdp_util.h"

// Device name
String device_name = "";

// ESP-NOW
uint8_t mac_address[6] = {0};

// SDP Interface
std::string packet_description_information = "Landmark information";
std::string serialization_format_information = "S";
std::vector<SDPData> data_for_information_data_packet;

// UWB
int uwb_id = -1;
std::string packet_description_uwb = "UWB Station";
std::string serialization_format_uwb = "i";
std::vector<SDPData> data_for_uwb_data_packet;

// Other
std::vector<SDPData> data;
int loop_counter = 0;

bool load_config_from_FS(fs::FS &fs, String filename = "/config.json")
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc))
  {
    return false;
  }
  if (not doc.containsKey("device_name") or not doc.containsKey("uwb_id") or not doc.containsKey("information"))
  {
    return false;
  }

  device_name = doc["device_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  String information_str = doc["information"].as<String>();
  data_for_information_data_packet.push_back(SDPData(std::string(information_str.c_str())));

  return true;
}

void setup()
{
  M5.begin(true, false, true, false);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 22, 21);

  // Load config from FS
  SPIFFS.begin();
  SD.begin();
  if (not load_config_from_FS(SD, "/config.json"))
  {
    if (not load_config_from_FS(SPIFFS, "/config.json"))
    {
      Serial.println("Failed to load config file");
      ESP.restart();
    }
  }

  // Initialization of SDP
  if (not init_sdp(mac_address, device_name))
  {
    Serial.println("Failed to initialize SDP");
    ESP.restart();
  }
  {
    Serial.println("Failed to initialize ESP-NOW");
    ESP.restart();
  }
  Serial.println("ESP NOW Initialized!");

  // UWB module
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  bool result = initUWB(false, uwb_id, Serial1);
  data_for_uwb_data_packet.push_back(SDPData(uwb_id));

  // LCD Print
  M5.Lcd.printf("SDP SWITCHBOT LIGHT HOST\n");
  M5.Lcd.printf("Name: %s\n", device_name.c_str());
  M5.Lcd.printf("ADDR: %2x:%2x:%2x:%2x:%2x:%2x\n",
                mac_address[0], mac_address[1], mac_address[2],
                mac_address[3], mac_address[4], mac_address[5]);
}

void loop()
{
  delay(5000);

  // Send SDP data packet
  if (not send_sdp_data_packet(packet_description_information, data_for_information_data_packet))
  {
    Serial.println("Failed to send SDP data packet");
  }
  if (not send_sdp_data_packet(packet_description_uwb, data_for_uwb_data_packet))
  {
    Serial.println("Failed to send SDP data packet");
  }
}
