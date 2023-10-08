#include <M5Core2.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <ArduinoJson.h>

#include "sdp/sdp.h"
#include "utils/config_loader.h"
#include "devices/stickv2_util.h"
#include "devices/uwb_module_util.h"

static LGFX lcd;
static LGFX_Sprite sprite_header(&lcd);
static LGFX_Sprite sprite_json(&lcd);

/* SDP Interface */
std::string packet_description = "Number of people in some place";
std::string serialization_format = "iS";

/* device information */
uint8_t mac_address[6];
String device_name;
String place_name;

/* UWB */
int uwb_id = -1;

bool load_config_from_FS(fs::FS &fs, const String &filename)
{
  StaticJsonDocument<1024> doc;
  if (not load_json_from_FS<1024>(fs, filename, doc))
  {
    return false;
  }
  if (not doc.containsKey("device_name") or not doc.containsKey("place_name") or not doc.containsKey("uwb_id"))
  {
    return false;
  }
  device_name = doc["device_name"].as<String>();
  place_name = doc["place_name"].as<String>();
  uwb_id = doc["uwb_id"].as<int>();
  return true;
}

void setup()
{
  // Initialize
  M5.begin(true, false, true, false);
  Serial1.begin(115200, SERIAL_8N1, 32, 33);
  Serial.begin(115200);

  // Display Initialization
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);
  sprite_header.createSprite(320, 40);
  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_json.createSprite(320, 200);
  sprite_json.fillScreen(0xFFFFFF);
  sprite_json.setTextColor(0x000000);
  sprite_header.println("ENR StickV2 Interface");
  sprite_header.pushSprite(0, 0);

  // Load config
  SD.begin();
  SPIFFS.begin();
  if (not load_config_from_FS(SD, String("/config.json")))
  {
    if (not load_config_from_FS(SPIFFS, String("/config.json")))
    {
      Serial.println("Failed to load config.");
      while (true)
      {
        delay(1000);
      }
    }
  }

  // UWB Initialization
  // TODO

  // SDP Initialization
  if (not init_sdp(NULL, String("SDP_STICKV2")))
  {
    Serial.println("SDP Initialization Failed.");
    while (true)
    {
      delay(1000);
    }
  }
}

void loop()
{
  auto last_read_stamp = millis();
  StaticJsonDocument<2048> doc;
  while (true)
  {
    if (not Serial1.available() and (millis() - last_read_stamp > 10000))
    {
      set_object_recognition_model(Serial1, String("./uploads/models/nanodet_80class"));
      Serial.println("Set objection dection mode.");
      last_read_stamp = millis();
    }
    else if (Serial1.available())
    {
      doc.clear();
      bool success = read_data_from_serial(Serial1, doc);
      if (success and doc.containsKey("num") and doc.containsKey("obj"))
      {
        int num_of_person = 0;
        long num_of_objects = doc["num"];
        for (int i = 0; i < num_of_objects; i++)
        {
          if (strncmp(doc["obj"][i]["type"], "person", 3) == 0)
          {
            ++num_of_person;
          }
        }
        std::vector<SDPData> data;
        data.push_back(SDPData(num_of_person));
        data.push_back(SDPData(std::string("73B2")));
        send_sdp_data_packet(packet_description, data);
      }
    }
  }
}
