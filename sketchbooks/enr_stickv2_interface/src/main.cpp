#include <esp_now.h>
#include <esp_system.h>

#include <M5Core2.h>
#include <WiFi.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <ArduinoJson.h>

#include <esp_now_ros/Packet.h>

#define BUFSIZE 2048

const char place[] = "73B2_TV_Front";

static LGFX lcd;
static LGFX_Sprite sprite_header(&lcd);
static LGFX_Sprite sprite_json(&lcd);

char buf[BUFSIZE];

esp_now_peer_info_t peer;

uint8_t packet[240];

StaticJsonDocument<BUFSIZE> doc;

void send_packet(uint32_t number_of_person, const char* place_name)
{
  *(uint16_t*)(packet + 0) = esp_now_ros::Packet::PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER;
  *(uint32_t*)(packet + 2) = number_of_person;
  strncpy((char*)(packet + 2 + 4), place_name, 200);
  esp_err_t result = esp_now_send(peer.peer_addr, (uint8_t*)packet, sizeof(packet) / sizeof(packet[0]));
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

  // ESP-NOW initialization
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  // Peer initialization
  for (int i = 0; i < 6; i++)
  {
    peer.peer_addr[i] = 0xff;
  }
  esp_err_t add_status = esp_now_add_peer(&peer);
}

void loop()
{
  auto last_read_stamp = millis();
  while (true)
  {
    if (not Serial1.available())
    {
      if (millis() > last_read_stamp + 10000)
      {
        Serial1.println("{\"function\": \"object_recognition\", \"args\": [\"./uploads/models/nanodet_80class\"]}");
        Serial.println("Send function command JSON.");
        last_read_stamp = millis();
      }
    }
    else
    {
      last_read_stamp = millis();

      int actual_length = Serial1.readBytesUntil('\n', buf, BUFSIZE);
      buf[actual_length] = 0;

      sprite_json.fillScreen(0xFFFFF);
      sprite_json.setCursor(0, 0);
      sprite_json.println("Read text:");
      sprite_json.println(buf);
      sprite_json.pushSprite(0, 40);

      Serial.print("Read text:");
      Serial.print(buf);
      Serial.print("\n");

      DeserializationError error = deserializeJson(doc, buf);
      if (error)
      {
        Serial.print("JSON deserialization Failed: ");
        Serial.println(error.f_str());
        continue;
      }

      if (doc.containsKey("num") and doc.containsKey("obj"))
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
        Serial.print("Detection result:\n");
        Serial.printf("number of objects: %ld\n", num_of_objects);
        Serial.printf("number of people: %d\n", num_of_person);
        Serial.print("\n");

        send_packet(num_of_person, place);
      }
      else
      {
        Serial.println("Unknown JSON Structure.");
      }
    }
  }
}
