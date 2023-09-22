#include <esp_now.h>
#include <esp_system.h>

#include <Arduino.h>
#include <WiFi.h>

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <esp_now_ros/Packet.h>

static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_event_info(&lcd);

void readUWB()
{
  if (Serial2.available())
  {
    auto DATA = Serial2.readStringUntil('\n');
    Serial.print(DATA);
  }
}

void testUWB()
{
  Serial2.write("AT\r\n"); // RESET 复位
  delay(100);
  readUWB();
}

void resetUWB()
{
  Serial2.write("AT+RST\r\n"); // RESET 复位
  delay(100);
  auto ret = Serial2.readStringUntil('\n');
  Serial.printf("ret of reset: %s\n", ret.c_str());
  String DATA;
  auto timeout = millis() + 1000;
  while (timeout > millis())
  {
    if (Serial2.available())
    {
      DATA = Serial2.readStringUntil('\n');
      Serial.println(DATA);
      timeout = millis() + 1000;
    }
  }
}

void initUWB(bool tag = true, int id = 0)
{
  String DATA;
  resetUWB();
  if (tag)
  {
    Serial2.printf("AT+anchor_tag=0\r\n", id);
    delay(100);
    DATA = Serial2.readStringUntil('\n');
    Serial.printf("ret of anchor_tag: %s\n", DATA.c_str());

    Serial2.write("AT+interval=5\r\n");
    delay(100);
    DATA = Serial2.readStringUntil('\n');
    Serial.printf("ret of interval: %s\n", DATA.c_str());

    resetUWB();

    Serial2.write("AT+switchdis=1\r\n");
    delay(100);
    DATA = Serial2.readStringUntil('\n');
    Serial.printf("ret of switchdis: %s\n", DATA.c_str());
  }
  else
  {
    Serial2.printf("AT+anchor_tag=1,%d\r\n", id);
    delay(100);
    DATA = Serial2.readStringUntil('\n');
    Serial.printf("ret of anchor_tag: %s\n", DATA.c_str());

    resetUWB();
  }

  sprite_event_info.printf("ID: %d\n", id);
  sprite_event_info.printf("mode: %s\n", tag ? "tag" : "anchor");
  sprite_event_info.pushSprite(0, lcd.height() / 3);
}

void setup()
{
  // Read device mac address
  uint8_t device_mac_address[6] = {0};
  esp_read_mac(device_mac_address, ESP_MAC_WIFI_STA);

  // LCD Initialization
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
  sprite_event_info.createSprite(lcd.width(), lcd.height() * 2 / 3);

  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_device_info.setTextSize(1.0, 1.0);
  sprite_event_info.setTextSize(1.0, 1.0);
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_event_info.setTextColor(0x000000);

  sprite_device_info.println("SDP UWB Sample");
  sprite_device_info.printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", device_mac_address[0], device_mac_address[1],
                            device_mac_address[2], device_mac_address[3], device_mac_address[4], device_mac_address[5]);
  sprite_device_info.pushSprite(0, 0);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 1, 2);
  delay(100);

  // initUWB(true, 0);
  initUWB(false, 1);
}

void loop()
{
  delay(100);
  readUWB();
}
