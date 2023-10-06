#include <optional>

#include <M5AtomS3.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include <ArduinoJson.h>

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "Time.h"
#include "sesami_util.h"
#include "iot_wifi_util.h"

/* Mofidy below */
String ssid = "";
String password = "";
String device_uuid = "";
String secret_key = "";
String api_key = "";

//
WiFiMulti WiFiMulti;
static LGFX lcd;
static LGFX_Sprite sprite_device_info(&lcd);
static LGFX_Sprite sprite_event_info(&lcd);

void setup()
{
  M5.begin(false, true, false, false);
  Serial2.begin(115200, SERIAL_8N1, 2, 1);

  // LovyanGFX Initialization
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);
  sprite_device_info.createSprite(lcd.width(), lcd.height() / 3);
  sprite_event_info.createSprite(lcd.width(), lcd.height() / 3 * 2);
  sprite_device_info.fillScreen(0xFFFFFF);
  sprite_event_info.fillScreen(0xFFFFFF);
  sprite_device_info.setTextColor(0x000000);
  sprite_event_info.setTextColor(0x000000);
  sprite_device_info.setTextSize(1);
  sprite_event_info.setTextSize(1);

  //
  sprite_device_info.setCursor(0, 0);
  sprite_device_info.println("SESAMI Client");
  sprite_device_info.pushSprite(0, 0);

  //
  if (ssid != "" and password != "")
  {
    initWiFi(ssid.c_str(), password.c_str(), sprite_event_info, lcd, WiFiMulti);
  }

  USBSerial.printf("ARDUINO_LOOP_STACK_SIZE: %d\n", getArduinoLoopTaskStackSize());
}

void loop()
{
  delay(50);
  String bufstring;
  StaticJsonDocument<1024> input_json;
  StaticJsonDocument<1024> result_json;
  JsonObject result_json_obj;
  StaticJsonDocument<2048> response_json;
  String message;
  bool success;
#ifdef USBMODE
  if (USBSerial.available())
  {
    bufstring = USBSerial.readStringUntil('\n');
#else
  if (Serial2.available())
  {
    bufstring = Serial2.readStringUntil('\n');
#endif
    DeserializationError error = deserializeJson(input_json, bufstring.c_str());

    if (error)
    {
      success = false;
      message = "deserializeJson() failed: " + String(error.c_str());
      USBSerial.println(message);
      show_device_info(message.c_str(), sprite_event_info, lcd);
      response_json["success"] = success;
      response_json["message"] = message;
      Serial2.printf("%s\n", response_json.as<String>().c_str());
      return;
    }

    if (not input_json.containsKey("command"))
    {
      success = false;
      message = "command key not found";
      USBSerial.println(message);
      show_device_info(message.c_str(), sprite_event_info, lcd);
      response_json["success"] = success;
      response_json["message"] = message;
      Serial2.printf("%s\n", response_json.as<String>().c_str());
      return;
    }

    String command = input_json["command"];
    if (command == String("toggle") or command == String("lock") or command == String("unlock"))
    {
      std::optional<String> ret;
      if (command == String("toggle"))
      {
        ret = operation_sesami(device_uuid, api_key, 88, secret_key);
      }
      else if (command == String("lock"))
      {
        ret = operation_sesami(device_uuid, api_key, 82, secret_key);
      }
      else if (command == String("unlock"))
      {
        ret = operation_sesami(device_uuid, api_key, 83, secret_key);
      }
      if (ret)
      {
        success = true;
        message = "toggle/lock/unlock success";
      }
      else
      {
        success = false;
        message = "operation_sesami() failed";
      }
      response_json["success"] = success;
      response_json["message"] = message;
    }
    else if (command == String("status"))
    {
      std::optional<String> ret = get_sesami_status(device_uuid, api_key);
      if (ret)
      {
        String result = ret.value();
        DeserializationError error = deserializeJson(result_json, result.c_str());
        if (error)
        {
          success = false;
          message = "deserializeJson() failed during get_sesami_status: " + String(error.c_str());
          response_json["success"] = success;
          response_json["message"] = message;
        }
        else
        {
          success = true;
          message = "get status success";
          response_json["success"] = success;
          response_json["message"] = message;
          response_json["result"] = result_json;
        }
      }
      else
      {
        success = false;
        message = "get_sesami_status() failed";
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == String("history"))
    {
      std::optional<String> ret = get_sesami_history(device_uuid, api_key);
      if (ret)
      {
        String result = ret.value();
        DeserializationError error = deserializeJson(result_json, result.c_str());
        if (error)
        {
          success = false;
          message = "deserializeJson() failed during get_sesami_history: " + String(error.c_str());
          response_json["success"] = success;
          response_json["message"] = message;
        }
        else
        {
          success = true;
          message = "get history success";
          response_json["success"] = success;
          response_json["message"] = message;
          response_json["result"] = result_json;
        }
      }
      else
      {
        success = false;
        message = "get_sesami_history() failed";
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == "get_time")
    {
      uint32_t t = (uint32_t)std::time(nullptr);
      success = true;
      message = "get_time success";
      result_json["time"] = t;
      response_json["success"] = success;
      response_json["message"] = message;
      response_json["result"] = result_json;
    }
    else if (command == String("config_wifi"))
    {
      if (not input_json.containsKey("ssid") or not input_json.containsKey("password"))
      {
        success = false;
        message = "ssid or password key not found";
        response_json["success"] = success;
        response_json["message"] = message;
      }
      else
      {
        String new_ssid = input_json["ssid"].as<String>();
        String new_password = input_json["password"].as<String>();
        if (new_ssid != "")
          ssid = new_ssid;
        if (new_password != "")
          password = new_password;
        success = initWiFi(ssid.c_str(), password.c_str(), sprite_event_info, lcd, WiFiMulti);
        if (success)
        {
          message = "config_wifi success. SSID: " + ssid + ", password: " + password + ", IP: " + WiFi.localIP().toString();
        }
        else
        {
          message = "config_wifi failed";
        }
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == String("config_sesami"))
    {
      if (not input_json.containsKey("device_uuid") or not input_json.containsKey("secret_key") or not input_json.containsKey("api_key"))
      {
        success = false;
        message = "device_uuid or secret_key or api_key key not found";
        response_json["success"] = success;
        response_json["message"] = message;
      }
      else
      {
        device_uuid = input_json["device_uuid"].as<String>();
        secret_key = input_json["secret_key"].as<String>();
        api_key = input_json["api_key"].as<String>();
        success = true;
        message = "config_sesami success";
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == String("get_device_config"))
    {
      success = true;
      message = "get_device_config success";
      result_json["ssid"] = ssid;
      result_json["password"] = password;
      result_json["device_uuid"] = device_uuid;
      result_json["secret_key"] = secret_key;
      result_json["api_key"] = api_key;
      response_json["success"] = success;
      response_json["message"] = message;
      response_json["result"] = result_json;
    }
    else
    {
      success = false;
      message = "Unknown command error: " + command;
      response_json["success"] = success;
      response_json["message"] = message;
    }
    USBSerial.printf("response_json: %s\n", response_json.as<String>().c_str());
    show_device_info((String("response_json") + response_json.as<String>()).c_str(), sprite_event_info, lcd);
    Serial2.printf("%s\n", response_json.as<String>().c_str());
  }
}
