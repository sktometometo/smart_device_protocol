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
#include "switchbot_util.h"
#include "iot_wifi_util.h"

/* Mofidy below */
String ssid = "";
String password = "";
String token = "";
String secret = "";

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
  sprite_device_info.println("Switchbot Client");
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
  if (USBSerial.available() or Serial2.available())
  {
    if (USBSerial.available())
    {
      bufstring = USBSerial.readStringUntil('\n');
    }
    else
    {
      bufstring = Serial2.readStringUntil('\n');
    }
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
    if (command == String("get_device_list"))
    {
      std::optional<String> ret = get_device_list(token, secret);
      if (ret)
      {
        success = true;
        message = "get_device_list success";
        String result = ret.value();
        deserializeJson(result_json, result.c_str());
        response_json["success"] = success;
        response_json["message"] = message;
        response_json["result"] = result_json;
      }
      else
      {
        success = false;
        message = "get_device_list failed";
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else if (command == String("get_device_status"))
    {
      if (not input_json.containsKey("device_id"))
      {
        success = false;
        message = "device_id key not found";
        response_json["success"] = success;
        response_json["message"] = message;
      }
      else
      {
        String device_id = input_json["device_id"];
        std::optional<String> ret = get_device_status(token, secret, device_id);
        if (ret)
        {
          String result = ret.value();
          DeserializationError error = deserializeJson(result_json, result.c_str());
          if (error)
          {
            success = false;
            message = "deserializeJson() failed during get_device_status: " + String(error.c_str()) + ", ret: " + result;
            response_json["success"] = success;
            response_json["message"] = message;
          }
          else
          {
            success = true;
            message = "get_device_status success";
            response_json["success"] = success;
            response_json["message"] = message;
            response_json["result"] = result_json;
          }
        }
        else
        {
          success = false;
          message = "get_device_status failed";
          response_json["success"] = success;
          response_json["message"] = message;
        }
      }
    }
    else if (command == "send_device_command")
    {
      if (not input_json.containsKey("device_id") or not input_json.containsKey("sb_command_type") or not input_json.containsKey("sb_command"))
      {
        success = false;
        message = "device_id or sb_command_type or sb_command key not found";
        response_json["success"] = success;
        response_json["message"] = message;
      }
      else
      {
        String device_id = input_json["device_id"];
        String sb_command_type = input_json["sb_command_type"];
        String sb_command = input_json["sb_command"];
        std::optional<String> ret = send_device_command(token, secret, device_id, sb_command_type, sb_command);
        if (ret)
        {
          success = true;
          message = "send_device_command success";
          String result = ret.value();
          deserializeJson(result_json, result.c_str());
          response_json["success"] = success;
          response_json["message"] = message;
          response_json["result"] = result_json;
        }
        else
        {
          success = false;
          message = "send_device_command failed";
          response_json["success"] = success;
          response_json["message"] = message;
        }
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
    else if (command == String("config_switchbot"))
    {
      if (not input_json.containsKey("token") or not input_json.containsKey("secret"))
      {
        success = false;
        message = "token or secret key not found";
        response_json["success"] = success;
        response_json["message"] = message;
      }
      else
      {
        token = input_json["token"].as<String>();
        secret = input_json["secret"].as<String>();
        success = true;
        message = "config_switchbot success";
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
      result_json["token"] = token;
      result_json["secret"] = secret;
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
