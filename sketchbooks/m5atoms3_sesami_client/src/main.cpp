#include <M5AtomS3.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <optional>

#include "Time.h"
#include <sesami_util.h>

/* Mofidy below */
const char* ssid = "";
const char* password = "";
String device_uuid = "";
String secret_key = "";
String api_key = "";

WiFiMulti WiFiMulti;

void setup()
{
  int sum = 0;
  M5.begin(true, true, false, false);
  WiFiMulti.addAP(ssid, password);
  M5.lcd.printf("== M5AtomS3 Sesami Client ==\n");
  M5.lcd.printf("Waiting connect to WiFi: %s ...", ssid);
  while (WiFiMulti.run() != WL_CONNECTED)
  {
    M5.lcd.print(".");
    delay(1000);
    sum += 1;
    if (sum == 8)
      M5.lcd.print("Conncet failed!");
  }
  M5.lcd.println("\nWiFi connected");
  M5.lcd.print("IP address: ");
  M5.lcd.println(WiFi.localIP());
  delay(500);

  Time.set_time();

  Serial2.begin(115200, SERIAL_8N1, 2, 1);
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
  if (Serial2.available())
  {
    bufstring = Serial2.readStringUntil('\n');
    DeserializationError error = deserializeJson(input_json, bufstring.c_str());

    if (error)
    {
      success = false;
      message = "deserializeJson() failed: " + String(error.c_str());
      USBSerial.println(message);
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
        message = "toggle success";
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
        success = true;
        message = "get status success";
        String result = ret.value();
        deserializeJson(result_json, result.c_str());
        response_json["success"] = success;
        response_json["message"] = message;
        response_json["result"] = result_json;
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
        success = true;
        message = "get history success";
        String result = ret.value();
        deserializeJson(result_json, result.c_str());
        response_json["success"] = success;
        response_json["message"] = message;
        response_json["result"] = result_json;
      }
      else
      {
        success = false;
        message = "get_sesami_history() failed";
        response_json["success"] = success;
        response_json["message"] = message;
      }
    }
    else
    {
      success = false;
      message = "command error: " + command;
      response_json["success"] = success;
      response_json["message"] = message;
    }
    USBSerial.printf("response_json: %s\n", response_json.as<String>().c_str());
    Serial2.printf("%s\n", response_json.as<String>().c_str());
  }
}
