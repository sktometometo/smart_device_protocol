#pragma once

#include <ArduinoJson.h>

#include <optional>

#include "iot_com_util/iot_host_util.h"

std::optional<bool> get_device_status(String);
bool control_device(String, bool, int);

bool config_wifi(String ssid, String password) {
  String ret = send_serial_command(String("") + "{\"command\":\"config_wifi\"," + "\"ssid\":\"" + ssid + "\"," +
                                       "\"password\":\"" + password + "\"}\n",
                                   20000);
}

std::optional<bool> get_device_status(String switchbot_device_id) {
  StaticJsonDocument<1024> result_json;
  String result = send_serial_command(
      String("") + "{\"command\":\"get_device_status\"," + "\"device_id\":\"" + switchbot_device_id + "\"}\n", 5000);
  DeserializationError error = deserializeJson(result_json, result);
  if (error or (result_json.containsKey("success") and not result_json["success"].as<bool>())) {
    return std::nullopt;
  } else {
    String power = result_json["result"]["body"]["power"];
    return power == "on";
  }
}

bool control_device(String switchbot_device_id, bool turn_on, int timeout = 10) {
  String command = turn_on ? "turnOn" : "turnOff";
  String ret = send_serial_command(String("") + "{\"command\":\"send_device_command\"," + "\"device_id\":\"" +
                                       switchbot_device_id + "\"," + "\"sb_command_type\":\"command\"," +
                                       "\"sb_command\":\" " + command + "\"}\n",
                                   10000);
  int deadline = millis() / 1000 + timeout;
  while (millis() / 1000 < deadline) {
    Serial.printf("Fetching result.");
    auto power = get_device_status(switchbot_device_id);
    if (power.has_value() and power.value() == turn_on) {
      return true;
    }
    delay(1000);
  }
  return false;
}