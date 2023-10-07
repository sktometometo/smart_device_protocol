#ifndef IOT_HOST_UTIL_H
#define IOT_HOST_UTIL_H

#include <Arduino.h>
#include <ArduinoJson.h>

String send_serial_command(String command, int timeout_duration = 5000)
{
  Serial2.print(command.c_str());
  auto timeout = millis() + timeout_duration;
  while (millis() < timeout)
  {
    delay(100);
    if (Serial2.available())
    {
      return Serial2.readStringUntil('\n');
    }
  }
  return "";
}

template <int N>
bool load_json_from_FS(fs::FS &fs, const String &filename, StaticJsonDocument<N> &doc)
{
  auto file = fs.open(filename.c_str());
  if (!file)
  {
    Serial.printf("Failed to open config file from %s\n", filename.c_str());
    file.close();
    return false;
  }
  DeserializationError error = deserializeJson(doc, file.readString());
  if (error)
  {
    Serial.println("Failed to parse config file");
    file.close();
    return false;
  }
  file.close();
  return true;
}
#endif // IOT_HOST_UTIL_H
