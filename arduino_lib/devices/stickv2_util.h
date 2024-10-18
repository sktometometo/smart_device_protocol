#ifndef STICKV2_UTIL_H
#define STICKV2_UTIL_H

#include <Arduino.h>
#include <ArduinoJson.h>

#define BUFSIZE 2048

bool send_data_to_serial(HardwareSerial &serial,
                         StaticJsonDocument<BUFSIZE> &doc) {
  String request;
  serializeJson(doc, request);
  Serial.printf("Send data: %s\n", request.c_str());
  serial.println(request);
  return true;
}

bool read_data_from_serial(HardwareSerial &serial,
                           StaticJsonDocument<BUFSIZE> &doc) {
  String response = serial.readStringUntil('\n');
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    return false;
  }
  return true;
}

bool set_object_recognition_model(HardwareSerial &serial,
                                  const String &model_path) {
  StaticJsonDocument<BUFSIZE> doc;
  doc["function"] = "object_recognition";
  doc["args"][0] = model_path;
  return send_data_to_serial(serial, doc);
}

bool set_face_recognition(HardwareSerial &serial) {
  StaticJsonDocument<BUFSIZE> doc;
  doc["function"] = "face_recognition";
  JsonArray args = doc.createNestedArray("args");
  return send_data_to_serial(serial, doc);
}

//
// Data Parser
//
int parse_object_recognition_response(const StaticJsonDocument<BUFSIZE> &doc,
                                      const String &target_class,
                                      float threashold) {
  if (doc.containsKey("num") and doc.containsKey("obj")) {
    int num_of_target = 0;
    long num_of_objects = doc["num"];
    for (int i = 0; i < num_of_objects; i++) {
      // Serial.printf(" %d th object: %s, %f\n",
      //               i,
      //               doc["obj"][i]["type"].as<String>().c_str(),
      //               doc["obj"][i]["prob"].as<float>());
      if (doc["obj"][i]["type"] == target_class and doc["obj"][i]["prob"].as<float>() > threashold) {
        ++num_of_target;
      }
    }
    return num_of_target;
  } else {
    return -1;
  }
}

bool parse_face_recognition_response(const StaticJsonDocument<BUFSIZE> &doc,
                                     float threshold_prob, float threshold_match, std::vector<String> &names) {
  if (doc.containsKey("num") and doc.containsKey("face")) {
    names.clear();
    long num_of_faces = doc["num"];
    for (int i = 0; i < num_of_faces; i++) {
      if (doc["face"][i]["prob"].as<float>() > threshold_prob and doc["face"][i]["match_prob"].as<float>() > threshold_match) {
        names.push_back(doc["face"][i]["name"].as<String>());
      }
    }
    return true;
  } else {
    return false;
  }
}

#endif