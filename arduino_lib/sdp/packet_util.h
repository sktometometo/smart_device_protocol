#ifndef SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__
#define SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__

#include <variant>
#include <vector>

typedef std::variant<int32_t, float, std::string, bool> SDPData;
typedef std::tuple<std::string, std::string> SDPInterfaceDescription;

std::string get_serialization_format(const std::vector<SDPData> &data) {
  std::string serialization_format;
  for (auto itr = data.begin(); itr != data.end(); ++itr) {
    if (std::holds_alternative<int32_t>(*itr)) {
      serialization_format += "i";
    } else if (std::holds_alternative<float>(*itr)) {
      serialization_format += "f";
    } else if (std::holds_alternative<std::string>(*itr)) {
      if (std::get<std::string>(*itr).size() > 16) {
        serialization_format += "S";
      } else {
        serialization_format += "s";
      }
    } else if (std::holds_alternative<bool>(*itr)) {
      serialization_format += "?";
    } else {
      serialization_format += "_";
    }
  }
  return serialization_format;
}

bool is_consistent_serialization_format(const std::string &serialization_format,
                                        const std::vector<SDPData> &data) {
  std::string estimated_serialization_format = get_serialization_format(data);
  if (serialization_format.length() != estimated_serialization_format.length()) {
    return false;
  }

  for (size_t i = 0; i < serialization_format.length(); ++i) {
    if (serialization_format[i] == 'S' and estimated_serialization_format[i] == 's') {
      continue;
    } else if (serialization_format[i] == '?' and estimated_serialization_format[i] == 'b') {
      continue;
    } else if (serialization_format[i] == 'b' and estimated_serialization_format[i] == '?') {
      continue;
    }
    if (serialization_format[i] != estimated_serialization_format[i]) {
      return false;
    }
  }

  return true;
}

#endif  // SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__