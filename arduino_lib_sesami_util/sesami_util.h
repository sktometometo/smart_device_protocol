#ifndef SESAMI_UTIL_H
#define SESAMI_UTIL_H

#include <optional>

String generateRandomTag(String secret_key, uint32_t date_sec);

std::optional<String> operation_sesami(String device_uuid, String api_key, int command, String secret_key);

std::optional<String> get_sesami_status(String device_uuid, String api_key);

std::optional<String> get_sesami_history(String device_uuid, String api_key);

#endif // SESAMI_UTIL_H