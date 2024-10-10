#pragma once

#include <Arduino.h>

#include <optional>

typedef struct
{
  int32_t floor_num;
  float floor_height;
} ElevatorConfig;

/**
 * Calculate floor number from altitude
 *
 * @param altitude Altitude in meters
 * @param elevator_config Elevator configuration
 * @param threshold Threshold for floor detection
 * @return Floor number
 *
 * C++ version of code below
 *
 * altitude_diff_list = [
                {
                    'floor': key,
                    'altitude_diff':
                    (entry['altitude'] - self.elevator_config[self.param_anchor_floor]['altitude']) - (self.state_altitude - self.param_anchor_altitude)
                }
                for key, entry in self.elevator_config.items()]
            nearest_entry = min(
                altitude_diff_list,
                key=lambda x: math.fabs(x['altitude_diff'])
            )
            self.state_current_floor = nearest_entry['floor']
 */
std::optional<int32_t> calc_floor(float altitude, const std::vector<ElevatorConfig> &elevator_config) {
  std::vector<std::pair<int32_t, float>> altitude_diff_list;
  for (uint8_t i = 0; i < elevator_config.size(); i++) {
    altitude_diff_list.push_back(std::make_pair(
        i,
        (elevator_config[i].floor_height - elevator_config[0].floor_height) - (altitude - elevator_config[0].floor_height)));
  }
  auto nearest_entry = std::min_element(
      altitude_diff_list.begin(),
      altitude_diff_list.end(),
      [](const std::pair<int32_t, float> &a, const std::pair<int32_t, float> &b) {
        return std::fabs(a.second) < std::fabs(b.second);
      });
  return nearest_entry->first;  // floor number
}