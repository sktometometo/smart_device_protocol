#pragma once

#include <Arduino.h>

std::tuple<float, float, float> calc_gravity_direction(std::vector<float> &accels_x, std::vector<float> &accels_y, std::vector<float> &accels_z) {
  float gravity_x = 0.0;
  float gravity_y = 0.0;
  float gravity_z = 0.0;
  for (auto &a : accels_x) {
    gravity_x += a;
  }
  for (auto &a : accels_y) {
    gravity_y += a;
  }
  for (auto &a : accels_z) {
    gravity_z += a;
  }
  gravity_x /= accels_x.size();
  gravity_y /= accels_y.size();
  gravity_z /= accels_z.size();
  return std::make_tuple(gravity_x, gravity_y, gravity_z);
}
