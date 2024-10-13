#pragma once

#include <Arduino.h>

#include <LovyanGFX.hpp>
#include <optional>

typedef struct
{
  int32_t floor_num;
  float floor_height;
} ElevatorConfig;

enum ElevatorMovingStatus {
  HALT,
  UP_ACCEL,
  UP_STABLE,
  UP_DECEL,
  DOWN_ACCEL,
  DOWN_STABLE,
  DOWN_DECEL
};

void print_elevator_config_vector(LGFX_Sprite &sprite, const std::vector<ElevatorConfig> &elevator_config, int32_t push_x, int32_t push_y);

/**
 * Calculate floor number from altitude
 *
 * @param altitude Altitude in meters
 * @param reference_altitude Reference altitude in meters
 * @param reference_floor Reference floor number
 * @param elevator_config Elevator configuration
 * @return Floor number
 */
std::optional<int32_t> calc_floor(float altitude, float reference_altitude, int32_t reference_floor, const std::vector<ElevatorConfig> &elevator_config);

/**
 * Calculate acceleration on gravity direction
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @return Acceleration on gravity direction
 */
float calc_accel_on_gravity(float accels_x, float accels_y, float accels_z, float gravity_x, float gravity_y, float gravity_z);

/**
 * WIthdraw gravity from acceleration
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @return Acceleration without gravity
 */
std::tuple<float, float, float> withdraw_gravity(float accels_x, float accels_y, float accels_z, float gravity_x, float gravity_y, float gravity_z);

/**
 * Calculate next step moving status from acceleration
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @param current_status Current moving status
 * @param moving_threshold Threshold for moving detection
 * @return Moving status
 */
ElevatorMovingStatus calc_moving_status(float accel_on_gravity, ElevatorMovingStatus current_status, float moving_threshold);

std::string moving_status_to_string(ElevatorMovingStatus status);