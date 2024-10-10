#pragma once

#include <Arduino.h>

#define SEALEVELPRESSURE_PA 101325.0   // Pa
#define GAS_CONSTANT 8.31446261815324  // J/(mol K)
#define GRAVITY 9.80665                // m/s^2

/**
 * Calculate altitude from pressure and temperature
 *
 * @param pressure Pressure in Pa
 * @param temp Temperature in Celsius
 * @param sea_level_pressure Sea level pressure in Pa
 * @return Altitude in meters
 */
float calc_altitude(float pressure, float temp, float sea_level_pressure = SEALEVELPRESSURE_PA) {
  float altitude = (pow(sea_level_pressure / pressure, 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;
  return altitude;
}