#if 1  // defined(M5STACK_CORE2)
#include <M5Core2.h>
#include <m5stack_utils/m5core2.h>
#elif defined(M5STACK_FIRE)
#include <M5Stack.h>
#include <m5stack_utils/m5stack.h>
#endif

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "elevator_status.h"

extern LGFX lcd;
extern LGFX_Sprite sprite_header;
extern LGFX_Sprite sprite_status;
extern LGFX_Sprite sprite_info;
extern LGFX_Sprite sprite_plot;

extern float sensor_accX;
extern float sensor_accY;
extern float sensor_accZ;
extern float sensor_gyroX;
extern float sensor_gyroY;
extern float sensor_gyroZ;
extern float sensor_pitch;
extern float sensor_roll;
extern float sensor_yaw;
extern float sensor_temp_mpu;
extern float sensor_temp_dps;
extern float sensor_pressure;

extern float altitude;
extern int32_t current_floor;
extern ElevatorMovingStatus current_status;

extern float gravity_x;
extern float gravity_y;
extern float gravity_z;

extern float accel_x_without_gravity;
extern float accel_y_without_gravity;
extern float accel_z_without_gravity;
extern float accel_on_gravity;

extern float initial_altitude;
extern int32_t initial_floor;

void init_lcd() {
  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(lcd.width(), lcd.height() / 3);      // Pos 0, 0
  sprite_status.createSprite(lcd.width() / 2, lcd.height() / 3);  // Pos 0, lcd.height() / 3
  sprite_info.createSprite(lcd.width() / 2, lcd.height() / 3);    // Pos lcd.width() / 2, lcd.height() / 3
  sprite_plot.createSprite(lcd.width(), lcd.height() / 3);        // Pos 0, lcd.height() / 3 * 2

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
  sprite_info.fillScreen(0xFFFFFF);
  sprite_info.setTextColor(0x000000);
  sprite_plot.fillScreen(0xFFFFFF);
  sprite_plot.setTextColor(0x000000);
}

void print_status() {
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  //   sprite_status.printf("Acc: %.2f, %.2f, %.2f\n", sensor_accX, sensor_accY, sensor_accZ);
  //   sprite_status.printf("Gyro: %.2f, %.2f, %.2f\n", sensor_gyroX, sensor_gyroY, sensor_gyroZ);
  //   sprite_status.printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", sensor_pitch, sensor_roll, sensor_yaw);
  sprite_status.printf("%.2f Pa, %.2f C\n", sensor_pressure, sensor_temp_dps);
  sprite_status.printf("Alt.: %.2f m\n", altitude);
  // sprite_status.printf("Grv.: %.2f, %.2f, %.2f\n", gravity_x, gravity_y, gravity_z);
  sprite_status.printf("Acc(wo grav): %.2f, %.2f, %.2f\n", accel_x_without_gravity, accel_y_without_gravity, accel_z_without_gravity);
  sprite_status.printf("Accel on grv dir: %.2f\n", accel_on_gravity);
  sprite_status.printf("Init floor: %d, alt: %.2f\n", initial_floor, initial_altitude);
  sprite_status.printf("Floor: %d\n", current_floor);
  //   sprite_status.printf("Status: %d\n", current_status);
  std::string status_str = moving_status_to_string(current_status);
  sprite_status.printf("Status: %s\n", status_str.c_str());
  sprite_status.pushSprite(0, lcd.height() / 3);
}