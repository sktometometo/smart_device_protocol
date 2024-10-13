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

void init_lcd() {
  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(lcd.width(), lcd.height() / 3);
  sprite_status.createSprite(lcd.width(), lcd.height() / 3);
  sprite_info.createSprite(lcd.width(), lcd.height() / 3);

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
  sprite_info.fillScreen(0xFFFFFF);
  sprite_info.setTextColor(0x000000);
}

void print_status() {
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  //   sprite_status.printf("Acc: %.2f, %.2f, %.2f\n", sensor_accX, sensor_accY, sensor_accZ);
  //   sprite_status.printf("Gyro: %.2f, %.2f, %.2f\n", sensor_gyroX, sensor_gyroY, sensor_gyroZ);
  //   sprite_status.printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", sensor_pitch, sensor_roll, sensor_yaw);
  sprite_status.printf("Pressure: %.2f\n", sensor_pressure);
  sprite_status.printf("Temp: %.2f, %.2f\n", sensor_temp_mpu, sensor_temp_dps);
  sprite_status.printf("Altitude: %.2f\n", altitude);
  sprite_status.printf("Gravity: %.2f, %.2f, %.2f\n", gravity_x, gravity_y, gravity_z);
  sprite_status.printf("Accel without gravity: %.2f, %.2f, %.2f\n", accel_x_without_gravity, accel_y_without_gravity, accel_z_without_gravity);
  sprite_status.printf("Accel on gravity: %.2f\n", accel_on_gravity);
  sprite_status.printf("Floor: %d\n", current_floor);
  //   sprite_status.printf("Status: %d\n", current_status);
  switch (current_status) {
    case HALT:
      sprite_status.printf("Status: HALT\n");
      break;
    case UP_ACCEL:
      sprite_status.printf("Status: UP_ACCEL\n");
      break;
    case UP_STABLE:
      sprite_status.printf("Status: UP_STABLE\n");
      break;
    case UP_DECEL:
      sprite_status.printf("Status: UP_DECEL\n");
      break;
    case DOWN_ACCEL:
      sprite_status.printf("Status: DOWN_ACCEL\n");
      break;
    case DOWN_STABLE:
      sprite_status.printf("Status: DOWN_STABLE\n");
      break;
    case DOWN_DECEL:
      sprite_status.printf("Status: DOWN_DECEL\n");
      break;
  }
  sprite_status.pushSprite(0, lcd.height() / 3);
}