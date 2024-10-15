#pragma once

#include <Arduino.h>
#include <M5Stack.h>
#include <Wire.h>

#define SERVO_ADDR 0x53

void init_servo_module(bool init_wire = true) {
  if (init_wire) {
    Wire.begin(21, 22, 100000UL);
  }
}

// addr 0x01 mean control the number 1 servo by us
void Servo_write_us(uint8_t number, uint16_t us) {
  Wire.beginTransmission(SERVO_ADDR);
  Wire.write(0x00 | number);
  Wire.write(us & 0x00ff);
  Wire.write(us >> 8 & 0x00ff);
  Wire.endTransmission();
}

// addr 0x11 mean control the number 1 servo by angle
void Servo_write_angle(uint8_t number, float servo_angle) {
  uint8_t angle;
  if (servo_angle < 0) {
    angle = 0;
  } else if (servo_angle > 180) {
    angle = 180;
  } else {
    angle = (uint8_t)servo_angle;
  }
  Wire.beginTransmission(SERVO_ADDR);
  Wire.write(0x10 | number);
  Wire.write(angle);
  Wire.endTransmission();
}
