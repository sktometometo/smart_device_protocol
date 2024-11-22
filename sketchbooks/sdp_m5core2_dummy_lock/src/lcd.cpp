#include "lcd.h"

#include <M5Core2.h>
#include <m5stack_utils/m5core2.h>

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

extern LGFX lcd;
extern LGFX_Sprite sprite_header;
extern LGFX_Sprite sprite_status;
extern LGFX_Sprite sprite_lock_image;

void init_lcd() {
  // LCD
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_header.createSprite(lcd.width(), lcd.height() / 4);                  // Pos 0, 0
  sprite_status.createSprite(lcd.width(), lcd.height() * 3 / 4);              // Pos 0, lcd.height() / 4
  sprite_lock_image.createSprite(lcd.width() / 2, lcd.height() * 3 / 4 / 2);  // Pos 0, lcd.height() / 4

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.0, 1.0);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
}

void show_lock_image() {
  sprite_lock_image.fillScreen(0xFFFFFF);
  Serial.println("hoge");
  sprite_lock_image.drawJpgFile(SPIFFS, "/lock.jpg", 0, 0, 0, 0, 0, 0, 1.0, 1.0);
  Serial.println("fuga");
  sprite_lock_image.pushSprite(0, lcd.height() / 4);
  Serial.println("piyo");
}

void show_unlock_image() {
  sprite_lock_image.fillScreen(0xFFFFFF);
  sprite_lock_image.drawJpgFile(SPIFFS, "/unlock.jpg", 0, 0, 0, 0, 0, 0, 1.0, 1.0);
  sprite_lock_image.pushSprite(0, lcd.height() / 4);
}