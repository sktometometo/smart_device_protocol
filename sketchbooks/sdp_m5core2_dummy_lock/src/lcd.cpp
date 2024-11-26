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

  sprite_header.createSprite(lcd.width(), lcd.height() / 8);                  // Pos 0, 0
  sprite_status.createSprite(lcd.width(), lcd.height() * 7 / 8);              // Pos 0, lcd.height() / 8
  sprite_lock_image.createSprite(lcd.width() / 2, lcd.height() * 7 / 8 / 2);  // Pos 0, lcd.height() / 8

  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setTextColor(0x000000);
  sprite_header.setTextSize(1.0, 1.0);
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setTextColor(0x000000);
}

void show_status(String &status) {
  sprite_status.fillScreen(0xFFFFFF);
  sprite_status.setCursor(0, 0);
  sprite_status.print(status);
  sprite_status.pushSprite(0, lcd.height() / 8);
}

void print_header(String &header) {
  sprite_header.fillScreen(0xFFFFFF);
  sprite_header.setCursor(0, 0);
  sprite_header.print(header);
  sprite_header.pushSprite(0, 0);
}

void show_lock_image() {
  sprite_lock_image.fillScreen(0xFFFFFF);
  bool draw_sucess = sprite_lock_image.drawJpgFile(SD, "/lock.jpg");
  if (!draw_sucess) {
    Serial.println("Failed to draw image");
  }
  // sprite_lock_image.pushSprite(lcd.width() / 4, lcd.height() / 8);
  sprite_lock_image.pushRotateZoom(&lcd, lcd.width() / 2, (int)(lcd.height() * (1.0 / 8 + 3.5 / 8)), 0.0, 2.0, 2.0);
}

void show_unlock_image() {
  sprite_lock_image.fillScreen(0xFFFFFF);
  sprite_lock_image.drawJpgFile(SD, "/unlock.jpg");
  // sprite_lock_image.pushSprite(lcd.width() / 4, lcd.height() / 8);
  sprite_lock_image.pushRotateZoom(&lcd, lcd.width() / 2, (int)(lcd.height() * (1.0 / 8 + 3.5 / 8)), 0.0, 2.0, 2.0);
}