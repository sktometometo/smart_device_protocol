#include "lcd.h"

#if 1
#include <M5Stack.h>

#include "m5stack_utils/m5stack.h"
#endif

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

extern LGFX lcd;
extern LGFX_Sprite sprite_header;
extern LGFX_Sprite sprite_status;

void init_lcd() {
  lcd.init();
  lcd.setRotation(3);
  lcd.setBrightness(255);
  lcd.setColorDepth(16);
  lcd.fillScreen(TFT_WHITE);

  sprite_header.createSprite(lcd.width(), lcd.height() / 3);  // Pos: 0, 0
  sprite_status.createSprite(lcd.width(), lcd.height() / 3);  // Pos: 0, lcd.height() / 3

  sprite_header.fillScreen(TFT_WHITE);
  sprite_header.setTextColor(TFT_BLACK);
  sprite_header.setTextSize(1.5, 1.5);
  sprite_status.fillScreen(TFT_WHITE);
  sprite_status.setTextColor(TFT_BLACK);
}