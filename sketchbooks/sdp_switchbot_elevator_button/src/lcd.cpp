#include "lcd.h"

#if defined(M5STACK_FIRE)
#include <M5Stack.h>

#include "m5stack_utils/m5stack.h"
#elif defined(M5STACK_CORE2)
#include <M5Core2.h>

#include "m5stack_utils/m5core2.h"
#endif

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

extern LGFX lcd;
extern LGFX_Sprite sprite_header;
extern LGFX_Sprite sprite_status;
extern LGFX_Sprite sprite_info;

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