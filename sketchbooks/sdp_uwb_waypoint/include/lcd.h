#include <LovyanGFX.hpp>

#include "sdp/sdp_util.h"

void init_lcd(LGFX &lcd, LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2,
              LGFX_Sprite &sprite_3);
void update_lcd(LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2,
                LGFX_Sprite &sprite_3);
void check_and_scroll(LGFX_Sprite &sprite);
void clear_sprite(LGFX_Sprite &sprite);

void init_lcd(LGFX &lcd, LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2,
              LGFX_Sprite &sprite_3) {
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.fillScreen(0xFFFFFF);

  sprite_1.createSprite(lcd.width(), lcd.height() / 3);
  sprite_2.createSprite(lcd.width(), lcd.height() / 3);
  sprite_3.createSprite(lcd.width(), lcd.height() / 3);

  sprite_1.fillScreen(0x000000);
  sprite_1.setTextColor(0xFFFFFF);
  sprite_2.fillScreen(0x000000);
  sprite_2.setTextColor(0xFFFFFF);
  sprite_3.fillScreen(0x000000);
  sprite_3.setTextColor(0xFFFFFF);
}

void update_lcd(LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2,
                LGFX_Sprite &sprite_3) {
  check_and_scroll(sprite_1);
  check_and_scroll(sprite_2);
  check_and_scroll(sprite_3);
  sprite_1.pushSprite(0, 0);
  sprite_2.pushSprite(0, sprite_1.height());
  sprite_3.pushSprite(0, sprite_1.height() + sprite_2.height());
}

void check_and_scroll(LGFX_Sprite &sprite) {
  int16_t x = sprite.getCursorX();
  int16_t y = sprite.getCursorY();
  int font_height = sprite.fontHeight();
  while (y >= sprite.height()) {
    sprite.scroll(0, -font_height);
    sprite.setCursor(x, y - font_height);
    x = sprite.getCursorX();
    y = sprite.getCursorY();
  }
}

void clear_sprite(LGFX_Sprite &sprite) {
  sprite.fillScreen(0xFFFFFF);
  sprite.setCursor(0, 0);
}