#include <LovyanGFX.hpp>

#include "sdp/sdp_util.h"

void init_lcd(LGFX &lcd, LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2, LGFX_Sprite &sprite_3)
{
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(128);
    lcd.setColorDepth(24);
    lcd.fillScreen(0xFFFFFF);

    sprite_1.createSprite(300, 50);
    sprite_2.createSprite(300, 50);
    sprite_3.createSprite(300, 50);

    sprite_1.fillScreen(0xFFFFFF);
    sprite_1.setTextColor(0x000000);
    sprite_2.fillScreen(0xFFFFFF);
    sprite_2.setTextColor(0x000000);
    sprite_3.fillScreen(0xFFFFFF);
    sprite_3.setTextColor(0x000000);
}

void update_lcd(LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2, LGFX_Sprite &sprite_3)
{
    sprite_1.pushSprite(0, 0);
    sprite_2.pushSprite(0, 60);
    sprite_3.pushSprite(0, 120);
}

void clear_sprite(LGFX_Sprite &sprite)
{
    sprite.fillScreen(0xFFFFFF);
    sprite.setCursor(0, 0);
}