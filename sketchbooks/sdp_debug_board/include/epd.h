#include <M5EPD.h>

void clear_epd(M5EPD_Canvas &canvas)
{
    canvas.clear();
    canvas.setCursor(0, 0);
}

void init_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_info, M5EPD_Canvas &canvas_message)
{
    canvas_title.createCanvas(540, 100);
    canvas_info.createCanvas(540, 60);
    canvas_message.createCanvas(540, 800);

    canvas_title.setTextSize(3);
    canvas_info.setTextSize(2);
    canvas_message.setTextSize(2);

    clear_epd(canvas_title);
    clear_epd(canvas_info);
    clear_epd(canvas_message);
}

void update_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_info, M5EPD_Canvas &canvas_message)
{
    canvas_title.pushCanvas(0, 0, UPDATE_MODE_DU4);
    canvas_info.pushCanvas(0, 100, UPDATE_MODE_DU4);
    canvas_message.pushCanvas(0, 160, UPDATE_MODE_DU4);
}
