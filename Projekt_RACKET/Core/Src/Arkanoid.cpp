#include "Arkanoid.hpp"
#include <stdlib.h>

extern "C" {
    #include "stm32f429i_discovery_lcd.h"
    #include "stm32f429i_discovery.h"
}


Arkanoid::Arkanoid() {
    max_w = 0;
    max_h = 0;
}

void Arkanoid::init() {
    max_w = BSP_LCD_GetXSize();
    max_h = BSP_LCD_GetYSize();

    plat_w = 60;
    plat_h = 10;
    plat_y = max_h - 20;
    plat_x = (max_w - plat_w) / 2;
    old_plat_x = plat_x;

    int brick_w = 40;
    int brick_h = 15;
    bx[0] = 30;  by[0] = 40;  balive[0] = 1;
    bx[1] = 170; by[1] = 70;  balive[1] = 1;
    bx[2] = 100; by[2] = 120; balive[2] = 1;

    size = 15;
    ball_x = plat_x + (plat_w / 2) - (size / 2);
    ball_y = plat_y - size - 5;
    speed_x = 1;
    speed_y = -1;

    BSP_LCD_Clear(LCD_COLOR_BLACK);

    BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
    for(int i=0; i<BRICK_COUNT; i++) {
        if(balive[i]) {
            BSP_LCD_FillRect(bx[i], by[i], brick_w, brick_h);
        }
    }

    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_FillRect(plat_x, plat_y, plat_w, plat_h);
}

void Arkanoid::updateAndRender(float roll_input) {
    if (max_w == 0) return;

    old_ball_x = ball_x;
    old_ball_y = ball_y;
    old_plat_x = plat_x;
    int brick_hit_index = -1;
    int brick_w = 40;
    int brick_h = 15;

    int center_x = (max_w / 2) - (plat_w / 2);
    int sensitivity = 5;

    plat_x = center_x + (int)(roll_input * sensitivity);

    if (plat_x < 0) plat_x = 0;
    if (plat_x + plat_w > max_w) plat_x = max_w - plat_w;

    ball_x += speed_x;
    if (ball_x <= 0) { ball_x = 0; speed_x = abs(speed_x); }
    if (ball_x + size >= max_w) { ball_x = max_w - size; speed_x = -abs(speed_x); }

    if (ball_x + size > plat_x && ball_x < plat_x + plat_w &&
        ball_y + size > plat_y && ball_y < plat_y + plat_h)
    {
        speed_y = -abs(speed_y);
    }

    for(int i=0; i<BRICK_COUNT; i++) {
        if (balive[i] == 1) {
            if (ball_x + size > bx[i] && ball_x < bx[i] + brick_w &&
                ball_y + size > by[i] && ball_y < by[i] + brick_h)
            {
                speed_y = -speed_y;
                balive[i] = 0;
                brick_hit_index = i;
            }
        }
    }

    ball_y += speed_y;
    if (ball_y <= 0) { ball_y = 0; speed_y = abs(speed_y); }
    if (ball_y + size >= max_h) {
        ball_x = plat_x + (plat_w/2);
        ball_y = plat_y - size - 5;
        speed_y = -abs(speed_y);
    }

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(old_ball_x, old_ball_y, size, size);

    if (abs(plat_x - old_plat_x) > 0) {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_FillRect(old_plat_x, plat_y, plat_w, plat_h);
        BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
        BSP_LCD_FillRect(plat_x, plat_y, plat_w, plat_h);
    } else {
        if (old_ball_x + size > plat_x && old_ball_x < plat_x + plat_w &&
            old_ball_y + size > plat_y && old_ball_y < plat_y + plat_h)
        {
             BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
             BSP_LCD_FillRect(plat_x, plat_y, plat_w, plat_h);
        }
    }

    for(int i=0; i<BRICK_COUNT; i++)
    {
        if (balive[i] == 1 && i != brick_hit_index)
        {
            if (old_ball_x + size > bx[i] && old_ball_x < bx[i] + brick_w &&
                old_ball_y + size > by[i] && old_ball_y < by[i] + brick_h)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
                BSP_LCD_FillRect(bx[i], by[i], brick_w, brick_h);
            }
        }
    }

    if (brick_hit_index != -1)
    {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_FillRect(bx[brick_hit_index], by[brick_hit_index], brick_w, brick_h);
    }

    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillRect(ball_x, ball_y, size, size);
}

Arkanoid game;

void Game_Init_CPP(void) {
    game.init();
}

void Game_Loop_CPP(float roll_val) {
    game.updateAndRender(roll_val);
}
