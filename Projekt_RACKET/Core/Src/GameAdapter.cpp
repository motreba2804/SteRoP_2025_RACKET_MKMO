#include "GameAdapter.h"
#include "World.h"
#include "STM32Renderer.h"
#include "InputState.h"
#include "FixedPoint.h"

extern "C" {
#include "stm32f429i_discovery_lcd.h"
}

static World* gameWorld = nullptr;
static STM32Renderer* gameRenderer = nullptr;

extern "C" void Game_Init(void) {
    int w = BSP_LCD_GetXSize();
    int h = BSP_LCD_GetYSize();

    gameWorld = new World(FIX(w), FIX(h));
    gameWorld->init();

    gameRenderer = new STM32Renderer(w, h);
}

extern "C" void Game_UpdateAndDraw(float dt, float normX, float normY, float rotation, bool isBtnPressed) {
    if (!gameWorld || !gameRenderer) return;

    InputState input;
    input.normX = FIX(normX);
    input.normY = FIX(normY);
    input.rotation = FIX(rotation);
    input.isShooting = isBtnPressed;

    gameWorld->update(FIX(dt), input);

    BSP_LCD_Clear(LCD_COLOR_BLACK);

    gameRenderer->draw(*gameWorld);
}
