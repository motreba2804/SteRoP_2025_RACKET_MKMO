#include "GameAdapter.h"
#include "World.h"
#include "STM32Renderer.h"
#include "InputState.h"
#include "FixedPoint.h"
#include <stdio.h>

extern "C" {
#include "stm32f429i_discovery_lcd.h"
}

static World* gameWorld = nullptr;
static STM32Renderer* gameRenderer = nullptr;

enum GameState {
    STATE_WAIT_FOR_START,
    STATE_PLAYING,
    STATE_GAME_OVER
};

static GameState currentState = STATE_WAIT_FOR_START;

extern "C" void Game_Init(void) {
    int w = BSP_LCD_GetXSize();
    int h = BSP_LCD_GetYSize();

    gameWorld = new World(FIX(w), FIX(h));
    gameWorld->init();

    gameRenderer = new STM32Renderer(w, h);

    currentState = STATE_WAIT_FOR_START;
}

extern "C" void Game_UpdateAndDraw(float dt, float normX, float normY, float rotation, bool isBtnPressed) {
    if (!gameWorld || !gameRenderer) return;

    InputState input;
    input.normX = FIX(normX);
    input.normY = FIX(normY);
    input.rotation = FIX(rotation);
    input.isShooting = isBtnPressed;

    switch (currentState) {
        case STATE_WAIT_FOR_START:
            gameWorld->update(FIX(dt), input, false);

            if (isBtnPressed) {
                currentState = STATE_PLAYING;
            }
            break;

        case STATE_PLAYING:
            gameWorld->update(FIX(dt), input, true);

            if (gameWorld->isGameLost()) {
                currentState = STATE_GAME_OVER;
            }
            break;

        case STATE_GAME_OVER:
            break;
    }

    BSP_LCD_Clear(LCD_COLOR_BLACK);

    gameRenderer->draw(*gameWorld);

    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);

    char scoreBuffer[32];
    sprintf(scoreBuffer, "Score: %d", gameWorld->getScore());
    BSP_LCD_DisplayStringAt(0, 5, (uint8_t*)scoreBuffer, LEFT_MODE);

    if (currentState == STATE_WAIT_FOR_START) {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_SetFont(&Font16);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 8, (uint8_t*)"Nacisnij przycisk", CENTER_MODE);
    }
    else if (currentState == STATE_GAME_OVER) {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_SetFont(&Font24);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 12, (uint8_t*)"KONIEC GRY!", CENTER_MODE);
    }
}
