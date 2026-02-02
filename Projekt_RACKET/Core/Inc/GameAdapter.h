#ifndef GAME_ADAPTER_H
#define GAME_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Game_Init(void);

void Game_UpdateAndDraw(float dt, float normX, float normY, float rotation, bool isBtnPressed);

#ifdef __cplusplus
}
#endif

#endif // GAME_ADAPTER_H
