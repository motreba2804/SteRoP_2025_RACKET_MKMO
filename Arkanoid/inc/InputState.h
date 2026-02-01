#pragma once
#include "FixedPoint.h"

struct InputState {
    fix16 normX;      // 0.0 (lewa) do 1.0 (prawa)
    fix16 normY;      // 0.0 (góra) do 1.0 (dół)
    fix16 rotation;   // Kąt w radianach (pełny zakres)
    bool isShooting;
};