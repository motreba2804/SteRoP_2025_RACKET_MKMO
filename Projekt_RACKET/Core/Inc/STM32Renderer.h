#pragma once
#include "World.h"
#include "FixedPoint.h"

class STM32Renderer {
private:
    int screenWidth;
    int screenHeight;

public:
    STM32Renderer(int w, int h);
    void draw(const World& world);
};
