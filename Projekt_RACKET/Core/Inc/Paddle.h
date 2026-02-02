#pragma once
#include "Vector2D.h"
#include "Ball.h"
#include "FixedPoint.h"
#include "InputState.h"

class Paddle {
private:
    Vector2D pos;
    Vector2D halfSize;
    fix16 angle;

    void getSinCos(fix16 ang, fix16& outSin, fix16& outCos) const;

public:

    Paddle(Vector2D centerPos = {}, Vector2D fullSize = {FIX(100), FIX(20)});

    void update(const InputState& input, fix16 worldW, fix16 worldH);

    bool checkCollision(Ball& ball);

    Vector2D getPosition() const { return pos; }
    Vector2D getSize() const { return halfSize * FIX(2); }
    fix16 getRotation() const { return angle; }
};
