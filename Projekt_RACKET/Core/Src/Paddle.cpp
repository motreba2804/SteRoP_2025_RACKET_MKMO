#include "Paddle.h"
#include <cmath>
#include <algorithm>

Paddle::Paddle(Vector2D centerPos, Vector2D fullSize)
        : pos(centerPos), angle(0) {
    halfSize = fullSize / FIX(2);
}

void Paddle::getSinCos(fix16 ang, fix16& outSin, fix16& outCos) const {
    float a = TO_FLOAT(ang);
    outSin = FIX(std::sin(a));
    outCos = FIX(std::cos(a));
}

void Paddle::update(const InputState& input, fix16 worldW, fix16 worldH) {
    angle = input.rotation;

    pos.x = MUL(input.normX, worldW);
    pos.y = MUL(input.normY, worldH);

}

bool Paddle::checkCollision(Ball& ball) {
    Vector2D bPos = ball.getPosition();
    fix16 r = ball.getRadius();

    Vector2D delta = bPos - pos;

    fix16 sinA, cosA;
    getSinCos(angle, sinA, cosA);

    fix16 localX = MUL(delta.x, cosA) + MUL(delta.y, sinA);
    fix16 localY = MUL(delta.y, cosA) - MUL(delta.x, sinA);

    fix16 closestLocalX = clamp(localX, -halfSize.x, halfSize.x);
    fix16 closestLocalY = clamp(localY, -halfSize.y, halfSize.y);

    fix16 distX = localX - closestLocalX;
    fix16 distY = localY - closestLocalY;

    if (abs(distX) > r || abs(distY) > r) return false;

    fix16 distSq = MUL(distX, distX) + MUL(distY, distY);
    if (distSq > MUL(r, r)) return false;

    Vector2D localNormal;

    if (distSq == 0) {
        localNormal = Vector2D(FIX(0), FIX(-1));
    } else {

        if (abs(distX) > abs(distY)) {
            localNormal = Vector2D(sign(distX), FIX(0));
        } else {
            localNormal = Vector2D(FIX(0), sign(distY));
        }
    }

    Vector2D worldNormal;
    worldNormal.x = MUL(localNormal.x, cosA) - MUL(localNormal.y, sinA);
    worldNormal.y = MUL(localNormal.x, sinA) + MUL(localNormal.y, cosA);

    ball.bounce(worldNormal);

    fix16 overlap = r - fix16(std::sqrt(TO_FLOAT(distSq)));
    if (overlap < FIX(1)) overlap = FIX(1);

    ball.setPosition(ball.getPosition() + (worldNormal * overlap));

    return true;
}
