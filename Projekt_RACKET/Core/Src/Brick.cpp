#include "Brick.h"
#include "FixedPoint.h"



Brick::Brick(const Vector2D& p, const Vector2D& s)
        : pos(p), size(s), alive(true) {}


Vector2D Brick::getPosition() const { return pos; }
Vector2D Brick::getSize() const { return size; }
bool Brick::isAlive() const { return alive; }


void Brick::setPosition(const Vector2D& p) { pos = p; }
void Brick::setSize(const Vector2D& s) { size = s; }
void Brick::setAlive(bool a) { alive = a; }


bool Brick::checkCollision(const Ball& ball, Vector2D& outNormal) const {
    if (!alive) return false;

    Vector2D c = ball.getPosition();
    fix16 r = ball.getRadius();

    fix16 closestX = clamp(c.x, pos.x, pos.x + size.x);
    fix16 closestY = clamp(c.y, pos.y, pos.y + size.y);

    fix16 dx = c.x - closestX;
    fix16 dy = c.y - closestY;


    if (abs(dx) > r || abs(dy) > r) {
        return false;
    }

    fix16 distSq = MUL(dx, dx) + MUL(dy, dy);
    fix16 radSq  = MUL(r, r);

    if (distSq > radSq)
        return false;

    if (abs(dx) > abs(dy)) {
        outNormal = Vector2D(sign(dx), FIX(0));
    } else {
        outNormal = Vector2D(FIX(0), sign(dy));
    }

    return true;
}
