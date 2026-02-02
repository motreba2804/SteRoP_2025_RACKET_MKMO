#include "Ball.h"


Ball::Ball(const Vector2D& p, const Vector2D& v, fix16 r)
        : pos(p), vel(v), radius(r), active(true) {}

void Ball::activate(Vector2D p, Vector2D v, fix16 r) {
    pos = p;
    vel = v;
    radius = r;
    active = true;
}

Vector2D Ball::getPosition() const { return pos; }
Vector2D Ball::getVelocity() const { return vel; }
fix16 Ball::getRadius() const { return radius; }
bool Ball::isActive() const { return active; }

void Ball::setPosition(const Vector2D& p) { pos = p; }
void Ball::setVelocity(const Vector2D& v) { vel = v; }
void Ball::setRadius(fix16 r) { radius = r; }
void Ball::setActive(bool a) { active = a; }

void Ball::update(fix16 dt) {
    pos.x += MUL(vel.x, dt);
    pos.y += MUL(vel.y, dt);
}

void Ball::bounce(const Vector2D& normal) {
    fix16 dot = vel.dot(normal);
    fix16 twoDot = dot << 1;
    vel.x = vel.x - MUL(twoDot, normal.x);
    vel.y = vel.y - MUL(twoDot, normal.y);
}
