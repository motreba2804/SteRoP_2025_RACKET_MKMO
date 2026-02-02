#pragma once
#include "Vector2D.h"

class Ball {
private:
    Vector2D pos;
    Vector2D vel;
    fix16 radius;
    bool active;

public:
    Ball(const Vector2D& p = {}, const Vector2D& v = {}, fix16 r = FIX(5));

    void activate(Vector2D p, Vector2D v, fix16 r);

    Vector2D getPosition() const;
    Vector2D getVelocity() const;
    fix16 getRadius() const;
    bool isActive() const;

    void setPosition(const Vector2D& p);
    void setVelocity(const Vector2D& v);
    void setRadius(fix16 r);
    void setActive(bool a);

    void update(fix16 dt);

    void bounce(const Vector2D& normal);
};
