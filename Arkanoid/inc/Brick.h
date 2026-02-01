#pragma once
#include "Vector2D.h"
#include "Ball.h"

class Brick {
private:
    Vector2D pos;     // lewy górny róg
    Vector2D size;    // szerokość i wysokość
    bool alive;       // czy cegła istnieje

public:
    Brick(const Vector2D& p = {}, const Vector2D& s = {FIX(60), FIX(20)});

    Vector2D getPosition() const;
    Vector2D getSize() const;
    bool isAlive() const;

    void setPosition(const Vector2D& p);
    void setSize(const Vector2D& s);
    void setAlive(bool a);

    bool checkCollision(const Ball& ball, Vector2D& outNormal) const;
};
