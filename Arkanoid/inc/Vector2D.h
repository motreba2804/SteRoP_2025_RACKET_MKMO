#pragma once
#include "FixedPoint.h"

struct Vector2D {
    fix16 x;
    fix16 y;

    Vector2D(fix16 X = 0, fix16 Y = 0) : x(X), y(Y) {}

    Vector2D operator+(const Vector2D& o) const { return {x + o.x, y + o.y}; }
    Vector2D operator-(const Vector2D& o) const { return {x - o.x, y - o.y}; }
    Vector2D operator*(fix16 s) const { return {MUL(x,s), MUL(y,s)}; }
    Vector2D operator/(fix16 s) const { return {DIV(x,s), DIV(y,s)}; }

    fix16 dot(const Vector2D& o) const { return MUL(x,o.x) + MUL(y,o.y); }
    fix16 cross(const Vector2D& o) const { return MUL(x,o.y) - MUL(y,o.x); }

    fix16 length() const {
        float fx = TO_FLOAT(x);
        float fy = TO_FLOAT(y);
        return FIX(std::sqrt(fx*fx + fy*fy));
    }

    Vector2D normalized() const {
        fix16 len = length();
        return len != 0 ? *this / len : Vector2D{0,0};
    }
};
