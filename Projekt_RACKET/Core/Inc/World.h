#pragma once
#include "Ball.h"
#include "Brick.h"
#include "Paddle.h"
#include "InputState.h"
#include "FixedPoint.h"
#include <array>

constexpr int MAX_BALLS = 5;
constexpr int MAX_BRICKS = 100;

class World {
private:
    fix16 width;
    fix16 height;
    int destroyedBricks;
    bool gameLost;

    fix16 spawnTimer;
    fix16 currentSpawnInterval;
    int rowsSpawnedCount;

public:
    std::array<Ball, MAX_BALLS> balls;
    int ballCount;

    std::array<Brick, MAX_BRICKS> bricks;
    int brickCount;

    Paddle paddle;

    World(fix16 w = FIX(800), fix16 h = FIX(600));

    void init();

    void update(fix16 dt, const InputState& input, bool isGameRunning);

    bool addBall(const Ball& ball);
    bool addBrick(const Brick& brick);

    void spawnNewRow();

    const Paddle& getPaddle() const { return paddle; }

    bool isGameLost() const { return gameLost; }

    int getScore() const { return destroyedBricks; }
};
