#include "World.h"

World::World(fix16 w, fix16 h) : width(w), height(h), ballCount(0), brickCount(0), destroyedBricks(0), gameLost(false), spawnTimer(0), currentSpawnInterval(FIX(20)), rowsSpawnedCount(0) {}

bool World::addBall(const Ball& ball){
    if(ballCount >= MAX_BALLS) return false;
    balls[ballCount++] = ball;
    return true;
}

bool World::addBrick(const Brick& brick){
    if(brickCount >= MAX_BRICKS) return false;
    bricks[brickCount++] = brick;
    return true;
}

void World::init(){
    ballCount = 0;
    brickCount = 0;
    destroyedBricks = 0;
    gameLost = false;
    spawnTimer = 0;

    currentSpawnInterval = FIX(20.0f);
    rowsSpawnedCount = 0;

    fix16 pW = FIX(40);
    fix16 pH = FIX(10);
    fix16 startX = FIX(120) - pW;
    fix16 startY = height - FIX(40);

    paddle = Paddle({startX, startY-10}, {pW, pH});

    addBall(Ball({startX, startY}, {FIX(75), FIX(-75)}, FIX(4)));

    for(int r=0; r<4; ++r){
        for(int c=0; c<5; ++c){
            fix16 x = FIX(10 + c*45);
            fix16 y = FIX(30 + r*20);
            addBrick(Brick({x, y}, {FIX(40), FIX(15)}));
        }
    }
}

void World::spawnNewRow() {
    fix16 rowHeight = FIX(20);

    for(int i = 0; i < brickCount; ++i) {
        if(bricks[i].isAlive()) {
            Vector2D pos = bricks[i].getPosition();
            pos.y += rowHeight;
            bricks[i].setPosition(pos);
        }
    }

    for(int c = 0; c < 5; ++c) {
        fix16 x = FIX(10 + c*45);
        fix16 y = FIX(30);
        Brick newBrick({x, y}, {FIX(40), FIX(15)});

        bool placed = false;
        for(int i = 0; i < brickCount; ++i) {
            if(!bricks[i].isAlive()) {
                bricks[i] = newBrick;
                placed = true;
                break;
            }
        }
        if(!placed) {
            addBrick(newBrick);
        }
    }
}

void World::update(fix16 dt, const InputState& input, bool isGameRunning) {
    paddle.update(input, width, height);

    if (!isGameRunning) {
        if (ballCount > 0) {
            Ball& b = balls[0];
            Vector2D paddlePos = paddle.getPosition();
            Vector2D paddleSize = paddle.getSize();
            fix16 r = b.getRadius();

            fix16 newX = paddlePos.x;
            fix16 newY = paddlePos.y - (paddleSize.y / FIX(2)) - r - FIX(2) - FIX(10);

            b.setPosition({newX, newY});
        }
        return;
    }

    spawnTimer += dt;
    if (spawnTimer >= currentSpawnInterval) {
        spawnTimer = 0;
        spawnNewRow();
        rowsSpawnedCount++;

        if (rowsSpawnedCount == 1) {
            currentSpawnInterval = FIX(16);
        }
        else if (rowsSpawnedCount == 3) {
            currentSpawnInterval = FIX(12);
        }
        else if (rowsSpawnedCount == 5) {
            currentSpawnInterval = FIX(10);
        }
    }

    fix16 limitY = (height * 3) / 5;

    for(int i = 0; i < brickCount; ++i) {
        if(bricks[i].isAlive()) {
            fix16 brickBottom = bricks[i].getPosition().y + bricks[i].getSize().y;

            if (brickBottom >= limitY) {
                gameLost = true;
            }
        }
    }

    for(int i = 0; i < ballCount; ++i) {
        Ball& ball = balls[i];
        if(!ball.isActive()) continue;

        ball.update(dt);

        Vector2D pos = ball.getPosition();
        fix16 r = ball.getRadius();
        Vector2D vel = ball.getVelocity();

        if(pos.x - r < 0) {
            pos.x = r;
            ball.setVelocity({-vel.x, vel.y});
        }
        else if(pos.x + r > width) {
            pos.x = width - r;
            ball.setVelocity({-vel.x, vel.y});
        }

        if(pos.y - r < 0) {
            pos.y = r;
            ball.setVelocity({vel.x, -vel.y});
        }
        else if(pos.y - r > height) {
             ball.setActive(false);
             gameLost = true;
        }

        ball.setPosition(pos);

        if (paddle.checkCollision(ball)) {
        }

        Vector2D normal;
        for(int j = 0; j < brickCount; ++j) {
            Brick& brick = bricks[j];
            if(!brick.isAlive()) continue;

            if(brick.checkCollision(ball, normal)) {
                ball.bounce(normal);
                Vector2D push = normal * FIX(2);
                ball.setPosition(ball.getPosition() + push);
                brick.setAlive(false);

                destroyedBricks++;
                if (destroyedBricks % 5 == 0) {
                    ball.setVelocity(ball.getVelocity() * FIX(1.1f));
                }
                break;
            }
        }
    }
}
