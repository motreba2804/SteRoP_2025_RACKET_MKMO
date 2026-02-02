#include "World.h"

World::World(fix16 w, fix16 h) : width(w), height(h), ballCount(0), brickCount(0) {}

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

    fix16 pW = FIX(40);
    fix16 pH = FIX(10);
    fix16 startX = (width - pW) / FIX(2);
    fix16 startY = height - FIX(40);

    paddle = Paddle({startX, startY}, {pW, pH});

    addBall(Ball({startX, startY+10}, {FIX(75), FIX(-75)}, FIX(4)));

    for(int r=0; r<4; ++r){
        for(int c=0; c<5; ++c){
            fix16 x = FIX(10 + c*45);
            fix16 y = FIX(30 + r*20);
            addBrick(Brick({x, y}, {FIX(40), FIX(15)}));
        }
    }
}

void World::update(fix16 dt, const InputState& input) {
    paddle.update(input, width, height);

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
        else if(pos.y + r > height) {
             pos.y = height - r;
             ball.setVelocity({vel.x, -vel.y});
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
                break;
            }
        }
    }
}
