#include "World.h"
#include <cstdio>


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

    paddle = Paddle({(width - FIX(100))/FIX(2), height - FIX(50)}, {FIX(100), FIX(20)});

    addBall(Ball({FIX(400), FIX(500)}, {FIX(100), FIX(-250)}, FIX(8)));

    for(int r=0;r<5;++r){
        for(int c=0;c<10;++c){
            fix16 x = FIX(50 + c*60);
            fix16 y = FIX(50 + r*25);
            addBrick(Brick({x,y},{FIX(60), FIX(20)}));
        }
    }
}

void World::update(fix16 dt, const InputState& input) {

    paddle.update(input, width, height);

    for(int i = 0; i < ballCount; ++i) {
        Ball& ball = balls[i];
        if(!ball.isActive()) continue;

        ball.update(dt);
        Vector2D normal;
        Vector2D pos = ball.getPosition();
        fix16 r = ball.getRadius();

        if(pos.x - r < 0) {
            pos.x = r;
            ball.setVelocity({-ball.getVelocity().x, ball.getVelocity().y});
        }
        else if(pos.x + r > width) {
            pos.x = width - r;
            ball.setVelocity({-ball.getVelocity().x, ball.getVelocity().y});
        }
        if(pos.y - r < 0) {
            pos.y = r;
            ball.setVelocity({ball.getVelocity().x, -ball.getVelocity().y});
        }
        else if(pos.y - r > height) {
            ball.setActive(false);
            continue;
        }
        ball.setPosition(pos);

        if (paddle.checkCollision(ball)) {
        }

        for(int j = 0; j < brickCount; ++j) {
            Brick& brick = bricks[j];
            if(!brick.isAlive()) continue;

            if(brick.checkCollision(ball, normal)) {
                ball.bounce(normal);
                // Wypychanie
                Vector2D push = normal * FIX(2);
                ball.setPosition(ball.getPosition() + push);
                brick.setAlive(false);
                break;
            }
        }
    }
}

void World::debugPrint() const {
    printf("---- WORLD DEBUG ----\n");
    for (int i = 0; i < ballCount; ++i) {
        const Ball& b = balls[i];
        Vector2D pos = b.getPosition();
        Vector2D vel = b.getVelocity();
        printf("Ball %d | pos=(%.2f, %.2f) vel=(%.2f, %.2f) alive=%d\n",
               i, TO_FLOAT(pos.x), TO_FLOAT(pos.y), TO_FLOAT(vel.x), TO_FLOAT(vel.y), b.isActive());
    }
    printf("Active Bricks: ");
    int activeCount = 0;
    for (int i = 0; i < brickCount; ++i) {
        if(bricks[i].isAlive()) activeCount++;
    }
    printf("%d / %d\n", activeCount, brickCount);
    printf("---------------------\n");
}