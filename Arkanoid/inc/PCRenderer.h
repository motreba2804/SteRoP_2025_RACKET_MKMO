#pragma once
#include <SFML/Graphics.hpp>
#include "World.h"
#include "FixedPoint.h" // Potrzebne do TO_FLOAT

class PCRenderer {
    sf::RenderWindow& window;
public:
    PCRenderer(sf::RenderWindow& w) : window(w) {}

    void draw(const World& world) {
        //RYSOWANIE CEGIEŁ

        for(int i=0; i<world.brickCount; ++i){
            const Brick& b = world.bricks[i];
            if(!b.isAlive()) continue;

            sf::RectangleShape rect;
            rect.setPosition(TO_FLOAT(b.getPosition().x), TO_FLOAT(b.getPosition().y));
            rect.setSize({TO_FLOAT(b.getSize().x), TO_FLOAT(b.getSize().y)});
            rect.setFillColor(sf::Color::Blue);
            window.draw(rect);
        }

        //RYSOWANIE PIŁEK

        for(int i=0; i<world.ballCount; ++i){
            const Ball& ball = world.balls[i];
            if(!ball.isActive()) continue;

            float r = TO_FLOAT(ball.getRadius());
            sf::CircleShape circle(r);

            circle.setOrigin(r, r);
            circle.setPosition(TO_FLOAT(ball.getPosition().x), TO_FLOAT(ball.getPosition().y));
            circle.setFillColor(sf::Color::White);
            window.draw(circle);
        }

        //RYSOWANIE PALETKI
        const Paddle& p = world.paddle; // Dostęp publiczny w World

        Vector2D pSize = p.getSize();
        Vector2D pPos = p.getPosition();
        float width = TO_FLOAT(pSize.x);
        float height = TO_FLOAT(pSize.y);

        sf::RectangleShape pShape;
        pShape.setSize({width, height});


        pShape.setOrigin(width / 2.0f, height / 2.0f);

        pShape.setPosition(TO_FLOAT(pPos.x), TO_FLOAT(pPos.y));

        // SFML używa stopni (0-360), nasza fizyka radianów (0 - 2PI)

        float angleRad = TO_FLOAT(p.getRotation());
        float angleDeg = angleRad * 57.29578f;

        pShape.setRotation(angleDeg);

        pShape.setFillColor(sf::Color::Red);

        pShape.setOutlineThickness(1);
        pShape.setOutlineColor(sf::Color::White);

        window.draw(pShape);
    }
};