#include <SFML/Graphics.hpp>
#include "World.h"

#include "InputState.h"
#include "PCRenderer.h"
#include "FixedPoint.h"
#include <cstdio> // Do printf

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Arkanoid STM32 Sim - DEBUG MODE");
    window.setFramerateLimit(60);

    World world(FIX(800), FIX(600));
    world.init();

    PCRenderer renderer(window);

    int debugCounter = 0;

    float currentRotation = 0.0f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::MouseWheelScrolled) {
                currentRotation += event.mouseWheelScroll.delta * 0.1f;
            }
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i pos = sf::Mouse::getPosition(window);
                    float xMouse = (float)pos.x;
                    float yMouse = (float)pos.y;
                    Ball newBall({FIX(xMouse), FIX(yMouse - 20.0f)},{FIX(0.0f), FIX(-100.0f)}, FIX(10));
                    world.addBall(newBall);
                }
            }
        }

        sf::Vector2i mousePos = sf::Mouse::getPosition(window);

        float ratioX = (float)mousePos.x / 800.0f;
        float ratioY = (float)mousePos.y / 600.0f;

        InputState input;
        input.normX = FIX(ratioX);
        input.normY = FIX(ratioY);
        input.rotation = FIX(currentRotation);
        input.isShooting = sf::Mouse::isButtonPressed(sf::Mouse::Left);


        world.update(FIX(0.016f), input);

        window.clear(sf::Color::Black);
        renderer.draw(world);
        window.display();

        if (++debugCounter % 60 == 0) {
            printf("INPUT: Mouse=(%d, %d) Ratio=(%.2f, %.2f)\n", mousePos.x, mousePos.y, ratioX, ratioY);

            Vector2D pPos = world.paddle.getPosition();
            printf("PADDLE: Pos=(%.2f, %.2f) Angle=%.2f\n\n",
                   TO_FLOAT(pPos.x), TO_FLOAT(pPos.y), TO_FLOAT(world.paddle.getRotation()));
        }
    }
    return 0;
}