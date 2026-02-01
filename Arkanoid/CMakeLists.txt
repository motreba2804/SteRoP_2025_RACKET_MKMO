cmake_minimum_required(VERSION 3.27)
project(Arkanoid)

set(CMAKE_CXX_STANDARD 17)

set(SFML_DIR "C:/SFML-2.5.1/lib/cmake/SFML")
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

set(SOURCES
        src/Ball.cpp
        inc/Ball.h
        src/Brick.cpp
        inc/Brick.h
        src/World.cpp
        inc/World.h
        inc/Vector2D.h
        src/main.cpp
        inc/FixedPoint.h
        inc/PCRenderer.h
        inc/InputState.h
        inc/Paddle.h
        src/Paddle.cpp


)

add_executable(Arkanoid ${SOURCES})

target_include_directories(Arkanoid PRIVATE inc C:/SFML/include)
target_link_libraries(Arkanoid sfml-graphics sfml-window sfml-system)