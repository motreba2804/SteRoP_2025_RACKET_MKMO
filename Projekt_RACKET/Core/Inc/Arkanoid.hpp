#ifndef ARKANOID_HPP
#define ARKANOID_HPP

#ifdef __cplusplus

class Arkanoid {
private:
    static const int BRICK_COUNT = 3;

    int bx[BRICK_COUNT];
    int by[BRICK_COUNT];
    int balive[BRICK_COUNT];

    int plat_x;
    int plat_y;
    int plat_w;
    int plat_h;
    int old_plat_x;

    int ball_x, ball_y;
    int old_ball_x, old_ball_y;
    int speed_x, speed_y;
    int size;

    int max_w, max_h;

public:
    Arkanoid();
    void init();

    void updateAndRender(float roll_input);
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

void Game_Init_CPP(void);
void Game_Loop_CPP(float roll_val);

#ifdef __cplusplus
}
#endif

#endif
