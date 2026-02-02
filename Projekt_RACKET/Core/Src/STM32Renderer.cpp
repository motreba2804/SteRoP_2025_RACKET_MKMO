#include "STM32Renderer.h"
#include <cmath>
#include <algorithm>

extern "C" {
#include "stm32f429i_discovery_lcd.h"
}

STM32Renderer::STM32Renderer(int w, int h) : screenWidth(w), screenHeight(h) {}

struct PointF { float x, y; };

void STM32Renderer::draw(const World& world) {
	int lineY = (screenHeight * 3) / 5;

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawHLine(0, lineY, screenWidth);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	for(int i = 0; i < world.brickCount; ++i) {
		const Brick& b = world.bricks[i];
		if(!b.isAlive()) continue;

		int x = (int)TO_FLOAT(b.getPosition().x);
		int y = (int)TO_FLOAT(b.getPosition().y);
		int w = (int)TO_FLOAT(b.getSize().x);
		int h = (int)TO_FLOAT(b.getSize().y);

		if (x >= screenWidth || y >= screenHeight || x + w <= 0 || y + h <= 0) continue;
		if (x < 0) { w += x; x = 0; }
		if (y < 0) { h += y; y = 0; }
		if (x + w > screenWidth) w = screenWidth - x;
		if (y + h > screenHeight) h = screenHeight - y;

		if (w > 0 && h > 0) BSP_LCD_FillRect(x, y, w, h);
	}

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	for(int i = 0; i < world.ballCount; ++i) {
		const Ball& ball = world.balls[i];
		if(!ball.isActive()) continue;

		int cx = (int)TO_FLOAT(ball.getPosition().x);
		int cy = (int)TO_FLOAT(ball.getPosition().y);
		int r = (int)TO_FLOAT(ball.getRadius());

		for (int dy = -r; dy <= r; dy++) {
			int posY = cy + dy;

			if (posY < 0 || posY >= screenHeight) continue;

			int halfWidth = (int)std::sqrt(r*r - dy*dy);
			int startX = cx - halfWidth;
			int width = 2 * halfWidth;

			if (startX < 0) {
				width += startX;
				startX = 0;
			}

			if (startX + width > screenWidth) {
				width = screenWidth - startX;
			}

			if (width > 0) {
				BSP_LCD_DrawHLine(startX, posY, width);
			}
		}
	}

	const Paddle& p = world.paddle;
	Vector2D center = p.getPosition();
	Vector2D size = p.getSize();
	fix16 angle = p.getRotation();

	float cx = TO_FLOAT(center.x);
	float cy = TO_FLOAT(center.y);
	float w2 = TO_FLOAT(size.x) / 2.0f;
	float h2 = TO_FLOAT(size.y) / 2.0f;
	float angRad = TO_FLOAT(angle);

	float c = cosf(angRad);
	float s = sinf(angRad);

	PointF pts[4];
	auto rotateX = [&](float x, float y) { return cx + (x * c - y * s); };
	auto rotateY = [&](float x, float y) { return cy + (x * s + y * c); };

	pts[0] = {rotateX(-w2, -h2), rotateY(-w2, -h2)};
	pts[1] = {rotateX( w2, -h2), rotateY( w2, -h2)};
	pts[2] = {rotateX( w2, h2), rotateY( w2, h2)};
	pts[3] = {rotateX(-w2, h2), rotateY(-w2, h2)};

	int minY = screenHeight, maxY = 0;
	for(int i=0; i<4; i++) {
		if(pts[i].y < minY) minY = (int)pts[i].y;
		if(pts[i].y > maxY) maxY = (int)pts[i].y;
	}

	if(minY < 0) minY = 0;
	if(maxY >= screenHeight) maxY = screenHeight - 1;

	BSP_LCD_SetTextColor(LCD_COLOR_RED);

	for(int y = minY; y <= maxY; y++) {
		float x1 = screenWidth + 1, x2 = -1;
		for(int i=0; i<4; i++) {
			PointF p1 = pts[i];
			PointF p2 = pts[(i+1)%4];
			if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
				float t = (y - p1.y) / (p2.y - p1.y);
				float intersectX = p1.x + t * (p2.x - p1.x);
				if(intersectX < x1) x1 = intersectX;
				if(intersectX > x2) x2 = intersectX;
			}
		}
		if(x2 >= x1) {
			int ix1 = (int)x1;
			int ix2 = (int)x2;
			if(ix1 < 0) ix1 = 0;
			if(ix2 >= screenWidth) ix2 = screenWidth - 1;
			if(ix2 >= ix1) BSP_LCD_DrawHLine(ix1, y, ix2 - ix1 + 1);
		}
	}
}
