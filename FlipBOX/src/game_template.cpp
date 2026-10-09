// game_template.cpp  —  copy, rename, and fill in.  See flipbox.h for the rules.
#include "flipbox.h"

namespace {                      // everything private to this game lives here

float boxX;                      // example state: a box that you steer left/right
float boxV;

void tpl_begin()                 // called every time the game is launched
{
    boxX = FB_SCREEN_W / 2.0f;
    boxV = 0.0f;
}

FbStatus tpl_update(float dt, const FbInput& in)
{
    // To quit from game logic: return FB_EXIT.  (A+B held ~1 s always quits, so
    // avoid making a lone A or B tap quit unless that is really what you want.)

    float target = 0.0f;
    if (in.held & FB_BTN_LEFT)  target -= 60.0f;   // px/s
    if (in.held & FB_BTN_RIGHT) target += 60.0f;

    boxV += (target - boxV) * (1.0f - expf(-12.0f * dt));   // smooth, frame-rate independent
    boxX += boxV * dt;
    if (boxX < 4.0f)                   boxX = 4.0f;
    if (boxX > FB_SCREEN_W - 4.0f)     boxX = FB_SCREEN_W - 4.0f;

    return FB_CONTINUE;
}

void tpl_render(U8G2& g)         // draw only; host clears before and sends after
{
    g.drawFrame(0, 0, FB_SCREEN_W, FB_SCREEN_H);
    g.drawBox((int)boxX - 4, FB_SCREEN_H - 12, 8, 8);
}

} // namespace

// id (used in main.cpp's list), launcher title (<= 19 chars), callbacks, optional end()
FB_DEFINE_GAME(template_game, "TEMPLATE", tpl_begin, tpl_update, tpl_render, nullptr);
