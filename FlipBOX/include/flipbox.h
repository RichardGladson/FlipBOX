// =============================================================================
//  flipbox.h  —  the contract between main.cpp (host) and every game .cpp
// =============================================================================
//
//  HOW TO WRITE A GAME FILE
//  ------------------------
//  1. Create  <yourgame>.cpp  next to main.cpp and  #include "flipbox.h".
//  2. Put ALL of the game's state and helper functions in an anonymous
//     namespace (or mark them `static`).  The only external symbol a game
//     exports is the one FB_DEFINE_GAME() creates.  This is what lets any
//     number of games link together without name collisions.
//  3. Provide these callbacks:
//
//       void     begin();                         // REQUIRED  entering the game; reset ALL state here
//       FbStatus update(float dt, const FbInput&);// REQUIRED  advance simulation; return FB_EXIT to quit
//       void     render(U8G2& g);                 // REQUIRED  draw only; no state changes
//       void     end();                           // OPTIONAL  pass nullptr if unused
//
//  4. Register it at the bottom of the game file:
//
//       FB_DEFINE_GAME(my_game, "MY GAME", begin, update, render, nullptr);
//
//  5. Add ONE line, X(my_game), to FLIPBOX_GAME_LIST in main.cpp.  Done.
//
//  RULES THE HOST RELIES ON
//  ------------------------
//   * update()/render() must never block: no delay(), no busy-waits.
//   * dt is seconds since the previous frame, already clamped by the host.
//     Integrate with it; do not assume a fixed frame rate.
//   * The host calls clearBuffer() before render() and sendBuffer() after it.
//     It also resets draw colour / font mode / bitmap mode / clip window to
//     U8g2 defaults before every render() call, so you start from a known state.
//   * Screen is FB_SCREEN_W x FB_SCREEN_H, monochrome, origin top-left.
//   * Hold FB_BTN_A + FB_BTN_B for about a second and the host returns to the
//     launcher regardless of what the game is doing.  Returning FB_EXIT from
//     update() does the same thing.
//   * Buttons held at the moment a game starts are hidden from that game until
//     they are released, so the press that launched you is never seen as input.
//   * Static storage for all games is linked in simultaneously; keep large
//     arrays modest (only one game runs at a time, but all of them occupy RAM).
//
// =============================================================================
#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

// ---- Display geometry -------------------------------------------------------
constexpr int FB_SCREEN_W = 128;
constexpr int FB_SCREEN_H = 64;

// ---- Buttons (bit masks) ----------------------------------------------------
constexpr uint8_t FB_BTN_UP    = 1u << 0;
constexpr uint8_t FB_BTN_DOWN  = 1u << 1;
constexpr uint8_t FB_BTN_LEFT  = 1u << 2;
constexpr uint8_t FB_BTN_RIGHT = 1u << 3;
constexpr uint8_t FB_BTN_A     = 1u << 4;
constexpr uint8_t FB_BTN_B     = 1u << 5;
constexpr uint8_t FB_BTN_COUNT = 6;

// Per-frame input snapshot handed to update().
struct FbInput {
    uint8_t held;      // buttons currently down
    uint8_t pressed;   // buttons that went down this frame   (edge)
    uint8_t released;  // buttons that went up   this frame   (edge)
};

enum FbStatus : uint8_t {
    FB_CONTINUE = 0,
    FB_EXIT     = 1     // return to the launcher
};

// ---- Game descriptor --------------------------------------------------------
struct FbGame {
    const char* name;                              // launcher title, <= 19 chars
    void      (*begin) ();                         // required
    FbStatus  (*update)(float dt, const FbInput&); // required
    void      (*render)(U8G2& g);                  // required
    void      (*end)   ();                         // optional (nullptr allowed)
};

// Defines the externally visible descriptor  fbgame_<id>.
// Use at file scope, followed by a semicolon:
//     FB_DEFINE_GAME(f1, "F1 RACING", begin, update, render, nullptr);
#define FB_DEFINE_GAME(id, title, beginFn, updateFn, renderFn, endFn)          \
    static_assert(sizeof(title) <= 20, "FB game title must be <= 19 chars");   \
    extern const FbGame fbgame_##id;                                           \
    const FbGame fbgame_##id = { title, beginFn, updateFn, renderFn, endFn }