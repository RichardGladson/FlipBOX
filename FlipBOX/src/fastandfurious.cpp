// src/Game2.cpp  --  Fast and Furious  (flipBOX)
//
// Straight-line 1050 m drag race against an independent AI opponent.
// Choose the Nissan Skyline GT-R (higher acceleration) or the Dodge Charger
// (higher top speed), time your launch, and spend two one-shot N2O charges
// (+30 km/h for 3 s each) plus a one-time POWER SHIFT (+12 km/h) wisely.
//
// Controls:
//   Title / menus : tap SELECT = confirm, hold SELECT 1 s = exit to launcher
//   Car select    : LEFT / RIGHT = choose car, tap SELECT = race
//   Countdown     : wait for GO!  (touching UP before GO = false start)
//   Race          : UP (hold) = throttle, DOWN = brake,
//                   LEFT = N2O #1, RIGHT = N2O #2, tap SELECT = POWER SHIFT
//   Results       : LEFT / RIGHT = choose, tap SELECT = restart / exit

// ----- FLIPBOX GAME CONTRACT v1 : BEGIN ------------------------------------
#include <Arduino.h>
#include <U8g2lib.h>

namespace fb {

constexpr uint8_t SCREEN_W = 128;
constexpr uint8_t SCREEN_H = 64;

// Button masks (used in Input::held / pressed / released)
constexpr uint8_t KEY_UP     = 1u << 0;
constexpr uint8_t KEY_DOWN   = 1u << 1;
constexpr uint8_t KEY_LEFT   = 1u << 2;
constexpr uint8_t KEY_RIGHT  = 1u << 3;
constexpr uint8_t KEY_SELECT = 1u << 4;

// Debounced input, identical for every game. Buttons that were still held when
// the game was entered stay invisible here until they are physically released.
struct Input {
  uint8_t held;      // currently down
  uint8_t pressed;   // went down  since the previous update()
  uint8_t released;  // went up    since the previous update()
};

// Timing, refreshed before every update().
struct Frame {
  uint32_t nowMs;    // millis()
  uint32_t dtUs;     // microseconds since previous frame (clamped, never 0 after frame 0)
  uint32_t index;    // frames since the game was entered (0 on the first one)
  uint16_t fps;      // measured frames/second, refreshed twice per second
};

enum Status : uint8_t { GAME_RUN = 0, GAME_EXIT = 1 };

// What every game exports.
struct Game {
  void   (*init)();                                        // once on entry (nullptr ok)
  Status (*update)(const Input& in, const Frame& f);       // return GAME_EXIT -> back to menu
  void   (*render)(U8G2& d);                               // draw only; buffer is cleared before
  void   (*shutdown)();                                    // once on exit (nullptr ok)
};

}  // namespace fb

// Put as the last line of a game file.  'id' is the symbol used in FB_GAME_LIST.
#define FB_GAME(id, initFn, updateFn, renderFn, shutdownFn)                    \
  extern const fb::Game fbGame_##id;                                           \
  const fb::Game fbGame_##id = { initFn, updateFn, renderFn, shutdownFn }
// ----- FLIPBOX GAME CONTRACT v1 : END --------------------------------------

namespace {

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------
// Physics is integer fixed-point: speeds in Q16.16 m/s, distances in Q16.16 m.
constexpr int32_t kmh16(int32_t kmh) { return (int32_t)(((int64_t)kmh * 5 * 65536) / 18); }
constexpr int32_t ms16(int32_t num, int32_t den) { return (int32_t)(((int64_t)num * 65536) / den); }

constexpr int32_t kRaceDist16    = 1050L << 16;     // race length: 1050 m (was 402 m; gives ~2x the race time)

constexpr int32_t kSkyTop16      = kmh16(205);      // Skyline GT-R top speed
constexpr int32_t kSkyAcc16      = ms16(116, 10);   // Skyline accel 11.6 m/s^2
constexpr int32_t kChgTop16      = kmh16(236);      // Charger top speed
constexpr int32_t kChgAcc16      = ms16(92, 10);    // Charger accel  9.2 m/s^2

constexpr int32_t kOppTop16      = kmh16(216);      // opponent top speed
constexpr int32_t kOppAcc16      = ms16(104, 10);   // opponent accel 10.4 m/s^2
constexpr uint32_t kOppReactMin  = 250000;          // opponent reaction: min (us)
constexpr uint32_t kOppReactSpr  = 350000;          //   ... plus random spread (us)
constexpr int32_t kOppN2OAt16    = (int32_t)(((int64_t)kRaceDist16 * 170) / 402);  // same point in the
                                                    //   race as before: ~42% of the track
constexpr int32_t kOppN2OTop16   = kmh16(25);       // opponent N2O: +25 km/h

constexpr int32_t kN2OTop16      = kmh16(30);       // player N2O: +30 km/h per charge
constexpr int32_t kN2OAcc16      = ms16(16, 10);    // +1.6 m/s^2 while a charge burns
constexpr uint32_t kN2ODurUs     = 3000000;         // N2O burn time: 3 s

constexpr int32_t kShiftTop16    = kmh16(12);       // POWER SHIFT: +12 km/h permanent
constexpr uint32_t kShiftBoostUs = 1000000;         //   ... and +20% accel for 1 s

constexpr uint32_t kLaunchPerfectUs = 220000;       // reaction <= this: PERFECT
constexpr uint32_t kLaunchGoodUs    = 450000;       // reaction <= this: GOOD
constexpr uint32_t kBoostPerfectUs  = 1200000;      // PERFECT: x1.30 accel for 1.2 s
constexpr uint32_t kBoostGoodUs     = 800000;       // GOOD:    x1.15 accel for 0.8 s
constexpr uint32_t kFalseStartUs    = 600000;       // false-start throttle lockout

constexpr int32_t kCoast16       = ms16(8, 10);     // 0.8 m/s^2 drag, throttle off
constexpr int32_t kBrake16       = ms16(6, 1);      // 6.0 m/s^2 brake

// Visual scales (Q16.16 px per metre). They only affect drawing, never physics:
constexpr int32_t kPxPerM16      = 65536;           // car-to-car spacing: 1 px per metre
constexpr int32_t kRoadPxPerM16  = 12 * 65536;      // road, dashes, buildings, gantry: 12 px/m
                                                    //   (~700 px/s at 205 km/h)
constexpr int32_t kFinishPxPerM16 = 4 * 65536;      // cars zooming off after the finish: 4 px/m

// Car sprite size
constexpr int32_t kCarW          = 24;
constexpr int32_t kCarH          = 9;

// Chase camera. "camX" = screen x of the PLAYER's front bumper. It slides so
// that both cars stay framed while the race is close, and falls back to the
// player alone when the gap is too big to show both.
constexpr int32_t kCamMinX       = kCarW + 2;       // player fully on screen (left)
constexpr int32_t kCamMaxX       = 128 - 2;         // player fully on screen (right)
constexpr int32_t kCamHome       = 76;              // level race: pair sits mid-screen
constexpr int32_t kCamAlone      = 54;              // opponent out of frame: player here
constexpr int32_t kCamFade       = 40;              // px over which we blend to kCamAlone
constexpr int32_t kCamEase       = 6;               // camera easing rate (1/s)

// Road layout (screen rows). Opponent lane = top, player lane = bottom.
constexpr int16_t kRoadTop       = 30;              // far edge (buildings stand on it)
constexpr int16_t kLaneDiv       = 42;              // dashed centre divider
constexpr int16_t kRoadBot       = 54;              // near edge
constexpr int16_t kOppCarY       = 32;              // opponent sprite top row
constexpr int16_t kPlrCarY       = 44;              // player sprite top row

// Lane divider dashes
constexpr int16_t kDashLen       = 32;              // long dashes: at this scroll speed a short
constexpr int16_t kDashPeriod    = 64;              //   period would appear to run backwards

// Background skyline: two layers (far + near) for density and depth
constexpr int32_t kBldBias       = 40000;           // keeps slot indices positive

constexpr uint32_t kExitHoldUs   = 1000000;         // hold SELECT this long to quit
constexpr uint32_t kCountdownUs  = 2800000;         // READY + 3 + 2 + 1, 0.7 s each

// ---------------------------------------------------------------------------
// Bitmaps (flash). XBM: row-major, LSB = leftmost pixel.
// ---------------------------------------------------------------------------
const uint8_t kBmpSkyline[] = {  // 24 x 9, sleek coupe + rear spoiler
  0, 124, 0, 0, 130, 0, 6, 131, 1, 206, 255, 31, 252, 255, 127,
  255, 255, 255, 135, 255, 240, 120, 0, 15, 120, 0, 15
};
const uint8_t kBmpCharger[] = {  // 24 x 9, boxy muscle car, upright cabin
  0, 255, 0, 128, 129, 1, 128, 129, 1, 248, 255, 63, 254, 255, 127,
  255, 255, 255, 135, 255, 240, 120, 0, 15, 120, 0, 15
};
const uint8_t kBmpFlag[] = { 127, 85, 42, 85, 34, 33, 1 };  // 7 x 7 checkered flag

// ---------------------------------------------------------------------------
// State (ALL runtime state is reset in gameInit()/resetRace())
// ---------------------------------------------------------------------------
enum State : uint8_t { ST_TITLE, ST_SELECT, ST_COUNT, ST_RACE, ST_FINISH, ST_RESULTS };

State    gState;
uint32_t gAnimUs;         // cosmetic clock (blinking, speed lines)
uint32_t gRng;            // xorshift32 PRNG state
uint32_t gSelHoldUs;      // SELECT hold timer (global exit)
uint8_t  gMenuCursor;     // car-select column / results menu cursor

uint8_t  gCar;            // 0 = Skyline, 1 = Charger
uint32_t gCdUs;           // countdown clock
uint32_t gRaceUs;         // race clock, starts at GO
bool     gFalseStart;
uint32_t gLockoutUs;      // false-start throttle lockout remaining

int32_t  gPV16, gPD16;    // player speed (Q16.16 m/s) / distance (Q16.16 m)
bool     gN1Avail, gN2Avail;
uint32_t gN1LeftUs, gN2LeftUs;
bool     gShiftAvail;
uint32_t gShiftBoostUs;
bool     gReactKnown;
uint32_t gReactTxtUs;     // show reaction text until this race time
uint32_t gLaunchBoostUs;
uint16_t gLaunchPct;      // 130 / 115 / 100
uint8_t  gReactGrade;     // 0 = none, 1 = GOOD, 2 = PERFECT
bool     gThrottleVis;    // throttle state, for the exhaust puff in render()
bool     gPFinished;
uint32_t gPTimeUs;

int32_t  gOV16, gOD16;    // opponent speed / distance
uint32_t gOReactUs;
bool     gON2OUsed;
uint32_t gON2OLeftUs;
bool     gOFinished;
uint32_t gOTimeUs;

int32_t  gCamX256;        // camera: player front-bumper screen x, Q24.8 px (cosmetic only)
int32_t  gFinCx;          // finish freeze-frame: camera x ...
int32_t  gFinScroll;      //   ... road scroll (world px at screen x = 0) ...
int32_t  gFinPD16;        //   ... and both cars' distances at the instant the
int32_t  gFinOD16;        //   player crossed the line (background stops here)

uint32_t gFinishWaitUs;   // ST_FINISH deadline for the opponent
bool     gWin;
uint32_t gGapUs;          // abs win/loss gap for the results screen
bool     gNewBest;

uint32_t gBestUs = 0;     // SESSION-PERSISTENT best player time; intentionally
                          // NOT reset in gameInit() (spec 10.2), lives until power-off.

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
uint32_t rnd() {                       // xorshift32
  gRng ^= gRng << 13; gRng ^= gRng >> 17; gRng ^= gRng << 5;
  return gRng;
}

// a = a0 * (1 - v/top)   (classic first-order acceleration curve)
int32_t accelToward16(int32_t v16, int32_t top16, int32_t a016) {
  return a016 - (int32_t)(((int64_t)a016 * v16) / top16);
}

void integrate(int32_t& v16, int32_t& d16, int32_t a16, uint32_t dt16) {
  v16 += (int32_t)(((int64_t)a16 * dt16) >> 16);
  if (v16 < 0) v16 = 0;
  d16 += (int32_t)(((int64_t)v16 * dt16) >> 16);
}

// hand-rolled formatting: no printf float, no truncation warnings, tiny stack
void fmtUInt(char* b, uint32_t v) {           // 0..999
  char t[4]; uint8_t n = 0;
  do { t[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < 3);
  uint8_t i = 0; while (n) b[i++] = t[--n];
  b[i] = 0;
}
void fmtSec1(char* b, uint32_t us) {          // "12.3s" (capped 99.9)
  uint32_t ds = us / 100000UL; if (ds > 999UL) ds = 999UL;
  uint32_t s = ds / 10UL, t = ds % 10UL;
  char* p = b;
  if (s >= 10) *p++ = (char)('0' + s / 10);
  *p++ = (char)('0' + s % 10); *p++ = '.'; *p++ = (char)('0' + t); *p++ = 's'; *p = 0;
}
void fmtSec2(char* b, uint32_t us) {          // "12.34" (capped 99.99)
  uint32_t cs = us / 10000UL; if (cs > 9999UL) cs = 9999UL;
  uint32_t s = cs / 100UL, c = cs % 100UL;
  char* p = b;
  if (s >= 10) *p++ = (char)('0' + s / 10);
  *p++ = (char)('0' + s % 10); *p++ = '.';
  *p++ = (char)('0' + c / 10); *p++ = (char)('0' + c % 10); *p = 0;
}

// Q16.16 metres -> whole screen pixels (floor). kPxPerM16 is Q16.16 px/m, so
// the product is Q32.32 and needs >> 32 (a plain >> 16 leaves a Q16.16 value).
int32_t m16ToPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kPxPerM16) >> 32);
}

// Where the camera WANTS the player's front bumper, given the gap (px) between
// the cars: gap > 0 = opponent ahead, gap < 0 = opponent behind.
int32_t camTargetX(int32_t gap) {
  // Range of player positions that keeps BOTH cars fully on screen.
  int32_t lo = kCamMinX + (gap < 0 ? -gap : 0);
  int32_t hi = kCamMaxX - (gap > 0 ?  gap : 0);
  if (lo > kCamMaxX) lo = kCamMaxX;
  if (hi < kCamMinX) hi = kCamMinX;

  int32_t t = kCamHome - gap / 2;               // centre the pair on screen
  if (t < lo) t = lo;
  if (t > hi) t = hi;

  // Too far apart to frame both: stop chasing the opponent, follow the player.
  const int32_t ag   = gap < 0 ? -gap : gap;
  const int32_t span = kCamMaxX - kCamMinX;     // widest gap that still fits both
  if (ag > span) {
    int32_t w = ag - span;
    if (w > kCamFade) w = kCamFade;
    t = (t * (kCamFade - w) + kCamAlone * w) / kCamFade;
  }
  return t;
}

// Ease the camera toward its target. snap = jump there (race start).
void updateCamera(uint32_t dt16, bool snap) {
  const int32_t gap = m16ToPx(gOD16) - m16ToPx(gPD16);
  const int32_t tgt = camTargetX(gap) << 8;
  if (snap) {
    gCamX256 = tgt;
  } else {
    int32_t k = (int32_t)(dt16 * (uint32_t)kCamEase);   // Q16 fraction of the way
    if (k > 65536) k = 65536;
    gCamX256 += (int32_t)(((int64_t)(tgt - gCamX256) * k) >> 16);
  }
  if (gCamX256 < (kCamMinX << 8)) gCamX256 = kCamMinX << 8;   // player never leaves
  if (gCamX256 > (kCamMaxX << 8)) gCamX256 = kCamMaxX << 8;   // the screen
}

// Same conversion for the (much faster) road / scenery scale.
int32_t m16ToRoadPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kRoadPxPerM16) >> 32);
}

// ...and for the cars' zoom-off after the finish line.
int32_t m16ToFinishPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kFinishPxPerM16) >> 32);
}

int32_t camX() { return (gCamX256 + 128) >> 8; }

void resetRace() {
  gCdUs = 0; gRaceUs = 0;
  gFalseStart = false; gLockoutUs = 0;
  gPV16 = 0; gPD16 = 0;
  gN1Avail = true; gN2Avail = true; gN1LeftUs = 0; gN2LeftUs = 0;
  gShiftAvail = true; gShiftBoostUs = 0;
  gReactKnown = false; gReactTxtUs = 0; gLaunchBoostUs = 0; gLaunchPct = 100;
  gReactGrade = 0; gThrottleVis = false;
  gPFinished = false; gPTimeUs = 0;
  gOV16 = 0; gOD16 = 0;
  gOReactUs = kOppReactMin + (rnd() % kOppReactSpr);
  gON2OUsed = false; gON2OLeftUs = 0;
  gOFinished = false; gOTimeUs = 0;
  gFinishWaitUs = 0; gWin = false; gGapUs = 0; gNewBest = false;
  updateCamera(0, true);                      // gOD16 = gPD16 = 0 here: level start
  gFinCx = camX(); gFinScroll = 0; gFinPD16 = 0; gFinOD16 = 0;
}

// One physics step for both cars. 'racing' = player throttle is live
// (false while the player car coasts out past the finish line).
void stepRace(const fb::Input& in, uint32_t dt16, uint32_t dt, bool racing) {
  if (gN1LeftUs      > dt) gN1LeftUs      -= dt; else gN1LeftUs      = 0;
  if (gN2LeftUs      > dt) gN2LeftUs      -= dt; else gN2LeftUs      = 0;
  if (gShiftBoostUs  > dt) gShiftBoostUs  -= dt; else gShiftBoostUs  = 0;
  if (gLaunchBoostUs > dt) gLaunchBoostUs -= dt; else gLaunchBoostUs = 0;

  // ---- player ----
  const int32_t nAct = (gN1LeftUs ? 1 : 0) + (gN2LeftUs ? 1 : 0);
  const int32_t top16 = (gCar == 0 ? kSkyTop16 : kChgTop16)
                      + nAct * kN2OTop16 + (gShiftAvail ? 0 : kShiftTop16);
  const bool throttle = racing && (in.held & fb::KEY_UP) && gLockoutUs == 0;
  gThrottleVis = throttle;
  int32_t a16;
  if (throttle) {
    a16 = accelToward16(gPV16, top16, gCar == 0 ? kSkyAcc16 : kChgAcc16);
    a16 += nAct * kN2OAcc16;
    if (a16 > 0) {
      if (gLaunchBoostUs) a16 = (int32_t)(((int64_t)a16 * gLaunchPct) / 100);
      if (gShiftBoostUs)  a16 = (a16 * 6) / 5;
    }
    if (in.held & fb::KEY_DOWN) a16 -= kBrake16;
  } else {
    a16 = -kCoast16;
    if (in.held & fb::KEY_DOWN) a16 -= kBrake16;
  }
  integrate(gPV16, gPD16, a16, dt16);

  // ---- opponent: fully independent model, never reads player state ----
  if (!gOFinished) {
    if (!gON2OUsed && gOD16 >= kOppN2OAt16) { gON2OUsed = true; gON2OLeftUs = kN2ODurUs; }
    const int32_t oTop16 = kOppTop16 + (gON2OLeftUs ? kOppN2OTop16 : 0);
    int32_t oa16 = 0;
    if (gRaceUs >= gOReactUs) {
      oa16 = accelToward16(gOV16, oTop16, kOppAcc16);
      if (gON2OLeftUs) oa16 += kN2OAcc16;
    }
    integrate(gOV16, gOD16, oa16, dt16);
    if (gOD16 >= kRaceDist16) { gOFinished = true; gOTimeUs = gRaceUs; }
  } else {
    integrate(gOV16, gOD16, -kCoast16, dt16);   // opponent coasts out
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void gameInit() {
  gRng = micros() ^ 0x9E3779B9UL;      // micros() allowed here for seeding (spec 7.4)
  if (gRng == 0) gRng = 0x12345678UL;
  gState = ST_TITLE;
  gAnimUs = 0; gSelHoldUs = 0; gMenuCursor = 0; gCar = 0;
  resetRace();
}

fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  const uint32_t dt   = f.dtUs;
  const uint32_t dt16 = (dt << 16) / 1000000UL;   // Q16.16 seconds
  gAnimUs += dt;

  // Global exit: hold SELECT for 1 s from anywhere (spec 10.3).
  const uint32_t selBefore = gSelHoldUs;
  if (in.held & fb::KEY_SELECT) gSelHoldUs += dt; else gSelHoldUs = 0;
  const bool selTap = (in.released & fb::KEY_SELECT) && (selBefore < kExitHoldUs);
  if (gSelHoldUs >= kExitHoldUs) return fb::GAME_EXIT;

  switch (gState) {

    case ST_TITLE:
      if (selTap) { gState = ST_SELECT; gMenuCursor = gCar; }
      break;

    case ST_SELECT:
      if ((in.pressed & fb::KEY_LEFT) || (in.pressed & fb::KEY_RIGHT)) gMenuCursor ^= 1;
      if (selTap) { gCar = gMenuCursor; resetRace(); gState = ST_COUNT; }
      break;

    case ST_COUNT:
      gCdUs += dt;
      if (in.held & fb::KEY_UP) gFalseStart = true;   // jumped the gun
      if (gCdUs >= kCountdownUs) {
        gState  = ST_RACE;
        gRaceUs = 0;
        gLockoutUs = gFalseStart ? kFalseStartUs : 0;
      }
      break;

    case ST_RACE: {
      gRaceUs += dt;
      if (gLockoutUs > dt) gLockoutUs -= dt; else gLockoutUs = 0;

      // reaction-based launch boost (a fresh UP edge after GO)
      if (!gReactKnown) {
        if (gLockoutUs == 0 && (in.pressed & fb::KEY_UP)) {
          gReactKnown = true;
          gReactTxtUs = gRaceUs + 1500000UL;
          if (gRaceUs <= kLaunchPerfectUs) {
            gLaunchBoostUs = kBoostPerfectUs; gLaunchPct = 130; gReactGrade = 2;
          } else if (gRaceUs <= kLaunchGoodUs) {
            gLaunchBoostUs = kBoostGoodUs;    gLaunchPct = 115; gReactGrade = 1;
          } else {
            gLaunchPct = 100; gReactGrade = 0;
          }
        } else if (gRaceUs > 1200000UL) {
          gReactKnown = true; gLaunchPct = 100;
        }
      }

      // two independent one-shot N2O charges
      if ((in.pressed & fb::KEY_LEFT)  && gN1Avail) { gN1Avail = false; gN1LeftUs = kN2ODurUs; }
      if ((in.pressed & fb::KEY_RIGHT) && gN2Avail) { gN2Avail = false; gN2LeftUs = kN2ODurUs; }
      // one-time POWER SHIFT
      if ((in.pressed & fb::KEY_SELECT) && gShiftAvail) { gShiftAvail = false; gShiftBoostUs = kShiftBoostUs; }

      stepRace(in, dt16, dt, true);

      if (gPD16 >= kRaceDist16) {
        gPFinished = true; gPTimeUs = gRaceUs;
        gFinishWaitUs = gRaceUs + 2500000UL;
        gFinCx = camX();                          // freeze the background right here:
        gFinPD16 = gPD16; gFinOD16 = gOD16;       // road, buildings and gantry stop, the
        gFinScroll = m16ToRoadPx(gPD16) - gFinCx; // cars keep going
        gState = ST_FINISH;
      }
    } break;

    case ST_FINISH: {
      gRaceUs += dt;
      stepRace(in, dt16, dt, false);
      if (gOFinished || gRaceUs >= gFinishWaitUs) {
        if (!gOFinished) {
          // opponent did not cross within 2.5 s: project its remaining time
          if (gOV16 > (1 << 14))
            gOTimeUs = gRaceUs + (uint32_t)(((int64_t)(kRaceDist16 - gOD16) * 1000000LL) / gOV16);
          else
            gOTimeUs = gRaceUs + 9900000UL;
          gOFinished = true;
        }
        gWin    = gPTimeUs < gOTimeUs;
        gGapUs  = gWin ? (gOTimeUs - gPTimeUs) : (gPTimeUs - gOTimeUs);
        gNewBest = (gBestUs == 0 || gPTimeUs < gBestUs);
        if (gNewBest) gBestUs = gPTimeUs;
        gMenuCursor = 0;
        gState = ST_RESULTS;
      }
    } break;

    case ST_RESULTS:
      if ((in.pressed & fb::KEY_LEFT) || (in.pressed & fb::KEY_RIGHT)) gMenuCursor ^= 1;
      if (selTap) {
        if (gMenuCursor == 0) { gState = ST_SELECT; gMenuCursor = gCar; }
        else return fb::GAME_EXIT;
      }
      break;
  }
  if (gState == ST_COUNT || gState == ST_RACE)   // camera is frozen once the race is over
    updateCamera(dt16, false);
  return fb::GAME_RUN;
}

// ---------------------------------------------------------------------------
// Render helpers (draw only; never touch gameplay state)
// ---------------------------------------------------------------------------
void drawCentered(U8G2& d, int16_t base, const char* s) {
  const int16_t w = (int16_t)d.getStrWidth(s);
  d.drawStr((int16_t)((fb::SCREEN_W - w) / 2), base, s);
}

// centered text on a dark backing box so it never blends into the scene
void banner(U8G2& d, int16_t base, const char* s) {
  const int16_t w = (int16_t)d.getStrWidth(s);
  const int16_t x = (int16_t)((fb::SCREEN_W - w) / 2);
  d.setDrawColor(0);
  d.drawBox((u8g2_uint_t)(x - 2), (u8g2_uint_t)(base - 11), (u8g2_uint_t)(w + 4), 13);
  d.setDrawColor(1);
  d.drawStr(x, base, s);
}

// u8g2_uint_t is unsigned, so anything that may start left of x = 0 must be
// clipped by hand. These take signed coordinates and clip to the 128x64 screen.
void clipPx(U8G2& d, int32_t x, int32_t y) {
  if (x < 0 || x >= fb::SCREEN_W || y < 0 || y >= fb::SCREEN_H) return;
  d.drawPixel((u8g2_uint_t)x, (u8g2_uint_t)y);
}
void clipH(U8G2& d, int32_t x0, int32_t x1, int32_t y) {      // inclusive x0..x1
  if (y < 0 || y >= fb::SCREEN_H || x1 < 0 || x0 >= fb::SCREEN_W) return;
  if (x0 < 0) x0 = 0;
  if (x1 > fb::SCREEN_W - 1) x1 = fb::SCREEN_W - 1;
  d.drawHLine((u8g2_uint_t)x0, (u8g2_uint_t)y, (u8g2_uint_t)(x1 - x0 + 1));
}
void clipV(U8G2& d, int32_t x, int32_t y0, int32_t y1) {      // inclusive y0..y1
  if (x < 0 || x >= fb::SCREEN_W || y1 < 0 || y0 >= fb::SCREEN_H) return;
  if (y0 < 0) y0 = 0;
  if (y1 > fb::SCREEN_H - 1) y1 = fb::SCREEN_H - 1;
  d.drawVLine((u8g2_uint_t)x, (u8g2_uint_t)y0, (u8g2_uint_t)(y1 - y0 + 1));
}

void clipBox(U8G2& d, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {   // inclusive
  if (x1 < x0 || y1 < y0) return;
  for (int32_t y = y0; y <= y1; ++y) clipH(d, x0, x1, y);
}

// A car sprite that may hang off either screen edge.
void drawCar(U8G2& d, int32_t x, int32_t y, const uint8_t* bmp) {
  if (x + kCarW <= 0 || x >= fb::SCREEN_W) return;
  if (x >= 0 && x + kCarW <= fb::SCREEN_W) {                  // fully visible: fast path
    d.drawXBM((u8g2_uint_t)x, (u8g2_uint_t)y, kCarW, kCarH, bmp);
    return;
  }
  for (int32_t r = 0; r < kCarH; ++r)                         // partly visible: per pixel
    for (int32_t c = 0; c < kCarW; ++c)
      if (bmp[r * (kCarW / 8) + (c >> 3)] & (1u << (c & 7)))
        clipPx(d, x + c, y + r);
}

// world-anchored horizontal segments (lane dashes), clipped manually.
// scrollPx = world pixel that sits at screen x = 0.
void scrollSegs(U8G2& d, int16_t y, int16_t segLen, int16_t period, int32_t scrollPx) {
  int32_t xs = (-scrollPx) % period;
  if (xs < 0) xs += period;
  xs -= period;
  for (; xs < fb::SCREEN_W; xs += period)
    clipH(d, xs, xs + segLen - 1, y);
}

uint32_t hash32(uint32_t x) {                 // stateless: same slot -> same building
  x ^= x >> 16; x *= 0x7FEB352DUL;
  x ^= x >> 15; x *= 0x846CA68BUL;
  x ^= x >> 16;
  return x;
}

// One building, OUTLINE ONLY. Its bottom row is the road's far edge.
// hideBehind: blank the inside first (background colour, not a fill) so the
// far layer does not show through a near building.
void drawBuilding(U8G2& d, int32_t x, int32_t w, int32_t h, uint32_t style, bool hideBehind) {
  const int32_t top = kRoadTop + 1 - h;       // rows top .. kRoadTop
  const int32_t r   = x + w - 1;
  if (hideBehind) {
    d.setDrawColor(0);
    clipBox(d, x + 1, top + 1, r - 1, kRoadTop - 1);
    d.setDrawColor(1);
  }
  clipV(d, x, top, kRoadTop - 1);
  clipV(d, r, top, kRoadTop - 1);
  switch (style) {
    case 1:                                   // stepped roof
      clipH(d, x, r, top);
      clipV(d, x + 3, top - 3, top - 1);
      clipV(d, r - 3, top - 3, top - 1);
      clipH(d, x + 3, r - 3, top - 3);
      break;
    case 2:                                   // flat roof + antenna
      clipH(d, x, r, top);
      clipV(d, x + w / 2, top - 4, top - 1);
      break;
    case 3: {                                 // pitched roof
      const int32_t half = w / 2;
      for (int32_t i = 0; i <= half; ++i) {
        const int32_t dy = (i * 3) / half;
        clipPx(d, x + i, top - dy);
        clipPx(d, r - i, top - dy);
      }
    } break;
    default:                                  // flat roof
      clipH(d, x, r, top);
      break;
  }
}

// One endless procedural layer. scrollPx = layer pixel at screen x = 0.
// pitch: slot width; wMin/wSpan: building width; hMin/hSpan: building height.
void drawSkylineLayer(U8G2& d, int32_t scrollPx, int32_t pitch, int32_t wMin, int32_t wSpan,
                      int32_t hMin, int32_t hSpan, uint32_t salt, bool near) {
  const int32_t base = scrollPx + kBldBias;
  int32_t slot = base / pitch;
  int32_t sx   = slot * pitch - base;         // screen x of this slot, in (-pitch, 0]
  for (; sx < fb::SCREEN_W; sx += pitch, ++slot) {
    const uint32_t h = hash32((uint32_t)slot + salt);
    const int32_t  w = wMin + (int32_t)(h % (uint32_t)wSpan);
    const int32_t  x = sx + (int32_t)((h >> 8) % (uint32_t)(pitch - w + 1));
    const int32_t  t = hMin + (int32_t)((h >> 16) % (uint32_t)hSpan);
    uint32_t style = (h >> 24) & 3u;
    if (!near && style == 1u) style = 0u;     // stepped roofs need a wider body
    drawBuilding(d, x, w, t, style, near);
  }
}

// Far layer (slower, shorter, denser) behind a near layer (faster, taller).
// roadScroll = the road's scroll; each layer takes a fraction of it (parallax).
void drawBuildings(U8G2& d, int32_t roadScroll) {
  drawSkylineLayer(d, roadScroll >> 1,       14,  8, 5, 5, 5, 0x51ED270BUL, false);   // 2 px/m
  drawSkylineLayer(d, (roadScroll * 3) >> 2, 18, 10, 6, 5, 6, 0x00000000UL, true);    // 3 px/m
}

void drawGantry(U8G2& d, int32_t worldD16, bool withFlag, int32_t scroll) {
  const int32_t gx = m16ToRoadPx(worldD16) - scroll;    // road scale, world-anchored
  if (gx < 0 || gx > fb::SCREEN_W - 1) return;
  const u8g2_uint_t x = (u8g2_uint_t)gx;
  d.drawVLine(x, 16, 39);                     // pole spans both lanes (y16..54)
  if (withFlag) {
    for (uint8_t r = 0; r < 3; ++r)
      for (uint8_t c = 0; c < 3; ++c)
        if (((r + c) & 1u) && gx + 1 + c * 2 <= fb::SCREEN_W - 1)
          d.drawBox((u8g2_uint_t)(gx + 1 + c * 2), (u8g2_uint_t)(16 + r * 2), 2, 2);
  }
}

void drawN2OBox(U8G2& d, int16_t x, const char* ch, bool avail, bool active) {
  d.setFont(u8g2_font_5x7_tr);
  if (avail) {                                // ready: filled, dark letter
    d.drawBox((u8g2_uint_t)x, 56, 8, 8);
    d.setDrawColor(0);
    d.drawStr((u8g2_uint_t)(x + 2), 63, ch);
    d.setDrawColor(1);
  } else if (active) {                        // burning: blinking frame + letter
    if ((gAnimUs >> 17) & 1u) d.drawFrame((u8g2_uint_t)x, 56, 8, 8);
    d.drawStr((u8g2_uint_t)(x + 2), 63, ch);
  } else {                                    // spent: empty frame
    d.drawFrame((u8g2_uint_t)x, 56, 8, 8);
  }
}

// ---------------------------------------------------------------------------
// Screens
// ---------------------------------------------------------------------------
void renderTitle(U8G2& d) {
  // animated checkered strip
  const uint8_t ph = (uint8_t)((gAnimUs >> 18) & 1u);
  for (uint8_t x = 0; x < fb::SCREEN_W; x = (uint8_t)(x + 6))
    if (((x / 6) + ph) & 1u) d.drawBox(x, 0, 6, 5);

  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(2, 13, "flipBOX");
  if (gBestUs) {
    char b[8];
    fmtSec1(b, gBestUs);
    d.drawStr(78, 13, "BEST");
    d.drawStr(102, 13, b);
  }
  d.setFont(u8g2_font_7x13B_tr);
  drawCentered(d, 27, "FAST AND");
  drawCentered(d, 40, "FURIOUS");
  d.setBitmapMode(1);
  d.drawXBM(16, 43, 24, 9, kBmpSkyline);
  d.drawXBM(88, 43, 24, 9, kBmpCharger);
  d.setFont(u8g2_font_5x7_tr);
  drawCentered(d, 62, "TAP SEL:START  HOLD:EXIT");
}

void drawStat(U8G2& d, int16_t x, int16_t base, const char* label, uint8_t v10) {
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr((u8g2_uint_t)x, (u8g2_uint_t)base, label);
  d.drawFrame((u8g2_uint_t)(x + 18), (u8g2_uint_t)(base - 5), 37, 6);
  d.drawBox((u8g2_uint_t)(x + 19), (u8g2_uint_t)(base - 4), (u8g2_uint_t)(35 * v10 / 10), 4);
}

void renderSelect(U8G2& d) {
  d.setFont(u8g2_font_6x13B_tr);
  drawCentered(d, 11, "CHOOSE YOUR CAR");
  for (uint8_t i = 0; i < 2; ++i) {
    const int16_t cx  = (int16_t)(2 + i * 64);
    const int16_t ccx = (int16_t)(cx + 30);
    if (gMenuCursor == i) d.drawFrame((u8g2_uint_t)cx, 14, 60, 42);
    d.setBitmapMode(1);
    d.drawXBM((u8g2_uint_t)(ccx - 12), 17, 24, 9, i == 0 ? kBmpSkyline : kBmpCharger);
    d.setFont(u8g2_font_5x7_tr);
    const char* nm = (i == 0) ? "SKYLINE" : "CHARGER";
    d.drawStr((u8g2_uint_t)(ccx - (int16_t)d.getStrWidth(nm) / 2), 33, nm);
    drawStat(d, (int16_t)(cx + 4), 41, "ACC", (i == 0) ? 9 : 7);
    drawStat(d, (int16_t)(cx + 4), 49, "TOP", (i == 0) ? 7 : 9);
    d.setFont(u8g2_font_5x7_tr);
    d.drawStr((u8g2_uint_t)(cx + 4), 56, "N2O");
    d.drawBox((u8g2_uint_t)(cx + 22), 51, 5, 5);
    d.drawBox((u8g2_uint_t)(cx + 29), 51, 5, 5);
  }
  d.setFont(u8g2_font_5x7_tr);
  drawCentered(d, 63, "L/R:CHOOSE  SEL:RACE");
}

// The drag strip. hud=false during the countdown (top/bottom HUD suppressed).
void renderScene(U8G2& d, bool hud) {
  // After the player crosses the line everything in the background is frozen at
  // its finish-line state; only the two cars keep moving (at finish-zoom scale).
  const bool    over   = (gState == ST_FINISH);
  const int32_t cx     = over ? gFinCx : camX();                     // player front bumper, screen x
  const int32_t scroll = over ? gFinScroll : m16ToRoadPx(gPD16) - cx; // world px at screen x = 0
  const int32_t kmh    = ((gPV16 * 18) / 5) >> 16;

  // ---- background: two-layer outline skyline (parallax) ----
  drawBuildings(d, scroll);

  // ---- road: far edge | opponent lane | dashed divider | player lane | near edge ----
  d.drawHLine(0, kRoadTop, fb::SCREEN_W);
  scrollSegs(d, kLaneDiv, kDashLen, kDashPeriod, scroll);
  d.drawHLine(0, kRoadBot, fb::SCREEN_W);
  drawGantry(d, 0, false, scroll);            // start pole (drives off-screen)
  drawGantry(d, kRaceDist16, true, scroll);   // finish gantry with flag

  // ---- cars (the opponent drives the model the player did not pick) ----
  int32_t plrFront, oppFront;
  if (over) {                                 // zoom away from the frozen finish frame
    plrFront = cx + m16ToFinishPx(gPD16 - gFinPD16);
    oppFront = cx + (m16ToPx(gFinOD16) - m16ToPx(gFinPD16)) + m16ToFinishPx(gOD16 - gFinOD16);
  } else {                                    // chase camera: spacing at car scale
    plrFront = cx;
    oppFront = cx + m16ToPx(gOD16) - m16ToPx(gPD16);
  }
  const int32_t oppLeft = oppFront - kCarW;
  const int32_t plrLeft = plrFront - kCarW;   // on screen during the race (camera clamp)

  d.setBitmapMode(1);
  drawCar(d, oppLeft, kOppCarY, gCar == 0 ? kBmpCharger : kBmpSkyline);
  if (!over && ((gAnimUs >> 17) & 1u)) {      // opponent out of frame: blinking arrow
    const int16_t ay = kOppCarY + kCarH / 2;
    if (oppLeft >= fb::SCREEN_W)
      d.drawTriangle(127, ay - 4, 127, ay + 4, 121, ay);
    else if (oppFront <= 0)
      d.drawTriangle(0, ay - 4, 0, ay + 4, 6, ay);
  }
  drawCar(d, plrLeft, kPlrCarY, gCar == 0 ? kBmpSkyline : kBmpCharger);

  // exhaust: nitro flame, or a small puff on throttle (only while the car is on screen)
  if (plrLeft >= 1 && plrLeft <= fb::SCREEN_W - 1) {
    const uint8_t nAct = (gN1LeftUs ? 1 : 0) + (gN2LeftUs ? 1 : 0);
    if (nAct) {
      int32_t fl = 4 + (int32_t)((gAnimUs >> 15) & 3u);
      if (nAct > 1) fl += 2;
      int32_t tip = plrLeft - 1 - fl;
      if (tip < 0) tip = 0;
      d.drawTriangle((int16_t)(plrLeft - 1), kPlrCarY + 3, (int16_t)(plrLeft - 1), kPlrCarY + 7,
                     (int16_t)tip, kPlrCarY + 5);
    } else if (gThrottleVis && ((gAnimUs >> 16) & 1u)) {
      clipPx(d, plrLeft - 2, kPlrCarY + 6);
    }
  }

  if (!hud) return;

  // ---- top HUD: big speed, position, race time ----
  char b[8];
  d.setFont(u8g2_font_logisoso16_tn);
  fmtUInt(b, (uint32_t)kmh);
  d.drawStr((u8g2_uint_t)(32 - (int16_t)d.getStrWidth(b)), 16, b);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(35, 15, "km/h");
  d.setFont(u8g2_font_6x13B_tr);
  d.drawStr(54, 12, gPD16 >= gOD16 ? "1ST" : "2ND");
  d.setFont(u8g2_font_5x7_tr);
  fmtSec1(b, gPFinished ? gPTimeUs : gRaceUs);
  d.drawStr((u8g2_uint_t)(127 - (int16_t)d.getStrWidth(b)), 7, b);

  // ---- bottom HUD: PWR + N2O charges, progress bar ----
  d.drawStr(1, 63, "N2O");
  drawN2OBox(d, 19, "P", gShiftAvail, gShiftBoostUs > 0);
  drawN2OBox(d, 28, "1", gN1Avail, gN1LeftUs > 0);
  drawN2OBox(d, 37, "2", gN2Avail, gN2LeftUs > 0);

  d.drawFrame(48, 57, 80, 7);
  int32_t pw = (int32_t)(((int64_t)gPD16 * 78) / kRaceDist16);
  if (pw > 78) pw = 78;
  if (pw > 0) d.drawBox(49, 58, (u8g2_uint_t)pw, 5);
  int32_t ow = (int32_t)(((int64_t)gOD16 * 78) / kRaceDist16);
  if (ow > 78) ow = 78;
  if (ow > 0) {
    d.setDrawColor(2);                        // XOR tick: stays visible on the fill
    d.drawVLine((u8g2_uint_t)(49 + ow), 57, 7);
    d.setDrawColor(1);
  }
  d.setBitmapMode(1);
  d.drawXBM(120, 49, 7, 7, kBmpFlag);
}

void renderCountdown(U8G2& d) {
  d.setFont(u8g2_font_7x13B_tr);
  if (gFalseStart) {
    if ((gAnimUs >> 17) & 1u) drawCentered(d, 14, "FALSE START!");
  } else {
    drawCentered(d, 14, "READY");
  }
  char dig = 0;
  if      (gCdUs >= 2100000UL) dig = '1';
  else if (gCdUs >= 1400000UL) dig = '2';
  else if (gCdUs >=  700000UL) dig = '3';
  if (dig) {
    const char b[2] = { dig, 0 };
    d.setFont(u8g2_font_logisoso24_tn);       // right of the staged cars, not on top of them
    d.drawStr((u8g2_uint_t)(104 - (int16_t)d.getStrWidth(b) / 2), 46, b);
  }
  d.setFont(u8g2_font_5x7_tr);
  drawCentered(d, 63, "UP:GAS  L/R:N2O  SEL:PWR");
}

void renderRaceOverlay(U8G2& d) {
  // Banners sit in the skyline band (rows 17..29) so they never cover the cars,
  // and share one slot: FALSE START > PERFECT/GOOD > GO!
  d.setFont(u8g2_font_6x13B_tr);
  if (gFalseStart && gRaceUs < 1600000UL) {
    banner(d, 28, "FALSE START");
  } else if (gRaceUs < gReactTxtUs && gReactGrade == 2) {
    banner(d, 28, "PERFECT!");
  } else if (gRaceUs < gReactTxtUs && gReactGrade == 1) {
    banner(d, 28, "GOOD");
  } else if (gRaceUs < 600000UL) {
    d.setFont(u8g2_font_7x13B_tr);
    banner(d, 28, "GO!");
  }
}

void renderFinish(U8G2& d) {
  d.setFont(u8g2_font_7x13B_tr);
  banner(d, 28, "FINISH!");
}

void renderResults(U8G2& d) {
  char b[8];
  d.setFont(u8g2_font_7x13B_tr);
  drawCentered(d, 13, gWin ? "YOU WIN!" : "2ND PLACE");
  d.setFont(u8g2_font_5x7_tr);
  if (gNewBest && ((gAnimUs >> 17) & 1u)) drawCentered(d, 21, "NEW BEST!");

  d.drawStr(4, 30, "YOU");
  d.setFont(u8g2_font_logisoso16_tn);
  fmtSec2(b, gPTimeUs);
  d.drawStr(26, 40, b);
  const int16_t w = (int16_t)d.getStrWidth(b);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr((u8g2_uint_t)(26 + w + 2), 40, "s");

  d.drawStr(4, 52, "OPP");
  fmtSec2(b, gOTimeUs);
  d.drawStr(26, 52, b);
  d.drawStr((u8g2_uint_t)(26 + (int16_t)d.getStrWidth(b) + 2), 52, "s");
  // gap
  char g[9];
  g[0] = gWin ? '+' : '-';
  fmtSec2(g + 1, gGapUs);
  uint8_t L = 0; while (g[L]) ++L;
  g[L] = 's'; g[L + 1] = 0;
  d.drawStr(62, 52, "GAP");
  d.drawStr(82, 52, g);

  // menu: RESTART / EXIT
  d.drawStr(22, 62, "RESTART");
  d.drawStr(84, 62, "EXIT");
  d.drawStr(gMenuCursor == 0 ? 14 : 76, 62, ">");
}

void renderExitCue(U8G2& d) {
  d.setDrawColor(0);
  d.drawBox(30, 26, 68, 13);
  d.setDrawColor(1);
  d.drawFrame(30, 26, 68, 13);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(34, 35, "EXITING");
  uint32_t w = (gSelHoldUs * 22UL) / kExitHoldUs;
  if (w > 22) w = 22;
  d.drawBox(72, 30, (u8g2_uint_t)w, 4);
}

void gameRender(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(1);              // transparent text over graphics
  d.setBitmapMode(0);
  switch (gState) {
    case ST_TITLE:   renderTitle(d);                 break;
    case ST_SELECT:  renderSelect(d);                break;
    case ST_COUNT:   renderScene(d, false); renderCountdown(d); break;
    case ST_RACE:    renderScene(d, true);  renderRaceOverlay(d); break;
    case ST_FINISH:  renderScene(d, true);  renderFinish(d);      break;
    case ST_RESULTS: renderResults(d);               break;
  }
  if (gSelHoldUs > 120000UL) renderExitCue(d);
}

}  // namespace

FB_GAME(FastAndFurious, gameInit, gameUpdate, gameRender, nullptr);
