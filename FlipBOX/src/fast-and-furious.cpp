// src/Game2.cpp  --  Fast and Furious  (flipBOX)
//
// Straight-line 1050 m drag race against an independent AI opponent.
// Modes: CLASSIC (2-lane original) and ROAD RAGE (4-lane with traffic).
// Choose the Nissan Skyline GT-R (higher acceleration) or the Dodge Charger
// (higher top speed), time your launch, and spend two one-shot N2O charges
// (+30 km/h for 3 s each, additive, with accel boost that reaches the extra
// 30 km/h in ~1 s even from top speed) plus a one-time POWER SHIFT (+12 km/h).
//
// Controls:
//   Title / menus : LEFT/RIGHT = choose, SELECT = confirm (acts on press),
//                   hold SELECT 1 s = exit to launcher
//   Mode select   : LEFT / RIGHT = CLASSIC / ROAD RAGE, SELECT = next
//   Car select    : LEFT / RIGHT = choose car, SELECT = race, X = back to mode
//   Countdown     : wait for GO!
//                   Classic: touching UP before GO = false start
//                   Road Rage: touching X before GO = false start
//   Race (Classic): UP (hold) = throttle, DOWN = brake,
//                   LEFT = N2O #1, RIGHT = N2O #2, tap SELECT = POWER SHIFT
//   Race (Road Rage):
//                   X (hold) = throttle
//                   UP / DOWN = change lane
//                   LEFT = N2O #1, RIGHT = N2O #2
//                   SELECT = POWER SHIFT
//   Results       : LEFT / RIGHT = choose, SELECT = menu / replay / exit
//
// Road Rage specifics:
//   4 broader lanes (no skyline scenery). Player camera locked to the
//   left third of the screen for reaction time against oncoming traffic.
//   Lanes 0-1 same direction (traffic slower than player). Lanes 2-3
//   opposite direction (traffic extremely fast). Full-size car sprites
//   for traffic. Crash into any traffic car = immediate loss.

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
constexpr uint8_t KEY_X      = 1u << 5;  // physical button on D4 / ESP32 GPIO4

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

// Optional launcher extras (provided by main.cpp, not part of contract v1):
//   keyPressedAtUs : micros() time stamp of the latest accepted press of ONE key
//   frameStartUs   : micros() time this frame's input was sampled at
//   displayLatencyUs: estimated delay from frameStartUs() to visible pixels
namespace fb {
uint32_t keyPressedAtUs(uint8_t keyMask);
uint32_t frameStartUs();
uint32_t displayLatencyUs();
}

namespace {

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------
// Physics is integer fixed-point: speeds in Q16.16 m/s, distances in Q16.16 m.
constexpr int32_t kmh16(int32_t kmh) { return (int32_t)(((int64_t)kmh * 5 * 65536) / 18); }
constexpr int32_t ms16(int32_t num, int32_t den) { return (int32_t)(((int64_t)num * 65536) / den); }

constexpr int32_t kRaceDist16    = 1050L << 16;     // race length: 1050 m

constexpr int32_t kSkyTop16      = kmh16(205);      // Skyline GT-R top speed
constexpr int32_t kSkyAcc16      = ms16(116, 10);   // Skyline accel 11.6 m/s^2
constexpr int32_t kChgTop16      = kmh16(236);      // Charger top speed
constexpr int32_t kChgAcc16      = ms16(92, 10);    // Charger accel  9.2 m/s^2

constexpr int32_t kOppTop16      = kmh16(216);      // opponent top speed
constexpr int32_t kOppAcc16      = ms16(104, 10);   // opponent accel 10.4 m/s^2
constexpr uint32_t kOppReactMin  = 250000;          // opponent reaction: min (us)
constexpr uint32_t kOppReactSpr  = 350000;          //   ... plus random spread (us)
constexpr int32_t kOppN2OAt16    = (int32_t)(((int64_t)kRaceDist16 * 170) / 402);
constexpr int32_t kOppN2OTop16   = kmh16(25);       // opponent N2O: +25 km/h

// Player N2O: each charge adds a full +30 km/h (additive) and a strong accel
// boost so the extra speed is reached in ~1 s even from the current top speed.
constexpr int32_t kN2OTop16      = kmh16(30);       // +30 km/h per charge (additive)
constexpr int32_t kN2OAcc16      = ms16(90, 10);    // strong extra accel while burning
constexpr uint32_t kN2ODurUs     = 3000000;         // N2O burn time: 3 s
constexpr uint32_t kN2ORampUs    = 1000000;         // target: reach +30 km/h in ~1 s

constexpr int32_t kShiftTop16    = kmh16(12);       // POWER SHIFT: +12 km/h permanent
constexpr uint32_t kShiftBoostUs = 1000000;         //   ... and +20% accel for 1 s

constexpr uint32_t kLaunchPerfectUs = 220000;       // reaction <= this: PERFECT
constexpr uint32_t kLaunchGoodUs    = 450000;       // reaction <= this: GOOD
constexpr uint32_t kBoostPerfectUs  = 1200000;      // PERFECT: x1.30 accel for 1.2 s
constexpr uint32_t kBoostGoodUs     = 800000;       // GOOD:    x1.15 accel for 0.8 s
constexpr uint32_t kFalseStartUs    = 600000;       // false-start throttle lockout

constexpr int32_t kCoast16       = ms16(8, 10);     // 0.8 m/s^2 drag, throttle off
constexpr int32_t kBrake16       = ms16(6, 1);      // 6.0 m/s^2 brake

// Visual scales (Q16.16 px per metre). They only affect drawing, never physics.
// Road scale raised so scenery moves markedly faster at high speed.
constexpr int32_t kPxPerM16      = 65536;           // car-to-car spacing: 1 px per metre
constexpr int32_t kRoadPxPerM16  = 22 * 65536;      // road/buildings/gantry: 22 px/m
                                                    //   (~1250 px/s at 205 km/h)
constexpr int32_t kFinishPxPerM16 = 18 * 65536;     // cars zooming off after finish: 18 px/m

// Car sprite size
constexpr int32_t kCarW          = 24;
constexpr int32_t kCarH          = 9;

// Chase camera
constexpr int32_t kCamMinX       = kCarW + 2;
constexpr int32_t kCamMaxX       = 128 - 2;
constexpr int32_t kCamHome       = 76;
constexpr int32_t kCamAlone      = 54;
constexpr int32_t kCamFade       = 40;
constexpr int32_t kCamEase       = 10;      // camera follow rate (1/s); higher = snappier

// Road layout (screen rows) — Classic 2-lane
constexpr int16_t kRoadTop       = 30;
constexpr int16_t kLaneDiv       = 42;
constexpr int16_t kRoadBot       = 54;
constexpr int16_t kOppCarY       = 32;
constexpr int16_t kPlrCarY       = 44;

// Road Rage 4-lane layout — even split, bottom clears HUD (progress bar ~y57)
// Usable band y=18..56 (38 px). Four 9-px lanes + dividers.
// Lane 0,1 = same direction (top half). Lane 2,3 = opposite (bottom half).
// Solid median between lane 1 and 2.
constexpr int16_t kRRLaneY[4]    = { 19, 28, 39, 48 };
constexpr int16_t kRRLaneH       = 9;
constexpr int16_t kRRRoadTop     = 18;
constexpr int16_t kRRRoadBot     = 56;
constexpr int16_t kRRMedianY     = 36;             // solid divider top/bottom halves

// Lane divider dashes
constexpr int16_t kDashLen       = 32;
constexpr int16_t kDashPeriod    = 64;

// Background skyline (Classic only)
constexpr int32_t kBldBias       = 40000;
// Parallax: skyline layers scroll at road speed >> shift. Kept slow enough that a
// layer moves less than half a building pitch per frame, which avoids the
// strobing / backwards-stepping look at high speed.
constexpr int32_t kParFarShift   = 3;               // far layer  = 1/8 road speed (pitch 14 px)
constexpr int32_t kParNearShift  = 2;               // near layer = 1/4 road speed (pitch 18 px)

constexpr uint32_t kExitHoldUs   = 1000000;         // hold SELECT this long to quit
constexpr uint32_t kCountdownUs  = 2800000;         // READY + 3 + 2 + 1
constexpr uint32_t kPostOffUs    = 1500000;         // wait after both cars off-screen

// Road Rage traffic
// On-camera relative speed = |v_traf - v_player| (camera locked to player).
// Same-dir: very slow absolute → large closing speed on screen.
// Opposite: huge absolute toward player → extreme on-camera speed.
constexpr uint8_t  kMaxTraffic   = 5;              // fewer cars on screen
// On-camera relative = |v_traf - v_player|. Push both directions much faster on screen.
constexpr int32_t  kTrafSameTop16 = kmh16(25);      // same-dir: very slow → huge closing rate
constexpr int32_t  kTrafOppTop16  = kmh16(650);     // opposite: extreme on-camera approach
constexpr int32_t  kTrafAcc16     = ms16(70, 10);   // traffic accel
constexpr int32_t  kTrafMinGap16  = 120L << 16;     // larger min gap → fewer simultaneous cars
constexpr uint32_t kLaneChangeUs  = 380000;         // cooldown between OPPONENT lane changes (tighter AI)
constexpr uint32_t kLaneAnimUs    = 140000;         // swerve animation duration
constexpr uint32_t kPlrLaneCdUs   = kLaneAnimUs;    // player: next lane change allowed once the swerve ends
constexpr uint32_t kLaneBufUs     = 200000;         // a lane request made during a swerve is kept this long
constexpr uint32_t kWanderPeriodUs = 2666667;       // opponent casual lane wander: ~1 per 2.7 s (frame-rate independent)
// Car outline length for same-lane collision (≈ sprite visual length in metres)
constexpr int32_t  kCarLen16     = 22L << 16;       // full car length ~22 m
constexpr int32_t  kCrashHalfW16 = 11L << 16;       // half-length for overlap test
constexpr uint8_t  kNumTrafBmps  = 6;              // number of background car sprites
constexpr uint32_t kCrashAnimUs  = 1600000;         // crash freeze + animation duration
constexpr uint32_t kFinishMaxUs  = 2500000;         // max wait in ST_FINISH before projecting opp time

// Road Rage camera: keep player fixed in the left third of the screen
constexpr int32_t kRRCamX        = 42;             // player front bumper X target

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
// Background traffic car sprites (24 x 9 each). Distinct silhouettes.
// XBM: row-major, LSB = leftmost pixel. Same format as player cars.
// ---------------------------------------------------------------------------
// 0: low sports coupe
const uint8_t kBmpTraf0[] = {
  0, 60, 0, 0, 66, 0, 12, 195, 3, 254, 255, 63, 255, 255, 127,
  255, 255, 255, 199, 255, 248, 112, 0, 14, 112, 0, 14
};
// 1: tall hatchback
const uint8_t kBmpTraf1[] = {
  0, 126, 0, 0, 129, 0, 128, 129, 1, 248, 255, 31, 252, 255, 63,
  254, 255, 127, 135, 255, 240, 120, 0, 15, 120, 0, 15
};
// 2: long sedan
const uint8_t kBmpTraf2[] = {
  0, 248, 0, 0, 4, 1, 14, 6, 3, 255, 255, 31, 255, 255, 63,
  255, 255, 127, 199, 255, 248, 96, 0, 6, 96, 0, 6
};
// 3: pickup / utility
const uint8_t kBmpTraf3[] = {
  0, 248, 1, 0, 4, 2, 0, 4, 2, 252, 255, 63, 254, 255, 127,
  255, 255, 255, 135, 255, 240, 192, 0, 3, 192, 0, 3
};
// 4: wedge sports
const uint8_t kBmpTraf4[] = {
  0, 28, 0, 0, 34, 0, 28, 227, 7, 254, 255, 31, 255, 255, 63,
  255, 255, 127, 143, 255, 240, 112, 0, 14, 112, 0, 14
};
// 5: boxy compact
const uint8_t kBmpTraf5[] = {
  0, 255, 0, 128, 0, 1, 128, 0, 1, 240, 255, 15, 248, 255, 31,
  252, 255, 63, 135, 255, 240, 120, 0, 15, 120, 0, 15
};

// Lookup table for traffic sprites
const uint8_t* const kTrafBmps[kNumTrafBmps] = {
  kBmpTraf0, kBmpTraf1, kBmpTraf2, kBmpTraf3, kBmpTraf4, kBmpTraf5
};

// ---------------------------------------------------------------------------
// Car sprites in blit-ready form.  Every car is converted ONCE (gameInit) from the
// XBM data above into 24 column words (bit r = row r), including a mirrored copy
// of each traffic car for oncoming lanes, so drawing is a handful of byte ORs
// into the display buffer instead of ~200 per-pixel calls.
// ---------------------------------------------------------------------------
enum SprId : uint8_t {
  SPR_SKY = 0,
  SPR_CHG = 1,
  SPR_TRAF0 = 2,                                   // 6 traffic cars
  SPR_TRAF_FLIP0 = SPR_TRAF0 + kNumTrafBmps,       // 6 mirrored traffic cars
  SPR_COUNT = SPR_TRAF_FLIP0 + kNumTrafBmps
};
uint16_t gSprCol[SPR_COUNT][kCarW];

// ---------------------------------------------------------------------------
// State (ALL runtime state is reset in gameInit()/resetRace())
// ---------------------------------------------------------------------------
enum State : uint8_t {
  ST_TITLE, ST_MODE, ST_SELECT, ST_COUNT, ST_RACE, ST_FINISH, ST_WAIT, ST_RESULTS, ST_CRASH
};
enum Mode  : uint8_t { MODE_CLASSIC = 0, MODE_ROADRAGE = 1 };

State    gState;
Mode     gMode;
uint32_t gAnimUs;
uint32_t gRng;
uint32_t gSelHoldUs;
uint8_t  gMenuCursor;

uint8_t  gCar;
uint32_t gCdUs;
uint32_t gRaceUs;
bool     gFalseStart;
uint32_t gLockoutUs;

int32_t  gPV16, gPD16;
bool     gN1Avail, gN2Avail;
uint32_t gN1LeftUs, gN2LeftUs;
bool     gShiftAvail;
uint32_t gShiftBoostUs;
bool     gReactKnown;
uint32_t gReactTxtUs;
uint32_t gLaunchBoostUs;
uint16_t gLaunchPct;
uint8_t  gReactGrade;
bool     gThrottleVis;
bool     gPFinished;
uint32_t gPTimeUs;

int32_t  gOV16, gOD16;
uint32_t gOReactUs;
bool     gON2OUsed;
uint32_t gON2OLeftUs;
bool     gOFinished;
uint32_t gOTimeUs;

int32_t  gCamX256;
int32_t  gFinCx;
int32_t  gFinScroll;
int32_t  gFinPD16;
int32_t  gFinOD16;

uint32_t gWaitUs;
bool     gWin;
uint32_t gGapUs;
bool     gNewBest;
bool     gCrashed;           // Road Rage loss by collision
int16_t  gCrashX;            // screen X of crash centre
int16_t  gCrashY;            // screen Y of crash centre
uint32_t gCrashAnimUs;       // elapsed crash animation time

// Road Rage lane state (0..3) + smooth Y animation
uint8_t  gPLane;
uint8_t  gOLane;
uint32_t gPLaneCdUs;
uint32_t gOLaneCdUs;
int8_t   gLaneBuf;           // buffered player lane request: -1 up, +1 down, 0 none
uint32_t gLaneBufUs;         // time left before the buffered request expires
int16_t  gPLaneY;            // current rendered Y (interpolated)
int16_t  gOLaneY;
int16_t  gPLaneYFrom;        // animation start Y
int16_t  gOLaneYFrom;
uint32_t gPLaneAnimUs;       // remaining anim time
uint32_t gOLaneAnimUs;

// Traffic cars
struct Traf {
  bool     alive;
  uint8_t  lane;             // 0..3
  bool     sameDir;          // true = same direction as player
  uint8_t  sprite;           // index into kTrafBmps
  int32_t  d16;              // world distance (Q16.16 m)
  int32_t  v16;              // speed (Q16.16 m/s), signed: + same, - opposite
};
Traf     gTraf[kMaxTraffic];
uint32_t gNextSpawnUs;

uint32_t gBestUs = 0;        // SESSION-PERSISTENT best; intentionally not reset in init()

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
uint32_t rnd() {
  gRng ^= gRng << 13; gRng ^= gRng >> 17; gRng ^= gRng << 5;
  return gRng;
}

int32_t accelToward16(int32_t v16, int32_t top16, int32_t a016) {
  return a016 - (int32_t)(((int64_t)a016 * v16) / top16);
}

void integrate(int32_t& v16, int32_t& d16, int32_t a16, uint32_t dt16) {
  v16 += (int32_t)(((int64_t)a16 * dt16) >> 16);
  if (v16 < 0) v16 = 0;
  d16 += (int32_t)(((int64_t)v16 * dt16) >> 16);
}

void integrateSigned(int32_t& v16, int32_t& d16, int32_t a16, uint32_t dt16) {
  // For opposite-direction cars that keep negative velocity
  v16 += (int32_t)(((int64_t)a16 * dt16) >> 16);
  d16 += (int32_t)(((int64_t)v16 * dt16) >> 16);
}

void fmtUInt(char* b, uint32_t v) {
  char t[4]; uint8_t n = 0;
  do { t[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < 3);
  uint8_t i = 0; while (n) b[i++] = t[--n];
  b[i] = 0;
}
void fmtSec1(char* b, uint32_t us) {
  uint32_t ds = us / 100000UL; if (ds > 999UL) ds = 999UL;
  uint32_t s = ds / 10UL, t = ds % 10UL;
  char* p = b;
  if (s >= 10) *p++ = (char)('0' + s / 10);
  *p++ = (char)('0' + s % 10); *p++ = '.'; *p++ = (char)('0' + t); *p++ = 's'; *p = 0;
}
void fmtSec2(char* b, uint32_t us) {
  uint32_t cs = us / 10000UL; if (cs > 9999UL) cs = 9999UL;
  uint32_t s = cs / 100UL, c = cs % 100UL;
  char* p = b;
  if (s >= 10) *p++ = (char)('0' + s / 10);
  *p++ = (char)('0' + s % 10); *p++ = '.';
  *p++ = (char)('0' + c / 10); *p++ = (char)('0' + c % 10); *p = 0;
}

int32_t m16ToPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kPxPerM16) >> 32);
}

int32_t camTargetX(int32_t gap) {
  int32_t lo = kCamMinX + (gap < 0 ? -gap : 0);
  int32_t hi = kCamMaxX - (gap > 0 ?  gap : 0);
  if (lo > kCamMaxX) lo = kCamMaxX;
  if (hi < kCamMinX) hi = kCamMinX;

  int32_t t = kCamHome - gap / 2;
  if (t < lo) t = lo;
  if (t > hi) t = hi;

  const int32_t ag   = gap < 0 ? -gap : gap;
  const int32_t span = kCamMaxX - kCamMinX;
  if (ag > span) {
    int32_t w = ag - span;
    if (w > kCamFade) w = kCamFade;
    t = (t * (kCamFade - w) + kCamAlone * w) / kCamFade;
  }
  return t;
}

void updateCamera(uint32_t dt16, bool snap) {
  int32_t tgt;
  if (gMode == MODE_ROADRAGE) {
    // Road Rage: lock player car to the left third of the screen so the
    // player always has reaction distance against oncoming traffic.
    tgt = kRRCamX << 8;
  } else {
    const int32_t gap = m16ToPx(gOD16) - m16ToPx(gPD16);
    tgt = camTargetX(gap) << 8;
  }
  if (snap) {
    gCamX256 = tgt;
  } else {
    int32_t k = (int32_t)(dt16 * (uint32_t)kCamEase);
    if (k > 65536) k = 65536;
    gCamX256 += (int32_t)(((int64_t)(tgt - gCamX256) * k) >> 16);
  }
  if (gCamX256 < (kCamMinX << 8)) gCamX256 = kCamMinX << 8;
  if (gCamX256 > (kCamMaxX << 8)) gCamX256 = kCamMaxX << 8;
}

int32_t m16ToRoadPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kRoadPxPerM16) >> 32);
}

int32_t m16ToFinishPx(int32_t m16) {
  return (int32_t)(((int64_t)m16 * kFinishPxPerM16) >> 32);
}

int32_t camX() { return (gCamX256 + 128) >> 8; }

// ---------------------------------------------------------------------------
// Traffic helpers (Road Rage)
// ---------------------------------------------------------------------------
void clearTraffic() {
  for (uint8_t i = 0; i < kMaxTraffic; ++i) gTraf[i].alive = false;
  gNextSpawnUs = 0;
}

void spawnTraffic(uint32_t nowUs) {
  if (nowUs < gNextSpawnUs) return;
  // Find free slot
  uint8_t slot = 255;
  for (uint8_t i = 0; i < kMaxTraffic; ++i) {
    if (!gTraf[i].alive) { slot = i; break; }
  }
  if (slot == 255) {
    gNextSpawnUs = nowUs + 400000UL;
    return;
  }

  // Decide direction and lane
  const bool same = (rnd() & 1u) != 0;
  uint8_t lane;
  if (same) {
    lane = (uint8_t)(rnd() & 1u);           // lanes 0 or 1
  } else {
    lane = 2 + (uint8_t)(rnd() & 1u);       // lanes 2 or 3
  }

  // Place ahead relative to player so it appears on screen
  int32_t base = gPD16;
  int32_t offset;
  if (same) {
    // Same dir (~120 km/h): spawn well ahead; player catches up so they
    // drift leftward on the fixed camera.
    offset = kTrafMinGap16 + (int32_t)((rnd() % 180) << 16);
  } else {
    // Opposite: spawn further ahead; they close extremely fast.
    offset = (180L << 16) + (int32_t)((rnd() % 220) << 16);
  }

  Traf& t = gTraf[slot];
  t.alive   = true;
  t.lane    = lane;
  t.sameDir = same;
  t.sprite  = (uint8_t)(rnd() % kNumTrafBmps);
  t.d16     = base + offset;
  if (same) {
    // Very slow absolute (~20–30 km/h) → very high on-camera closing speed
    t.v16 = kTrafSameTop16 + (int32_t)((rnd() % 11) - 5) * (1 << 12);
    if (t.v16 < kmh16(18)) t.v16 = kmh16(18);
    if (t.v16 > kmh16(32)) t.v16 = kmh16(32);
  } else {
    // Extreme absolute speed toward the player → extreme on-camera speed
    t.v16 = -(kTrafOppTop16 + (int32_t)((rnd() % 60) << 12));
  }

  // Longer spawn interval → fewer cars on screen at once
  gNextSpawnUs = nowUs + 700000UL + (rnd() % 900000UL);
}

// Same-lane outline overlap test (car length along the track)
static bool carsOverlap(int32_t dA, int32_t dB) {
  int32_t dd = dA - dB;
  if (dd < 0) dd = -dd;
  return dd < kCarLen16;
}

bool checkCrash() {
  // Lane-wise only: crash if same lane and car outlines overlap
  for (uint8_t i = 0; i < kMaxTraffic; ++i) {
    const Traf& t = gTraf[i];
    if (!t.alive || t.lane != gPLane) continue;
    if (carsOverlap(t.d16, gPD16)) return true;
  }
  return false;
}

void stepTraffic(uint32_t dt16, uint32_t dt, uint32_t nowUs) {
  (void)dt;
  spawnTraffic(nowUs);

  for (uint8_t i = 0; i < kMaxTraffic; ++i) {
    Traf& t = gTraf[i];
    if (!t.alive) continue;

    // Constant-speed integration
    t.d16 += (int32_t)(((int64_t)t.v16 * dt16) >> 16);

    // Cull far behind or far ahead of the player
    int32_t rel = t.d16 - gPD16;
    if (rel < -(180L << 16) || rel > (350L << 16)) {
      t.alive = false;
      continue;
    }
  }
}

// ---------------------------------------------------------------------------
// Race reset
// ---------------------------------------------------------------------------
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
  gWaitUs = 0; gWin = false; gGapUs = 0; gNewBest = false;
  gCrashed = false;
  gCrashX = 64; gCrashY = 32; gCrashAnimUs = 0;
  gPLane = 1;           // start in a same-dir lane
  gOLane = 0;
  gPLaneCdUs = 0;
  gOLaneCdUs = 0;
  gLaneBuf = 0;
  gLaneBufUs = 0;
  gPLaneY = kRRLaneY[gPLane];
  gOLaneY = kRRLaneY[gOLane];
  gPLaneYFrom = gPLaneY;
  gOLaneYFrom = gOLaneY;
  gPLaneAnimUs = 0;
  gOLaneAnimUs = 0;
  clearTraffic();
  updateCamera(0, true);
  gFinCx = camX(); gFinScroll = 0; gFinPD16 = 0; gFinOD16 = 0;
}

// Lane swerve: rush sideways with slight overshoot, then settle into the lane.
// Not a floaty glide — feels like a quick body roll / swerve.  Phase 1 eases OUT
// (fast first, then slowing), so the car visibly reacts on the very first frame
// after the key press instead of creeping off the mark.
int16_t laneSwerveY(int16_t from, int16_t to, uint32_t remainUs) {
  if (remainUs == 0 || remainUs >= kLaneAnimUs) return to;
  const uint32_t prog = kLaneAnimUs - remainUs;
  const int32_t dy = (int32_t)to - (int32_t)from;
  // Phase 1 (~0–55%): accelerate past the target (~18% overshoot)
  const uint32_t p1 = (kLaneAnimUs * 11) / 20;
  if (prog < p1) {
    const int32_t q  = 1000 - (int32_t)((prog * 1000) / p1);       // time left in phase 1, 1000..0
    const int32_t eo = 1000 - (q * q) / 1000;                      // ease-out: 0..1000, steep at the start
    return (int16_t)(from + (dy * eo * 118) / 100000);
  }
  // Phase 2: snap back from overshoot to the lane centre
  const int32_t over = from + (dy * 118) / 100;
  const int32_t p = (int32_t)(((prog - p1) * 1000) / (kLaneAnimUs - p1));
  return (int16_t)(over + ((int32_t)to - over) * p / 1000);
}

// One physics step. 'racing' = player throttle is live.
void stepRace(const fb::Input& in, uint32_t dt16, uint32_t dt, bool racing) {
  if (gN1LeftUs      > dt) gN1LeftUs      -= dt; else gN1LeftUs      = 0;
  if (gN2LeftUs      > dt) gN2LeftUs      -= dt; else gN2LeftUs      = 0;
  if (gShiftBoostUs  > dt) gShiftBoostUs  -= dt; else gShiftBoostUs  = 0;
  if (gLaunchBoostUs > dt) gLaunchBoostUs -= dt; else gLaunchBoostUs = 0;
  if (gON2OLeftUs    > dt) gON2OLeftUs    -= dt; else gON2OLeftUs    = 0;
  if (gPLaneCdUs     > dt) gPLaneCdUs     -= dt; else gPLaneCdUs     = 0;
  if (gOLaneCdUs     > dt) gOLaneCdUs     -= dt; else gOLaneCdUs     = 0;

  // ---- player ----
  const int32_t nAct = (gN1LeftUs ? 1 : 0) + (gN2LeftUs ? 1 : 0);
  const int32_t baseTop = (gCar == 0 ? kSkyTop16 : kChgTop16)
                        + (gShiftAvail ? 0 : kShiftTop16);
  const int32_t top16 = baseTop + nAct * kN2OTop16;

  // Classic: UP = throttle, DOWN = brake
  // Road Rage: X = throttle (no brake; UP/DOWN reserved for lanes)
  const bool throttle = racing && gLockoutUs == 0 &&
    (gMode == MODE_ROADRAGE ? (in.held & fb::KEY_X) : (in.held & fb::KEY_UP));
  gThrottleVis = throttle;
  int32_t a16;
  if (throttle) {
    a16 = accelToward16(gPV16, top16, gCar == 0 ? kSkyAcc16 : kChgAcc16);
    a16 += nAct * kN2OAcc16;
    if (a16 > 0) {
      if (gLaunchBoostUs) a16 = (int32_t)(((int64_t)a16 * gLaunchPct) / 100);
      if (gShiftBoostUs)  a16 = (a16 * 6) / 5;
    }
    if (gMode == MODE_CLASSIC && (in.held & fb::KEY_DOWN)) a16 -= kBrake16;
  } else {
    a16 = -kCoast16;
    if (gMode == MODE_CLASSIC && (in.held & fb::KEY_DOWN)) a16 -= kBrake16;
  }
  integrate(gPV16, gPD16, a16, dt16);

  // Road Rage: UP / DOWN change lane (±1 only) + swerve animation.
  // A request made while the previous swerve / cooldown is still running is
  // remembered for kLaneBufUs and executed the moment the car is free, so quick
  // double-dodges are not eaten by the cooldown.
  if (gMode == MODE_ROADRAGE) {
    if (racing) {
      if (in.pressed & fb::KEY_UP)        { gLaneBuf = -1; gLaneBufUs = kLaneBufUs; }
      else if (in.pressed & fb::KEY_DOWN) { gLaneBuf = +1; gLaneBufUs = kLaneBufUs; }

      if (gLaneBuf != 0) {
        if (gPLaneCdUs == 0 && gPLaneAnimUs == 0) {
          const int8_t nl = (int8_t)((int8_t)gPLane + gLaneBuf);
          if (nl >= 0 && nl <= 3) {
            gPLaneYFrom = gPLaneY;
            gPLane = (uint8_t)nl;             // exactly one lane
            gPLaneAnimUs = kLaneAnimUs;
            gPLaneCdUs = kPlrLaneCdUs;
          }
          gLaneBuf = 0;
          gLaneBufUs = 0;
        } else if (gLaneBufUs > dt) {
          gLaneBufUs -= dt;
        } else {
          gLaneBuf = 0;
          gLaneBufUs = 0;
        }
      }
    }
    if (gPLaneAnimUs > 0) {
      if (gPLaneAnimUs > dt) gPLaneAnimUs -= dt; else gPLaneAnimUs = 0;
      gPLaneY = laneSwerveY(gPLaneYFrom, kRRLaneY[gPLane], gPLaneAnimUs);
    } else {
      gPLaneY = kRRLaneY[gPLane];
    }
  }

  // ---- opponent: calmer AI, only ±1 lane, light avoidance, no phasing ----
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

    if (gMode == MODE_ROADRAGE) {
      int32_t gap = gOD16 - gPD16;
      if (gap < 0) gap = -gap;

      // Only shift one lane at a time; long cooldown; avoid only when truly close
      if (gOLaneCdUs == 0 && gOLaneAnimUs == 0) {
        int8_t delta = 0;

        if (gOLane == gPLane && gap < (45L << 16)) {
          // Same lane and close: step one lane away from the median if possible
          if (gOLane <= 1) delta = (gOLane < 3) ? 1 : -1;
          else delta = (gOLane > 0) ? -1 : 1;
          // Prefer not landing on player if that would still be same (impossible with ±1
          // unless player is adjacent — then pick the other way)
          {
            int8_t alt = (int8_t)gOLane + delta;
            if (alt < 0) alt = 1;
            if (alt > 3) alt = 2;
            if ((uint8_t)alt == gPLane) delta = -delta;
          }
        } else if ((rnd() % kWanderPeriodUs) < dt) {
          // Rare casual wander, ±1 only (time-based, so it does not speed up with the frame rate)
          delta = (rnd() & 1) ? 1 : -1;
        }

        int8_t nl = (int8_t)gOLane + delta;
        if (nl < 0) nl = 0;
        if (nl > 3) nl = 3;
        // Never enter player's lane while longitudinally overlapping / very close
        if ((uint8_t)nl == gPLane && gap < (50L << 16)) {
          nl = (int8_t)gOLane;   // cancel
        }

        if ((uint8_t)nl != gOLane) {
          gOLaneYFrom = gOLaneY;
          gOLane = (uint8_t)nl;
          gOLaneAnimUs = kLaneAnimUs;
          gOLaneCdUs = kLaneChangeUs + (rnd() % (kLaneChangeUs / 2));
        }
      }

      if (gOLaneAnimUs > 0) {
        if (gOLaneAnimUs > dt) gOLaneAnimUs -= dt; else gOLaneAnimUs = 0;
        gOLaneY = laneSwerveY(gOLaneYFrom, kRRLaneY[gOLane], gOLaneAnimUs);
      } else {
        gOLaneY = kRRLaneY[gOLane];
      }

      // Solid separation whenever same logical lane and outlines overlap
      // (also blocks phasing mid-swerve into the player's lane)
      if (gOLane == gPLane && carsOverlap(gOD16, gPD16)) {
        if (gOD16 >= gPD16) {
          gOD16 = gPD16 + kCarLen16;
          if (gOV16 < gPV16) gOV16 = gPV16;   // can't sink backward through player
        } else {
          gOD16 = gPD16 - kCarLen16;
          if (gOV16 > gPV16) gOV16 = gPV16;
        }
      }
    }
  } else {
    integrate(gOV16, gOD16, -kCoast16, dt16);
    if (gMode == MODE_ROADRAGE) gOLaneY = kRRLaneY[gOLane];
  }

  // Traffic + crash check (lane-wise outline only)
  if (gMode == MODE_ROADRAGE && racing) {
    stepTraffic(dt16, dt, gRaceUs);
    if (checkCrash()) {
      gCrashed = true;
      gPFinished = true;
      gPTimeUs = gRaceUs;
    }
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void buildSprites();   // defined with the render helpers below

void gameInit() {
  buildSprites();
  gRng = micros() ^ 0x9E3779B9UL;
  if (gRng == 0) gRng = 0x12345678UL;
  gState = ST_TITLE;
  gMode = MODE_CLASSIC;
  gAnimUs = 0; gSelHoldUs = 0; gMenuCursor = 0; gCar = 0;
  resetRace();
}

fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  const uint32_t dt   = f.dtUs;
  const uint32_t dt16 = (dt << 16) / 1000000UL;
  gAnimUs += dt;

  // SELECT confirms the moment it is pressed (no wait for the release).
  // Global exit: hold SELECT for 1 s from anywhere (the confirm that the first
  // press triggered is harmless: the exit still fires after one second).
  if (in.held & fb::KEY_SELECT) gSelHoldUs += dt; else gSelHoldUs = 0;
  const bool selTap = (in.pressed & fb::KEY_SELECT) != 0;
  if (gSelHoldUs >= kExitHoldUs) return fb::GAME_EXIT;

  switch (gState) {

    case ST_TITLE:
      if (selTap) { gState = ST_MODE; gMenuCursor = (uint8_t)gMode; }
      break;

    case ST_MODE:
      if ((in.pressed & fb::KEY_LEFT) || (in.pressed & fb::KEY_RIGHT))
        gMenuCursor ^= 1;
      if (selTap) {
        gMode = (Mode)gMenuCursor;
        gState = ST_SELECT;
        gMenuCursor = gCar;
      }
      break;

    case ST_SELECT:
      if ((in.pressed & fb::KEY_LEFT) || (in.pressed & fb::KEY_RIGHT)) gMenuCursor ^= 1;
      if (in.pressed & fb::KEY_X) {
        gState = ST_MODE;
        gMenuCursor = (uint8_t)gMode;
      }
      if (selTap) { gCar = gMenuCursor; resetRace(); gState = ST_COUNT; }
      break;

    case ST_COUNT:
      gCdUs += dt;
      // Classic false-start on UP; Road Rage false-start on X
      if (gMode == MODE_ROADRAGE) {
        if ((in.held | in.pressed) & fb::KEY_X) gFalseStart = true;
      } else {
        if ((in.held | in.pressed) & fb::KEY_UP) gFalseStart = true;
      }
      if (gCdUs >= kCountdownUs) {
        gState  = ST_RACE;
        gRaceUs = 0;
        gLockoutUs = gFalseStart ? kFalseStartUs : 0;
      }
      break;

    case ST_RACE: {
      gRaceUs += dt;
      if (gLockoutUs > dt) gLockoutUs -= dt; else gLockoutUs = 0;

      // reaction-based launch boost
      // Classic reacts on UP press; Road Rage reacts on X press
      if (!gReactKnown) {
        const bool launchPress = (gMode == MODE_ROADRAGE)
          ? (in.pressed & fb::KEY_X)
          : (in.pressed & fb::KEY_UP);
        if (gLockoutUs == 0 && launchPress) {
          gReactKnown = true;
          gReactTxtUs = gRaceUs + 1500000UL;

          // Reaction = race clock at the instant the key actually went down
          // (not at the frame boundary), minus the time the GO frame needed to
          // reach the panel, so it measures the player and not the display.
          const uint8_t  launchKey = (gMode == MODE_ROADRAGE) ? fb::KEY_X : fb::KEY_UP;
          const uint32_t ago       = fb::frameStartUs() - fb::keyPressedAtUs(launchKey);
          uint32_t react = (ago < gRaceUs) ? (gRaceUs - ago) : 0;
          const uint32_t lat = fb::displayLatencyUs();
          react = (react > lat) ? (react - lat) : 0;

          if (react <= kLaunchPerfectUs) {
            gLaunchBoostUs = kBoostPerfectUs; gLaunchPct = 130; gReactGrade = 2;
          } else if (react <= kLaunchGoodUs) {
            gLaunchBoostUs = kBoostGoodUs;    gLaunchPct = 115; gReactGrade = 1;
          } else {
            gLaunchPct = 100; gReactGrade = 0;
          }
        } else if (gRaceUs > 1200000UL) {
          gReactKnown = true; gLaunchPct = 100;
        }
      }

      // N2O: LEFT = charge 1, RIGHT = charge 2 (both modes)
      // POWER SHIFT: SELECT
      if ((in.pressed & fb::KEY_LEFT)  && gN1Avail) { gN1Avail = false; gN1LeftUs = kN2ODurUs; }
      if ((in.pressed & fb::KEY_RIGHT) && gN2Avail) { gN2Avail = false; gN2LeftUs = kN2ODurUs; }
      if ((in.pressed & fb::KEY_SELECT) && gShiftAvail) {
        gShiftAvail = false; gShiftBoostUs = kShiftBoostUs;
      }

      stepRace(in, dt16, dt, true);

      if (gCrashed) {
        // Freeze world snapshot and place crash FX at player car
        gFinCx = camX();
        gFinPD16 = gPD16; gFinOD16 = gOD16;
        gFinScroll = m16ToRoadPx(gPD16) - gFinCx;
        gCrashX = (int16_t)gFinCx - (int16_t)(kCarW / 2);
        gCrashY = gPLaneY + (int16_t)(kCarH / 2);
        gCrashAnimUs = 0;
        gWaitUs = 0;
        gState = ST_CRASH;
        break;
      }

      if (gPD16 >= kRaceDist16) {
        gPFinished = true; gPTimeUs = gRaceUs;
        gFinCx = camX();
        gFinPD16 = gPD16; gFinOD16 = gOD16;
        gFinScroll = m16ToRoadPx(gPD16) - gFinCx;
        gState = ST_FINISH;
      }
    } break;

    case ST_CRASH:
      // World is frozen; only the crash animation advances
      gCrashAnimUs += dt;
      gWaitUs += dt;
      if (gWaitUs >= kCrashAnimUs) {
        gWin = false;
        gGapUs = 0;
        gNewBest = false;
        gMenuCursor = 0;
        gState = ST_RESULTS;
      }
      break;

    case ST_FINISH: {
      gRaceUs += dt;
      // Keep throttle live so the player can still blast off-screen
      stepRace(in, dt16, dt, true);

      const int32_t plrFront = gFinCx + m16ToFinishPx(gPD16 - gFinPD16);
      const int32_t oppFront = gFinCx + (m16ToPx(gFinOD16) - m16ToPx(gFinPD16))
                             + m16ToFinishPx(gOD16 - gFinOD16);
      const bool plrOff = (plrFront - kCarW > fb::SCREEN_W) || (plrFront < 0);
      const bool oppOff = (oppFront - kCarW > fb::SCREEN_W) || (oppFront < 0);

      if (!gOFinished && gOD16 >= kRaceDist16) {
        gOFinished = true; gOTimeUs = gRaceUs;
      }

      // Don't stall waiting for a distant/off-screen opponent:
      // once the player is off-screen, or a max timer elapses, proceed.
      const uint32_t finElapsed = gRaceUs - gPTimeUs;
      const bool oppFar = (gOD16 + (200L << 16) < gPD16) || (gPD16 + (200L << 16) < gOD16);
      if ((plrOff && (oppOff || oppFar || gOFinished)) || finElapsed >= kFinishMaxUs) {
        if (!gOFinished) {
          // Project remaining opponent time so results aren't blocked
          if (gOV16 > (1 << 14))
            gOTimeUs = gRaceUs + (uint32_t)(((int64_t)(kRaceDist16 - gOD16) * 1000000LL) / gOV16);
          else
            gOTimeUs = gRaceUs + 5000000UL;
          gOFinished = true;
        }
        gWaitUs = 0;
        gState = ST_WAIT;
      }
    } break;

    case ST_WAIT: {
      gWaitUs += dt;
      if (gWaitUs >= kPostOffUs) {
        if (!gOFinished) {
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
      if (in.pressed & fb::KEY_LEFT) {
        if (gMenuCursor == 0) gMenuCursor = 2; else --gMenuCursor;
      } else if (in.pressed & fb::KEY_RIGHT) {
        gMenuCursor = (uint8_t)((gMenuCursor + 1) % 3);
      }
      if (selTap) {
        if (gMenuCursor == 0) {          // Back to menu
          gState = ST_MODE;
          gMenuCursor = (uint8_t)gMode;
        } else if (gMenuCursor == 1) {   // Replay
          resetRace();
          gState = ST_COUNT;
        } else {                         // Exit
          return fb::GAME_EXIT;
        }
      }
      break;
  }
  if (gState == ST_COUNT || gState == ST_RACE)
    updateCamera(dt16, false);
  return fb::GAME_RUN;
}

// ---------------------------------------------------------------------------
// Render helpers
// ---------------------------------------------------------------------------
void drawCentered(U8G2& d, int16_t base, const char* s) {
  const int16_t w = (int16_t)d.getStrWidth(s);
  d.drawStr((int16_t)((fb::SCREEN_W - w) / 2), base, s);
}

void banner(U8G2& d, int16_t base, const char* s) {
  const int16_t w = (int16_t)d.getStrWidth(s);
  const int16_t x = (int16_t)((fb::SCREEN_W - w) / 2);
  d.setDrawColor(0);
  d.drawBox((u8g2_uint_t)(x - 2), (u8g2_uint_t)(base - 11), (u8g2_uint_t)(w + 4), 13);
  d.setDrawColor(1);
  d.drawStr(x, base, s);
}

// ---------------------------------------------------------------------------
// Direct frame-buffer drawing.  U8g2's full buffer is page-organised: byte
// buf[(y >> 3) * 128 + x], bit (y & 7), bit 0 = top.  The helpers below write
// that memory directly (with their own clipping) and honour the current U8g2
// draw colour (0 = clear, 1 = set, 2 = xor), so they are drop-in replacements
// for the former per-pixel / per-row U8g2 calls.
// ---------------------------------------------------------------------------
inline uint8_t drawCol(U8G2& d) { return d.getU8g2()->draw_color; }

inline void applyMask(uint8_t* p, uint8_t m, uint8_t col) {
  if (col == 1)      *p = (uint8_t)(*p | m);
  else if (col == 0) *p = (uint8_t)(*p & (uint8_t)~m);
  else               *p = (uint8_t)(*p ^ m);
}

// Inclusive rectangle, clipped to the screen.
void fillRect(U8G2& d, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
  if (x1 < x0 || y1 < y0) return;
  if (x1 < 0 || y1 < 0 || x0 >= fb::SCREEN_W || y0 >= fb::SCREEN_H) return;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > fb::SCREEN_W - 1) x1 = fb::SCREEN_W - 1;
  if (y1 > fb::SCREEN_H - 1) y1 = fb::SCREEN_H - 1;
  uint8_t* const buf = d.getBufferPtr();
  const uint8_t col = drawCol(d);
  for (int32_t pg = y0 >> 3; pg <= (y1 >> 3); ++pg) {
    const int32_t top = pg << 3;
    const int32_t lo  = (y0 > top)     ? (y0 - top) : 0;
    const int32_t hi  = (y1 < top + 7) ? (y1 - top) : 7;
    const uint8_t m   = (uint8_t)((0xFFu << lo) & (0xFFu >> (7 - hi)));
    uint8_t* const row = buf + pg * fb::SCREEN_W;
    if (col == 1)      for (int32_t x = x0; x <= x1; ++x) row[x] = (uint8_t)(row[x] | m);
    else if (col == 0) for (int32_t x = x0; x <= x1; ++x) row[x] = (uint8_t)(row[x] & (uint8_t)~m);
    else               for (int32_t x = x0; x <= x1; ++x) row[x] = (uint8_t)(row[x] ^ m);
  }
}

void clipPx(U8G2& d, int32_t x, int32_t y) {
  if (x < 0 || x >= fb::SCREEN_W || y < 0 || y >= fb::SCREEN_H) return;
  applyMask(d.getBufferPtr() + (y >> 3) * fb::SCREEN_W + x, (uint8_t)(1u << (y & 7)), drawCol(d));
}
void clipH(U8G2& d, int32_t x0, int32_t x1, int32_t y) { fillRect(d, x0, y, x1, y); }
void clipV(U8G2& d, int32_t x, int32_t y0, int32_t y1) { fillRect(d, x, y0, x, y1); }
void clipBox(U8G2& d, int32_t x0, int32_t y0, int32_t x1, int32_t y1) { fillRect(d, x0, y0, x1, y1); }

// Build the blit-ready column words of one sprite from XBM data (optionally mirrored).
void buildSprite(uint8_t id, const uint8_t* bmp, bool flip) {
  for (int32_t c = 0; c < kCarW; ++c) {
    const int32_t sc = flip ? (kCarW - 1 - c) : c;
    uint16_t col = 0;
    for (int32_t r = 0; r < kCarH; ++r)
      if (bmp[r * (kCarW / 8) + (sc >> 3)] & (1u << (sc & 7)))
        col = (uint16_t)(col | (1u << r));
    gSprCol[id][c] = col;
  }
}

void buildSprites() {
  buildSprite(SPR_SKY, kBmpSkyline, false);
  buildSprite(SPR_CHG, kBmpCharger, false);
  for (uint8_t i = 0; i < kNumTrafBmps; ++i) {
    buildSprite((uint8_t)(SPR_TRAF0 + i),      kTrafBmps[i], false);
    buildSprite((uint8_t)(SPR_TRAF_FLIP0 + i), kTrafBmps[i], true);
  }
}

// Transparent blit (set bits drawn in the current colour), clipped left/right.
void drawSprite(U8G2& d, int32_t x, int32_t y, uint8_t id) {
  if (x + kCarW <= 0 || x >= fb::SCREEN_W) return;
  if (y < 0 || y >= fb::SCREEN_H) return;
  uint8_t* const buf = d.getBufferPtr();
  const uint8_t  col = drawCol(d);
  const int32_t  pg  = y >> 3;
  const int32_t  sh  = y & 7;
  const int32_t  c0  = (x < 0) ? -x : 0;
  const int32_t  c1  = (x + kCarW > fb::SCREEN_W) ? (fb::SCREEN_W - x) : kCarW;
  const bool     two = (pg + 1) < (fb::SCREEN_H >> 3);
  for (int32_t c = c0; c < c1; ++c) {
    const uint32_t bits = (uint32_t)gSprCol[id][c] << sh;
    uint8_t* const p = buf + pg * fb::SCREEN_W + x + c;
    if (bits & 0xFFu) applyMask(p, (uint8_t)bits, col);
    if (two && (bits >> 8)) applyMask(p + fb::SCREEN_W, (uint8_t)(bits >> 8), col);
  }
}

void scrollSegs(U8G2& d, int16_t y, int16_t segLen, int16_t period, int32_t scrollPx) {
  int32_t xs = (-scrollPx) % period;
  if (xs < 0) xs += period;
  xs -= period;
  for (; xs < fb::SCREEN_W; xs += period)
    clipH(d, xs, xs + segLen - 1, y);
}

uint32_t hash32(uint32_t x) {
  x ^= x >> 16; x *= 0x7FEB352DUL;
  x ^= x >> 15; x *= 0x846CA68BUL;
  x ^= x >> 16;
  return x;
}

void drawBuilding(U8G2& d, int32_t x, int32_t w, int32_t h, uint32_t style, bool hideBehind) {
  const int32_t top = kRoadTop + 1 - h;
  const int32_t r   = x + w - 1;
  if (hideBehind) {
    d.setDrawColor(0);
    clipBox(d, x + 1, top + 1, r - 1, kRoadTop - 1);
    d.setDrawColor(1);
  }
  clipV(d, x, top, kRoadTop - 1);
  clipV(d, r, top, kRoadTop - 1);
  switch (style) {
    case 1:
      clipH(d, x, r, top);
      clipV(d, x + 3, top - 3, top - 1);
      clipV(d, r - 3, top - 3, top - 1);
      clipH(d, x + 3, r - 3, top - 3);
      break;
    case 2:
      clipH(d, x, r, top);
      clipV(d, x + w / 2, top - 4, top - 1);
      break;
    case 3: {
      const int32_t half = w / 2;
      for (int32_t i = 0; i <= half; ++i) {
        const int32_t dy = (i * 3) / half;
        clipPx(d, x + i, top - dy);
        clipPx(d, r - i, top - dy);
      }
    } break;
    default:
      clipH(d, x, r, top);
      break;
  }
}

void drawSkylineLayer(U8G2& d, int32_t scrollPx, int32_t pitch, int32_t wMin, int32_t wSpan,
                      int32_t hMin, int32_t hSpan, uint32_t salt, bool near) {
  const int32_t base = scrollPx + kBldBias;
  int32_t slot = base / pitch;
  int32_t sx   = slot * pitch - base;
  for (; sx < fb::SCREEN_W; sx += pitch, ++slot) {
    const uint32_t h = hash32((uint32_t)slot + salt);
    const int32_t  w = wMin + (int32_t)(h % (uint32_t)wSpan);
    const int32_t  x = sx + (int32_t)((h >> 8) % (uint32_t)(pitch - w + 1));
    const int32_t  t = hMin + (int32_t)((h >> 16) % (uint32_t)hSpan);
    uint32_t style = (h >> 24) & 3u;
    if (!near && style == 1u) style = 0u;
    drawBuilding(d, x, w, t, style, near);
  }
}

void drawBuildings(U8G2& d, int32_t roadScroll) {
  drawSkylineLayer(d, roadScroll >> kParFarShift,  14,  8, 5, 5, 5, 0x51ED270BUL, false);
  drawSkylineLayer(d, roadScroll >> kParNearShift, 18, 10, 6, 5, 6, 0x00000000UL, true);
}

void drawGantry(U8G2& d, int32_t worldD16, bool withFlag, int32_t scroll) {
  const int32_t gx = m16ToRoadPx(worldD16) - scroll;
  if (gx < 0 || gx > fb::SCREEN_W - 1) return;
  const u8g2_uint_t x = (u8g2_uint_t)gx;
  d.drawVLine(x, 16, 39);
  if (withFlag) {
    for (uint8_t r = 0; r < 3; ++r)
      for (uint8_t c = 0; c < 3; ++c)
        if (((r + c) & 1u) && gx + 1 + c * 2 <= fb::SCREEN_W - 1)
          d.drawBox((u8g2_uint_t)(gx + 1 + c * 2), (u8g2_uint_t)(16 + r * 2), 2, 2);
  }
}

void drawN2OBox(U8G2& d, int16_t x, const char* ch, bool avail, bool active) {
  d.setFont(u8g2_font_5x7_tr);
  if (avail) {
    d.drawBox((u8g2_uint_t)x, 56, 8, 8);
    d.setDrawColor(0);
    d.drawStr((u8g2_uint_t)(x + 2), 63, ch);
    d.setDrawColor(1);
  } else if (active) {
    if ((gAnimUs >> 17) & 1u) d.drawFrame((u8g2_uint_t)x, 56, 8, 8);
    d.drawStr((u8g2_uint_t)(x + 2), 63, ch);
  } else {
    d.drawFrame((u8g2_uint_t)x, 56, 8, 8);
  }
}

// ---------------------------------------------------------------------------
// Screens
// ---------------------------------------------------------------------------
void renderTitle(U8G2& d) {
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

void renderMode(U8G2& d) {
  d.setFont(u8g2_font_6x13B_tr);
  drawCentered(d, 11, "SELECT MODE");

  const int16_t x0 = 4, x1 = 66;
  const int16_t cardW = 58, cardH = 36;

  // Classic card — solid fill + inverted text when selected
  if (gMenuCursor == 0) {
    d.setDrawColor(1);
    d.drawBox(x0, 16, cardW, cardH);
    d.setDrawColor(0);
  } else {
    d.setDrawColor(1);
    d.drawFrame(x0, 16, cardW, cardH);
  }
  d.setFont(u8g2_font_6x13B_tr);
  d.drawStr(x0 + 8, 28, "CLASSIC");
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(x0 + 10, 38, "2 LANE");
  d.drawStr(x0 + 12, 47, "DRAG");
  d.setDrawColor(1);

  // Road Rage card — solid fill + inverted text when selected
  if (gMenuCursor == 1) {
    d.setDrawColor(1);
    d.drawBox(x1, 16, cardW, cardH);
    d.setDrawColor(0);
  } else {
    d.setDrawColor(1);
    d.drawFrame(x1, 16, cardW, cardH);
  }
  d.setFont(u8g2_font_6x13B_tr);
  d.drawStr(x1 + 2, 28, "ROAD RAGE");
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(x1 + 10, 38, "4 LANE");
  d.drawStr(x1 + 6, 47, "+TRAFFIC");
  d.setDrawColor(1);

  d.setFont(u8g2_font_5x7_tr);
  drawCentered(d, 62, "L/R:CHOOSE  SEL:NEXT");
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
  drawCentered(d, 63, "L/R:CHOOSE  SEL:RACE  X:BACK");
}

void renderScene(U8G2& d, bool hud) {
  const bool    over   = (gState == ST_FINISH || gState == ST_WAIT || gState == ST_CRASH);
  const int32_t cx     = over ? gFinCx : camX();
  const int32_t scroll = over ? gFinScroll : m16ToRoadPx(gPD16) - cx;
  const int32_t kmh    = ((gPV16 * 18) / 5) >> 16;

  // Skyline scenery only in Classic; Road Rage uses the full height for lanes
  if (gMode == MODE_CLASSIC) {
    drawBuildings(d, scroll);
  }

  if (gMode == MODE_CLASSIC) {
    d.drawHLine(0, kRoadTop, fb::SCREEN_W);
    scrollSegs(d, kLaneDiv, kDashLen, kDashPeriod, scroll);
    d.drawHLine(0, kRoadBot, fb::SCREEN_W);
  } else {
    // ---- Redesigned 4-lane road ----
    // Outer edges
    d.drawHLine(0, kRRRoadTop, fb::SCREEN_W);
    d.drawHLine(0, kRRRoadBot, fb::SCREEN_W);

    // Dashed lane markers between 0-1 and between 2-3
    scrollSegs(d, 27, 16, 28, scroll);   // between lane 0 and 1
    scrollSegs(d, 47, 16, 28, scroll);   // between lane 2 and 3

    // Solid double median separating top 2 (same-dir) from bottom 2 (opposite)
    d.drawHLine(0, kRRMedianY, fb::SCREEN_W);
    d.drawHLine(0, kRRMedianY + 1, fb::SCREEN_W);
  }

  drawGantry(d, 0, false, scroll);
  drawGantry(d, kRaceDist16, true, scroll);

  // Traffic cars (Road Rage) — face the direction they are actually moving
  // v16 >= 0 (same dir as player): face right (finish line)
  // v16 <  0 (oncoming): face left (toward the player)
  if (gMode == MODE_ROADRAGE) {
    d.setBitmapMode(1);
    for (uint8_t i = 0; i < kMaxTraffic; ++i) {
      const Traf& t = gTraf[i];
      if (!t.alive) continue;
      int32_t tx;
      if (over) {
        tx = cx + m16ToFinishPx(t.d16 - gFinPD16) - kCarW;
      } else {
        tx = cx + m16ToPx(t.d16) - m16ToPx(gPD16) - kCarW;
      }
      int16_t ty = kRRLaneY[t.lane];
      const uint8_t si = (uint8_t)(t.sprite % kNumTrafBmps);
      if (t.v16 >= 0) {
        drawSprite(d, tx, ty, (uint8_t)(SPR_TRAF0 + si));        // same direction → face forward
      } else {
        drawSprite(d, tx, ty, (uint8_t)(SPR_TRAF_FLIP0 + si));   // oncoming → face the player
      }
    }
  }

  // Player & opponent cars
  int32_t plrFront, oppFront;
  if (over) {
    plrFront = cx + m16ToFinishPx(gPD16 - gFinPD16);
    oppFront = cx + (m16ToPx(gFinOD16) - m16ToPx(gFinPD16))
             + m16ToFinishPx(gOD16 - gFinOD16);
  } else {
    plrFront = cx;
    oppFront = cx + m16ToPx(gOD16) - m16ToPx(gPD16);
  }
  const int32_t oppLeft = oppFront - kCarW;
  const int32_t plrLeft = plrFront - kCarW;

  const uint8_t plrSpr = (gCar == 0) ? SPR_SKY : SPR_CHG;
  const uint8_t oppSpr = (gCar == 0) ? SPR_CHG : SPR_SKY;

  d.setBitmapMode(1);
  if (gMode == MODE_CLASSIC) {
    drawSprite(d, oppLeft, kOppCarY, oppSpr);
    if (!over && ((gAnimUs >> 17) & 1u)) {
      const int16_t ay = kOppCarY + kCarH / 2;
      if (oppLeft >= fb::SCREEN_W)
        d.drawTriangle(127, ay - 4, 127, ay + 4, 121, ay);
      else if (oppFront <= 0)
        d.drawTriangle(0, ay - 4, 0, ay + 4, 6, ay);
    }
    drawSprite(d, plrLeft, kPlrCarY, plrSpr);
  } else {
    // Road Rage: Y from smooth lane animation
    int16_t oy = gOLaneY;
    int16_t py = gPLaneY;
    drawSprite(d, oppLeft, oy, oppSpr);
    if (!over && ((gAnimUs >> 17) & 1u)) {
      const int16_t ay = oy + kCarH / 2;
      if (oppLeft >= fb::SCREEN_W)
        d.drawTriangle(127, ay - 4, 127, ay + 4, 121, ay);
      else if (oppFront <= 0)
        d.drawTriangle(0, ay - 4, 0, ay + 4, 6, ay);
    }
    drawSprite(d, plrLeft, py, plrSpr);
  }

  // Exhaust / N2O flame
  if (plrLeft >= 1 && plrLeft <= fb::SCREEN_W - 1) {
    const uint8_t nAct = (gN1LeftUs ? 1 : 0) + (gN2LeftUs ? 1 : 0);
    int16_t flameY = (gMode == MODE_CLASSIC) ? (kPlrCarY + 3) : (gPLaneY + 3);
    if (nAct) {
      int32_t fl = 4 + (int32_t)((gAnimUs >> 15) & 3u);
      if (nAct > 1) fl += 2;
      int32_t tip = plrLeft - 1 - fl;
      if (tip < 0) tip = 0;
      d.drawTriangle((int16_t)(plrLeft - 1), flameY,
                     (int16_t)(plrLeft - 1), flameY + 4,
                     (int16_t)tip, flameY + 2);
    } else if (gThrottleVis && ((gAnimUs >> 16) & 1u)) {
      clipPx(d, plrLeft - 2, flameY + 3);
    }
  }

  if (!hud) return;

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

  d.drawStr(1, 63, "N2O");
  drawN2OBox(d, 19, "P", gShiftAvail, gShiftBoostUs > 0);
  drawN2OBox(d, 28, "1", gN1Avail, gN1LeftUs > 0);
  drawN2OBox(d, 37, "2", gN2Avail, gN2LeftUs > 0);

  // Progress bar
  d.drawFrame(48, 57, 80, 7);
  int32_t pw = (int32_t)(((int64_t)gPD16 * 78) / kRaceDist16);
  if (pw > 78) pw = 78;
  if (pw > 0) d.drawBox(49, 58, (u8g2_uint_t)pw, 5);
  int32_t ow = (int32_t)(((int64_t)gOD16 * 78) / kRaceDist16);
  if (ow > 78) ow = 78;
  if (ow > 0) {
    d.setDrawColor(2);
    d.drawVLine((u8g2_uint_t)(49 + ow), 57, 7);
    d.setDrawColor(1);
  }
  d.setBitmapMode(1);
  d.drawXBM(120, 49, 7, 7, kBmpFlag);

  // Road Rage lane indicator
  if (gMode == MODE_ROADRAGE) {
    d.setFont(u8g2_font_5x7_tr);
    char lb[6];
    lb[0] = 'L'; lb[1] = (char)('1' + gPLane); lb[2] = 0;
    d.drawStr(110, 15, lb);
  }
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
    d.setFont(u8g2_font_logisoso24_tn);
    d.drawStr((u8g2_uint_t)(104 - (int16_t)d.getStrWidth(b) / 2), 46, b);
  }
  d.setFont(u8g2_font_5x7_tr);
  if (gMode == MODE_CLASSIC)
    drawCentered(d, 63, "UP:GAS  L/R:N2O  SEL:PWR");
  else
    drawCentered(d, 63, "X:GAS  U/D:LANE  L/R:N2O");
}

void renderRaceOverlay(U8G2& d) {
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

void renderCrash(U8G2& d) {
  // Expanding burst + debris; world underneath is already frozen
  const uint32_t t = gCrashAnimUs;
  const int16_t cx = gCrashX;
  const int16_t cy = gCrashY;

  // Flash the impact zone for the first ~200 ms
  if (t < 200000UL) {
    if ((t / 40000UL) & 1u) {
      d.setDrawColor(1);
      d.drawBox((u8g2_uint_t)(cx - 14), (u8g2_uint_t)(cy - 10), 28, 20);
      d.setDrawColor(0);
      d.drawBox((u8g2_uint_t)(cx - 10), (u8g2_uint_t)(cy - 6), 20, 12);
      d.setDrawColor(1);
    }
  }

  // Expanding ring (approximate circles with boxes/pixels)
  const int16_t r = (int16_t)(4 + (t / 50000UL));
  if (r < 40) {
    // 8-point diamond / star burst
    for (int16_t i = -r; i <= r; i += 2) {
      int16_t j = r - (i < 0 ? -i : i);
      clipPx(d, cx + i, cy - j);
      clipPx(d, cx + i, cy + j);
      clipPx(d, cx - j, cy + i);
      clipPx(d, cx + j, cy + i);
    }
  }

  // Debris particles flying outward
  static const int8_t kDx[8] = { -3, -2, -1, 0, 1, 2, 3, 2 };
  static const int8_t kDy[8] = { -2,  1, -3, 2,-1, 3, 0,-2 };
  const int16_t spread = (int16_t)(t / 30000UL);
  for (uint8_t i = 0; i < 8; ++i) {
    int16_t px = cx + (int16_t)kDx[i] * spread / 2;
    int16_t py = cy + (int16_t)kDy[i] * spread / 2;
    if (spread > 2) {
      clipPx(d, px, py);
      clipPx(d, px + 1, py);
      clipPx(d, px, py + 1);
    }
  }

  // Secondary sparks late in the anim
  if (t > 400000UL && t < 1200000UL) {
    for (uint8_t i = 0; i < 6; ++i) {
      int16_t sx = cx + (int16_t)(((i * 17 + (t >> 14)) & 15) - 7);
      int16_t sy = cy + (int16_t)(((i * 13 + (t >> 15)) & 11) - 5);
      if ((t >> 16) & 1u) clipPx(d, sx, sy);
    }
  }

  // Text banner after a short delay
  if (t > 300000UL) {
    d.setFont(u8g2_font_7x13B_tr);
    banner(d, 14, "CRASH!");
  }
  if (t > 700000UL) {
    d.setFont(u8g2_font_5x7_tr);
    drawCentered(d, 62, "RACE OVER");
  }
}

void renderResults(U8G2& d) {
  char b[8];
  d.setFont(u8g2_font_7x13B_tr);
  if (gCrashed) {
    drawCentered(d, 12, "YOU LOST");
    d.setFont(u8g2_font_5x7_tr);
    drawCentered(d, 22, "YOU WRECKED YOUR CAR");
  } else if (gWin) {
    drawCentered(d, 13, "YOU WIN!");
  } else {
    drawCentered(d, 13, "YOU LOST");
  }
  d.setFont(u8g2_font_5x7_tr);
  if (gNewBest && !gCrashed && ((gAnimUs >> 17) & 1u)) drawCentered(d, 21, "NEW BEST!");

  d.drawStr(4, 30, "YOU");
  d.setFont(u8g2_font_logisoso16_tn);
  if (gCrashed) {
    d.drawStr(26, 40, "--");
  } else {
    fmtSec2(b, gPTimeUs);
    d.drawStr(26, 40, b);
  }
  const int16_t w = (int16_t)d.getStrWidth(gCrashed ? "--" : b);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr((u8g2_uint_t)(26 + w + 2), 40, "s");

  d.drawStr(4, 52, "OPP");
  if (!gCrashed) {
    fmtSec2(b, gOTimeUs);
    d.drawStr(26, 52, b);
    d.drawStr((u8g2_uint_t)(26 + (int16_t)d.getStrWidth(b) + 2), 52, "s");

    char g[9];
    g[0] = gWin ? '+' : '-';
    fmtSec2(g + 1, gGapUs);
    uint8_t L = 0; while (g[L]) ++L;
    g[L] = 's'; g[L + 1] = 0;
    d.drawStr(62, 52, "GAP");
    d.drawStr(82, 52, g);
  } else {
    d.drawStr(26, 52, "---");
  }

  // Three options: MENU | REPLAY | EXIT
  d.drawStr(8, 62, "MENU");
  d.drawStr(48, 62, "REPLAY");
  d.drawStr(96, 62, "EXIT");
  // Cursor indicator
  if (gMenuCursor == 0)      d.drawStr(2, 62, ">");
  else if (gMenuCursor == 1) d.drawStr(42, 62, ">");
  else                       d.drawStr(90, 62, ">");
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
  d.setFontMode(1);
  d.setBitmapMode(0);
  switch (gState) {
    case ST_TITLE:   renderTitle(d);                 break;
    case ST_MODE:    renderMode(d);                  break;
    case ST_SELECT:  renderSelect(d);                break;
    case ST_COUNT:   renderScene(d, false); renderCountdown(d); break;
    case ST_RACE:    renderScene(d, true);  renderRaceOverlay(d); break;
    case ST_FINISH:  renderScene(d, true);  renderFinish(d);      break;
    case ST_WAIT:    renderScene(d, true);  break;
    case ST_CRASH:   renderScene(d, true);  renderCrash(d);       break;
    case ST_RESULTS: renderResults(d);               break;
  }
  if (gSelHoldUs > 120000UL) renderExitCue(d);
}

}  // namespace

FB_GAME(FastAndFurious, gameInit, gameUpdate, gameRender, nullptr);
