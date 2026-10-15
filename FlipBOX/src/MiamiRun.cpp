// src/MiamiRun.cpp  --  Miami Street Run   (flipBOX)
//
// MILESTONE 1: driving prototype ("does it feel fast?").   (rev 2: controls remapped,
// road markings spaced evenly)
// Top-down arcade street racer inspired by the opening race of "2 Fast 2 Furious".
// This build has ONE car on a hard-coded test loop: fixed-step arcade slip physics,
// drift kick, basic nitro, look-ahead camera, title and pause menus.
//
// Controls (race)   LEFT / RIGHT  steer
//                   UP            throttle (or automatic, see the title menu)
//                   DOWN          brake while rolling forward, reverse from a standstill
//                   X             handbrake: tap while steering at speed to kick into a drift,
//                                 hold to keep sliding
//                   SELECT        pause
//                   Nitro         built, but not bound to a button yet (see kNitroKey)
// Title / Pause     UP / DOWN choose, SELECT confirm.
//                   Quit is on both menus (that is the exit path back to the launcher).

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

#include <math.h>
#include <stdint.h>
#include <stdio.h>

namespace {

// =============================================================================
// Fixed-point conventions
//   Q16.16  positions (px), velocities (px/s), accelerations (px/s^2), rates (1/s)
//   Q15     sine / cosine table entries and unit vectors
//   Q8      steering lock (-256 .. +256)
//   Q4      route centre line (1/16 px) so collision stays smooth on curves
//   angle   uint32, 2^32 = 360 deg.  The top 16 bits are "angle16" (65536 = 360 deg);
//           0 = east (+x), and angles grow clockwise on screen (y points down).
//   Signed left shifts are written as multiplications (they are undefined for
//   negative values in C++11); right shifts of negatives are arithmetic on Xtensa.
// =============================================================================
constexpr int32_t kOne = 65536;                                       // 1.0 in Q16.16
constexpr int32_t qf(int32_t num, int32_t den) { return (int32_t)(((int64_t)num * 65536) / den); }
constexpr int32_t deg16(int32_t deg) { return (deg * 65536) / 360; }  // degrees -> angle16 units

// =============================================================================
// TUNING  (units in comments; everything that changes the feel is here)
// =============================================================================
// ---- Simulation ---------------------------------------------------------------
constexpr uint32_t kStepUs   = 8000;     // fixed physics step, microseconds (125 Hz)
constexpr int      kMaxSteps = 6;        // max physics steps per frame (spiral-of-death cap)
constexpr int32_t  kDt16     = (int32_t)(((int64_t)kStepUs << 16) / 1000000);  // step, Q16.16 s

// ---- Engine and body ----------------------------------------------------------
constexpr int32_t kTopSpeed     = 110 * kOne;   // px/s, flat-out without nitro
constexpr int32_t kAccel        = 150 * kOne;   // px/s^2, engine force at full throttle
constexpr int32_t kDrag         = (int32_t)(((int64_t)kAccel << 16) / kTopSpeed);  // 1/s: top speed = accel / drag
constexpr int32_t kBrake        = 300 * kOne;   // px/s^2, braking
constexpr int32_t kBrakeMinSpeed = 8 * kOne;    // px/s, below this DOWN reverses instead of braking
constexpr int32_t kReverseAccel = 90 * kOne;    // px/s^2
constexpr int32_t kReverseTop   = 35 * kOne;    // px/s
constexpr int32_t kReverseDelayUs = 200000;     // us: after braking to a stop with DOWN held, reverse starts this much later

// ---- Nitro --------------------------------------------------------------------
// Nitro is fully built (tank, burn, kick, flame, HUD) but has no button yet. When the extra
// button exists, set kNitroKey to its fb:: key mask and the whole system switches on.
constexpr uint8_t kNitroKey = 0;                // 0 = not bound to any button
constexpr int32_t kNitroAccelMul   = qf(3, 2);  // engine force multiplier while burning (top speed ~165 px/s)
constexpr int32_t kNitroBurnUs     = 1800000;   // us, one charge
constexpr int32_t kNitroBufferUs   = 200000;    // us, an X press during a burn is remembered this long
constexpr int32_t kNitroKick       = 15 * kOne; // px/s, instant forward shove when a charge fires
constexpr int32_t kNitroRechargeUs = 6000000;   // us per charge (M1 only; M3 replaces it with drift banking)
constexpr int      kNitroMax       = 3;         // charges in the tank
constexpr int      kNitroStart     = 2;         // charges at the start

// ---- Grip and drifting --------------------------------------------------------
constexpr int32_t kGrip             = 9 * kOne;     // 1/s, sideways-speed bleed (higher = more grip)
constexpr int32_t kGripDrift        = qf(17, 10);   // 1/s, grip while drifting
constexpr int32_t kGripNitroMul     = qf(8, 10);    // grip multiplier under nitro (twitchier)
constexpr int32_t kGripRoughMul     = qf(7, 10);    // grip multiplier on the shoulder
constexpr int32_t kOversteerSpeed   = 95 * kOne;    // px/s, above this hard steering starts to slide
constexpr int32_t kOversteerSteer   = 200;          // Q8 steering lock that counts as "hard"
constexpr int32_t kOversteerGripMul = qf(6, 10);    // grip multiplier during natural oversteer
constexpr int32_t kSlipToForward    = qf(6, 10);    // share of bled sideways speed turned into forward speed (capped at top speed)
constexpr int32_t kDriftMinSpeed    = 60 * kOne;    // px/s, handbrake kick needs at least this
constexpr int32_t kDriftKickUs      = 450000;       // us of low grip after an X tap
constexpr int32_t kDriftHoldUs      = 120000;       // us of low grip kept alive while X stays held
constexpr int32_t kDriftSnap        = deg16(10);    // heading snap into the turn on a kick (angle16), only while steering
constexpr int32_t kDriftYawMul      = qf(125, 100); // steering authority multiplier while drifting
constexpr int32_t kDriftScrub       = 45 * kOne;    // px/s^2 speed lost while handbrake-sliding
constexpr int32_t kDriftFlagSlip    = 18 * kOne;    // px/s sideways speed that counts as "drifting" (HUD)
constexpr int32_t kDriftFlagSpeed   = 40 * kOne;    // px/s minimum speed for the drifting flag

// ---- Steering -----------------------------------------------------------------
constexpr int32_t kSteerUp   = 256 * (int32_t)kStepUs / 80000;   // Q8 per step: ~80 ms to full lock
constexpr int32_t kSteerBack = 256 * (int32_t)kStepUs / 50000;   // Q8 per step: ~50 ms back to centre
constexpr int      kYawN = 6;
// Yaw rate at full lock, by speed: slow cars barely turn, mid speed turns best, top speed a bit less.
const int32_t kYawSpeed[kYawN] = { 0, 10 * kOne, 35 * kOne, 110 * kOne, 170 * kOne, 220 * kOne };  // px/s
const int32_t kYawRate[kYawN]  = { 0, deg16(120), deg16(185), deg16(140), deg16(112), deg16(100) }; // angle16/s
constexpr int32_t kAssistMinSpeed = 30 * kOne;   // px/s, auto-countersteer only above this
constexpr int32_t kAssistGain     = 22000;       // angle16/s per radian of slip (auto-countersteer when not steering)
constexpr int32_t kAssistMax      = 25000;       // angle16/s cap

// ---- World / surface ----------------------------------------------------------
constexpr int32_t kRoadHalf  = 14;               // px, road half width (full grip)
constexpr int32_t kShoulder  = 6;                // px, rough shoulder beyond the road edge
constexpr int32_t kWallHalf  = kRoadHalf + kShoulder;  // px, centre line to wall
constexpr int32_t kCarRadius = 3;                // px, collision circle
constexpr int32_t kRoughDragMul   = qf(22, 10);  // drag multiplier on the shoulder
constexpr int32_t kRoughAccelMul  = qf(7, 10);   // engine multiplier on the shoulder
constexpr int32_t kWallRestitution = qf(25, 100); // bounce
constexpr int32_t kScrapePerSpeed = qf(4, 1000);  // tangential speed lost per px/s of impact speed
constexpr int32_t kScrapeMax      = qf(6, 10);    // cap on that loss per contact step
constexpr int32_t kWallRub        = qf(8, 10);    // 1/s, tangential speed lost while touching a wall

// ---- Camera -------------------------------------------------------------------
constexpr int32_t kLookMul  = qf(3, 10);         // s of velocity the camera leads the car by
constexpr int32_t kLookMaxX = 34;                // px, max horizontal lead
constexpr int32_t kLookMaxY = 20;                // px, max vertical lead
constexpr int32_t kCamAlpha = qf(85, 1000);      // follow smoothing per step (~90 ms time constant)

// ---- Presentation -------------------------------------------------------------
constexpr int32_t kSpeedDisplayMul = 2;          // HUD number = px/s * this
constexpr int     kSpeedLineMinPx  = 125;        // px/s, streaks start above this (or under nitro)
constexpr int32_t kTitleSpeed      = 80 * kOne;  // px/s, title background scroll
constexpr float kDashPeriodCentre = 24.0f;       // px, centre-line dash period (fitted per loop so it closes evenly)
constexpr float kDashDutyCentre   = 0.40f;       // share of the period that is painted
constexpr float kDashPeriodKerb   = 12.0f;       // px, road-edge dash period
constexpr float kDashDutyKerb     = 0.50f;

// =============================================================================
// Route: a hard-coded test loop (M2 replaces it with the real Miami route)
// =============================================================================
// Control polygon of the centre line in world pixels, travelled in this order and closed
// back to the first point. Each corner is rounded with the radius in kRtR.
constexpr int16_t kRtX[] = { 100, 560, 560, 480, 480, 260, 260, 400, 400, 100 };
constexpr int16_t kRtY[] = { 350, 350, 250, 250,  70,  70, 140, 140, 230, 230 };
constexpr int16_t kRtR[] = {  40,  50,  30,  30,  45,  34,  34,  26,  40,  40 };
constexpr int kRtN = (int)(sizeof(kRtX) / sizeof(kRtX[0]));
constexpr int32_t kSpawnX = 220;                 // px, start position on the first straight
constexpr int32_t kSpawnY = 350;
constexpr float kArcStepRad     = 0.2618f;       // 15 degrees per arc segment
constexpr float kMaxStraightSeg = 12.0f;         // px, straights are cut into pieces for dashes

constexpr int kMaxVerts = 192;
constexpr int kDashCh   = 3;                     // dashed lines: 0 centre, 1 left road edge, 2 right road edge
constexpr int kMaxRuns  = 160;

// =============================================================================
// State (ALL of it is reset in gameInit())
// =============================================================================
enum State : uint8_t { ST_TITLE = 0, ST_PLAY = 1, ST_PAUSE = 2 };

struct Vert {
  int16_t  cx, cy;                 // centre line, Q4 px
  int16_t  kLx, kLy, kRx, kRy;     // road edge (kerb), left / right of travel, whole px
  int16_t  wLx, wLy, wRx, wRy;     // wall, left / right of travel, whole px
  uint16_t len;                    // length of the segment to the next vertex, Q4 px
};

struct Car {
  int32_t  x, y;                   // Q16.16 px
  int32_t  vx, vy;                 // Q16.16 px/s
  uint32_t ang;                    // 2^32 = 360 deg
  int32_t  steer;                  // Q8, smoothed steering lock
  int32_t  driftUs;                // us of low grip left
  int32_t  nitroBurnUs;            // us of nitro left
  int32_t  nitroBufUs;             // us an X press is still remembered
  int32_t  nitroRechargeUs;        // us towards the next free charge
  int32_t  revWaitUs;              // us stopped with DOWN held after braking (reverse waits for this)
  int32_t  d2;                     // squared distance to the centre line (Q4 px squared)
  int16_t  seg;                    // nearest route segment
  uint8_t  nitro;                  // charges in the tank
  bool     drifting;
  bool     rough;
  bool     brakeLatch;             // DOWN was pressed while rolling forward: it is a brake, not (yet) reverse
};

struct CarCmd {
  int8_t steerDir;                 // -1 left, 0, +1 right
  bool   throttle;
  bool   brake;                    // DOWN
  bool   handbrake;                // X
};

State    gState;
int      gMenuSel;
bool     gAutoThrottle;
bool     gQuit;
uint32_t gAnim;                    // frame counter, cosmetic animation only
uint32_t gAcc;                     // physics accumulator, us
Car      gCar;
int32_t  gCamX, gCamY;             // Q16.16 camera centre
int      gSpeedDisp;               // HUD speed, cached in update()
bool     gReversing;               // HUD: rolling backwards
int      gTitleSeg;                // title background: segment and distance along it
int32_t  gTitleT;                  // Q16.16 px
int32_t  gTitleX, gTitleY;         // Q16.16 title camera centre

// Route data (built in init)
Vert     gV[kMaxVerts];
bool     gStraight[kMaxVerts];     // segment i (vertex i -> i+1) lies on a straight
int      gN;
uint8_t  gRunA[kMaxRuns], gRunB[kMaxRuns];   // merged quads for the corridor fill
int      gRuns;
int      gStartV;                  // vertex where the start line is drawn
int32_t  gCum[kDashCh][kMaxVerts]; // arc length at each vertex along the centre / left edge / right edge, Q8 px
int32_t  gTotal[kDashCh];          // full loop length of each of those lines, Q8 px
int32_t  gDashPeriod[kDashCh];     // fitted dash period, Q8 px
int32_t  gDashLen[kDashCh];        // painted part of the period, Q8 px

// Tables (built in init)
int16_t  gSin[256];                // Q15 sine, one turn
uint16_t gSprBody[32][11];         // car sprite rows, 32 headings, 11x11, bit c = column c
uint16_t gSprHalo[32][11];         // dark outline mask of the same

uint8_t* gFb;                      // frame buffer, valid only inside render()

// =============================================================================
// Small helpers
// =============================================================================
inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }
inline int32_t mulQ(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 16); }
inline int32_t mulDt(int32_t rate) { return (int32_t)(((int64_t)rate * kDt16) >> 16); }  // rate * step
inline int32_t clampI(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint32_t isqrt32(uint32_t v) {
  uint32_t r = 0, bit = 1u << 30;
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
    else r >>= 1;
    bit >>= 2;
  }
  return r;
}

uint32_t hash2(int32_t x, int32_t y) {
  uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}

int floorDiv(int v, int d) { return v >= 0 ? v / d : -((-v + d - 1) / d); }

int16_t rnd16(float v) { return (int16_t)floorf(v + 0.5f); }

// Sine / cosine of angle16 (0..65535), Q15, linearly interpolated.
int32_t sinQ15(uint32_t a16) {
  const uint32_t i = a16 >> 8;
  const int32_t s0 = gSin[i & 255u], s1 = gSin[(i + 1u) & 255u];
  return s0 + (((s1 - s0) * (int32_t)(a16 & 255u)) >> 8);
}
int32_t cosQ15(uint32_t a16) { return sinQ15((a16 + 16384u) & 0xFFFFu); }

// Yaw rate at full lock for a speed (px/s Q16.16), angle16 per second.
int32_t yawRate16(int32_t sp) {
  if (sp <= kYawSpeed[0]) return kYawRate[0];
  for (int i = 1; i < kYawN; ++i) {
    if (sp < kYawSpeed[i]) {
      const int32_t s0 = kYawSpeed[i - 1], r0 = kYawRate[i - 1];
      return r0 + (int32_t)(((int64_t)(kYawRate[i] - r0) * (sp - s0)) / (kYawSpeed[i] - s0));
    }
  }
  return kYawRate[kYawN - 1];
}

// =============================================================================
// Building the tables and the route (init time only: floats are fine here)
// =============================================================================
void buildSinTable() {
  for (int i = 0; i < 256; ++i) {
    const float s = sinf((float)i * (6.2831853f / 256.0f)) * 32767.0f;
    gSin[i] = (int16_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
  }
}

void routePush(float x, float y, bool straightLeg) {
  if (gN >= kMaxVerts) return;
  const int32_t qx = (int32_t)floorf(x * 16.0f + 0.5f);
  const int32_t qy = (int32_t)floorf(y * 16.0f + 0.5f);
  if (gN > 0) {
    const int32_t dx = qx - gV[gN - 1].cx, dy = qy - gV[gN - 1].cy;
    if (dx * dx + dy * dy < 64) return;          // closer than half a pixel: duplicate
    gStraight[gN - 1] = straightLeg;             // flag describes the segment that ends here
  }
  gV[gN].cx = (int16_t)qx;
  gV[gN].cy = (int16_t)qy;
  ++gN;
}

// Round the corners of the control polygon into arcs and cut the straights into short pieces.
void buildRoute() {
  float t1x[kRtN], t1y[kRtN], t2x[kRtN], t2y[kRtN];
  float ccx[kRtN], ccy[kRtN], a0[kRtN], phi[kRtN], sg[kRtN];
  for (int i = 0; i < kRtN; ++i) {
    const int ip = (i + kRtN - 1) % kRtN, inx = (i + 1) % kRtN;
    float ux = (float)(kRtX[i] - kRtX[ip]), uy = (float)(kRtY[i] - kRtY[ip]);
    float wx = (float)(kRtX[inx] - kRtX[i]), wy = (float)(kRtY[inx] - kRtY[i]);
    const float ul = sqrtf(ux * ux + uy * uy), wl = sqrtf(wx * wx + wy * wy);
    ux /= ul; uy /= ul; wx /= wl; wy /= wl;
    float dot = ux * wx + uy * wy;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    const float p = acosf(dot);                                  // turn angle
    const float s = (ux * wy - uy * wx) >= 0.0f ? 1.0f : -1.0f;  // +1 = clockwise on screen
    const float r = (float)kRtR[i];
    const float t = (p > 0.01f) ? r * tanf(p * 0.5f) : 0.0f;     // tangent length
    const float px = (float)kRtX[i], py = (float)kRtY[i];
    t1x[i] = px - ux * t; t1y[i] = py - uy * t;
    t2x[i] = px + wx * t; t2y[i] = py + wy * t;
    ccx[i] = t1x[i] + (-uy * s) * r;                             // arc centre, on the inside of the turn
    ccy[i] = t1y[i] + (ux * s) * r;
    a0[i] = atan2f(t1y[i] - ccy[i], t1x[i] - ccx[i]);
    phi[i] = p; sg[i] = s;
  }
  gN = 0;
  for (int i = 0; i < kRtN; ++i) {
    const int inx = (i + 1) % kRtN;
    const float r = (float)kRtR[i];
    if (phi[i] > 0.01f) {
      int steps = (int)ceilf(phi[i] / kArcStepRad);
      if (steps < 1) steps = 1;
      for (int k = 0; k <= steps; ++k) {
        const float a = a0[i] + sg[i] * phi[i] * (float)k / (float)steps;
        routePush(ccx[i] + r * cosf(a), ccy[i] + r * sinf(a), k == 0);
      }
    } else {
      routePush((float)kRtX[i], (float)kRtY[i], true);
    }
    const float lx = t1x[inx] - t2x[i], ly = t1y[inx] - t2y[i];
    const float len = sqrtf(lx * lx + ly * ly);
    const int m = (int)ceilf(len / kMaxStraightSeg);
    for (int j = 1; j < m; ++j)
      routePush(t2x[i] + lx * (float)j / (float)m, t2y[i] + ly * (float)j / (float)m, true);
  }
  if (gN > 1) {                                                  // close the loop
    const int32_t dx = gV[gN - 1].cx - gV[0].cx, dy = gV[gN - 1].cy - gV[0].cy;
    if (dx * dx + dy * dy < 64) --gN;
    else gStraight[gN - 1] = true;
  }
}

// Road edge / wall offset points (mitred) and segment lengths.
void buildOffsets() {
  const float k16 = 1.0f / 16.0f;
  for (int i = 0; i < gN; ++i) {
    const int ip = (i == 0) ? gN - 1 : i - 1;
    const int inx = (i + 1 == gN) ? 0 : i + 1;
    const float x = (float)gV[i].cx * k16, y = (float)gV[i].cy * k16;
    float d1x = x - (float)gV[ip].cx * k16, d1y = y - (float)gV[ip].cy * k16;
    float d2x = (float)gV[inx].cx * k16 - x, d2y = (float)gV[inx].cy * k16 - y;
    float l1 = sqrtf(d1x * d1x + d1y * d1y), l2 = sqrtf(d2x * d2x + d2y * d2y);
    if (l1 < 0.001f) l1 = 1.0f;
    if (l2 < 0.001f) l2 = 1.0f;
    d1x /= l1; d1y /= l1; d2x /= l2; d2y /= l2;
    float mx = -d1y - d2y, my = d1x + d2x;       // sum of the right-hand normals (-dy, dx)
    const float ml = sqrtf(mx * mx + my * my);
    float scale = 1.0f;
    if (ml < 0.05f) { mx = -d1y; my = d1x; }
    else {
      mx /= ml; my /= ml;
      const float cosHalf = mx * (-d1y) + my * d1x;
      scale = 1.0f / (cosHalf < 0.6f ? 0.6f : cosHalf);
    }
    const float kr = (float)kRoadHalf * scale, wr = (float)kWallHalf * scale;
    gV[i].kRx = rnd16(x + mx * kr); gV[i].kRy = rnd16(y + my * kr);
    gV[i].kLx = rnd16(x - mx * kr); gV[i].kLy = rnd16(y - my * kr);
    gV[i].wRx = rnd16(x + mx * wr); gV[i].wRy = rnd16(y + my * wr);
    gV[i].wLx = rnd16(x - mx * wr); gV[i].wLy = rnd16(y - my * wr);
    const int32_t dx = gV[inx].cx - gV[i].cx, dy = gV[inx].cy - gV[i].cy;
    gV[i].len = (uint16_t)(sqrtf((float)(dx * dx + dy * dy)) + 0.5f);
  }
}

// Straight stretches are filled with one quad, curve pieces with one quad each.
void buildRuns() {
  gRuns = 0;
  int i = 0;
  while (i < gN) {
    int j = i + 1;
    if (gStraight[i]) { while (j < gN && gStraight[j]) ++j; }
    if (gRuns < kMaxRuns) {
      gRunA[gRuns] = (uint8_t)i;
      gRunB[gRuns] = (uint8_t)(j >= gN ? 0 : j);
      ++gRuns;
    }
    i = j;
  }
}

// Dashes are placed by distance travelled along each line (never by vertex count), so they are
// evenly spaced on straights and curves alike. Each period is fitted so the pattern also closes
// cleanly where the loop meets itself.
void buildDashes() {
  const float period[kDashCh] = { kDashPeriodCentre, kDashPeriodKerb, kDashPeriodKerb };
  const float duty[kDashCh]   = { kDashDutyCentre,   kDashDutyKerb,   kDashDutyKerb };
  for (int ch = 0; ch < kDashCh; ++ch) {
    float cum = 0.0f;
    for (int i = 0; i < gN; ++i) {
      const Vert& a = gV[i];
      const Vert& b = gV[(i + 1 == gN) ? 0 : i + 1];
      float ax, ay, bx, by;
      if (ch == 0)      { ax = (float)a.cx * (1.0f / 16.0f); ay = (float)a.cy * (1.0f / 16.0f);
                          bx = (float)b.cx * (1.0f / 16.0f); by = (float)b.cy * (1.0f / 16.0f); }
      else if (ch == 1) { ax = (float)a.kLx; ay = (float)a.kLy; bx = (float)b.kLx; by = (float)b.kLy; }
      else              { ax = (float)a.kRx; ay = (float)a.kRy; bx = (float)b.kRx; by = (float)b.kRy; }
      gCum[ch][i] = (int32_t)(cum * 256.0f + 0.5f);
      cum += sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    }
    int n = (int)floorf(cum / period[ch] + 0.5f);
    if (n < 1) n = 1;
    const float p = cum / (float)n;
    gTotal[ch] = (int32_t)(cum * 256.0f + 0.5f);
    gDashPeriod[ch] = (int32_t)(p * 256.0f + 0.5f);
    gDashLen[ch] = (int32_t)(p * duty[ch] * 256.0f + 0.5f);
  }
}

int nearestVertex(int32_t qx, int32_t qy) {
  int best = 0;
  int32_t bd = 0x7FFFFFFF;
  for (int i = 0; i < gN; ++i) {
    const int32_t dx = gV[i].cx - qx, dy = gV[i].cy - qy;
    const int32_t d = dx * dx + dy * dy;
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

// 32 headings of an 11x11 top-down car: bright body, dark windscreen, dark outline halo.
void buildSprites() {
  for (int f = 0; f < 32; ++f) {
    const float ang = (float)f * (6.2831853f / 32.0f);
    const float ca = cosf(ang), sa = sinf(ang);
    uint16_t mask[11], cock[11];
    for (int r = 0; r < 11; ++r) { mask[r] = 0; cock[r] = 0; }
    for (int r = 0; r < 11; ++r) {
      for (int c = 0; c < 11; ++c) {
        int hit = 0, hitC = 0;
        for (int sy = 0; sy < 3; ++sy) {
          for (int sx = 0; sx < 3; ++sx) {
            const float dx = (float)(c - 5) + ((float)sx - 1.0f) * 0.33f;
            const float dy = (float)(r - 5) + ((float)sy - 1.0f) * 0.33f;
            const float lx = dx * ca + dy * sa;          // car frame: +x = nose
            const float ly = -dx * sa + dy * ca;
            float hw = 2.4f;
            if (lx > 2.0f) hw -= 0.32f * (lx - 2.0f);    // tapered nose
            if (lx >= -4.5f && lx <= 4.5f && fabsf(ly) <= hw) ++hit;
            if (lx >= -0.8f && lx <= 1.4f && fabsf(ly) <= 1.15f) ++hitC;
          }
        }
        if (hit >= 4)  mask[r] = (uint16_t)(mask[r] | (1u << c));
        if (hitC >= 5) cock[r] = (uint16_t)(cock[r] | (1u << c));
      }
    }
    for (int r = 0; r < 11; ++r) {
      gSprBody[f][r] = (uint16_t)(mask[r] & (uint16_t)~cock[r]);
      uint16_t h = 0;
      for (int rr = r - 1; rr <= r + 1; ++rr) {
        if (rr < 0 || rr > 10) continue;
        h = (uint16_t)(h | mask[rr] | (uint16_t)(mask[rr] << 1) | (uint16_t)(mask[rr] >> 1));
      }
      gSprHalo[f][r] = (uint16_t)(h & 0x7FFu);
    }
  }
}

// =============================================================================
// Road queries (collision works on the centre line, so curves are smooth)
// =============================================================================
// Squared distance (Q4 px squared) from point p to segment i; also returns the closest point.
int32_t segDist2(int i, int32_t px, int32_t py, int32_t& qx, int32_t& qy) {
  const Vert& a = gV[i];
  const Vert& b = gV[(i + 1 == gN) ? 0 : i + 1];
  const int32_t abx = b.cx - a.cx, aby = b.cy - a.cy;
  const int32_t apx = px - a.cx, apy = py - a.cy;
  const int32_t den = abx * abx + aby * aby;
  const int32_t t = apx * abx + apy * aby;
  if (t <= 0) { qx = a.cx; qy = a.cy; }
  else if (t >= den) { qx = b.cx; qy = b.cy; }
  else { qx = a.cx + (abx * t) / den; qy = a.cy + (aby * t) / den; }
  const int32_t dx = px - qx, dy = py - qy;
  return dx * dx + dy * dy;
}

// Cheap local search around the last known segment (the car moves < 2 px per step).
void roadLocate(Car& c, int32_t& qx, int32_t& qy) {
  const int32_t px = c.x >> 12, py = c.y >> 12;        // Q16.16 -> Q4
  int32_t best = 0x7FFFFFFF, bx = 0, by = 0;
  int bi = c.seg;
  for (int k = -3; k <= 3; ++k) {
    int i = c.seg + k;
    if (i < 0) i += gN; else if (i >= gN) i -= gN;
    int32_t cx, cy;
    const int32_t d = segDist2(i, px, py, cx, cy);
    if (d < best) { best = d; bx = cx; by = cy; bi = i; }
  }
  c.seg = (int16_t)bi; c.d2 = best; qx = bx; qy = by;
}

// Full search, used when placing a car.
int roadLocateAll(int32_t pxQ4, int32_t pyQ4) {
  int best = 0;
  int32_t bd = 0x7FFFFFFF;
  for (int i = 0; i < gN; ++i) {
    int32_t cx, cy;
    const int32_t d = segDist2(i, pxQ4, pyQ4, cx, cy);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

constexpr int32_t kRoadHalfSq  = (kRoadHalf * 16) * (kRoadHalf * 16);
constexpr int32_t kWallLimitQ4 = (kWallHalf - kCarRadius) * 16;        // centre-line distance where a car touches the wall
constexpr int32_t kWallLimitSq = kWallLimitQ4 * kWallLimitQ4;

// Push the car back inside the walls, bounce, scrape.
void wallResolve(Car& c, int32_t qx, int32_t qy) {
  const int32_t px = c.x >> 12, py = c.y >> 12;
  const int32_t dx = qx - px, dy = qy - py;               // towards the road centre (Q4)
  const int32_t d = (int32_t)isqrt32((uint32_t)c.d2);     // Q4
  if (d <= 0) return;
  const int32_t nx = (int32_t)(((int64_t)dx * 32768) / d);   // unit normal, Q15
  const int32_t ny = (int32_t)(((int64_t)dy * 32768) / d);
  const int32_t pen16 = (d - kWallLimitQ4) * 4096;           // Q16.16 px
  c.x += (int32_t)(((int64_t)nx * pen16) >> 15);
  c.y += (int32_t)(((int64_t)ny * pen16) >> 15);

  int32_t scrape = mulDt(kWallRub);
  const int32_t vn = (int32_t)(((int64_t)c.vx * nx + (int64_t)c.vy * ny) >> 15);   // > 0: heading for the centre
  if (vn < 0) {                                           // moving into the wall: bounce
    const int32_t j = (int32_t)(((int64_t)(-vn) * (kOne + kWallRestitution)) >> 16);
    c.vx += (int32_t)(((int64_t)nx * j) >> 15);
    c.vy += (int32_t)(((int64_t)ny * j) >> 15);
    scrape += mulQ(-vn, kScrapePerSpeed);
  }
  if (scrape > kScrapeMax) scrape = kScrapeMax;
  const int32_t vn2 = (int32_t)(((int64_t)c.vx * nx + (int64_t)c.vy * ny) >> 15);
  const int32_t vtx = c.vx - (int32_t)(((int64_t)nx * vn2) >> 15);   // tangential part
  const int32_t vty = c.vy - (int32_t)(((int64_t)ny * vn2) >> 15);
  c.vx -= mulQ(vtx, scrape);
  c.vy -= mulQ(vty, scrape);
  c.d2 = kWallLimitSq;
}

// =============================================================================
// Car physics (one fixed 8 ms step)
// =============================================================================
void fireNitro(Car& c) {
  c.nitro--;
  c.nitroBurnUs = kNitroBurnUs;
  c.nitroBufUs = 0;
  const uint32_t a16 = c.ang >> 16;
  c.vx += (int32_t)(((int64_t)cosQ15(a16) * kNitroKick) >> 15);
  c.vy += (int32_t)(((int64_t)sinQ15(a16) * kNitroKick) >> 15);
}

void carNitroPress(Car& c) {
  if (c.nitroBurnUs <= 0 && c.nitro > 0) fireNitro(c);
  else c.nitroBufUs = kNitroBufferUs;                      // remember the tap
}

// X just went down (handbrake pulled): at speed the rear lets go; if we are steering, the nose snaps in.
void carHandbrakeKick(Car& c, int steerDir) {
  const uint32_t a16 = c.ang >> 16;
  const int32_t vf = (int32_t)(((int64_t)c.vx * cosQ15(a16) + (int64_t)c.vy * sinQ15(a16)) >> 15);
  if (vf < kDriftMinSpeed) return;                                // too slow to slide
  c.driftUs = kDriftKickUs;
  if (steerDir != 0) c.ang += ((uint32_t)((int32_t)steerDir * kDriftSnap)) << 16;  // modular arithmetic, wraps correctly
}

void carStep(Car& c, const CarCmd& in) {
  // --- steering: buttons are digital, the lock ramps so it never feels like a switch
  const int32_t target = (int32_t)in.steerDir * 256;
  if (c.steer < target) {
    c.steer += (c.steer < 0) ? kSteerBack : kSteerUp;
    if (c.steer > target) c.steer = target;
  } else if (c.steer > target) {
    c.steer -= (c.steer > 0) ? kSteerBack : kSteerUp;
    if (c.steer < target) c.steer = target;
  }

  // --- velocity split into forward / sideways parts of the heading
  const uint32_t a16 = c.ang >> 16;
  const int32_t fxq = cosQ15(a16), fyq = sinQ15(a16);        // forward, Q15
  const int32_t rxq = -fyq, ryq = fxq;                       // right-hand side, Q15
  int32_t vf = (int32_t)(((int64_t)c.vx * fxq + (int64_t)c.vy * fyq) >> 15);
  int32_t vl = (int32_t)(((int64_t)c.vx * rxq + (int64_t)c.vy * ryq) >> 15);
  const bool nitro = c.nitroBurnUs > 0;

  // --- handbrake: X held at speed keeps the rear loose (the kick itself is applied on the press)
  if (in.handbrake && vf > kDriftMinSpeed && c.driftUs < kDriftHoldUs)
    c.driftUs = kDriftHoldUs;
  const bool driftOn = c.driftUs > 0;
  if (c.driftUs > 0) { c.driftUs -= (int32_t)kStepUs; if (c.driftUs < 0) c.driftUs = 0; }

  // --- engine, drag, handbrake drag, DOWN (brake or reverse)
  int32_t accel = in.throttle ? kAccel : 0;
  if (nitro) accel = mulQ(kAccel, kNitroAccelMul);
  int32_t drag = kDrag;
  if (c.rough) { accel = mulQ(accel, kRoughAccelMul); drag = mulQ(drag, kRoughDragMul); }
  vf += mulDt(accel - mulQ(vf, drag));
  if (in.handbrake && driftOn && vf > 0) { vf -= mulDt(kDriftScrub); if (vf < 0) vf = 0; }
  if (in.brake) {
    if (vf > kBrakeMinSpeed) {                                   // rolling forward: DOWN is a brake
      c.brakeLatch = true;
      c.revWaitUs = 0;
      vf -= mulDt(kBrake);
      if (vf < 0) vf = 0;
    } else if (c.brakeLatch && c.revWaitUs < kReverseDelayUs) {  // just braked to a stop: hold, then reverse
      vf -= mulDt(kBrake);
      if (vf < 0) { vf = 0; c.revWaitUs += (int32_t)kStepUs; }
    } else {                                                     // standstill or rolling back: DOWN is reverse
      vf -= mulDt(kReverseAccel);
      if (vf < -kReverseTop) vf = -kReverseTop;
    }
  } else {
    c.brakeLatch = false;
    c.revWaitUs = 0;
  }

  // --- grip: sideways speed bleeds off, part of it becomes forward speed (keeps drifts flowing)
  int32_t grip = kGrip;
  if (driftOn) grip = kGripDrift;
  else if (iabs32(c.steer) > kOversteerSteer && vf > kOversteerSpeed) grip = mulQ(grip, kOversteerGripMul);
  if (nitro) grip = mulQ(grip, kGripNitroMul);
  if (c.rough) grip = mulQ(grip, kGripRoughMul);
  int32_t k = mulDt(grip);
  if (k > kOne / 2) k = kOne / 2;
  const int32_t bleed = mulQ(vl, k);                    // lost to the tyres (part of it is handed to forward speed)
  vl -= bleed + mulQ(vl, mulDt(drag));                  // air / rolling drag acts on the sideways speed too, so sliding is never faster than driving
  if (vf >= 0) {                                        // slide speed is not lost, but it never pushes past top speed
    const int32_t topNow = nitro ? mulQ(kTopSpeed, kNitroAccelMul) : kTopSpeed;
    if (vf < topNow) {
      int32_t gain = mulQ(iabs32(bleed), kSlipToForward);
      if (vf + gain > topNow) gain = topNow - vf;
      vf += gain;
    }
  }

  // --- yaw: steering turns the heading; with hands off, the car self-countersteers out of a slide
  const int32_t sp = iabs32(vf);
  int32_t yaw = (int32_t)(((int64_t)yawRate16(sp) * c.steer) >> 8);
  if (driftOn) yaw = mulQ(yaw, kDriftYawMul);
  if (vf < 0) yaw = -yaw;
  if (in.steerDir == 0 && vf > kAssistMinSpeed) {
    int32_t slip = (int32_t)(((int64_t)vl * 65536) / vf);    // ~ slip angle, Q16.16 radians
    slip = clampI(slip, -kOne, kOne);
    yaw += clampI((int32_t)(((int64_t)slip * kAssistGain) >> 16), -kAssistMax, kAssistMax);
  }
  c.ang += (uint32_t)(int32_t)((int64_t)yaw * kDt16);       // angle16/s * Q16.16 s = ang units

  // --- recompose the velocity with the heading the split used, then move
  c.vx = (int32_t)(((int64_t)vf * fxq + (int64_t)vl * rxq) >> 15);
  c.vy = (int32_t)(((int64_t)vf * fyq + (int64_t)vl * ryq) >> 15);
  c.x += (int32_t)(((int64_t)c.vx * kDt16) >> 16);
  c.y += (int32_t)(((int64_t)c.vy * kDt16) >> 16);
  c.drifting = (iabs32(vl) > kDriftFlagSlip) && (sp > kDriftFlagSpeed);

  // --- road: shoulder slows, wall bounces
  int32_t qx, qy;
  roadLocate(c, qx, qy);
  if (c.d2 > kWallLimitSq) wallResolve(c, qx, qy);
  c.rough = c.d2 > kRoadHalfSq;

  // --- nitro timers
  if (c.nitroBurnUs > 0) { c.nitroBurnUs -= (int32_t)kStepUs; if (c.nitroBurnUs < 0) c.nitroBurnUs = 0; }
  if (c.nitroBufUs > 0) {
    c.nitroBufUs -= (int32_t)kStepUs;
    if (c.nitroBufUs > 0 && c.nitroBurnUs <= 0 && c.nitro > 0) fireNitro(c);
  }
  if (c.nitro < kNitroMax) {
    c.nitroRechargeUs += (int32_t)kStepUs;
    if (c.nitroRechargeUs >= kNitroRechargeUs) { c.nitroRechargeUs = 0; c.nitro++; }
  } else {
    c.nitroRechargeUs = 0;
  }
}

void spawnCar() {
  Car& c = gCar;
  c.x = kSpawnX * kOne;
  c.y = kSpawnY * kOne;
  c.vx = 0; c.vy = 0;
  c.ang = 0;                                              // facing east along the first straight
  c.steer = 0; c.driftUs = 0;
  c.nitroBurnUs = 0; c.nitroBufUs = 0; c.nitroRechargeUs = 0;
  c.nitro = (uint8_t)kNitroStart;
  c.revWaitUs = 0; c.brakeLatch = false;
  c.drifting = false; c.rough = false;
  c.seg = (int16_t)roadLocateAll(kSpawnX * 16, kSpawnY * 16);
  int32_t qx, qy;
  roadLocate(c, qx, qy);
}

// =============================================================================
// Camera: leads the car along its velocity, follows fast so it never lags
// =============================================================================
void camSnap() { gCamX = gCar.x; gCamY = gCar.y; }

void camStep() {
  int32_t lx = mulQ(gCar.vx, kLookMul), ly = mulQ(gCar.vy, kLookMul);
  lx = clampI(lx, -kLookMaxX * kOne, kLookMaxX * kOne);
  ly = clampI(ly, -kLookMaxY * kOne, kLookMaxY * kOne);
  gCamX += (int32_t)(((int64_t)(gCar.x + lx - gCamX) * kCamAlpha) >> 16);
  gCamY += (int32_t)(((int64_t)(gCar.y + ly - gCamY) * kCamAlpha) >> 16);
}

// =============================================================================
// Drawing primitives (direct frame-buffer writes, bounds-checked)
//   Layout: byte = (y >> 3) * 128 + x, bit = y & 7, 1 = lit.
// =============================================================================
inline void pxSet(int x, int y) {
  if ((unsigned)x < 128u && (unsigned)y < 64u) gFb[(y >> 3) * 128 + x] |= (uint8_t)(1u << (y & 7));
}
inline void pxClr(int x, int y) {
  if ((unsigned)x < 128u && (unsigned)y < 64u) gFb[(y >> 3) * 128 + x] &= (uint8_t)~(1u << (y & 7));
}

void lineSet(int x0, int y0, int x1, int y1) {
  if ((x0 < 0 && x1 < 0) || (x0 > 127 && x1 > 127) || (y0 < 0 && y1 < 0) || (y0 > 63 && y1 > 63)) return;
  const int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
  const int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;   // dy <= 0
  int err = dx + dy;
  for (;;) {
    pxSet(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// Clear (darken) a convex quad. Edges are always interpolated bottom-to-top so
// neighbouring quads that share an edge agree on every pixel and leave no gaps.
void fillQuadClear(const int* xs, const int* ys) {
  int ymin = ys[0], ymax = ys[0], xmin = xs[0], xmax = xs[0];
  for (int i = 1; i < 4; ++i) {
    if (ys[i] < ymin) ymin = ys[i];
    if (ys[i] > ymax) ymax = ys[i];
    if (xs[i] < xmin) xmin = xs[i];
    if (xs[i] > xmax) xmax = xs[i];
  }
  if (ymax < 0 || ymin > 63 || xmax < 0 || xmin > 127) return;
  if (ymin < 0) ymin = 0;
  if (ymax > 63) ymax = 63;
  for (int y = ymin; y <= ymax; ++y) {
    int xl = 100000, xr = -100000;
    for (int e = 0; e < 4; ++e) {
      int x0 = xs[e], y0 = ys[e], x1 = xs[(e + 1) & 3], y1 = ys[(e + 1) & 3];
      if (y0 == y1) {
        if (y == y0) {
          if (x0 < xl) xl = x0;
          if (x1 < xl) xl = x1;
          if (x0 > xr) xr = x0;
          if (x1 > xr) xr = x1;
        }
        continue;
      }
      if (y0 > y1) { const int tx = x0; x0 = x1; x1 = tx; const int ty = y0; y0 = y1; y1 = ty; }
      if (y < y0 || y > y1) continue;
      const int x = x0 + ((x1 - x0) * (y - y0)) / (y1 - y0);
      if (x < xl) xl = x;
      if (x > xr) xr = x;
    }
    if (xl > xr) continue;
    if (xl < 0) xl = 0;
    if (xr > 127) xr = 127;
    uint8_t* row = gFb + (y >> 3) * 128;
    const uint8_t m = (uint8_t)~(1u << (y & 7));
    for (int x = xl; x <= xr; ++x) row[x] &= m;
  }
}

int lerpRound(int a, int b, int32_t num, int32_t den) {          // a + (b - a) * num / den, rounded
  const int64_t v = (int64_t)(b - a) * num;
  const int64_t h = den / 2;
  return a + (int)(v >= 0 ? (v + h) / den : -((-v + h) / den));
}

// Paint the dashes of line 'ch' that fall on segment i (screen coordinates a -> b).
void drawDashes(int ch, int i, int ax, int ay, int bx, int by) {
  const int32_t c0 = gCum[ch][i];
  const int32_t c1 = (i + 1 == gN) ? gTotal[ch] : gCum[ch][i + 1];
  const int32_t period = gDashPeriod[ch], dash = gDashLen[ch];
  const int32_t span = c1 - c0;
  if (span <= 0 || period <= 0) return;
  for (int32_t base = c0 - (c0 % period); base < c1; base += period) {
    const int32_t s0 = base < c0 ? c0 : base;
    const int32_t e = base + dash;
    const int32_t s1 = e > c1 ? c1 : e;
    if (s1 <= s0) continue;
    lineSet(lerpRound(ax, bx, s0 - c0, span), lerpRound(ay, by, s0 - c0, span),
            lerpRound(ax, bx, s1 - c0, span), lerpRound(ay, by, s1 - c0, span));
  }
}

void blitSprite(const uint16_t* rows, int ox, int oy, bool lit) {
  for (int r = 0; r < 11; ++r) {
    const int y = oy + r;
    if ((unsigned)y >= 64u) continue;
    const uint16_t m = rows[r];
    if (m == 0) continue;
    for (int c = 0; c < 11; ++c) {
      if ((m & (1u << c)) == 0) continue;
      if (lit) pxSet(ox + c, y); else pxClr(ox + c, y);
    }
  }
}

// =============================================================================
// World rendering
// =============================================================================
// Sparse dots that are anchored to the world, so the ground visibly slides past.
void drawGround(int ox, int oy) {
  const uint8_t mask = (uint8_t)(0x11u << ((uint32_t)(-oy) & 3u));
  for (int x = 0; x < 128; ++x) {
    if ((((uint32_t)(x + ox)) & 3u) != 0u) continue;
    for (int p = 0; p < 8; ++p) gFb[p * 128 + x] |= mask;
  }
}

void drawWorld(int ox, int oy) {
  drawGround(ox, oy);

  // pass 1: darken the corridor (road + shoulder) so the dots only remain outside the walls
  for (int k = 0; k < gRuns; ++k) {
    const Vert& a = gV[gRunA[k]];
    const Vert& b = gV[gRunB[k]];
    const int xs[4] = { a.wLx - ox, b.wLx - ox, b.wRx - ox, a.wRx - ox };
    const int ys[4] = { a.wLy - oy, b.wLy - oy, b.wRy - oy, a.wRy - oy };
    fillQuadClear(xs, ys);
  }

  // pass 2: wall (solid), road edge (dashed), centre dashes
  for (int i = 0; i < gN; ++i) {
    const Vert& a = gV[i];
    const Vert& b = gV[(i + 1 == gN) ? 0 : i + 1];
    int minX = a.wLx, maxX = a.wLx, minY = a.wLy, maxY = a.wLy;
    const int16_t px[3] = { a.wRx, b.wLx, b.wRx };
    const int16_t py[3] = { a.wRy, b.wLy, b.wRy };
    for (int j = 0; j < 3; ++j) {
      if (px[j] < minX) minX = px[j];
      if (px[j] > maxX) maxX = px[j];
      if (py[j] < minY) minY = py[j];
      if (py[j] > maxY) maxY = py[j];
    }
    if (maxX - ox < -1 || minX - ox > 128 || maxY - oy < -1 || minY - oy > 64) continue;
    lineSet(a.wLx - ox, a.wLy - oy, b.wLx - ox, b.wLy - oy);
    lineSet(a.wRx - ox, a.wRy - oy, b.wRx - ox, b.wRy - oy);
    drawDashes(0, i, ((a.cx + 8) >> 4) - ox, ((a.cy + 8) >> 4) - oy, ((b.cx + 8) >> 4) - ox, ((b.cy + 8) >> 4) - oy);
    drawDashes(1, i, a.kLx - ox, a.kLy - oy, b.kLx - ox, b.kLy - oy);
    drawDashes(2, i, a.kRx - ox, a.kRy - oy, b.kRx - ox, b.kRy - oy);
  }

  // start line: two-pixel chequer across the road
  const Vert& s = gV[gStartV];
  const int x0 = s.kLx - ox, y0 = s.kLy - oy, x1 = s.kRx - ox, y1 = s.kRy - oy;
  const int adx = x1 > x0 ? x1 - x0 : x0 - x1, ady = y1 > y0 ? y1 - y0 : y0 - y1;
  const int n = adx > ady ? adx : ady;
  if (n > 0) {
    for (int i = 0; i <= n; ++i) {
      const int x = x0 + ((x1 - x0) * i) / n, y = y0 + ((y1 - y0) * i) / n;
      const int cell = (i >> 1) & 1;
      if (adx < ady) { pxSet(x + cell, y); }
      else { pxSet(x, y + cell); }
    }
  }
}

// Wind streaks, anchored to the world so they stream past at the real speed.
void drawSpeedLines(int ox, int oy, uint32_t a16, int speedPx, bool nitro) {
  int len = (speedPx - kSpeedLineMinPx) / 6 + 6;
  if (nitro && len < 9) len = 9;
  if (len > 18) len = 18;
  const int ex = (int)(((int64_t)cosQ15(a16) * len) >> 16);     // half-length along the heading
  const int ey = (int)(((int64_t)sinQ15(a16) * len) >> 16);
  const int cell = 24;
  const int cx0 = floorDiv(ox - 20, cell), cx1 = floorDiv(ox + 128 + 20, cell);
  const int cy0 = floorDiv(oy - 20, cell), cy1 = floorDiv(oy + 64 + 20, cell);
  for (int cy = cy0; cy <= cy1; ++cy) {
    for (int cx = cx0; cx <= cx1; ++cx) {
      const uint32_t h = hash2(cx, cy);
      if ((h & 7u) >= 3u) continue;
      const int sx = cx * cell + (int)((h >> 3) % (uint32_t)cell) - ox;
      const int sy = cy * cell + (int)((h >> 11) % (uint32_t)cell) - oy;
      lineSet(sx - ex, sy - ey, sx + ex, sy + ey);
    }
  }
}

void drawCar(const Car& c, int ox, int oy, bool flicker) {
  const int sx = (c.x >> 16) - ox, sy = (c.y >> 16) - oy;
  const uint32_t a16 = c.ang >> 16;
  const int fx = (int)cosQ15(a16), fy = (int)sinQ15(a16);        // Q15 heading
  if (c.nitroBurnUs > 0) {                                       // flame behind the car
    const int tail = 5, tip = 9 + (flicker ? 4 : 1) + (int)((c.nitroBurnUs >> 17) & 1);
    const int bx = sx - ((fx * tail) >> 15), by = sy - ((fy * tail) >> 15);
    const int tx = sx - ((fx * tip) >> 15), ty = sy - ((fy * tip) >> 15);
    const int sidex = (-fy * 3) >> 16, sidey = (fx * 3) >> 16;   // half-width ~1.5 px
    lineSet(bx + sidex, by + sidey, tx, ty);
    lineSet(bx - sidex, by - sidey, tx, ty);
    lineSet(bx, by, tx, ty);
  }
  const int frame = (int)(((a16 + 1024u) >> 11) & 31u);
  blitSprite(gSprHalo[frame], sx - 5, sy - 5, false);
  blitSprite(gSprBody[frame], sx - 5, sy - 5, true);
}

// =============================================================================
// Screens
// =============================================================================
void drawMenuRow(U8G2& d, int x, int baseline, int w, bool sel, const char* text) {
  d.setFontMode(1);
  if (sel) {
    d.setDrawColor(1);
    d.drawBox(x, baseline - 9, w, 11);
    d.setDrawColor(0);
    d.drawStr(x + 3, baseline, text);
    d.setDrawColor(1);
  } else {
    d.setDrawColor(1);
    d.drawStr(x + 3, baseline, text);
  }
}

void renderHud(U8G2& d) {
  d.setFontMode(1);
  // speed, bottom-left
  d.setDrawColor(0);
  d.drawBox(0, 50, 26, 14);
  d.setDrawColor(1);
  d.setFont(u8g2_font_6x13B_tr);
  char buf[8];
  const int shown = (int)clampI(gSpeedDisp, 0, 999);
  if (gReversing) snprintf(buf, sizeof(buf), "R%2d", shown > 99 ? 99 : shown);
  else snprintf(buf, sizeof(buf), "%3d", shown);
  d.drawStr(2, 62, buf);
  // nitro tank and burn bar, bottom-right (hidden until a nitro button exists)
  if (kNitroKey != 0) {
    d.setDrawColor(0);
    d.drawBox(98, 46, 30, 18);
    d.setDrawColor(1);
    for (int i = 0; i < kNitroMax; ++i) {
      const int x = 102 + i * 8;
      if (i < (int)gCar.nitro) d.drawBox(x, 57, 6, 6);
      else d.drawFrame(x, 57, 6, 6);
    }
    if (gCar.nitroBurnUs > 0) {
      const int w = (int)(((int64_t)gCar.nitroBurnUs * 22) / kNitroBurnUs);
      d.drawFrame(102, 50, 24, 5);
      d.drawBox(103, 51, w, 3);
    }
  }
  // drift tag, top-left (top-right stays free for the launcher's FPS overlay)
  if (gCar.drifting) {
    d.setDrawColor(0);
    d.drawBox(0, 0, 28, 9);
    d.setDrawColor(1);
    d.setFont(u8g2_font_5x7_tr);
    d.drawStr(1, 7, "DRIFT");
  }
}

void renderPlay(U8G2& d) {
  const int ox = (gCamX >> 16) - 64, oy = (gCamY >> 16) - 32;
  drawWorld(ox, oy);
  const int speedPx = (int)(isqrt32((uint32_t)(((int64_t)(gCar.vx >> 16) * (gCar.vx >> 16)) +
                                                ((int64_t)(gCar.vy >> 16) * (gCar.vy >> 16)))));
  const bool nitro = gCar.nitroBurnUs > 0;
  if (nitro || speedPx > 135) drawSpeedLines(ox, oy, gCar.ang >> 16, speedPx, nitro);
  drawCar(gCar, ox, oy, (gAnim & 1u) != 0u);
  renderHud(d);
}

void renderTitle(U8G2& d) {
  drawWorld((gTitleX >> 16) - 64, (gTitleY >> 16) - 32);
  d.setFontMode(1);
  d.setDrawColor(0);
  d.drawBox(0, 0, 32, 9);                    // flipBOX tag, top-left
  d.drawBox(0, 10, 128, 15);                 // title bar
  d.drawBox(12, 26, 104, 36);                // menu
  d.setDrawColor(1);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(1, 7, "flipBOX");
  d.drawFrame(0, 10, 128, 15);
  d.setFont(u8g2_font_7x13B_tr);
  d.drawStr(8, 22, "MIAMI STREET RUN");
  d.setFont(u8g2_font_6x10_tr);
  drawMenuRow(d, 14, 36, 100, gMenuSel == 0, "RACE");
  drawMenuRow(d, 14, 47, 100, gMenuSel == 1, gAutoThrottle ? "THROTTLE: AUTO" : "THROTTLE: MANUAL");
  drawMenuRow(d, 14, 58, 100, gMenuSel == 2, "QUIT");
}

void renderPause(U8G2& d) {
  renderPlay(d);
  d.setFontMode(1);
  d.setDrawColor(0);
  d.drawBox(22, 8, 84, 50);
  d.setDrawColor(1);
  d.drawFrame(22, 8, 84, 50);
  d.setFont(u8g2_font_6x10_tr);
  d.drawStr(50, 19, "PAUSED");
  drawMenuRow(d, 28, 33, 72, gMenuSel == 0, "RESUME");
  drawMenuRow(d, 28, 44, 72, gMenuSel == 1, "RESTART");
  drawMenuRow(d, 28, 55, 72, gMenuSel == 2, "QUIT");
}

// =============================================================================
// Game flow
// =============================================================================
void titleAdvance(uint32_t dtUs) {
  const int32_t dt16 = (int32_t)((dtUs << 16) / 1000000u);       // dtUs <= 50000: no overflow
  gTitleT += (int32_t)(((int64_t)kTitleSpeed * dt16) >> 16);
  for (;;) {
    const int32_t segLen = (int32_t)gV[gTitleSeg].len * 4096;    // Q4 -> Q16.16
    if (segLen <= 0 || gTitleT < segLen) break;
    gTitleT -= segLen;
    gTitleSeg = (gTitleSeg + 1 == gN) ? 0 : gTitleSeg + 1;
  }
  const Vert& a = gV[gTitleSeg];
  const Vert& b = gV[(gTitleSeg + 1 == gN) ? 0 : gTitleSeg + 1];
  const int32_t segLen = (int32_t)a.len * 4096;
  if (segLen > 0) {
    gTitleX = (int32_t)a.cx * 4096 + (int32_t)(((int64_t)((int32_t)(b.cx - a.cx) * 4096) * gTitleT) / segLen);
    gTitleY = (int32_t)a.cy * 4096 + (int32_t)(((int64_t)((int32_t)(b.cy - a.cy) * 4096) * gTitleT) / segLen);
  }
}

void startRace() {
  spawnCar();
  camSnap();
  gAcc = 0;
  gSpeedDisp = 0;
  gState = ST_PLAY;
}

void updateTitle(const fb::Input& in, const fb::Frame& f) {
  titleAdvance(f.dtUs);
  if (in.pressed & fb::KEY_UP)   gMenuSel = (gMenuSel == 0) ? 2 : gMenuSel - 1;
  if (in.pressed & fb::KEY_DOWN) gMenuSel = (gMenuSel == 2) ? 0 : gMenuSel + 1;
  if (gMenuSel == 1 && (in.pressed & (fb::KEY_LEFT | fb::KEY_RIGHT))) gAutoThrottle = !gAutoThrottle;
  if (in.pressed & fb::KEY_SELECT) {
    if (gMenuSel == 0) startRace();
    else if (gMenuSel == 1) gAutoThrottle = !gAutoThrottle;
    else gQuit = true;
  }
}

void updatePause(const fb::Input& in) {
  if (in.pressed & fb::KEY_UP)   gMenuSel = (gMenuSel == 0) ? 2 : gMenuSel - 1;
  if (in.pressed & fb::KEY_DOWN) gMenuSel = (gMenuSel == 2) ? 0 : gMenuSel + 1;
  if (in.pressed & fb::KEY_SELECT) {
    if (gMenuSel == 0) { gAcc = 0; gState = ST_PLAY; }
    else if (gMenuSel == 1) startRace();
    else gQuit = true;
  }
}

void updatePlay(const fb::Input& in, const fb::Frame& f) {
  if (in.pressed & fb::KEY_SELECT) { gState = ST_PAUSE; gMenuSel = 0; return; }

  CarCmd cmd;
  cmd.steerDir = (int8_t)(((in.held & fb::KEY_RIGHT) ? 1 : 0) - ((in.held & fb::KEY_LEFT) ? 1 : 0));
  cmd.brake = (in.held & fb::KEY_DOWN) != 0;
  cmd.handbrake = (in.held & fb::KEY_X) != 0;
  cmd.throttle = gAutoThrottle ? !cmd.brake : ((in.held & fb::KEY_UP) != 0);
  if (in.pressed & fb::KEY_X) carHandbrakeKick(gCar, cmd.steerDir);
  if (kNitroKey != 0 && (in.pressed & kNitroKey)) carNitroPress(gCar);

  gAcc += f.dtUs;
  int steps = 0;
  while (gAcc >= kStepUs && steps < kMaxSteps) {
    carStep(gCar, cmd);
    camStep();
    gAcc -= kStepUs;
    ++steps;
  }
  if (gAcc >= kStepUs) gAcc = 0;                       // fell behind: drop the rest instead of spiralling

  const int32_t vx = gCar.vx >> 16, vy = gCar.vy >> 16;
  gSpeedDisp = (int)(isqrt32((uint32_t)(vx * vx + vy * vy))) * (int)kSpeedDisplayMul;
  if (gSpeedDisp > 999) gSpeedDisp = 999;
  const uint32_t a16 = gCar.ang >> 16;
  const int32_t vfQ = (int32_t)(((int64_t)gCar.vx * cosQ15(a16) + (int64_t)gCar.vy * sinQ15(a16)) >> 15);
  gReversing = (vfQ >> 16) < -3;
}

void gameInit() {
  buildSinTable();
  buildRoute();
  buildOffsets();
  buildRuns();
  buildDashes();
  buildSprites();
  gStartV = nearestVertex(204 * 16, kSpawnY * 16);
  gState = ST_TITLE;
  gMenuSel = 0;
  gAutoThrottle = false;
  gQuit = false;
  gAnim = 0;
  gAcc = 0;
  gSpeedDisp = 0;
  gReversing = false;
  gTitleSeg = roadLocateAll(kSpawnX * 16, kSpawnY * 16);
  gTitleT = 0;
  titleAdvance(0);
  spawnCar();
  camSnap();
}

fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  ++gAnim;
  if (gState == ST_TITLE) updateTitle(in, f);
  else if (gState == ST_PLAY) updatePlay(in, f);
  else updatePause(in);
  return gQuit ? fb::GAME_EXIT : fb::GAME_RUN;
}

void gameRender(U8G2& d) {
  gFb = d.getBufferPtr();                    // re-fetched every frame, never kept across frames
  d.setDrawColor(1);
  d.setFontMode(1);
  if (gState == ST_TITLE) renderTitle(d);
  else if (gState == ST_PLAY) renderPlay(d);
  else renderPause(d);
}

}  // namespace

FB_GAME(MiamiRun, gameInit, gameUpdate, gameRender, nullptr);
