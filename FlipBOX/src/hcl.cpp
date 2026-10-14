// src/HillClimb.cpp  --  Hill Climb   (flipBOX)
//
// Side-scrolling hill-driving physics game: drive as far as you can,
// keep the car upright, grab coins and fuel cans, survive the bridges.
// The track gets rougher the further you drive (bumpy by ~500 m,
// brutal by ~1000 m).  Flips are momentum-gated: only bumps that
// unload a wheel at speed can flip you -- throttle alone on flat
// ground cannot; it just gives a wheelie stance.
//
//   RIGHT : accelerate          / backflip torque while airborne
//   LEFT  : brake + reverse     / frontflip torque while airborne
//   SELECT: start, pause, confirm menu entries
//
// Physics: rigid chassis with two spring/damper wheels, fixed 120 Hz
// substep accumulator (frame-rate independent, bounded catch-up).
// Drive/brake act at the wheel contact point.  Loop-out / endo
// torques are gated by forward speed and by wheel unloading, so they
// only fire when a bump has actually thrown the car.  Bridges are
// chains of spring-coupled planks that sag under the wheels.
// Terrain: stateless deterministic multi-octave value noise with a
// distance-based difficulty ramp (no storage).
// Floats are used only for the small per-substep / per-point math
// (hardware single-precision FPU); never per pixel, never double.

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

#include <stdio.h>
#include <string.h>
#include <math.h>

namespace {

// ===========================================================================
// Tuning constants (units in comments)
// ===========================================================================
constexpr float PHYS_DT     = 1.0f / 120.0f;  // physics substep [s]
constexpr uint32_t PHYS_US  = 8333;           // physics substep [us]
constexpr uint32_t MAX_SUBSTEPS = 6;          // bounded catch-up per frame

constexpr float GRAV        = 820.0f;   // gravity [px/s^2]
constexpr float SPRING_K    = 280.0f;   // wheel spring [px/s^2 per px penetration]
constexpr float SPRING_C    = 16.0f;    // wheel damper [px/s^2 per px/s]
constexpr float N_MAX       = 3000.0f;  // max wheel normal force [px/s^2]
constexpr float INERTIA     = 65.0f;    // pitch inertia (low = impacts rotate car)
constexpr float ENGINE      = 430.0f;   // engine accel [px/s^2] (snappy throttle)
constexpr float BRAKE_F     = 650.0f;   // brake accel [px/s^2] (snappy brake)
constexpr float REV_F       = 140.0f;   // reverse accel [px/s^2]
constexpr float MAX_VX      = 285.0f;   // forward speed cap [px/s]
constexpr float MAX_REV     = 95.0f;    // reverse speed cap [px/s]
constexpr float ROLL_RES    = 0.5f;     // rolling resistance [1/s]
constexpr float GND_DRAG2   = 0.004f;   // quadratic ground drag [1/(px/s s)]
constexpr float AIR_DRAG    = 0.03f;    // airborne drag [1/s]
constexpr float AIR_TORQUE  = 12.0f;    // in-air pitch control [rad/s^2] (snappy)
constexpr float MAX_W       = 10.0f;    // |angular velocity| cap [rad/s]
constexpr float GND_ANGDAMP = 2.5f;     // pitch damping while grounded [1/s]
constexpr float PEN_CLAMP   = 10.0f;    // anti-tunnel penetration clamp [px]
constexpr float WHEEL_R     = 4.0f;     // wheel radius [px]
constexpr float WHEELBASE   = 9.0f;     // axle distance from centre [px]
constexpr float WHEEL_DY    = 4.0f;     // axle below chassis centre [px]
constexpr float START_X     = 60.0f;    // spawn world x [px]
constexpr float PX_PER_M    = 10.0f;    // world px per displayed metre
// flip behaviour: contact couple (mild stance) + momentum-gated loop-out.
// LOOP/ENDO torques ONLY engage above their min speed and scale with
// vx/MAX_VX, so a bump must unload a wheel at speed for a flip to happen;
// the suspension's load-proportional restoring torque beats them on flat.
constexpr float PITCH_COUPLE = 1.2f;    // multiplier on drive/brake pitch couple
constexpr float LOOP_T      = 11000.0f; // backflip torque, rear-only, gated [px^2/s^2]
constexpr float ENDO_T      = 10000.0f; // frontflip torque, front-only, gated [px^2/s^2]
constexpr float LOOP_MIN_VX = 120.0f;   // no loop-out below this speed [px/s]
constexpr float ENDO_MIN_VX = 140.0f;   // no endo below this speed [px/s]
// crash rules
constexpr float CORNER_PEN  = 0.5f;     // body-corner penetration = crash [px]
constexpr float BAD_ANGLE   = 1.9f;     // ~109 deg: resting sideways/roof
constexpr float CRASH_TUMBLE = 1.1f;    // crash-cam time before results [s]
// fuel
constexpr float TANK        = 100.0f;   // tank capacity [units]
constexpr float FUEL_GAS    = 3.1f;     // drain while throttling [units/s]
constexpr float FUEL_IDLE   = 0.32f;    // drain while playing [units/s]
constexpr float STILL_TIME  = 1.6f;     // out-of-fuel stopped time [s]
// terrain (world y grows downward, base surface ~44)
constexpr float GROUND_BASE = 44.0f;    // mean surface world y [px]
constexpr float AMP_A = 11.0f;  constexpr float SP_A = 128.0f;  // big hills
constexpr float AMP_B = 6.0f;   constexpr float SP_B = 36.0f;   // medium (rough from start)
constexpr float AMP_C = 2.6f;   constexpr float SP_C = 15.0f;   // chatter/bumps
constexpr float AMP_J = 2.0f;   constexpr float SP_J = 96.0f;   // jump kickers
constexpr float AMP_A_D = 10.0f;  // extra big-hill amp at max difficulty
constexpr float AMP_B_D = 8.0f;   // extra medium amp at max difficulty
constexpr float AMP_C_D = 4.5f;   // extra chatter amp at max difficulty
constexpr float AMP_J_D = 9.0f;   // extra kicker amp at max difficulty
constexpr float FLAT_END     = 520.0f;  // amplitude ramp-in from spawn [px]
constexpr float DIFF_START_M = 20.0f;   // difficulty ramp start [m]
constexpr float DIFF_END_M   = 1000.0f; // difficulty ramp end ("very hard") [m]
constexpr float DIFF_ARC     = 6.0f;    // extra coin-arc height at max diff [px]
// camera
constexpr float CAM_X_OFF    = 44.0f;   // car's screen x [px]
constexpr float CAM_Y_OFF    = 34.0f;   // vertical target below car [px]
constexpr float CAM_Y_RATE   = 9.5f;    // vertical follow rate [1/s]
// items
constexpr int   COIN_N       = 28;      // coin slots (ring)
constexpr int   FUEL_N       = 4;       // fuel can slots (ring)
constexpr float ITEM_WIN     = 16.0f;   // item planning window [px]
constexpr int   FUEL_WIN     = 32;      // fuel can every N windows (512 px)
constexpr int   ARC_WIN      = 6;       // coin arc every N windows (96 px)
constexpr int   HUD_TOP      = 9;       // first free terrain row (below HUD)
// wheel wobble (visual only)
constexpr float TILT_K       = 350.0f;  // wobble spring [1/s^2]
constexpr float TILT_D       = 7.0f;    // wobble damping [1/s]
constexpr float TILT_MAX     = 0.4f;    // max wobble amplitude [rad]
// wobbly bridges
constexpr int   BRIDGE_N     = 3;       // active bridges (ring)
constexpr int   PLANK_N      = 13;      // planks per bridge
constexpr float BRIDGE_SPACING = 5.0f;  // plank spacing [px] (span 60 px)
constexpr float BRIDGE_HALF  = 30.0f;   // half span [px]
constexpr int   BRIDGE_WIN   = 40;      // candidate every N windows (640 px)
constexpr float BRIDGE_K     = 120.0f;  // wheel spring while on a bridge
constexpr float BRIDGE_NMAX  = 2000.0f; // normal-force cap on a bridge
constexpr float PLANK_MASS   = 0.6f;    // plank node mass
constexpr float PLANK_REST_K = 45.0f;   // plank spring toward rest [1/s^2 per px]
constexpr float PLANK_DAMP   = 2.2f;    // plank damping [1/s]
constexpr float PLANK_CHAIN  = 60.0f;   // coupling to neighbour planks [1/s^2 per px]
constexpr float PLANK_SAG    = 3.0f;    // rest sag at mid-span [px]

// ===========================================================================
// State (ALL of it is reset in gameInit / startRun)
// ===========================================================================
enum GState : uint8_t { ST_TITLE, ST_PLAY, ST_PAUSE, ST_OVER };

struct CarS {
  float x, y;         // chassis centre, world [px]
  float vx, vy;       // [px/s]
  float ang, angVel;  // pitch [rad] (+ = nose down), [rad/s]
  float wheelRot;     // wheel spin angle [rad] (visual)
};
struct CoinS { float x, y; bool active; };
struct FuelS { float x, y; bool active; };
struct BridgeS {
  bool   active;
  float  x0, x1;              // span [px]
  float  restY[PLANK_N];      // rest heights (chord + sag)
  float  y[PLANK_N];          // current plank heights
  float  vy[PLANK_N];         // plank velocities [px/s]
};

CarS     car;
float    gWheelDispY[2];      // displayed wheel-centre world y [px]
bool     gGrounded[2];        // wheel contact (0 = rear, 1 = front)
bool     gAir;                // both wheels off the ground
float    gSpin;               // wheel angular velocity [rad/s]
float    gWobPh;              // airborne visual wobble phase [rad]
float    gWheelTilt[2];       // wheel wobble angle [rad] (visual)
float    gWheelTiltVel[2];    // wheel wobble angular velocity [rad/s]
bool     gPrevGrounded[2];    // for landing/takeoff detection
float    gPrevSl[2];          // previous wheel-local terrain slope
uint32_t gSeed;
uint32_t gAcc;                // physics accumulator [us]
GState   gState;
uint8_t  gMenuSel;
float    gOverGrace;          // results-screen input grace [s]
bool     gCrashed;
float    gCrashT;
uint8_t  gReason;             // 0 = crash, 1 = out of fuel
float    gStillT;
float    gFuel;
uint16_t gCoinsRun;
float    gDistPx;
float    gCamX, gCamY;
float    gTitleWheel;
bool     gBlink;
float    gBlinkT;
CoinS    gCoinsA[COIN_N];     uint8_t gCoinNext;
FuelS    gFuelA[FUEL_N];      uint8_t gFuelNext;
BridgeS  gBridges[BRIDGE_N];  uint8_t gBridgeNext;
float    gItemGenX;           // items generated up to this world x [px]
float    gCosT[128];          // cos(i*pi/128), for noise interpolation

uint16_t gBestM = 0;       // session-persistent best distance [m]
                         // deliberately NOT reset in gameInit (spec 10.2)

// ===========================================================================
// Helpers
// ===========================================================================
static inline int IR(float v) { return (int)floorf(v + 0.5f); }

static uint32_t hashU32(uint32_t v) {
  v ^= v >> 16; v *= 0x45d9f3bu; v ^= v >> 16; v *= 0x45d9f3bu; v ^= v >> 16;
  return v;
}

// difficulty factor at world x: 0 at DIFF_START_M, 1 at DIFF_END_M
static float diffAt(float x) {
  float m = x / PX_PER_M;
  float d = (m - DIFF_START_M) / (DIFF_END_M - DIFF_START_M);
  if (d < 0.0f) d = 0.0f;
  if (d > 1.0f) d = 1.0f;
  return d;
}

// one octave of cosine-interpolated value noise in [0,1]
static float noiseOct(float x, float spacing, uint32_t salt) {
  float f = x / spacing;
  float fi = floorf(f);
  float t = f - fi;
  int32_t i = (int32_t)fi;
  uint32_t a = hashU32((uint32_t)i * 2654435761u + salt + gSeed);
  uint32_t b = hashU32((uint32_t)(i + 1) * 2654435761u + salt + gSeed);
  float va = (float)(a >> 16) * (1.0f / 65535.0f);
  float vb = (float)(b >> 16) * (1.0f / 65535.0f);
  float w = 0.5f - 0.5f * gCosT[(int)(t * 128.0f) & 127];
  return va + (vb - va) * w;
}

// terrain surface world y at world x (stateless, deterministic, continuous,
// increasingly rough with distance)
static float terrainY(float x) {
  float ramp = (x - 40.0f) / FLAT_END;
  if (ramp < 0.0f) ramp = 0.0f;
  if (ramp > 1.0f) ramp = 1.0f;
  float d = diffAt(x);
  float aA = AMP_A + AMP_A_D * d;
  float aB = AMP_B + AMP_B_D * d;
  float aC = AMP_C + AMP_C_D * d;
  float aJ = AMP_J + AMP_J_D * d;
  return GROUND_BASE
       + (noiseOct(x, SP_A, 0x1111u) - 0.5f) * 2.0f * aA * ramp
       + (noiseOct(x, SP_B, 0x2222u) - 0.5f) * 2.0f * aB * ramp
       + (noiseOct(x, SP_C, 0x3333u) - 0.5f) * 2.0f * aC * ramp
       + (noiseOct(x, SP_J, 0x4444u) - 0.5f) * 2.0f * aJ * ramp;
}

// -------------------------------------------------------------------------
// Bridges: bridgeAt returns the bridge index covering x (or -1) plus the
// left node index and fraction; groundAt is the physics-facing ground height.
// -------------------------------------------------------------------------
static int bridgeAt(float x, int* nodeIdx, float* frac) {
  for (int b = 0; b < BRIDGE_N; b++) {
    const BridgeS& br = gBridges[b];
    if (!br.active) continue;
    if (x >= br.x0 && x <= br.x1) {
      float t = (x - br.x0) / BRIDGE_SPACING;
      int idx = (int)t;
      if (idx > PLANK_N - 2) idx = PLANK_N - 2;
      *nodeIdx = idx;
      *frac = t - (float)idx;
      return b;
    }
  }
  return -1;
}

static float groundAt(float x) {
  int ni; float f;
  int b = bridgeAt(x, &ni, &f);
  if (b < 0) return terrainY(x);
  const BridgeS& br = gBridges[b];
  return br.y[ni] + (br.y[ni + 1] - br.y[ni]) * f;
}

// bridge candidate fits if the terrain mid-span is a real valley below the chord
static bool bridgeCandidateFits(float x0, float x1) {
  float yA = terrainY(x0), yB = terrainY(x1);
  float mid = (yA + yB) * 0.5f;
  return terrainY((x0 + x1) * 0.5f) > mid + 4.0f;
}

// per-frame plank dynamics (ends pinned, spring + damping + chain coupling)
static void stepBridges(float dt) {
  for (int b = 0; b < BRIDGE_N; b++) {
    BridgeS& br = gBridges[b];
    if (!br.active) continue;
    br.y[0] = br.restY[0];                br.vy[0] = 0.0f;
    br.y[PLANK_N - 1] = br.restY[PLANK_N - 1]; br.vy[PLANK_N - 1] = 0.0f;
    for (int i = 1; i < PLANK_N - 1; i++) {
      float f = -PLANK_REST_K * (br.y[i] - br.restY[i])
              - PLANK_DAMP * br.vy[i]
              + PLANK_CHAIN * ((br.y[i - 1] - br.y[i]) + (br.y[i + 1] - br.y[i]));
      br.vy[i] += f * dt;
    }
    for (int i = 1; i < PLANK_N - 1; i++) br.y[i] += br.vy[i] * dt;
  }
}

// ===========================================================================
// Physics (one fixed substep)
// ===========================================================================
static void physStep(float dt, bool thr, bool brk) {
  car.vy += GRAV * dt;

  // ground tangent at the car (drive direction + slope gravity); bridge-aware
  float slope = (groundAt(car.x + 3.0f) - groundAt(car.x - 3.0f)) * (1.0f / 6.0f);
  float inv = 1.0f / sqrtf(1.0f + slope * slope);
  float tx = inv, ty = slope * inv;

  float c = cosf(car.ang), s = sinf(car.ang);
  bool anyG = false;

  // --- wheels: spring / damper against ground or planks -------------------
  for (int i = 0; i < 2; i++) {
    float ox = (i == 0) ? -WHEELBASE : WHEELBASE;
    float rx = ox * c - WHEEL_DY * s;
    float ry = ox * s + WHEEL_DY * c;
    float wx = car.x + rx;
    int ni = 0; float nf = 0.0f;
    int bi = bridgeAt(wx, &ni, &nf);
    float gy = (bi >= 0)
             ? (gBridges[bi].y[ni] + (gBridges[bi].y[ni + 1] - gBridges[bi].y[ni]) * nf)
             : terrainY(wx);
    float pen = car.y + ry + WHEEL_R - gy;
    gGrounded[i] = (pen > 0.0f);
    if (pen > 0.0f) {
      anyG = true;
      float penDot = car.vy + car.angVel * rx;       // penetration rate
      float K     = (bi >= 0) ? BRIDGE_K  : SPRING_K;
      float nMax  = (bi >= 0) ? BRIDGE_NMAX : N_MAX;
      float N = K * pen + SPRING_C * penDot;         // normal force (mass = 1)
      if (N < 0.0f) N = 0.0f;
      if (N > nMax) N = nMax;
      car.vy     -= N * dt;                          // vertical push
      car.angVel += (-rx * N / INERTIA) * dt;        // pitch torque
      if (bi >= 0) {
        // reaction: the car's weight bends the planks downwards
        BridgeS& br = gBridges[bi];
        br.vy[ni]     += N * (1.0f - nf) * dt / PLANK_MASS;
        br.vy[ni + 1] += N * nf * dt / PLANK_MASS;
      } else if (pen > PEN_CLAMP) {                  // anti-tunnel (terrain only)
        car.y -= (pen - PEN_CLAMP) * 0.6f;
        if (car.vy > 250.0f) car.vy = 250.0f;
      }
    }
  }

  // --- engine / brake, applied at the wheel contact point -----------------
  float drive = 0.0f;
  float dOx = -WHEELBASE;                    // driven wheel: rear
  if (thr && gFuel > 0.0f && !gCrashed) drive = ENGINE;
  if (brk && !gCrashed) {                   // LEFT+RIGHT: brake wins
    drive = (car.vx > 30.0f) ? -BRAKE_F : -REV_F;
    dOx = WHEELBASE;                        // braking acts at the front
  }
  if (drive < 0.0f && car.vx < -MAX_REV) drive = 0.0f;
  if (anyG && drive != 0.0f) {
    car.vx += tx * drive * dt;
    car.vy += ty * drive * dt;
    // couple through the contact point -> natural wheelie / brake dive
    float crx = dOx * c - (WHEEL_DY + WHEEL_R) * s;
    float cry = dOx * s + (WHEEL_DY + WHEEL_R) * c;
    float couple = PITCH_COUPLE;
    if (drive < 0.0f)                        // momentum: dive scales with speed
      couple *= 0.4f + fabsf(car.vx) / MAX_VX;
    car.angVel += (crx * ty * drive - cry * tx * drive) * couple / INERTIA * dt;
  }

  // --- momentum-gated flip-over torques -----------------------------------
  // These ONLY engage above LOOP/ENDO_MIN_VX and scale with speed, i.e. a
  // bump must have unloaded the other axle while the car carries momentum.
  // Below the gate (or on flat ground, where the suspension's load-
  // proportional restoring torque dominates) throttle/brake alone give a
  // mild wheelie stance / brake dive and nothing more.
  if (!gCrashed) {
    if (thr && gGrounded[0] && !gGrounded[1] && car.vx > LOOP_MIN_VX)
      car.angVel -= LOOP_T * (car.vx / MAX_VX) / INERTIA * dt;
    if (brk && gGrounded[1] && !gGrounded[0] && car.vx > ENDO_MIN_VX)
      car.angVel += ENDO_T * (car.vx / MAX_VX) / INERTIA * dt;
  }

  if (anyG) {
    float gA = GRAV * ty;                   // gravity along the slope
    car.vx += tx * gA * dt;
    car.vy += ty * gA * dt;
    car.vx -= car.vx * ROLL_RES * dt;       // rolling resistance
    car.vx -= car.vx * fabsf(car.vx) * GND_DRAG2 * dt;
    car.angVel *= 1.0f - GND_ANGDAMP * dt;
  } else {
    // airborne pitch control: gas = backflip, brake = frontflip
    if (thr && !gCrashed) car.angVel -= AIR_TORQUE * dt;
    if (brk && !gCrashed) car.angVel += AIR_TORQUE * dt;
    car.vx -= car.vx * AIR_DRAG * dt;
  }

  if (car.vx > MAX_VX) car.vx = MAX_VX;
  if (car.vx < -MAX_REV) car.vx = -MAX_REV;
  if (car.angVel > MAX_W) car.angVel = MAX_W;
  if (car.angVel < -MAX_W) car.angVel = -MAX_W;

  car.x   += car.vx * dt;
  car.y   += car.vy * dt;
  car.ang += car.angVel * dt;
  if (car.ang > 3.14159f) car.ang -= 6.28318f;
  else if (car.ang < -3.14159f) car.ang += 6.28318f;
  if (car.x < 6.0f) { car.x = 6.0f; if (car.vx < 0.0f) car.vx = 0.0f; }

  // --- crash detection (chassis corners / roof, bad rest angle) -----------
  if (!gCrashed) {
    float cc = cosf(car.ang), ss = sinf(car.ang);
    static const float corner[3][2] = {
      {-11.0f, -6.0f}, {11.0f, -6.0f}, {0.0f, -10.0f}
    };
    for (int k = 0; k < 3; k++) {
      float lx = corner[k][0], ly = corner[k][1];
      float wx = car.x + lx * cc - ly * ss;
      float wy = car.y + lx * ss + ly * cc;
      if (wy > groundAt(wx) + CORNER_PEN) {
        gCrashed = true; gReason = 0; gCrashT = CRASH_TUMBLE;
        break;
      }
    }
    if (!gCrashed && anyG && fabsf(car.ang) > BAD_ANGLE) {
      gCrashed = true; gReason = 0; gCrashT = CRASH_TUMBLE;
    }
  }
}

// ===========================================================================
// Items and bridges (deterministic placement by window index, ring buffers)
// ===========================================================================
static void addCoin(float x, float y) {
  CoinS& cn = gCoinsA[gCoinNext];
  gCoinNext = (uint8_t)((gCoinNext + 1) % COIN_N);
  cn.x = x; cn.y = y; cn.active = true;
}

static void genItemsUpTo(float targetX) {
  while (gItemGenX < targetX) {
    int32_t wi = (int32_t)(gItemGenX / ITEM_WIN);
    float bx = (float)wi * ITEM_WIN;
    if (bx > 90.0f) {                       // keep the spawn area clean
      uint32_t h = hashU32((uint32_t)wi * 2654435761u + 0xA5A5u + gSeed);
      int wmod = (int)(wi % BRIDGE_WIN);
      if (wmod == 20) {                     // bridge candidate site
        float cx = bx + 8.0f;
        if (bridgeCandidateFits(cx - BRIDGE_HALF, cx + BRIDGE_HALF) &&
            ((h & 1u) == 0u || diffAt(cx) > 0.35f)) {
          BridgeS& br = gBridges[gBridgeNext];
          gBridgeNext = (uint8_t)((gBridgeNext + 1) % BRIDGE_N);
          br.active = true;
          br.x0 = cx - BRIDGE_HALF;
          br.x1 = cx + BRIDGE_HALF;
          float yA = terrainY(br.x0), yB = terrainY(br.x1);
          for (int i = 0; i < PLANK_N; i++) {
            float t = (float)i / (float)(PLANK_N - 1);
            br.restY[i] = yA + (yB - yA) * t + PLANK_SAG * 4.0f * t * (1.0f - t);
            br.y[i] = br.restY[i];
            br.vy[i] = 0.0f;
          }
        }
      } else if (wmod < 18 || wmod > 26) {  // keep clear of bridge sites
        if (wi % FUEL_WIN == 17) {
          float fx = bx + 8.0f;
          FuelS& fc = gFuelA[gFuelNext];
          gFuelNext = (uint8_t)((gFuelNext + 1) % FUEL_N);
          fc.x = fx; fc.y = terrainY(fx) - 13.0f; fc.active = true;
        } else if (wi % ARC_WIN == 3 && (h & 3) != 3) {   // ~75% of slots
          // arcs rise with difficulty: more air time makes them reachable
          float arc = 9.0f + (float)((h >> 4) & 7u) + DIFF_ARC * diffAt(bx);
          for (int k = 0; k < 5; k++) {
            float cx = bx + 4.0f + (float)k * 13.0f;
            float lift = (k == 2) ? arc
                       : ((k == 1 || k == 3) ? arc * 0.55f : arc * 0.2f);
            addCoin(cx, terrainY(cx) - 10.0f - lift);
          }
        }
      }
    }
    gItemGenX += ITEM_WIN;
  }
}

static void collectItems() {
  for (int i = 0; i < COIN_N; i++) {
    CoinS& cn = gCoinsA[i];
    if (cn.active &&
        cn.x > car.x - 14.0f && cn.x < car.x + 14.0f &&
        cn.y > car.y - 14.0f && cn.y < car.y + 11.0f) {
      cn.active = false;
      gCoinsRun++;
    }
  }
  for (int i = 0; i < FUEL_N; i++) {
    FuelS& fc = gFuelA[i];
    if (fc.active &&
        fc.x > car.x - 13.0f && fc.x < car.x + 13.0f &&
        fc.y > car.y - 12.0f && fc.y < car.y + 10.0f) {
      fc.active = false;
      gFuel = TANK;
    }
  }
}

// ===========================================================================
// Run control
// ===========================================================================
static void startRun() {
  gSeed = hashU32((uint32_t)micros() + 0x1B573EEFu);   // seeding only
  for (int i = 0; i < COIN_N; i++) gCoinsA[i].active = false;
  for (int i = 0; i < FUEL_N; i++) gFuelA[i].active = false;
  for (int i = 0; i < BRIDGE_N; i++) gBridges[i].active = false;
  gCoinNext = 0; gFuelNext = 0; gBridgeNext = 0;
  gItemGenX = 96.0f;

  car.x = START_X;
  car.y = terrainY(START_X) - 8.6f;
  car.vx = 0.0f; car.vy = 0.0f;
  car.ang = 0.0f; car.angVel = 0.0f;
  car.wheelRot = 0.0f;
  gWheelDispY[0] = gWheelDispY[1] = car.y + WHEEL_DY;
  gGrounded[0] = gGrounded[1] = true;
  gPrevGrounded[0] = gPrevGrounded[1] = true;
  gWheelTilt[0] = gWheelTilt[1] = 0.0f;
  gWheelTiltVel[0] = gWheelTiltVel[1] = 0.0f;
  gPrevSl[0] = gPrevSl[1] = 0.0f;
  gAir = false; gSpin = 0.0f; gWobPh = 0.0f;

  gFuel = TANK; gCoinsRun = 0; gDistPx = 0.0f;
  gCrashed = false; gCrashT = 0.0f; gReason = 0; gStillT = 0.0f;
  gAcc = 0;
  gCamX = car.x - CAM_X_OFF;
  gCamY = car.y - CAM_Y_OFF;
  genItemsUpTo(gCamX + 300.0f);
  gState = ST_PLAY;
  gMenuSel = 0;
}

static void enterOver() {
  gState = ST_OVER;
  gMenuSel = 0;
  gOverGrace = 0.55f;                     // swallow crash-mash input
  uint32_t m = (uint32_t)(gDistPx / PX_PER_M);
  if (m > 9999u) m = 9999u;
  if (m > gBestM) gBestM = (uint16_t)m;
}

// ===========================================================================
// Lifecycle
// ===========================================================================
static void gameInit() {
  gSeed = hashU32((uint32_t)micros() + 0xC0FFEE11u);   // seeding only
  for (int i = 0; i < 128; i++)
    gCosT[i] = cosf((float)i * 0.024543693f);          // pi/128

  car.x = 0.0f; car.y = 0.0f; car.vx = 0.0f; car.vy = 0.0f;
  car.ang = 0.0f; car.angVel = 0.0f; car.wheelRot = 0.0f;
  gWheelDispY[0] = gWheelDispY[1] = 0.0f;
  gGrounded[0] = gGrounded[1] = false;
  gPrevGrounded[0] = gPrevGrounded[1] = false;
  gWheelTilt[0] = gWheelTilt[1] = 0.0f;
  gWheelTiltVel[0] = gWheelTiltVel[1] = 0.0f;
  gPrevSl[0] = gPrevSl[1] = 0.0f;
  gAir = false; gSpin = 0.0f; gWobPh = 0.0f;
  gAcc = 0;
  gFuel = TANK; gCoinsRun = 0; gDistPx = 0.0f;
  gCrashed = false; gCrashT = 0.0f; gReason = 0; gStillT = 0.0f;
  gCamX = 0.0f; gCamY = 0.0f;
  gTitleWheel = 0.0f; gBlink = false; gBlinkT = 0.0f;
  for (int i = 0; i < COIN_N; i++) gCoinsA[i].active = false;
  for (int i = 0; i < FUEL_N; i++) gFuelA[i].active = false;
  for (int i = 0; i < BRIDGE_N; i++) gBridges[i].active = false;
  gCoinNext = 0; gFuelNext = 0; gBridgeNext = 0;
  gItemGenX = 96.0f;
  gState = ST_TITLE; gMenuSel = 0; gOverGrace = 0.0f;
  // gBestM is session-persistent on purpose: NOT reset here (spec 10.2)
}

static fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  float dtF = (float)f.dtUs * 0.000001f;
  switch (gState) {

  case ST_TITLE:
    gTitleWheel += 4.0f * dtF;
    if (in.pressed & fb::KEY_SELECT) startRun();
    break;

  case ST_PLAY: {
    if (!gCrashed) {
      if (in.pressed & fb::KEY_SELECT) {          // pause
        gState = ST_PAUSE; gMenuSel = 0; gAcc = 0;
        break;
      }
    } else {
      gCrashT -= dtF;                             // crash cam (ragdoll)
      if (gCrashT <= 0.0f || (in.pressed & fb::KEY_SELECT)) {
        enterOver();
        break;
      }
    }

    bool thr = !gCrashed && (in.held & fb::KEY_RIGHT) != 0;
    bool brk = !gCrashed && (in.held & fb::KEY_LEFT)  != 0;

    if (!gCrashed) {                              // fuel
      if (thr) gFuel -= FUEL_GAS * dtF;
      gFuel -= FUEL_IDLE * dtF;
      if (gFuel < 0.0f) gFuel = 0.0f;
    }

    // fixed-step physics with bounded catch-up; thr/brk are sampled once
    // per frame and applied at the very next 120 Hz substep, so button
    // response is bounded by one frame, nothing else
    gAcc += f.dtUs;
    uint32_t cap = MAX_SUBSTEPS * PHYS_US;
    if (gAcc > cap) gAcc = cap;
    while (gAcc >= PHYS_US) {
      physStep(PHYS_DT, thr, brk);
      gAcc -= PHYS_US;
    }

    // plank dynamics (bridges wobble back after the car passes)
    stepBridges(dtF);

    // camera (car locked horizontally, smoothed vertically, wide clamp
    // so big jumps stay framed)
    gCamX = car.x - CAM_X_OFF;
    float tY = car.y - CAM_Y_OFF;
    float k = CAM_Y_RATE * dtF; if (k > 1.0f) k = 1.0f;
    gCamY += (tY - gCamY) * k;
    if (car.y - gCamY > 54.0f) gCamY = car.y - 54.0f;
    if (car.y - gCamY < -10.0f) gCamY = car.y + 10.0f;

    // forward progress (never decreases)
    if (car.x - START_X > gDistPx) gDistPx = car.x - START_X;

    // wheel visuals
    gAir = !gGrounded[0] && !gGrounded[1];
    if (gAir) {
      gWobPh += 11.0f * dtF;
      gSpin *= 1.0f - 1.2f * dtF;                 // free spin decays
    } else {
      gSpin = car.vx / WHEEL_R;
    }
    car.wheelRot += gSpin * dtF;

    // wheel wobble: damped oscillator excited by landings, takeoffs
    // and terrain chatter (purely visual, layered on the physics)
    {
      float c = cosf(car.ang), s = sinf(car.ang);
      float wdt = dtF; if (wdt > 0.033f) wdt = 0.033f;
      for (int i = 0; i < 2; i++) {
        float ox = (i == 0) ? -WHEELBASE : WHEELBASE;
        float wx = car.x + ox * c - WHEEL_DY * s;
        if (gGrounded[i] && !gPrevGrounded[i]) {  // touchdown impulse
          float imp = -car.vy;
          if (imp < 0.0f) imp = 0.0f;
          if (imp > 300.0f) imp = 300.0f;
          gWheelTiltVel[i] += imp * 0.009f * ((i == 0) ? 1.0f : -1.0f);
        } else if (!gGrounded[i] && gPrevGrounded[i]) {
          gWheelTiltVel[i] -= 0.5f;               // takeoff kick
        }
        if (gGrounded[i]) {                       // chatter from slope change
          float sl = (terrainY(wx + 3.0f) - terrainY(wx - 3.0f)) * (1.0f / 6.0f);
          gWheelTiltVel[i] += (sl - gPrevSl[i]) * 6.0f;
          gPrevSl[i] = sl;
        }
        gWheelTiltVel[i] += (-TILT_K * gWheelTilt[i] - TILT_D * gWheelTiltVel[i]) * wdt;
        gWheelTilt[i] += gWheelTiltVel[i] * wdt;
        if (gWheelTilt[i] > TILT_MAX) gWheelTilt[i] = TILT_MAX;
        if (gWheelTilt[i] < -TILT_MAX) gWheelTilt[i] = -TILT_MAX;
        gPrevGrounded[i] = gGrounded[i];
      }
    }

    // suspension display easing (grounded wheels sit on the ground/planks)
    {
      float c = cosf(car.ang), s = sinf(car.ang);
      float ke = 20.0f * dtF; if (ke > 1.0f) ke = 1.0f;
      for (int i = 0; i < 2; i++) {
        float ox = (i == 0) ? -WHEELBASE : WHEELBASE;
        float wx = car.x + ox * c - WHEEL_DY * s;
        float ry = car.y + ox * s + WHEEL_DY * c;
        float target = gGrounded[i] ? (groundAt(wx) - WHEEL_R) : ry;
        gWheelDispY[i] += (target - gWheelDispY[i]) * ke;
      }
    }

    genItemsUpTo(gCamX + 300.0f);
    collectItems();

    gBlinkT += dtF;                               // low-fuel blink
    if (gBlinkT >= 0.25f) { gBlinkT = 0.0f; gBlink = !gBlink; }

    // out of fuel: run ends once the car has stopped
    if (!gCrashed && gFuel <= 0.0f) {
      bool slow = (gGrounded[0] || gGrounded[1]) && fabsf(car.vx) < 6.0f;
      gStillT = slow ? gStillT + dtF : 0.0f;
      if (gStillT > STILL_TIME) { gReason = 1; enterOver(); }
    }
    break; }

  case ST_PAUSE:
    if (in.pressed & (fb::KEY_LEFT | fb::KEY_UP))
      gMenuSel = (uint8_t)((gMenuSel + 2) % 3);
    if (in.pressed & (fb::KEY_RIGHT | fb::KEY_DOWN))
      gMenuSel = (uint8_t)((gMenuSel + 1) % 3);
    if (in.pressed & fb::KEY_SELECT) {
      if (gMenuSel == 0) { gState = ST_PLAY; gAcc = 0; }   // resume
      else if (gMenuSel == 1) startRun();                  // restart
      else return fb::GAME_EXIT;                           // quit
    }
    break;

  case ST_OVER:
    if (gOverGrace > 0.0f) { gOverGrace -= dtF; break; }
    if (in.pressed & (fb::KEY_LEFT | fb::KEY_UP))
      gMenuSel = (uint8_t)((gMenuSel + 2) % 3);
    if (in.pressed & (fb::KEY_RIGHT | fb::KEY_DOWN))
      gMenuSel = (uint8_t)((gMenuSel + 1) % 3);
    if (in.pressed & fb::KEY_SELECT) {
      if (gMenuSel == 0) startRun();                       // retry
      else if (gMenuSel == 1) gState = ST_TITLE;           // title
      else return fb::GAME_EXIT;                           // quit
    }
    break;
  }
  return fb::GAME_RUN;
}

// ===========================================================================
// Rendering
// ===========================================================================
static void drawTerrain(U8G2& d, int camXi, float camY) {
  uint8_t* buf = d.getBufferPtr();
  for (int sx = 0; sx < 128; sx++) {
    int hs = IR(terrainY((float)(camXi + sx)) - camY);
    if (hs < HUD_TOP) hs = HUD_TOP;              // keep the HUD strip clear
    if (hs > 63) continue;
    int p0 = hs >> 3;
    int yEnd = (p0 + 1) << 3;
    if (yEnd > 64) yEnd = 64;
    for (int y = hs; y < yEnd; y++) {            // 2px solid surface + dither
      if (y < hs + 2 || !(((sx + y) & 1)))
        buf[(y >> 3) * 128 + sx] |= (uint8_t)(1u << (y & 7));
    }
    for (int p = p0 + 1; p < 8; p++)             // full pages: checkerboard
      buf[p * 128 + sx] |= (uint8_t)((sx & 1) ? 0xAAu : 0x55u);
  }
}

static void drawBridges(U8G2& d, int camXi, float camY) {
  for (int b = 0; b < BRIDGE_N; b++) {
    const BridgeS& br = gBridges[b];
    if (!br.active) continue;
    if (br.x1 < (float)camXi - 4.0f || br.x0 > (float)camXi + 132.0f) continue;
    for (int i = 0; i < PLANK_N - 1; i++) {      // planks (2px, small gaps)
      int x0 = IR(br.x0 + (float)i * BRIDGE_SPACING) - camXi;
      int x1 = IR(br.x0 + (float)(i + 1) * BRIDGE_SPACING) - camXi;
      int y0 = IR(br.y[i] - camY);
      int y1 = IR(br.y[i + 1] - camY);
      d.drawLine(x0, y0, x1, y1);
      d.drawLine(x0, y0 + 1, x1, y1 + 1);
    }
    int ex0 = IR(br.x0) - camXi, ex1 = IR(br.x1) - camXi;
    int ey0 = IR(br.restY[0] - camY), ey1 = IR(br.restY[PLANK_N - 1] - camY);
    d.drawVLine(ex0, ey0, 9); d.drawVLine(ex0 + 1, ey0, 9);   // end posts
    d.drawVLine(ex1, ey1, 9); d.drawVLine(ex1 - 1, ey1, 9);
    d.drawLine(ex0, ey0 + 4, ex1, ey1 + 4);                   // bottom chord
    for (int i = 3; i <= PLANK_N - 4; i += 3) {               // hangers
      int hx = IR(br.x0 + (float)i * BRIDGE_SPACING) - camXi;
      int hy = IR(br.y[i] - camY);
      d.drawVLine(hx, hy + 1, 4);
    }
  }
}

static void drawItems(U8G2& d, int camXi, float camY) {
  for (int i = 0; i < COIN_N; i++) {
    const CoinS& cn = gCoinsA[i];
    if (!cn.active) continue;
    int sx = IR(cn.x) - camXi;
    if (sx < -4 || sx > 131) continue;
    int sy = IR(cn.y - camY);
    if (sy < HUD_TOP || sy > 63) continue;
    d.drawCircle(sx, sy, 3);
    d.drawDisc(sx, sy, 1);
  }
  for (int i = 0; i < FUEL_N; i++) {
    const FuelS& fc = gFuelA[i];
    if (!fc.active) continue;
    int sx = IR(fc.x) - camXi;
    if (sx < -4 || sx > 131) continue;
    int sy = IR(fc.y - camY);
    if (sy < 14 || sy > 58) continue;            // 7x9 canister fully on screen
    d.drawBox(sx - 3, sy - 4, 7, 9);
    d.setDrawColor(0);                           // dark "F" mark
    d.drawVLine(sx - 1, sy - 2, 5);
    d.drawHLine(sx - 1, sy - 2, 4);
    d.drawHLine(sx - 1, sy, 3);
    d.setDrawColor(1);
  }
}

// the only rotated object: the car sprite (tilt = per-wheel wobble angle)
static void drawCarSprite(U8G2& d, float scx, float scy, float ang,
                          float wRot, int wy0, int wy1, float t0, float t1) {
  float c = cosf(ang), s = sinf(ang);
  for (int i = 0; i < 2; i++) {                  // wheels + struts + wobbling spokes
    float ox = (i == 0) ? -WHEELBASE : WHEELBASE;
    float tilt = (i == 0) ? t0 : t1;
    float wAng = wRot + tilt;
    float spc = cosf(wAng), sps = sinf(wAng);
    int s1x = IR(3.0f * spc), s1y = IR(3.0f * sps);
    int wsx = IR(scx + ox * c - WHEEL_DY * s) + IR(tilt * 2.0f);
    int wsy = (i == 0) ? wy0 : wy1;
    d.drawCircle(wsx, wsy, (int)WHEEL_R);
    d.drawLine(wsx - s1x, wsy - s1y, wsx + s1x, wsy + s1y);
    d.drawLine(wsx + s1y, wsy - s1x, wsx - s1y, wsy + s1x);
    int ax = IR(scx + ox * c - 1.0f * s);        // body anchor (ox, 1)
    int ay = IR(scy + ox * s + 1.0f * c);
    d.drawLine(ax, ay, wsx, wsy);                // suspension strut
  }
  // body outline: rear, roll cage, windshield, hood, bumper, floor
  static const float poly[8][2] = {
    {-11.0f, -1.0f}, {-11.0f, -6.0f}, {-4.0f, -6.0f}, {-1.0f, -10.0f},
    { 3.0f, -10.0f}, { 5.0f,  -6.0f}, {11.0f, -5.0f}, {11.0f,  -1.0f}
  };
  int px[8], py[8];
  for (int k = 0; k < 8; k++) {
    px[k] = IR(scx + poly[k][0] * c - poly[k][1] * s);
    py[k] = IR(scy + poly[k][0] * s + poly[k][1] * c);
  }
  for (int k = 0; k < 8; k++) {
    int k2 = (k + 1) & 7;
    d.drawLine(px[k], py[k], px[k2], py[k2]);
  }
  // driver: helmet + torso
  int hx = IR(scx + 1.0f * c + 8.0f * s);
  int hy = IR(scy + 1.0f * s - 8.0f * c);
  int bx = IR(scx + 2.0f * s);
  int by = IR(scy - 2.0f * c);
  d.drawDisc(hx, hy, 2);
  d.drawLine(hx, hy, bx, by);
}

static void drawCarWorld(U8G2& d, int camXi, float camY) {
  float scx = car.x - (float)camXi;
  float scy = car.y - camY;
  float ang = car.ang;
  if (gAir) ang += sinf(gWobPh) * 0.045f;        // visual-only airborne wobble
  drawCarSprite(d, scx, scy, ang, car.wheelRot,
                IR(gWheelDispY[0] - camY), IR(gWheelDispY[1] - camY),
                gWheelTilt[0], gWheelTilt[1]);
}

static void drawHud(U8G2& d) {
  char b[12];
  d.setFont(u8g2_font_6x10_tr);
  d.setFontMode(0);                              // solid bg: readable over hills
  snprintf(b, sizeof b, "%um", (unsigned)(gDistPx / PX_PER_M));
  d.drawStr(0, 9, b);
  d.drawCircle(38, 4, 2);                        // coin icon + count
  d.drawDisc(38, 4, 1);
  snprintf(b, sizeof b, "x%u", (unsigned)gCoinsRun);
  d.drawStr(43, 9, b);
  d.drawStr(71, 9, "F");                         // fuel gauge
  d.drawFrame(80, 0, 47, 9);
  int w = (int)((43.0f * gFuel) / TANK);
  if (w > 0 && (gFuel > 25.0f || gBlink)) d.drawBox(82, 2, w, 5);
}

static void drawPause(U8G2& d) {
  d.setDrawColor(0); d.drawBox(28, 8, 72, 50);
  d.setDrawColor(1);  d.drawFrame(28, 8, 72, 50);
  d.setFontMode(1);
  d.setFont(u8g2_font_7x13B_tr);
  int w = d.getStrWidth("PAUSED");
  d.drawStr(64 - w / 2, 22, "PAUSED");
  d.setFont(u8g2_font_6x10_tr);
  static const char* const opts[3] = {"RESUME", "RESTART", "QUIT"};
  for (int k = 0; k < 3; k++) {
    int y = 34 + k * 10;
    int tw = d.getStrWidth(opts[k]);
    int x = 64 - tw / 2;
    if (k == (int)gMenuSel) {
      d.drawBox(x - 3, y - 8, tw + 6, 11);
      d.setDrawColor(0);
      d.drawStr(x, y, opts[k]);
      d.setDrawColor(1);
    } else {
      d.drawStr(x, y, opts[k]);
    }
  }
}

static void drawOver(U8G2& d) {
  d.setDrawColor(0); d.drawBox(10, 1, 108, 62);
  d.setDrawColor(1);  d.drawFrame(10, 1, 108, 62);
  d.setFontMode(1);
  d.setFont(u8g2_font_7x13B_tr);
  const char* r = (gReason == 1) ? "OUT OF FUEL" : "CRASHED!";
  int w = d.getStrWidth(r);
  d.drawStr(64 - w / 2, 14, r);
  d.setFont(u8g2_font_6x10_tr);
  char b[16];
  snprintf(b, sizeof b, "DIST %um", (unsigned)(gDistPx / PX_PER_M));
  d.drawStr(18, 28, b);
  snprintf(b, sizeof b, "COINS %u", (unsigned)gCoinsRun);
  d.drawStr(18, 38, b);
  snprintf(b, sizeof b, "BEST %um", (unsigned)gBestM);
  d.drawStr(18, 47, b);
  static const char* const opts[3] = {"RETRY", "TITLE", "QUIT"};
  int tw[3]; int total = 0;
  for (int k = 0; k < 3; k++) { tw[k] = d.getStrWidth(opts[k]); total += tw[k]; }
  total += 16;
  int x = (128 - total) / 2;
  for (int k = 0; k < 3; k++) {
    if (k == (int)gMenuSel) {
      d.drawBox(x - 2, 51, tw[k] + 4, 11);
      d.setDrawColor(0);
      d.drawStr(x, 59, opts[k]);
      d.setDrawColor(1);
    } else {
      d.drawStr(x, 59, opts[k]);
    }
    x += tw[k] + 8;
  }
}

static void renderTitle(U8G2& d) {
  const float tcx = 56.0f;                       // parked car world x (camXi = 0)
  float camY = terrainY(tcx) - 42.0f;
  drawTerrain(d, 0, camY);
  float sl = (terrainY(tcx + 3.0f) - terrainY(tcx - 3.0f)) * (1.0f / 6.0f);
  float ang = sl / (1.0f + 0.28f * sl * sl);
  float scy = terrainY(tcx) - 8.0f - camY;
  int wy0 = IR(terrainY(tcx - WHEELBASE) - WHEEL_R - camY);
  int wy1 = IR(terrainY(tcx + WHEELBASE) - WHEEL_R - camY);
  drawCarSprite(d, tcx, scy, ang, gTitleWheel, wy0, wy1, 0.0f, 0.0f);

  d.setFontMode(1);
  d.setFont(u8g2_font_7x13B_tr);
  int w = d.getStrWidth("HILL CLIMB");
  d.drawStr(64 - w / 2, 12, "HILL CLIMB");
  d.setFont(u8g2_font_5x7_tr);
  char b[16];
  snprintf(b, sizeof b, "BEST %um", (unsigned)gBestM);
  w = d.getStrWidth(b);
  d.drawStr(64 - w / 2, 21, b);
  d.setDrawColor(0); d.drawBox(0, 48, 128, 16);  // info band
  d.setDrawColor(1);
  d.setFont(u8g2_font_6x10_tr);
  w = d.getStrWidth("SELECT: START");
  d.drawStr(64 - w / 2, 56, "SELECT: START");
  d.setFont(u8g2_font_5x7_tr);
  w = d.getStrWidth("flipBOX");
  d.drawStr(64 - w / 2, 63, "flipBOX");
}

static void gameRender(U8G2& d) {
  d.setDrawColor(1);
  if (gState == ST_TITLE) { renderTitle(d); return; }
  int camXi = (int)floorf(gCamX);
  drawTerrain(d, camXi, gCamY);
  drawBridges(d, camXi, gCamY);
  drawItems(d, camXi, gCamY);
  drawCarWorld(d, camXi, gCamY);
  drawHud(d);
  if (gState == ST_PAUSE) drawPause(d);
  else if (gState == ST_OVER) drawOver(d);
}

}  // namespace

FB_GAME(HillClimb, gameInit, gameUpdate, gameRender, nullptr);