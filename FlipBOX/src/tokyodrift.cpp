// ============================================================================
// src/Game3.cpp  --  TOKYO DRIFT  (flipBOX)
//
// True top-down, momentum-based arcade drifting racer for the 128x64
// monochrome OLED. The player's car sits low in the playable area so most of
// the screen shows the road ahead, while the track scrolls AND rotates around
// it: the camera looks along the car's MOMENTUM (velocity) direction, while
// the car body shows the true heading, so drifts visibly tilt the car.
// Large directional arrows are painted on the road surface itself and
// move/rotate with the world (they are transformed with the exact same
// world->camera transform as the road geometry).
//
// Screen layout: rows 0..kPlayH-1 are the playable/road area; the bottom
// kHudH rows are one dedicated HUD strip (LAP | SPEED | TIME). World
// rendering is clipped to the playable area so it can never reach the HUD.
//
// Controls
//   UP                  accelerate
//   DOWN                brake / reverse
//   LEFT / RIGHT        steer
//   DOWN + LEFT/RIGHT   drift (needs some speed)
//   SELECT              title: start | race: pause | menus: confirm
//   hold SELECT ~1 s    quit to launcher (anywhere, with progress bar)
//
// Race flow: TITLE -> COUNTDOWN (3-2-1-GO) -> RACE (3 laps) -> RESULTS
//            plus PAUSE (resume / restart / quit).
// ============================================================================

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

#include <stdio.h>   // snprintf (no heap, no String)
#include <math.h>    // sinf/atan2f used ONLY at init time (table building)

namespace {

// ---------------------------------------------------------------------------
// Tuning constants (units in comments) -- tweak the feel here
// ---------------------------------------------------------------------------
constexpr uint8_t  kLaps          = 3;          // race length, laps
constexpr int      kTrackN        = 192;        // centreline points in the loop
constexpr int16_t  kRoadHalf      = 16;         // road half-width, px
constexpr float    kMaxSpeed      = 175.f;      // px/s, forward top speed
constexpr float    kMaxReverse    = 45.f;       // px/s
constexpr float    kAccel         = 150.f;      // px/s^2
constexpr float    kBrake         = 230.f;      // px/s^2
constexpr float    kDrag          = 0.45f;      // 1/s, linear aero drag
constexpr float    kRoll          = 14.f;       // px/s^2, coasting resistance
constexpr float    kGrip          = 6.5f;       // 1/s, lateral grip (normal)
constexpr float    kDriftGrip     = 1.6f;       // 1/s, lateral grip while drifting
constexpr float    kSteer         = 150.f;      // brads/s steering rate (256 brads = 360 deg)
constexpr float    kDriftSteerMul = 1.75f;      // steering boost while drifting
constexpr float    kDriftMinSpeed = 45.f;       // px/s forward speed needed to drift
constexpr float    kGrassDrag     = 2.6f;       // extra 1/s drag when off the road
constexpr float    kCamRate       = 5.0f;       // 1/s, camera chase rate
constexpr float    kCamVelBlend   = 0.85f;      // 0..1, camera look: 0 = bonnet, 1 = pure momentum
constexpr float    kCamMaxSlip    = 48.f;       // brads, max look offset from the bonnet (~67 deg)
constexpr float    kCamSpd0       = 25.f;       // px/s, below this the camera just follows the bonnet
constexpr float    kCamSpd1       = 60.f;       // px/s, momentum look fully faded in from here
constexpr float    kCamFwdFull    = 20.f;       // px/s forward speed for full momentum look (0 when reversing)
constexpr float    kCamMaxRate    = 100.f;      // brads/s, hard cap on camera rotation speed
constexpr float    kKmhScale      = 1.35f;      // px/s -> displayed km/h
constexpr uint32_t kQuitHoldUs    = 1000000UL;  // hold SELECT this long to exit
constexpr uint32_t kCdStepUs      = 700000UL;   // countdown step (3,2,1,GO)
constexpr int      kVisBack       = 5;          // centreline points drawn behind car
constexpr int      kVisFwd        = 48;         // ... and ahead of the car
constexpr int      kVisN          = kVisBack + kVisFwd;
constexpr int      kArrowEvery    = 6;          // one road arrow per N points
constexpr float    kCullR2        = 250.f * 250.f; // px^2, ignore points beyond
constexpr int16_t  kHudH          = 9;          // bottom HUD strip height, px (rule + gap + 6 px text + pad)
constexpr int16_t  kPlayH         = fb::SCREEN_H - kHudH;  // playable rows 0..kPlayH-1
constexpr int16_t  kHudY          = kPlayH;     // first HUD row (the separator rule)
constexpr int16_t  kCarX0         = fb::SCREEN_W / 2;  // car screen position
constexpr int16_t  kCarY0         = 46;         // low in the play area -> long view ahead
constexpr int      kTrailN        = 22;         // skid-trail ring size
constexpr int      kPuffN         = 18;         // smoke particle count

// Compile-time layout guards: the car (<= 7 px from its centre, +1 rumble)
// must fit inside the play area, and the drawn window must not wrap onto itself.
static_assert(kCarY0 + 8 <= kPlayH, "car must stay inside the playable area");
static_assert(kCarY0 - 8 >= 0,      "car must stay inside the playable area");
static_assert(kVisN < kTrackN,      "visible window must be shorter than the loop");

// ---------------------------------------------------------------------------
// Sine table (Q15, 256 brads = full circle), built once per game entry
// ---------------------------------------------------------------------------
int16_t gSinT[256];

inline int32_t sinB(uint8_t a) { return gSinT[a]; }
inline int32_t cosB(uint8_t a) { return gSinT[(uint8_t)(a + 64)]; }

void buildTables() {
  for (int i = 0; i < 256; ++i)
    gSinT[i] = (int16_t)(sinf((float)i * (6.2831853f / 256.f)) * 32767.f);
}

// Runtime-safe atan2 in brads (no libm): returns -128..128, error < 0.25 brad.
// Uses atan(t) ~ t / (1 + 0.28 t^2) on the first octant, then folds.
float atan2B(float y, float x) {
  const float ax = x < 0.f ? -x : x;
  const float ay = y < 0.f ? -y : y;
  if (ax < 1e-4f && ay < 1e-4f) return 0.f;
  float a;
  if (ay <= ax) { const float t = ay / ax; a = (t / (1.f + 0.28f * t * t)) * (256.f / 6.2831853f); }
  else          { const float t = ax / ay; a = 64.f - (t / (1.f + 0.28f * t * t)) * (256.f / 6.2831853f); }
  if (x < 0.f) a = 128.f - a;
  return (y < 0.f) ? -a : a;
}

// ---------------------------------------------------------------------------
// Track: closed loop defined by radius harmonics (always loops cleanly).
// World coordinates in whole pixels (float), +y up, angles in brads.
// ---------------------------------------------------------------------------
float   gTX[kTrackN];   // centreline x
float   gTY[kTrackN];   // centreline y
uint8_t gTTan[kTrackN]; // tangent angle (brads)
int8_t  gTCur[kTrackN]; // curvature ahead (signed brads over ~8 pts)

void buildTrack() {
  for (int i = 0; i < kTrackN; ++i) {
    const uint8_t th = (uint8_t)((i * 256) / kTrackN);
    const float s1 = sinB(th)                    * (1.f / 32768.f);
    const float c1 = cosB(th)                    * (1.f / 32768.f);
    const float r  = 600.f
                   + 170.f * (sinB((uint8_t)(th * 2))         * (1.f / 32768.f))
                   + 105.f * (sinB((uint8_t)(th * 3 + 64))    * (1.f / 32768.f))
                   +  60.f * (sinB((uint8_t)(th * 5 + 190))   * (1.f / 32768.f));
    gTX[i] = r * c1;
    gTY[i] = r * s1;
  }
  for (int i = 0; i < kTrackN; ++i) {
    const int p = (i + kTrackN - 1) % kTrackN;
    const int n = (i + 1) % kTrackN;
    // atan2f at init time only (allowed): tangent angle in brads
    float a = atan2f(gTY[n] - gTY[p], gTX[n] - gTX[p]) * (256.f / 6.2831853f);
    if (a < 0.f) a += 256.f;
    gTTan[i] = (uint8_t)(int32_t)a;
  }
  for (int i = 0; i < kTrackN; ++i) {
    const int b = (i + kTrackN - 8) % kTrackN;
    const int a = (i + 8) % kTrackN;
    gTCur[i] = (int8_t)(gTTan[a] - gTTan[b]);   // wraps: signed bend amount
  }
}

// ---------------------------------------------------------------------------
// Game state (ALL of it reset in gameInit()/startRace())
// ---------------------------------------------------------------------------
enum State : uint8_t { ST_TITLE, ST_COUNT, ST_RACE, ST_PAUSE, ST_RESULT };

State    gState;
float    gCarX, gCarY;      // world position, px
float    gVX, gVY;          // world velocity, px/s
float    gHead;             // car heading, brads 0..256
float    gCamA;             // camera "up" direction, brads 0..256
float    gSpeed;            // |v| estimate, px/s (HUD)
bool     gDrift;            // currently drifting
int8_t   gDriftDir;         // +1 = drifting left(CCW), -1 = right
bool     gOffTrack;
int      gSeg, gPrevSeg;    // nearest centreline index
int      gLap;              // 1..kLaps
uint32_t gRaceMs, gLapMs;
uint32_t gBestLapMs;        // reset per race entry
float    gDriftAcc;         // drift points accumulator
uint32_t gCdUs;             // countdown timer
uint32_t gSelHoldUs;        // SELECT hold timer (quit)
uint32_t gTitleMs;          // title blink timer
uint8_t  gMenuIdx;          // pause / results cursor
uint32_t gSpawnUs, gTrailUs;// effect timers
int8_t   gRumble;           // off-track jitter, px
uint32_t gBestTotalMs;      // SESSION-PERSISTENT best race time (not reset)

uint32_t gRng;
uint32_t rnd() { gRng ^= gRng << 13; gRng ^= gRng >> 17; gRng ^= gRng << 5; return gRng; }

struct Puff { float x, y; uint16_t life; };   // life in ms, 0 = dead
Puff    gPuff[kPuffN];

float    gTrLX[kTrailN], gTrLY[kTrailN];      // skid trail, left wheel
float    gTrRX[kTrailN], gTrRY[kTrailN];      // skid trail, right wheel
uint16_t gTrAge[kTrailN];                     // ms since written, >1100 = gone
uint8_t  gTrHead;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void startRace() {
  gCarX = gTX[1]; gCarY = gTY[1];
  gVX = 0.f; gVY = 0.f;
  gHead = (float)gTTan[1];
  gCamA = gHead;
  gSeg = 1; gPrevSeg = 1;
  gLap = 1;
  gRaceMs = 0; gLapMs = 0; gBestLapMs = 0;
  gDriftAcc = 0.f;
  gDrift = false; gDriftDir = 0;
  gOffTrack = false;
  gSpeed = 0.f;
  gRumble = 0;
  gSpawnUs = 0; gTrailUs = 0;
  for (int i = 0; i < kPuffN; ++i) gPuff[i].life = 0;
  for (int i = 0; i < kTrailN; ++i) gTrAge[i] = 60000;
  gTrHead = 0;
}

void locate() {
  float best = 1e30f;
  int   bi   = gSeg;
  for (int k = -6; k <= 10; ++k) {
    int i = (gSeg + k) % kTrackN;
    if (i < 0) i += kTrackN;
    const float dx = gTX[i] - gCarX, dy = gTY[i] - gCarY;
    const float d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; bi = i; }
  }
  gPrevSeg = gSeg;
  gSeg = bi;
  gOffTrack = best > (float)((kRoadHalf + 3) * (kRoadHalf + 3)); // squared compare, no sqrt
}

void spawnPuff(float bx, float by) {
  for (int i = 0; i < kPuffN; ++i) {
    if (gPuff[i].life == 0) {
      const int o = (int)(rnd() % 7) - 3;          // small lateral scatter
      const uint8_t hb = (uint8_t)(int32_t)gHead;
      const float hx = cosB(hb) * (1.f / 32768.f), hy = sinB(hb) * (1.f / 32768.f);
      gPuff[i].x = bx + hy * (float)o;             // right vector = (hy,-hx)
      gPuff[i].y = by - hx * (float)o;
      gPuff[i].life = 500;
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// Physics -- all dt-driven (frame-rate independent), float per-frame only
// ---------------------------------------------------------------------------
void stepRace(const fb::Input& in, float dt, uint32_t dtUs) {
  const uint32_t dtMs = dtUs / 1000;
  const bool up    = (in.held & fb::KEY_UP)    != 0;
  const bool dn    = (in.held & fb::KEY_DOWN)  != 0;
  // LEFT = +1 (counter-clockwise = left when facing +heading)
  const int  steer = ((in.held & fb::KEY_LEFT) ? 1 : 0) - ((in.held & fb::KEY_RIGHT) ? 1 : 0);

  const uint8_t hb = (uint8_t)(int32_t)gHead;
  const float hx = cosB(hb) * (1.f / 32768.f);
  const float hy = sinB(hb) * (1.f / 32768.f);

  // decompose velocity into heading frame: forward / lateral
  float vF = gVX * hx + gVY * hy;
  float vL = -gVX * hy + gVY * hx;

  locate();

  // ---- drift state: DOWN + steer while carrying speed ----
  gDrift = dn && (steer != 0) && (vF > kDriftMinSpeed);
  if (gDrift) gDriftDir = (int8_t)steer;

  // ---- lateral grip (drifting slashes it -> the car slides) ----
  const float grip = gDrift ? kDriftGrip : kGrip;
  float g = grip * dt; if (g > 1.f) g = 1.f;
  vL -= vL * g;

  // ---- longitudinal ----
  if (up) vF += kAccel * dt;
  if (dn) vF -= (gDrift ? kBrake * 0.20f : kBrake) * dt;   // drift keeps momentum
  vF -= vF * kDrag * dt;
  if (!up && !dn) {
    const float r = kRoll * dt;
    if (vF > r) vF -= r; else if (vF < -r) vF += r; else vF = 0.f;
  }
  if (gOffTrack) {
    float g2 = kGrassDrag * dt; if (g2 > 1.f) g2 = 1.f;
    vF -= vF * g2;
    float g3 = 4.f * dt;    if (g3 > 1.f) g3 = 1.f;
    vL -= vL * g3;
  }
  if (vF >  kMaxSpeed)   vF =  kMaxSpeed;
  if (vF < -kMaxReverse) vF = -kMaxReverse;

  // ---- steering (sign of vF flips it naturally in reverse) ----
  float sf = vF / 55.f;
  if (sf > 1.f) sf = 1.f; else if (sf < -1.f) sf = -1.f;
  float topFade = 1.f - 0.30f * (vF / kMaxSpeed);
  if (topFade < 0.7f) topFade = 0.7f;
  gHead += (float)steer * kSteer * sf * topFade * (gDrift ? kDriftSteerMul : 1.f) * dt;
  while (gHead >= 256.f) gHead -= 256.f;
  while (gHead <   0.f)  gHead += 256.f;

  // ---- reassemble velocity, integrate position ----
  gVX = hx * vF - hy * vL;
  gVY = hy * vF + hx * vL;
  gCarX += gVX * dt;
  gCarY += gVY * dt;
  gSpeed = (vF < 0 ? -vF : vF) + 0.5f * (vL < 0 ? -vL : vL);   // cheap |v| approx

  // ---- camera: look along the MOMENTUM (velocity) direction, not the bonnet ----
  // target = heading + blend * confidence * (velocity angle - heading), where the
  // offset is capped, and confidence fades to 0 at low speed and when reversing
  // (the velocity direction is meaningless / backwards there). The camera then
  // eases toward the target with a capped rotation rate: no snapping, no flips.
  float target = gHead;
  const float sp2 = gVX * gVX + gVY * gVY;
  if (vF > 0.f && sp2 > kCamSpd0 * kCamSpd0) {
    float wS = (sp2 - kCamSpd0 * kCamSpd0) / (kCamSpd1 * kCamSpd1 - kCamSpd0 * kCamSpd0);
    if (wS > 1.f) wS = 1.f;
    float wF = vF * (1.f / kCamFwdFull);
    if (wF > 1.f) wF = 1.f;
    float off = atan2B(gVY, gVX) - gHead;    // signed look offset: + = momentum is left of the bonnet
    while (off >  128.f) off -= 256.f;
    while (off < -128.f) off += 256.f;
    if (off >  kCamMaxSlip) off =  kCamMaxSlip;
    else if (off < -kCamMaxSlip) off = -kCamMaxSlip;
    target = gHead + kCamVelBlend * wS * wF * off;
  }
  float cd = target - gCamA;
  while (cd >  128.f) cd -= 256.f;
  while (cd < -128.f) cd += 256.f;
  float ck = kCamRate * dt; if (ck > 1.f) ck = 1.f;
  float cs = cd * ck;
  const float cLim = kCamMaxRate * dt;
  if (cs > cLim) cs = cLim; else if (cs < -cLim) cs = -cLim;
  gCamA += cs;
  while (gCamA >= 256.f) gCamA -= 256.f;
  while (gCamA <   0.f)  gCamA += 256.f;

  // ---- lap counting (crossing start/finish going forward) ----
  if (gPrevSeg > (kTrackN * 3) / 4 && gSeg < kTrackN / 4 && gLapMs > 8000UL && vF > 0.f) {
    if (gBestLapMs == 0 || gLapMs < gBestLapMs) gBestLapMs = gLapMs;
    gLapMs = 0;
    ++gLap;
  }

  // ---- drift scoring + effects ----
  if (gDrift) {
    gDriftAcc += (vL < 0 ? -vL : vL) * dt * 6.f;
    gSpawnUs += dtUs;
    if (gSpawnUs >= 45000UL) {
      gSpawnUs = 0;
      spawnPuff(gCarX - hx * 5.f, gCarY - hy * 5.f);   // rear axle smoke
    }
    gTrailUs += dtUs;
    if (gTrailUs >= 35000UL) {
      gTrailUs = 0;
      const uint8_t w = gTrHead;
      gTrLX[w] = gCarX - 3.f * hy - 4.f * hx;  gTrLY[w] = gCarY + 3.f * hx - 4.f * hy;
      gTrRX[w] = gCarX + 3.f * hy - 4.f * hx;  gTrRY[w] = gCarY - 3.f * hx - 4.f * hy;
      gTrAge[w] = 0;
      gTrHead = (uint8_t)((w + 1) % kTrailN);
    }
  } else {
    gSpawnUs = 0; gTrailUs = 0;
  }

  // ---- off-track rumble + dust ----
  if (gOffTrack && gSpeed > 40.f) {
    gRumble = (int8_t)((rnd() & 1) ? 1 : -1);
    gSpawnUs += dtUs;
    if (gSpawnUs >= 60000UL) { gSpawnUs = 0; spawnPuff(gCarX - hx * 4.f, gCarY - hy * 4.f); }
  } else {
    gRumble = 0;
  }

  // ---- age effects ----
  for (int i = 0; i < kPuffN; ++i) {
    if (gPuff[i].life)
      gPuff[i].life = (gPuff[i].life > dtMs) ? (uint16_t)(gPuff[i].life - dtMs) : 0;
  }
  for (int i = 0; i < kTrailN; ++i) {
    if (gTrAge[i] < 60000)
      gTrAge[i] = (uint16_t)((gTrAge[i] + dtMs > 60000) ? 60000 : gTrAge[i] + dtMs);
  }
}

// ---------------------------------------------------------------------------
// init / update
// ---------------------------------------------------------------------------
void gameInit() {
  buildTables();
  buildTrack();
  gRng = micros() ^ 0x9E3779B9UL;   // micros(): allowed for seeding only
  if (!gRng) gRng = 1;
  gState     = ST_TITLE;
  gTitleMs   = 0;
  gSelHoldUs = 0;
  gMenuIdx   = 0;
  gCdUs      = 0;
  startRace();
  // NOTE: gBestTotalMs deliberately NOT reset -> session best survives re-entry.
}

fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  const float    dt = (float)f.dtUs * 1e-6f;

  // hold SELECT ~1 s anywhere -> back to launcher
  if (in.held & fb::KEY_SELECT) {
    gSelHoldUs += f.dtUs;
    if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
  } else {
    gSelHoldUs = 0;
  }

  switch (gState) {
    case ST_TITLE:
      gTitleMs += f.dtUs / 1000;
      if (in.pressed & fb::KEY_SELECT) {
        startRace();
        gCdUs  = 0;
        gState = ST_COUNT;
      }
      break;

    case ST_COUNT:
      gCdUs += f.dtUs;
      if (gCdUs >= 4UL * kCdStepUs) {          // 3,2,1,GO! -> green
        gState  = ST_RACE;
        gRaceMs = 0;
        gLapMs  = 0;
      }
      break;

    case ST_RACE:
      gRaceMs += f.dtUs / 1000;
      gLapMs  += f.dtUs / 1000;
      stepRace(in, dt, f.dtUs);
      if (in.pressed & fb::KEY_SELECT) {
        gState = ST_PAUSE;
        gMenuIdx = 0;
      } else if (gLap > kLaps) {
        gState = ST_RESULT;
        gMenuIdx = 0;
        if (gBestTotalMs == 0 || gRaceMs < gBestTotalMs) gBestTotalMs = gRaceMs;
      }
      break;

    case ST_PAUSE:
      if (in.pressed & (fb::KEY_UP | fb::KEY_DOWN))
        gMenuIdx = (uint8_t)((gMenuIdx + ((in.pressed & fb::KEY_DOWN) ? 1 : 2)) % 3);
      if (in.pressed & fb::KEY_SELECT) {
        if      (gMenuIdx == 0) gState = ST_RACE;
        else if (gMenuIdx == 1) { startRace(); gCdUs = 0; gState = ST_COUNT; }
        else                    return fb::GAME_EXIT;
      }
      break;

    case ST_RESULT:
      if (in.pressed & (fb::KEY_UP | fb::KEY_DOWN)) gMenuIdx ^= 1;
      if (in.pressed & fb::KEY_SELECT) {
        if (gMenuIdx == 0) { startRace(); gCdUs = 0; gState = ST_COUNT; }
        else               return fb::GAME_EXIT;
      }
      break;
  }
  return fb::GAME_RUN;
}

// ---------------------------------------------------------------------------
// Rendering. Camera rotation C: the world direction gCamA appears pointing
// UP on screen, so camera coords of a relative vector r are
//   cx = rx*cosC + ry*sinC ; cy = -rx*sinC + ry*cosC  with C = gCamA - 64 brads
//   screen = (kCarX0 + cx, kCarY0 - cy)
// ---------------------------------------------------------------------------

// Cohen-Sutherland clip to the PLAYABLE rect (128 x kPlayH, i.e. everything
// above the HUD strip; U8g2's unsigned coords must never receive a negative
// value, and no world line may ever reach the HUD rows).
inline uint8_t clipCode(int16_t x, int16_t y) {
  return (uint8_t)((x < 0 ? 1 : 0) | (x > (int16_t)(fb::SCREEN_W - 1) ? 2 : 0) |
                   (y < 0 ? 4 : 0) | (y > (int16_t)(kPlayH - 1) ? 8 : 0));
}

bool clipSeg(int16_t& x0, int16_t& y0, int16_t& x1, int16_t& y1) {
  uint8_t c0 = clipCode(x0, y0), c1 = clipCode(x1, y1);
  for (int it = 0; it < 8; ++it) {
    if (!(c0 | c1)) return true;
    if (c0 & c1)    return false;
    const uint8_t c = c0 ? c0 : c1;
    int16_t x, y;
    if      (c & 8) { x = (int16_t)(x0 + (int16_t)(((int32_t)(x1 - x0) * ((int16_t)(kPlayH - 1) - y0)) / (y1 - y0))); y = (int16_t)(kPlayH - 1); }
    else if (c & 4) { x = (int16_t)(x0 + (int16_t)(((int32_t)(x1 - x0) * (0 - y0)) / (y1 - y0))); y = 0; }
    else if (c & 2) { y = (int16_t)(y0 + (int16_t)(((int32_t)(y1 - y0) * ((int16_t)(fb::SCREEN_W - 1) - x0)) / (x1 - x0))); x = (int16_t)(fb::SCREEN_W - 1); }
    else            { y = (int16_t)(y0 + (int16_t)(((int32_t)(y1 - y0) * (0 - x0)) / (x1 - x0))); x = 0; }
    if (c == c0) { x0 = x; y0 = y; c0 = clipCode(x0, y0); }
    else         { x1 = x; y1 = y; c1 = clipCode(x1, y1); }
  }
  return false;
}

void seg(U8G2& d, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  if (clipSeg(x0, y0, x1, y1))
    d.drawLine((u8g2_uint_t)x0, (u8g2_uint_t)y0, (u8g2_uint_t)x1, (u8g2_uint_t)y1);
}

// world point -> screen; returns false when outside the coarse view radius
inline bool xform(float wx, float wy, float cosC, float sinC, int16_t& sx, int16_t& sy) {
  const float rx = wx - gCarX, ry = wy - gCarY;
  const float px = rx * cosC + ry * sinC;
  const float py = -rx * sinC + ry * cosC;
  if (px * px + py * py > kCullR2) return false;
  sx = (int16_t)(kCarX0 + px);
  sy = (int16_t)(kCarY0 - py);
  return true;
}

// Directional arrow painted ON the road. scrA = world tangent angle - camera
// rotation, so the arrow turns with the road and points along the track.
void drawArrow(U8G2& d, int16_t cx, int16_t cy, uint8_t scrA, bool strong) {
  const int32_t s = sinB(scrA), c = cosB(scrA);
  const int16_t dx = (int16_t)((c * 7) >> 15);          // screen dir: (c, -s)
  const int16_t dy = (int16_t)(-(s * 7) >> 15);
  const int16_t tx = (int16_t)(cx + dx), ty = (int16_t)(cy + dy);
  seg(d, (int16_t)(cx - dx), (int16_t)(cy - dy), tx, ty);
  const uint8_t a1 = (uint8_t)(scrA + 160), a2 = (uint8_t)(scrA - 160);
  const int16_t w1x = (int16_t)((cosB(a1) * 4) >> 15), w1y = (int16_t)(-(sinB(a1) * 4) >> 15);
  const int16_t w2x = (int16_t)((cosB(a2) * 4) >> 15), w2y = (int16_t)(-(sinB(a2) * 4) >> 15);
  seg(d, tx, ty, (int16_t)(tx + w1x), (int16_t)(ty + w1y));
  seg(d, tx, ty, (int16_t)(tx + w2x), (int16_t)(ty + w2y));
  if (strong) {                                        // double chevron = hard corner
    const int16_t ox = (int16_t)((c * 4) >> 15), oy = (int16_t)(-(s * 4) >> 15);
    seg(d, (int16_t)(tx - ox), (int16_t)(ty - oy), (int16_t)(tx - ox + w1x), (int16_t)(ty - oy + w1y));
    seg(d, (int16_t)(tx - ox), (int16_t)(ty - oy), (int16_t)(tx - ox + w2x), (int16_t)(ty - oy + w2y));
  }
}

// Top-down car silhouette, rotated. scrA=64 -> pointing straight up.
void drawCarShape(U8G2& d, int16_t cx0, int16_t cy0, uint8_t scrA) {
  const int32_t s = sinB(scrA), c = cosB(scrA);
  static const int8_t pts[5][2] = { {0, 7}, {3, 4}, {3, -5}, {-3, -5}, {-3, 4} };
  int16_t X[5], Y[5];
  for (int i = 0; i < 5; ++i) {
    X[i] = (int16_t)(cx0 + (int16_t)((pts[i][0] * s + pts[i][1] * c) >> 15));
    Y[i] = (int16_t)(cy0 + (int16_t)((pts[i][0] * c - pts[i][1] * s) >> 15));
  }
  d.setDrawColor(1);
  d.drawTriangle(X[0], Y[0], X[1], Y[1], X[4], Y[4]);           // nose wedge
  d.drawTriangle(X[1], Y[1], X[2], Y[2], X[4], Y[4]);           // body
  d.drawTriangle(X[2], Y[2], X[3], Y[3], X[4], Y[4]);
  // wheels: short white lines alongside the body
  static const int8_t wh[4][4] = { {4, 4, 4, 2}, {-4, 4, -4, 2}, {4, -2, 4, -4}, {-4, -2, -4, -4} };
  for (int i = 0; i < 4; ++i) {
    const int16_t ax = (int16_t)(cx0 + (int16_t)((wh[i][0] * s + wh[i][1] * c) >> 15));
    const int16_t ay = (int16_t)(cy0 + (int16_t)((wh[i][0] * c - wh[i][1] * s) >> 15));
    const int16_t bx = (int16_t)(cx0 + (int16_t)((wh[i][2] * s + wh[i][3] * c) >> 15));
    const int16_t by = (int16_t)(cy0 + (int16_t)((wh[i][2] * c - wh[i][3] * s) >> 15));
    seg(d, ax, ay, bx, by);
  }
  // windows: dark triangles inside the body
  static const int8_t win[2][3][2] = { {{0, 3}, {2, 1}, {-2, 1}}, {{2, -2}, {-2, -2}, {0, -4}} };
  d.setDrawColor(0);
  for (int t = 0; t < 2; ++t) {
    int16_t wx[3], wy[3];
    for (int i = 0; i < 3; ++i) {
      wx[i] = (int16_t)(cx0 + (int16_t)((win[t][i][0] * s + win[t][i][1] * c) >> 15));
      wy[i] = (int16_t)(cy0 + (int16_t)((win[t][i][0] * c - win[t][i][1] * s) >> 15));
    }
    d.drawTriangle(wx[0], wy[0], wx[1], wy[1], wx[2], wy[2]);
  }
  d.setDrawColor(1);
}

void renderWorld(U8G2& d) {
  const uint8_t C    = (uint8_t)(int32_t)(gCamA - 64.f);
  const float   cosC = cosB(C) * (1.f / 32768.f);
  const float   sinC = sinB(C) * (1.f / 32768.f);

  int16_t mx[kVisN], my[kVisN];           // centreline on screen
  int16_t ex[kVisN], ey[kVisN];           // left edge
  int16_t fx[kVisN], fy[kVisN];           // right edge
  int8_t  ox[kVisN], oy[kVisN];           // 1 px step OUTSIDE the left edge (right edge uses the negative)
  int16_t ti[kVisN];                      // track index per slot
  uint8_t ok[kVisN];

  const int first = gSeg - kVisBack;
  for (int k = 0; k < kVisN; ++k) {
    int i = (first + k) % kTrackN;
    if (i < 0) i += kTrackN;
    ti[k] = (int16_t)i;
    ok[k] = 0;
    int16_t sx, sy;
    if (!xform(gTX[i], gTY[i], cosC, sinC, sx, sy)) continue;
    // left-of-tangent normal, rotated into camera space
    const uint8_t t = gTTan[i];
    const float nx = -(sinB(t) * (1.f / 32768.f));
    const float ny =  (cosB(t) * (1.f / 32768.f));
    const float nxc = nx * cosC + ny * sinC;
    const float nyc = -nx * sinC + ny * cosC;
    mx[k] = sx;                 my[k] = sy;
    ex[k] = (int16_t)(sx + nxc * kRoadHalf); ey[k] = (int16_t)(sy - nyc * kRoadHalf);
    fx[k] = (int16_t)(sx - nxc * kRoadHalf); fy[k] = (int16_t)(sy + nyc * kRoadHalf);
    // outward step for the thick road edge: dominant screen axis of (centre -> left edge)
    const int16_t odx = (int16_t)(ex[k] - sx), ody = (int16_t)(ey[k] - sy);
    const int16_t adx = (int16_t)(odx < 0 ? -odx : odx), ady = (int16_t)(ody < 0 ? -ody : ody);
    if (adx >= ady) { ox[k] = (int8_t)(odx < 0 ? -1 : 1); oy[k] = 0; }
    else            { ox[k] = 0; oy[k] = (int8_t)(ody < 0 ? -1 : 1); }
    ok[k] = 1;
  }

  for (int k = 1; k < kVisN; ++k) {
    if (!ok[k] || !ok[k - 1]) continue;
    const int i = ti[k];
    // road edges: the true boundary plus one extra pixel just OUTSIDE it, so the
    // edge reads 2 px thick while the drivable road keeps its exact width
    seg(d, ex[k - 1], ey[k - 1], ex[k], ey[k]);
    seg(d, (int16_t)(ex[k - 1] + ox[k - 1]), (int16_t)(ey[k - 1] + oy[k - 1]),
           (int16_t)(ex[k] + ox[k]),         (int16_t)(ey[k] + oy[k]));
    seg(d, fx[k - 1], fy[k - 1], fx[k], fy[k]);
    seg(d, (int16_t)(fx[k - 1] - ox[k - 1]), (int16_t)(fy[k - 1] - oy[k - 1]),
           (int16_t)(fx[k] - ox[k]),         (int16_t)(fy[k] - oy[k]));
    // centre dashes (4 points on / 4 off)
    if ((i & 7) < 4) seg(d, mx[k - 1], my[k - 1], mx[k], my[k]);
    // road arrows: tangent direction at that point -> rotate with the road
    if (i % kArrowEvery == 3 && mx[k] >= 8 && mx[k] <= (int16_t)(fb::SCREEN_W - 8)
                             && my[k] >= 6 && my[k] <= (int16_t)(kPlayH - 6)) {
      const uint8_t scrA = (uint8_t)(gTTan[i] - C);
      const bool strong = (gTCur[i] > 12 || gTCur[i] < -12);
      drawArrow(d, mx[k], my[k], scrA, strong);
    }
    // trees, alternating sides
    if (i % 14 == 7) {
      const bool left = ((i / 14) & 1) == 0;
      const int16_t bx = left ? ex[k] : fx[k], by = left ? ey[k] : fy[k];
      const int16_t txx = (int16_t)(mx[k] + (int16_t)(((bx - mx[k]) * 19) / 16));
      const int16_t tyy = (int16_t)(my[k] + (int16_t)(((by - my[k]) * 19) / 16));
      if (txx >= 2 && txx <= (int16_t)(fb::SCREEN_W - 3) && tyy >= 2 && tyy <= (int16_t)(kPlayH - 3))
        d.drawCircle((u8g2_uint_t)txx, (u8g2_uint_t)tyy, 2);
    }
    // start/finish checker line
    if (i == 0) {
      const int16_t tdx = (int16_t)(mx[k] - mx[k - 1]), tdy = (int16_t)(my[k] - my[k - 1]);
      for (int m = -14; m <= 14; m += 4) {
        int16_t bx = (int16_t)(mx[k] + (int16_t)(((ex[k] - mx[k]) * m) / 16));
        int16_t by = (int16_t)(my[k] + (int16_t)(((ey[k] - my[k]) * m) / 16));
        if ((((m + 14) / 4) & 1)) { bx = (int16_t)(bx + tdx / 4); by = (int16_t)(by + tdy / 4); }
        if (bx >= 1 && bx <= (int16_t)(fb::SCREEN_W - 3) && by >= 1 && by <= (int16_t)(kPlayH - 3))
          d.drawBox((u8g2_uint_t)(bx - 1), (u8g2_uint_t)(by - 1), 3, 3);
      }
    }
  }

  // skid marks (world-anchored, fade out)
  for (int sIdx = 0; sIdx < kTrailN; ++sIdx) {
    const uint16_t a = gTrAge[sIdx];
    if (a > 1100) continue;
    const int p = (sIdx == 0) ? kTrailN - 1 : sIdx - 1;
    if (gTrAge[p] <= a || gTrAge[p] > 1100 || gTrAge[p] - a > 200) continue;
    int16_t x0, y0, x1, y1;
    if (xform(gTrLX[p], gTrLY[p], cosC, sinC, x0, y0) && xform(gTrLX[sIdx], gTrLY[sIdx], cosC, sinC, x1, y1))
      seg(d, x0, y0, x1, y1);
    if (xform(gTrRX[p], gTrRY[p], cosC, sinC, x0, y0) && xform(gTrRX[sIdx], gTrRY[sIdx], cosC, sinC, x1, y1))
      seg(d, x0, y0, x1, y1);
  }

  // smoke / dust puffs
  for (int i2 = 0; i2 < kPuffN; ++i2) {
    if (!gPuff[i2].life) continue;
    int16_t sx, sy;
    if (!xform(gPuff[i2].x, gPuff[i2].y, cosC, sinC, sx, sy)) continue;
    if (sx < 1 || sx > (int16_t)(fb::SCREEN_W - 3) || sy < 1 || sy > (int16_t)(kPlayH - 3)) continue;
    if (gPuff[i2].life > 300) d.drawBox((u8g2_uint_t)sx, (u8g2_uint_t)sy, 2, 2);
    else                      d.drawPixel((u8g2_uint_t)sx, (u8g2_uint_t)sy);
  }
}

void renderCar(U8G2& d) {
  const uint8_t C    = (uint8_t)(int32_t)(gCamA - 64.f);
  const uint8_t scrA = (uint8_t)((uint8_t)(int32_t)gHead - C);
  drawCarShape(d, (int16_t)(kCarX0 + gRumble), (int16_t)(kCarY0 + gRumble), scrA);
}

void fmtTime(char* b, uint32_t ms) {
  snprintf(b, 14, "%lu:%02lu.%lu", (unsigned long)(ms / 60000UL),
           (unsigned long)((ms / 1000UL) % 60UL), (unsigned long)((ms / 100UL) % 10UL));
}

// One compact HUD strip at the very bottom: [ LAP x/3 | NNN KM/H | m:ss.t ].
// Row kHudY is a 1 px separator rule, then a 1 px gap, 6 px of text and a 1 px
// bottom pad (kHudH = 9). The strip is outside the playable area (rows
// 0..kPlayH-1), so it can never cover road.
void renderHud(U8G2& d) {
  char buf[16];
  d.setDrawColor(0);
  d.drawBox(0, (u8g2_uint_t)kHudY, fb::SCREEN_W, (u8g2_uint_t)kHudH);   // clean strip
  d.setDrawColor(1);
  d.drawHLine(0, (u8g2_uint_t)kHudY, fb::SCREEN_W);                     // separator rule
  d.setFontMode(1);
  d.setFont(u8g2_font_5x7_tr);
  const u8g2_uint_t by = (u8g2_uint_t)(fb::SCREEN_H - 1);               // baseline: 5x7 ink = rows 57..62
  int lap = gLap;
  if (lap > (int)kLaps) lap = (int)kLaps;
  if (lap < 1) lap = 1;
  snprintf(buf, sizeof(buf), "LAP %d/%d", lap, (int)kLaps);
  d.drawStr(2, by, buf);                                                // left
  snprintf(buf, sizeof(buf), "%3d KM/H", (int)(gSpeed * kKmhScale));    // fixed width -> no jitter
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(buf)) / 2), by, buf);   // centre
  fmtTime(buf, gRaceMs);
  d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), by, buf);     // right
}

void renderTitle(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(1);
  d.setFont(u8g2_font_6x13B_tr);
  const char* t = "TOKYO DRIFT";
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(t)) / 2), 13, t);
  d.drawHLine(20, 17, fb::SCREEN_W - 40);
  drawCarShape(d, kCarX0, 29, 64);                        // demo car pointing up
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(2, 41, "UP GAS  DOWN BRAKE");
  d.drawStr(2, 49, "L/R STEER  DN+L/R DRIFT");
  if (((gTitleMs / 500UL) & 1UL)) {
    const char* p = "PRESS SELECT";
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(p)) / 2), 61, p);
  } else if (gBestTotalMs) {
    char buf[16];
    fmtTime(buf, gBestTotalMs);
    d.drawStr(2, 61, "BEST");
    d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), 61, buf);
  }
}

void renderCountdown(U8G2& d) {
  const uint32_t ph = gCdUs / kCdStepUs;
  d.setDrawColor(1);
  d.setFontMode(1);
  if (ph < 3) {
    char b[2] = { (char)('3' - (char)ph), '\0' };
    d.setFont(u8g2_font_logisoso24_tn);
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(b)) / 2), 46, b);
  } else {
    const char* g = "GO!";
    d.setFont(u8g2_font_6x13B_tr);
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(g)) / 2), 40, g);
  }
}

void renderPause(U8G2& d) {
  d.setDrawColor(0);
  d.drawBox(24, 10, 80, 44);
  d.setDrawColor(1);
  d.drawFrame(24, 10, 80, 44);
  d.setFontMode(1);
  d.setFont(u8g2_font_6x13B_tr);
  const char* t = "PAUSED";
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(t)) / 2), 22, t);
  static const char* opt[3] = { "RESUME", "RESTART RACE", "QUIT TO MENU" };
  d.setFont(u8g2_font_5x7_tr);
  for (uint8_t i = 0; i < 3; ++i) {
    d.drawStr(34, (u8g2_uint_t)(33 + i * 8), opt[i]);
    if (i == gMenuIdx) d.drawStr(28, (u8g2_uint_t)(33 + i * 8), ">");
  }
}

void renderResult(U8G2& d) {
  char buf[16];
  d.setDrawColor(1);
  d.setFontMode(1);
  d.setFont(u8g2_font_6x13B_tr);
  const char* t = "RACE COMPLETE";
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(t)) / 2), 13, t);
  d.setFont(u8g2_font_5x7_tr);
  fmtTime(buf, gRaceMs);
  d.drawStr(2, 26, "TIME");      d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), 26, buf);
  fmtTime(buf, gBestLapMs);
  d.drawStr(2, 34, "BEST LAP");  d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), 34, buf);
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)gDriftAcc);
  d.drawStr(2, 42, "DRIFT PTS"); d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), 42, buf);
  if (gBestTotalMs == gRaceMs) {
    const char* nb = "NEW BEST!";
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(nb)) / 2), 50, nb);
  }
  static const char* opt[2] = { "RACE AGAIN", "QUIT TO MENU" };
  d.drawStr(24, 61, opt[0]); d.drawStr(78, 61, opt[1]);
  d.drawStr((u8g2_uint_t)(gMenuIdx ? 72 : 18), 61, ">");
}

void renderQuitBar(U8G2& d) {
  if (gSelHoldUs < 150000UL) return;
  d.setDrawColor(0);
  d.drawBox(34, 38, 60, 12);
  d.setDrawColor(1);
  d.drawFrame(34, 38, 60, 12);
  d.setFontMode(1);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(40, 46, "EXIT");
  d.drawFrame(62, 42, 26, 5);
  const uint32_t w = (gSelHoldUs * 24UL) / kQuitHoldUs;
  if (w) d.drawBox(63, 43, (u8g2_uint_t)((w > 24) ? 24 : w), 3);
}

void gameRender(U8G2& d) {
  switch (gState) {
    case ST_TITLE:
      renderTitle(d);
      break;
    case ST_COUNT:
      renderWorld(d);
      renderCar(d);
      renderHud(d);
      renderCountdown(d);
      break;
    case ST_RACE:
      renderWorld(d);
      renderCar(d);
      renderHud(d);
      break;
    case ST_PAUSE:
      renderWorld(d);
      renderCar(d);
      renderHud(d);
      renderPause(d);
      break;
    case ST_RESULT:
      renderResult(d);
      break;
  }
  renderQuitBar(d);
}

}  // namespace

FB_GAME(TokyoDrift, gameInit, gameUpdate, gameRender, nullptr);
