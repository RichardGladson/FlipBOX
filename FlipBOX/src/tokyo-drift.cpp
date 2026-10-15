// ============================================================================
// src/Game3.cpp  --  TOKYO DRIFT  (flipBOX)
//
// True top-down, momentum-based arcade drifting racer for the 128x64
// monochrome OLED. The player's car sits low in the playable area so most of
// the screen shows the road ahead, while the track scrolls AND rotates around
// it: the camera looks along the car's MOMENTUM (velocity) direction, while
// the car body shows the true heading, so drifts visibly tilt the car.
// Road cues are painted on the road surface itself and move/rotate with the
// world (same world->camera transform as the road geometry): predictive
// corner arrows lean toward the coming bend (left/right, stronger lean =
// sharper corner) and dotted lines mark drift-initiation zones before suitable
// corners. Both, and the HUD next-corner chevrons, are derived once per track
// from the curvature table (analyseCorners), nothing is hard-coded per corner.
// Bordered, striped kerbs run on the edge the bend turns toward: a long thin lead-in well before the
// corner, a build-up, a bulge at the apex, then a fade-out; thicker for sharper corners (visual only,
// drawn outside the road edge). Road width depends on the track (wide = easy,
// narrow = expert).
//
// Screen layout: rows 0..kPlayH-1 are the playable/road area; the bottom
// kHudH rows are one dedicated HUD strip (LAP | SPEED | TIME). World
// rendering is clipped to the playable area so it can never reach the HUD.
// A compact minimap sits in the bottom-left corner of the playable area.
//
// Controls
//   UP                  accelerate
//   DOWN                brake / reverse (nothing else)
//   LEFT / RIGHT        steer (smoothed, reduced sensitivity)
//   X + LEFT/RIGHT      handbrake drift (needs speed; steering sets the angle,
//                       countersteer / release to recover, big angles cost speed)
//   SELECT              menus: confirm | race: pause
//   LEFT (track menu)   back
//   hold SELECT ~1 s    quit to launcher (anywhere, with progress bar)
//
// INPUT NOTE: the original input contract only had UP/DOWN/LEFT/RIGHT/SELECT.
// The smallest possible change was made: one new mask, KEY_X = bit 5, in the
// contract block below (bits 5..7 of Input::held were unused). The game reads
// it exactly like the other keys. The host input driver must set that bit
// for the X button; if the host never reports it, drifting is unavailable
// (see the tuning flag kXIsHandbrake at the bottom of the constants).
//
// Flow: TITLE -> MODE (single / multi) -> TRACK (6 tracks) -> COUNTDOWN
//       -> RACE (3 laps) -> RESULTS, plus PAUSE (resume / restart / tracks / quit).
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
constexpr uint8_t KEY_X      = 1u << 5;   // ADDED: handbrake / drift button (see INPUT NOTE)

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
#include <math.h>    // sinf/atan2f/sqrtf: init-time table building + a few cheap runtime sqrts

namespace {

// ---------------------------------------------------------------------------
// Tuning constants (units in comments) -- tweak the feel here
// ---------------------------------------------------------------------------
constexpr uint8_t  kLaps          = 3;          // race length, laps
constexpr int      kTrackMax      = 400;        // max centreline points (arrays)
constexpr int      kTrackMin      = 96;         // min centreline points
constexpr float    kSpacing       = 12.f;       // px between centreline points (uniform)
constexpr float    kWallMargin    = 4.f;        // px: wall limit = road half-width - this (wheels touch the edge)
constexpr float    kMaxSpeed      = 175.f;      // px/s, forward top speed
constexpr float    kMaxReverse    = 45.f;       // px/s
constexpr float    kAccel         = 150.f;      // px/s^2
constexpr float    kBrake         = 230.f;      // px/s^2
constexpr float    kDrag          = 0.45f;      // 1/s, linear aero drag
constexpr float    kDriftDragMul  = 0.45f;      // drag multiplier while drifting (longer drifts)
constexpr float    kRoll          = 14.f;       // px/s^2, coasting resistance
constexpr float    kGrip          = 6.5f;       // 1/s, lateral grip (normal)
constexpr float    kDriftGrip     = 1.6f;       // 1/s, lateral grip while drifting
constexpr float    kDriftKeep     = 0.85f;      // 0..1, share of sideways momentum turned into forward speed
constexpr float    kSteer         = 100.f;      // brads/s steering rate (was 150) (256 brads = 360 deg)
constexpr float    kSteerRamp     = 6.0f;       // 1/s, steer input ramps towards the key (smoothing)
constexpr float    kSteerRelax    = 10.0f;      // 1/s, steer input returns to centre
constexpr float    kDriftSteerMul = 1.6f;       // steering boost while drifting
constexpr float    kDriftMinSpeed = 45.f;       // px/s forward speed needed to drift
constexpr float    kDriftSteerMin = 0.30f;      // steer (0..1) needed to START a drift
constexpr float    kDriftHoldMin  = 0.12f;      // steer needed to keep one going
constexpr float    kDriftKIn      = 9.0f;       // 1/s, drift intensity ramps in (~0.11 s)
constexpr float    kDriftKOut     = 3.0f;       // 1/s, ...and out (~0.33 s): grip returns progressively
constexpr float    kDriftKick     = 9.0f;       // brads, one-off yaw kick when X is pressed with steering
constexpr float    kKickWindow    = 0.30f;      // s after pressing X in which steering still triggers the kick
constexpr float    kSlipRef       = 40.f;       // brads, slip angle at which sideways grip has risen by kSlipGrip
constexpr float    kSlipGrip      = 1.5f;       // extra grip factor at kSlipRef (squared): makes the angle self-limiting
constexpr float    kSlipSweet     = 26.f;       // brads (~37 deg): beyond this the penalty grows steeply
constexpr float    kSlipMax       = 50.f;       // brads (~70 deg): from here sideways motion is fully penalised
constexpr float    kSlipScrub     = 1.0f;       // 1/s speed loss at kSlipMax (scaled by drift intensity)
constexpr float    kSlipBase      = 0.65f;      // 1/s speed loss at the sweet-spot angle: a drift is never free, it must be earned
constexpr float    kCounterBoost  = 1.15f;       // steering authority bonus when countersteering a slide
constexpr float    kDeepSteerCut  = 0.5f;       // steering authority loss when pushing deeper into a big angle
constexpr float    kWallLoss      = 0.5f;       // max one-off tangential speed loss on a hard wall hit
constexpr float    kWallDrag      = 1.8f;       // 1/s, scrape drag while touching the wall
constexpr float    kWallAlign     = 90.f;       // brads/s, nose turns along the wall while sliding
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
constexpr int      kVisBack       = 8;          // centreline points drawn behind car
constexpr int      kVisFwd        = 30;         // ... and ahead of the car
constexpr int      kVisN          = kVisBack + kVisFwd;
constexpr int      kCornerMax     = 48;         // max corners tracked per track
constexpr int      kCorOn         = 14;         // |gTCur| (brads) at which a bend starts
constexpr int      kCorOff        = 8;          // ... and ends (hysteresis)
constexpr int      kCorMinTurn    = 20;         // brads (~28 deg): smaller bends are not "significant corners"
constexpr int      kSevMed        = 34;         // apex |gTCur| >= this = medium (radius ~85 px)
constexpr int      kSevSharp      = 52;         // ... >= this = sharp (radius ~55 px)
constexpr int      kArrowLead     = 7;          // points before the corner entry of the nearest road arrow (+2 per severity level)
constexpr int      kArrowGap      = 3;          // points between the arrows of one corner
constexpr int      kZoneTurn      = 36;         // brads (~50 deg): minimum total turn for a drift zone
constexpr int      kZoneRunUp     = 5;          // points of road needed between the previous corner's exit and a zone corner
constexpr int      kZoneMaxVSafe  = 140;        // px/s: only corners that really slow a grip car deserve a zone
constexpr int      kZoneLines     = 3;          // max dotted lines per zone (every 2nd point, starting 1 point before entry)
constexpr uint8_t  kKerbPeak[3]   = { 2, 4, 6 };  // kerb thickness (px beyond the road edge line) at the apex: gentle / medium / sharp
constexpr uint8_t  kKerbBase[3]   = { 1, 2, 2 };  // thickness of the long lead-in section before the corner
constexpr float    kHudLookS      = 0.9f;       // s of travel the HUD corner indicator looks ahead
constexpr float    kHudLookMin    = 90.f;       // px
constexpr float    kHudLookMax    = 220.f;      // px
constexpr float    kCullR2        = 250.f * 250.f; // px^2, ignore points beyond
constexpr int16_t  kHudH          = 9;          // bottom HUD strip height, px (rule + gap + 6 px text + pad)
constexpr int16_t  kPlayH         = fb::SCREEN_H - kHudH;  // playable rows 0..kPlayH-1
constexpr int16_t  kHudY          = kPlayH;     // first HUD row (the separator rule)
constexpr int16_t  kCarX0         = fb::SCREEN_W / 2;  // car screen position
constexpr int16_t  kCarY0         = 46;         // low in the play area -> long view ahead
constexpr int      kTrailN        = 22;         // skid-trail ring size
constexpr int      kPuffN         = 18;         // smoke particle count
constexpr int      kTrackCount    = 6;          // selectable tracks
constexpr int      kAiN           = 3;          // AI opponents in multiplayer
constexpr int      kCarsMax       = 1 + kAiN;
constexpr float    kCarR2         = 11.f * 11.f;// px^2, car-car contact distance
constexpr float    kYawCap        = 1.9f;       // rad/s, AI planning yaw rate (below real capability)
constexpr int16_t  kMapW          = 34;         // minimap box (incl. frame), px
constexpr int16_t  kMapH          = 26;
constexpr int16_t  kMapY          = kPlayH - kMapH;   // top row of the minimap box
constexpr int16_t  kMapCx         = 16;         // player position inside the minimap
constexpr int16_t  kMapCy         = kPlayH - 8;
constexpr float    kMapScale      = 0.15f;      // minimap px per world px
// Set to false only if the host cannot report KEY_X: the handbrake then falls
// back to holding UP+DOWN together so the car stays drivable. (Default: X.)
constexpr bool     kXIsHandbrake  = true;

// Compile-time layout guards: the car (<= 7 px from its centre, +1 rumble)
// must fit inside the play area, and the drawn window must not wrap onto itself.
static_assert(kCarY0 + 8 <= kPlayH, "car must stay inside the playable area");
static_assert(kCarY0 - 8 >= 0,      "car must stay inside the playable area");
static_assert(kVisN < kTrackMin,    "visible window must be shorter than the loop");

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

inline float wrapB(float a) {          // -> -128..128
  while (a >  128.f) a -= 256.f;
  while (a < -128.f) a += 256.f;
  return a;
}
inline float fabsx(float v) { return v < 0.f ? -v : v; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------------
// Tracks: closed loops defined by radius harmonics r(theta) = r0 + sum a*sin(k*theta+ph)
// (always loop cleanly, never self-intersect: a polar curve with r > 0). Six
// parameter sets, tuned offline for Easy -> Expert: every one is bend-dominated
// (no straight longer than ~280 px), min bend radius shrinks from ~100 px to
// ~30 px, direction changes and hairpins increase, and separate road sections
// stay > 110 px apart. The loop is then re-sampled at uniform arc length.
// ---------------------------------------------------------------------------
struct Harm { float a; uint8_t k; float ph; };
struct TrackDef {
  const char* name;
  const char* level;
  uint8_t     half;     // road half-width, px: wide for easy tracks, narrow for hard ones
  float       r0;
  Harm        h[4];
};

const TrackDef kTracks[kTrackCount] = {
  { "SAKURA",  "EASY", 22,    420.f, {{21.f, 7, 5.99f}, {14.f, 6, 2.17f}, {19.f, 4, 3.52f}, {0.f, 0, 0.f}} },
  { "ODAIBA",  "MEDIUM", 20,  480.f, {{101.f, 2, 5.94f}, {15.f, 9, 4.03f}, {41.f, 7, 5.96f}, {0.f, 0, 0.f}} },
  { "SHIBUYA", "TRICKY", 18,  480.f, {{7.f, 11, 2.06f}, {62.f, 5, 0.19f}, {36.f, 10, 5.63f}, {0.f, 0, 0.f}} },
  { "HAKONE",  "HARD", 16,    400.f, {{17.f, 8, 4.67f}, {31.f, 11, 5.99f}, {37.f, 3, 3.34f}, {0.f, 0, 0.f}} },
  { "IROHA",   "EXTREME", 14, 480.f, {{42.f, 2, 2.82f}, {33.f, 9, 1.18f}, {35.f, 11, 1.24f}, {0.f, 0, 0.f}} },
  { "USUI",    "EXPERT", 13,  440.f, {{21.f, 2, 0.21f}, {10.f, 7, 2.77f}, {49.f, 11, 5.55f}, {0.f, 0, 0.f}} },
};

int     gN;                     // centreline points of the current track
int16_t gRoadHalf;              // road half-width of the current track, px (render AND collision)
float   gWallLim;               // car-centre limit from the centreline = gRoadHalf - kWallMargin
float   gSp;                    // mean point spacing, px
float   gTX[kTrackMax];         // centreline x
float   gTY[kTrackMax];         // centreline y
float   gSX[kTrackMax];         // unit direction of segment i -> i+1
float   gSY[kTrackMax];
uint8_t gTTan[kTrackMax];       // tangent angle (brads)
int8_t  gTCur[kTrackMax];       // bend here (signed brads over +-3 pts = 72 px; no int8 wrap even in hairpins)
uint8_t gVSafe[kTrackMax];      // AI planning speed through the bend here, px/s
float   gBX0, gBY0, gBX1, gBY1; // bounding box (menu preview)

// Corner table, derived once per track from gTCur / gVSafe (no per-corner hard-coding).
struct Corner {
  uint16_t entry, apex, exit;   // centreline indices
  int8_t   dir;                 // +1 = left, -1 = right
  uint8_t  sev;                 // 0 gentle, 1 medium, 2 sharp
  uint8_t  turn;                // total turn, brads
  bool     zone;                // a drift-initiation zone is painted before it
};
Corner  gCor[kCornerMax];
int     gCorN;
// per-point road marks: bits 0-1 arrow level (0 none, 1..3 = bend of the arrow), bit 2 arrow left,
// bit 3 drift-zone line
uint8_t gMark[kTrackMax];
int8_t  gNextCor[kTrackMax];    // index of the next corner whose entry lies ahead of this point
// per-point kerb thickness (px beyond the road edge line), one array per edge. A corner's kerb sits on the
// edge the bend turns toward: a long thin lead-in (visible well before the corner), then it builds up to a
// bulge at the apex and fades out at the exit. Two arrays so an S-bend can carry both edges at once.
// Visual only: the drivable width and the wall limit never look at these.
uint8_t gKerbL[kTrackMax];
uint8_t gKerbR[kTrackMax];


float trackRadius(const TrackDef& T, float t) {
  float r = T.r0;
  for (int i = 0; i < 4; ++i)
    if (T.h[i].a != 0.f) r += T.h[i].a * sinf((float)T.h[i].k * t + T.h[i].ph);
  return r;
}

// ---------------------------------------------------------------------------
// Corner analysis. Segments the existing bend table gTCur into corners (hysteresis,
// split at direction changes), grades them gentle / medium / sharp from the apex
// bend, then derives the road marks (predictive arrows + drift zones) and the
// "next corner" lookup used by the HUD. Runs once per track, init time only.
// ---------------------------------------------------------------------------
inline int absi(int v) { return v < 0 ? -v : v; }

void analyseCorners() {
  gCorN = 0;
  for (int i = 0; i < gN; ++i) { gMark[i] = 0; gNextCor[i] = -1; gKerbL[i] = 0; gKerbR[i] = 0; }
  int s0 = 0, bq = 1000;                         // start the scan on the quietest point (never mid-corner)
  for (int i = 0; i < gN; ++i) { const int q = absi(gTCur[i]); if (q < bq) { bq = q; s0 = i; } }

  int i = 0;
  while (i < gN && gCorN < kCornerMax) {
    const int c0 = gTCur[(s0 + i) % gN];
    if (absi(c0) < kCorOn) { ++i; continue; }
    const int sign = c0 > 0 ? 1 : -1;
    int peak = absi(c0), pk = i, j = i + 1;
    for (; j < gN; ++j) {
      const int cj = gTCur[(s0 + j) % gN];
      if (absi(cj) < kCorOff) break;
      if (((cj > 0) ? 1 : -1) != sign && absi(cj) >= kCorOn) break;
      if (absi(cj) > peak) { peak = absi(cj); pk = j; }
    }
    int turn = 0;                                // total signed turn through the corner
    for (int k = i; k < j; ++k) {
      const int a = (s0 + k) % gN, b = (s0 + k + 1) % gN;
      turn += (int)wrapB((float)gTTan[b] - (float)gTTan[a]);
    }
    if (absi(turn) >= kCorMinTurn) {
      Corner& c = gCor[gCorN++];
      c.entry = (uint16_t)((s0 + i) % gN);
      c.apex  = (uint16_t)((s0 + pk) % gN);
      c.exit  = (uint16_t)((s0 + j - 1) % gN);
      c.dir   = (int8_t)sign;
      c.sev   = (uint8_t)(peak >= kSevSharp ? 2 : (peak >= kSevMed ? 1 : 0));
      c.turn  = (uint8_t)(absi(turn) > 255 ? 255 : absi(turn));
      c.zone  = false;
    }
    i = j;
  }
  // corners were found in scan order; entries wrap once around the loop

  // ---- kerbs: lead-in (as long as the corner itself => total length 2x) -> build-up -> bulge at the apex -> fade ----
  for (int ci = 0; ci < gCorN; ++ci) {
    const Corner& c = gCor[ci];
    const int span = ((int)c.exit - (int)c.entry + gN) % gN;
    const int pa   = ((int)c.apex - (int)c.entry + gN) % gN;
    int room = gN / 2;                                   // never reach back past the previous corner's apex
    for (int o = 0; o < gCorN; ++o) {
      if (o == ci) continue;
      const int d = ((int)c.entry - (int)gCor[o].apex + gN) % gN;
      if (d > 0 && d < room) room = d;
    }
    int lead = span;
    if (lead > room - 1) lead = room - 1;
    if (lead < 0) lead = 0;
    const float peak = (float)kKerbPeak[c.sev], base = (float)kKerbBase[c.sev];
    for (int u = -lead; u <= span; ++u) {
      float e;
      if (u < 0) {                                       // lead-in: short fade-in, then a steady thin kerb
        float f = (float)(u + lead) * 0.5f; if (f > 1.f) f = 1.f;
        e = base * f;
      } else if (u <= pa) {                              // build-up toward the apex
        float r = (pa > 0) ? (float)u / (float)pa : 1.f;
        r = r * r * (3.f - 2.f * r);
        e = base + (peak - base) * r;
      } else {                                           // fade out toward the exit
        float r = (span > pa) ? (float)(span - u) / (float)(span - pa) : 0.f;
        r = r * r * (3.f - 2.f * r);
        e = peak * r;
      }
      const uint8_t lvl = (uint8_t)(e + 0.5f);
      if (lvl == 0) continue;
      const int idx = (((int)c.entry + u) % gN + gN) % gN;
      uint8_t& dst = (c.dir > 0) ? gKerbL[idx] : gKerbR[idx];
      if (lvl > dst) dst = lvl;
    }
  }

  // ---- road marks ----
  for (int ci = 0; ci < gCorN; ++ci) {
    Corner& c = gCor[ci];
    // the closest earlier corner: its exit bounds the zone, its apex bounds the arrows
    int po = -1, gap = 1 << 20;
    for (int o = 0; o < gCorN; ++o) {
      if (o == ci) continue;
      const int g = ((int)c.entry - (int)gCor[o].exit + gN) % gN;
      if (g < gap) { gap = g; po = o; }
    }
    const int apexDist = (po >= 0) ? (((int)c.entry - (int)gCor[po].apex + gN) % gN) : gN / 2;

    // predictive arrows: further out for sharper corners (they need more braking room); one arrow for
    // gentle bends, a pair for medium/sharp ones, a trio for hairpins. On dense tracks the chain is
    // pulled in so it never reaches back past the previous corner's apex (no cues on top of an apex).
    int n = (c.sev >= 1) ? 2 : 1;
    if (c.sev == 2 && absi((int)gTCur[c.apex]) >= 80) n = 3;
    int off0 = kArrowLead + 2 * c.sev;
    if (off0 > apexDist - 2) off0 = apexDist - 2;
    if (off0 >= 3) {
      for (int a = 0; a < n; ++a) {
        const int off = off0 + a * kArrowGap;
        if (off > apexDist - 2) break;
        const int p = ((int)c.entry - off + 4 * gN) % gN;
        bool busy = false;
        for (int d = -1; d <= 1; ++d) if (gMark[(p + d + gN) % gN]) busy = true;
        if (busy) continue;
        gMark[p] = (uint8_t)((c.sev + 1) | (c.dir > 0 ? 4 : 0));
      }
    }

    // drift-initiation zone: only real turns that slow a grip car, with road to get set up on.
    // Dotted lines every 2nd point starting 1 point before the entry, kept inside the gap.
    if (c.sev >= 1 && c.turn >= kZoneTurn && gap >= kZoneRunUp && gVSafe[c.apex] <= kZoneMaxVSafe) {
      int lines = 0;
      for (int z = 0; z < kZoneLines; ++z) if (1 + 2 * z <= gap - 2) ++lines;
      bool ok = (lines >= 2);
      for (int z = 0; ok && z < lines; ++z) {
        const int p = ((int)c.entry - 1 - 2 * z + 4 * gN) % gN;
        if (gMark[p] & 3) ok = false;
      }
      if (ok) {
        c.zone = true;
        for (int z = 0; z < lines; ++z) gMark[((int)c.entry - 1 - 2 * z + 4 * gN) % gN] |= 8;
      }
    }
  }

  // ---- "next corner" lookup: nearest entry strictly ahead of each point ----
  for (int p = 0; p < gN; ++p) {
    int bestD = gN + 1, bi = -1;
    for (int ci = 0; ci < gCorN; ++ci) {
      const int d = ((int)gCor[ci].entry - p + gN) % gN;
      if (d > 0 && d < bestD) { bestD = d; bi = ci; }
    }
    gNextCor[p] = (int8_t)bi;
  }
}

void buildTrack(int idx) {
  const TrackDef& T = kTracks[idx];
  gRoadHalf = (int16_t)T.half;
  gWallLim  = (float)T.half - kWallMargin;
  const int   S  = 3000;
  const float dT = 6.2831853f / (float)S;
  // pass 1: loop length
  float L = 0.f, px = trackRadius(T, 0.f), py = 0.f;
  for (int s = 1; s <= S; ++s) {
    const float t = (float)s * dT, r = trackRadius(T, t);
    const float x = r * cosf(t), y = r * sinf(t);
    const float dx = x - px, dy = y - py;
    L += sqrtf(dx * dx + dy * dy);
    px = x; py = y;
  }
  int n = (int)(L / kSpacing + 0.5f);
  if (n < kTrackMin) n = kTrackMin;
  if (n > kTrackMax) n = kTrackMax;
  const float sp = (L / (float)n) * 0.9999f;
  // pass 2: emit a point every 'sp' px of arc length
  int cnt = 0;
  px = trackRadius(T, 0.f); py = 0.f;
  gTX[cnt] = px; gTY[cnt] = py; ++cnt;
  float acc = 0.f;
  for (int s = 1; s <= S && cnt < n; ++s) {
    const float t = (float)s * dT, r = trackRadius(T, t);
    const float x = r * cosf(t), y = r * sinf(t);
    float dx = x - px, dy = y - py;
    float d = sqrtf(dx * dx + dy * dy);
    while (cnt < n && acc + d >= sp && d > 1e-4f) {
      const float u = (sp - acc) / d;
      px += dx * u; py += dy * u;
      gTX[cnt] = px; gTY[cnt] = py; ++cnt;
      dx = x - px; dy = y - py; d = sqrtf(dx * dx + dy * dy);
      acc = 0.f;
    }
    acc += d;
    px = x; py = y;
  }
  gN  = cnt;
  gSp = L / (float)gN;

  gBX0 = gBX1 = gTX[0]; gBY0 = gBY1 = gTY[0];
  for (int i = 0; i < gN; ++i) {
    const int nx = (i + 1) % gN;
    const float dx = gTX[nx] - gTX[i], dy = gTY[nx] - gTY[i];
    float l = sqrtf(dx * dx + dy * dy); if (l < 1e-4f) l = 1e-4f;
    gSX[i] = dx / l; gSY[i] = dy / l;
    if (gTX[i] < gBX0) gBX0 = gTX[i];
    if (gTX[i] > gBX1) gBX1 = gTX[i];
    if (gTY[i] < gBY0) gBY0 = gTY[i];
    if (gTY[i] > gBY1) gBY1 = gTY[i];
  }
  for (int i = 0; i < gN; ++i) {
    const int p = (i + gN - 1) % gN;
    const int n2 = (i + 1) % gN;
    // atan2f at init time only (allowed): tangent angle in brads
    float a = atan2f(gTY[n2] - gTY[p], gTX[n2] - gTX[p]) * (256.f / 6.2831853f);
    if (a < 0.f) a += 256.f;
    gTTan[i] = (uint8_t)(int32_t)a;
  }
  for (int i = 0; i < gN; ++i) {
    const int b = (i + gN - 3) % gN;
    const int a = (i + 3) % gN;
    gTCur[i] = (int8_t)(gTTan[a] - gTTan[b]);   // signed bend amount over 6 segments
  }
  // AI planning speed: v = yaw-rate / curvature, curvature from a tight +-2 point
  // bend (48 px) so hairpins are not averaged away; min-filtered with neighbours
  for (int i = 0; i < gN; ++i) {
    float worst = 175.f;
    for (int d2 = -1; d2 <= 1; ++d2) {
      const int c  = (i + d2 + gN) % gN;
      const int b  = (c + gN - 2) % gN;
      const int a  = (c + 2) % gN;
      const float ang = fabsx(wrapB((float)gTTan[a] - (float)gTTan[b])) * (6.2831853f / 256.f);  // rad over 4*sp px
      float v = 175.f;
      if (ang > 0.02f) v = kYawCap * (4.f * gSp) / ang;
      if (v < worst) worst = v;
    }
    gVSafe[i] = (uint8_t)clampf(worst, 45.f, 175.f);
  }
  analyseCorners();
}

// ---------------------------------------------------------------------------
// Car state (player + AI share the same physics)
// ---------------------------------------------------------------------------
struct Car {
  float    x, y, vx, vy;      // world position / velocity
  float    head;              // heading, brads 0..256
  float    speed;             // |v| estimate, px/s
  float    vF, vL;            // heading-frame velocity after the last step
  float    steer;             // smoothed steer input -1..1
  float    lat;               // signed offset from the centreline (+ = left)
  bool     drift;
  int8_t   driftDir;          // +1 = drifting left(CCW), -1 = right
  bool     wall;              // touched the boundary this frame
  float    driftK;            // 0..1 drift intensity (blends grip / steering / drag; eases in and out)
  float    slip;              // signed slip angle, brads (atan2 of sideways / forward velocity)
  float    armT;              // s left in which the X-press kick may fire
  bool     handPrev, kicked;  // X edge detection / kick already used for this press
  float    wallHit;           // impact speed of this frame's wall touch
  int16_t  seg, prevSeg, sj;  // nearest point / previous / segment index of lat
  int16_t  lap;               // 0 = behind the line, 1..kLaps, kLaps+1 = finished
  bool     done;
  uint32_t finishMs;
  // ---- AI only ----
  uint8_t  style;             // sprite 0..2
  float    pace;              // 0..1 share of top speed
  float    driftLike;         // 0 = grip driver, 1 = loves drifting
  float    laneBias, lane, noise, noiseT;
  uint8_t  recMode;           // 0 = choose, 1 = reverse off the wall, 2 = turn round forward
  uint16_t recSw;             // ms since the last forward/reverse swap
  uint16_t goMs, stuckMs, wrongMs, recMs, driftMs, driftCool, mistakeMs, noiseMs, fxMs, wallMs;
};

struct Ctl { bool up, dn, hand; float steer; };

enum State : uint8_t { ST_TITLE, ST_MODE, ST_TRACK, ST_COUNT, ST_RACE, ST_PAUSE, ST_RESULT };

State    gState;
Car      gCars[kCarsMax];   // [0] = player
#define  gP (gCars[0])
bool     gMulti;            // multiplayer (AI opponents)
uint8_t  gTrackSel;         // selected track
int      gCarsN;            // cars in this race (1 or kCarsMax)
float    gCamA;             // camera "up" direction, brads 0..256
uint32_t gRaceMs, gLapMs;
uint32_t gBestLapMs;        // reset per race entry
int      gLapHigh;          // highest lap the player has reached
uint8_t  gPlace;            // final place (multiplayer)
float    gDriftAcc;         // drift points accumulator
uint32_t gCdUs;             // countdown timer
uint32_t gSelHoldUs;        // SELECT hold timer (quit)
uint32_t gTitleMs;          // title blink timer
uint8_t  gMenuIdx;          // pause / results / mode cursor
uint32_t gSpawnUs, gTrailUs;// effect timers
int8_t   gRumble;           // wall-scrape jitter, px
uint32_t gBestTotalMs[kTrackCount];  // SESSION-PERSISTENT best race time per track (not reset)

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
void locateCar(Car& c) {
  float best = 1e30f;
  int   bi   = c.seg;
  for (int k = -6; k <= 10; ++k) {
    int i = (c.seg + k) % gN;
    if (i < 0) i += gN;
    const float dx = gTX[i] - c.x, dy = gTY[i] - c.y;
    const float d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; bi = i; }
  }
  c.prevSeg = c.seg;
  c.seg     = (int16_t)bi;
  // exact lateral offset from the segment the car is actually beside
  int j = bi;
  const float tb = (c.x - gTX[bi]) * gSX[bi] + (c.y - gTY[bi]) * gSY[bi];
  if (tb < 0.f) j = (bi + gN - 1) % gN;
  c.sj  = (int16_t)j;
  c.lat = (c.x - gTX[j]) * (-gSY[j]) + (c.y - gTY[j]) * gSX[j];
}

void placeCar(Car& c, int seg, float lat) {
  seg = ((seg % gN) + gN) % gN;
  c.x = gTX[seg] - gSY[seg] * lat;
  c.y = gTY[seg] + gSX[seg] * lat;
  c.vx = c.vy = 0.f;
  c.head = (float)gTTan[seg];
  c.seg = c.prevSeg = c.sj = (int16_t)seg;
  locateCar(c);
  c.prevSeg = c.seg;
  c.speed = c.vF = c.vL = 0.f;
  c.steer = 0.f; c.drift = false; c.driftDir = 0; c.driftK = 0.f; c.slip = 0.f; c.armT = 0.f; c.handPrev = false; c.kicked = false; c.wall = false; c.wallHit = 0.f;
  c.lap = 0; c.done = false; c.finishMs = 0;
}

void spawnPuff(float bx, float by, uint8_t hb) {
  for (int i = 0; i < kPuffN; ++i) {
    if (gPuff[i].life == 0) {
      const int o = (int)(rnd() % 7) - 3;          // small lateral scatter
      const float hx = cosB(hb) * (1.f / 32768.f), hy = sinB(hb) * (1.f / 32768.f);
      gPuff[i].x = bx + hy * (float)o;             // right vector = (hy,-hx)
      gPuff[i].y = by - hx * (float)o;
      gPuff[i].life = 500;
      return;
    }
  }
}

// Place everything for a fresh race. Cars start behind the line (lap 0); the
// first crossing starts lap 1, so every car covers exactly the same distance.
void startRace() {
  gCarsN = gMulti ? kCarsMax : 1;
  // grid: slot s sits 2 points (24 px) behind slot s-1, alternating sides
  static const int8_t kPlayerSlot = 2;
  int slot = 0;
  for (int i = 0; i < gCarsN; ++i) {
    int s;
    if (!gMulti) s = 0;
    else if (i == 0) s = kPlayerSlot;
    else { s = slot; if (s == kPlayerSlot) { ++slot; s = slot; } ++slot; }
    placeCar(gCars[i], gN - 2 - 2 * s, (s & 1) ? gWallLim * 0.5f : -gWallLim * 0.5f);
  }
  for (int i = 1; i < gCarsN; ++i) {
    Car& c = gCars[i];
    const int a = i - 1;
    c.style     = (uint8_t)a;
    c.pace      = 0.76f + 0.022f * (float)gTrackSel + (a == 0 ? 0.03f : (a == 1 ? 0.0f : -0.02f));
    c.driftLike = (a == 0) ? 1.f : (a == 1 ? 0.6f : 0.f);
    c.laneBias  = gWallLim * ((a == 0) ? -0.25f : (a == 1 ? 0.25f : 0.f));
    c.lane = c.laneBias; c.noise = c.noiseT = 0.f;
    c.goMs = (uint16_t)(120 + (rnd() % 350));
    c.recMode = 0; c.recSw = 0;
    c.stuckMs = c.wrongMs = c.recMs = c.driftMs = c.driftCool = c.mistakeMs = c.noiseMs = c.fxMs = c.wallMs = 0;
  }
  gCamA = gP.head;
  gLapHigh = 0;
  gRaceMs = 0; gLapMs = 0; gBestLapMs = 0;
  gPlace = 1;
  gDriftAcc = 0.f;
  gRumble = 0;
  gSpawnUs = 0; gTrailUs = 0;
  for (int i = 0; i < kPuffN; ++i) gPuff[i].life = 0;
  for (int i = 0; i < kTrailN; ++i) gTrAge[i] = 60000;
  gTrHead = 0;
}

// Race position of car 'me' (1 = leading). Finished cars rank by finish time.
uint8_t calcPlace(int me) {
  const Car& m = gCars[me];
  const int  pm = m.lap * gN + m.seg;
  uint8_t p = 1;
  for (int i = 0; i < gCarsN; ++i) {
    if (i == me) continue;
    const Car& o = gCars[i];
    bool ahead;
    if (o.done || m.done) ahead = o.done && (!m.done || o.finishMs < m.finishMs);
    else                  ahead = (o.lap * gN + o.seg) > pm;
    if (ahead) ++p;
  }
  return p;
}

// ---------------------------------------------------------------------------
// Physics -- all dt-driven (frame-rate independent). Shared by player and AI.
// ---------------------------------------------------------------------------
void driveCar(Car& c, const Ctl& k, float dt) {
  const uint8_t hb = (uint8_t)(int32_t)c.head;
  const float hx = cosB(hb) * (1.f / 32768.f);
  const float hy = sinB(hb) * (1.f / 32768.f);

  // decompose velocity into heading frame: forward / lateral
  float vF = c.vx * hx + c.vy * hy;
  float vL = -c.vx * hy + c.vy * hx;

  // ---- smoothed steering input ----
  {
    const float tg = k.steer;
    const bool  relax = (tg == 0.f) || (fabsx(tg) < fabsx(c.steer));
    const float rate = (relax ? kSteerRelax : kSteerRamp) * dt;
    const float df = tg - c.steer;
    if (df > rate) c.steer += rate; else if (df < -rate) c.steer -= rate; else c.steer = tg;
  }

  // ---- slip angle: how far the velocity points away from the nose (brads, signed) ----
  // vL < 0 = sliding to the right of the nose (a LEFT drift), vL > 0 = sliding left.
  const float s    = (vF > 15.f) ? atan2B(vL, vF) : 0.f;
  const float beta = fabsx(s);
  c.slip = s;
  const float excess = clampf((beta - kSlipSweet) / (kSlipMax - kSlipSweet), 0.f, 1.5f);  // 0 = clean, 1 = full penalty

  // ---- drift state: X (handbrake) + steering while carrying speed ----
  const float aS = fabsx(c.steer);
  const bool holding = k.hand && vF > kDriftMinSpeed * 0.8f &&
                       aS > (c.driftK > 0.3f ? kDriftHoldMin : kDriftSteerMin);
  c.drift = holding;
  if (holding) c.driftDir = (int8_t)(c.steer > 0.f ? 1 : -1);
  const bool counter = (c.steer * s > 0.f);                // steering toward the slide's own side: reduces the angle

  // initiation kick: pressing X while steering (within a short window) swings the nose into the
  // turn. Scaled by speed and steering, zero below the minimum speed, once per press, never automatic.
  if (k.hand && !c.handPrev) { c.armT = kKickWindow; c.kicked = false; }
  c.handPrev = k.hand;
  if (!k.hand) { c.armT = 0.f; c.kicked = false; }
  else if (c.armT > 0.f) c.armT -= dt;
  if (holding && !c.kicked && c.armT > 0.f && aS >= kDriftSteerMin && c.driftK < 0.4f && beta < 12.f) {
    const float sfk = clampf((vF - 35.f) * (1.f / 70.f), 0.3f, 1.f);
    c.head += (c.steer > 0.f ? 1.f : -1.f) * kDriftKick * sfk * clampf(aS, 0.3f, 1.f);
    c.kicked = true;
  }

  // drift intensity eases in quickly and out slowly (countersteering speeds the recovery up)
  if (holding) {
    c.driftK += kDriftKIn * dt;  if (c.driftK > 1.f) c.driftK = 1.f;
  } else {
    float r = kDriftKOut * (counter ? 1.8f : 1.f);
    c.driftK -= r * dt;          if (c.driftK < 0.f) c.driftK = 0.f;
  }
  const float K = c.driftK;

  // ---- lateral grip: drops with drift intensity, but RISES with slip angle so a slide self-limits ----
  const float ratio = beta * (1.f / kSlipRef);
  const float grip = (kGrip + (kDriftGrip - kGrip) * K) * (1.f + K * kSlipGrip * ratio * ratio);
  float g = grip * dt; if (g > 1.f) g = 1.f;
  const float vL0 = vL;
  vL -= vL * g;
  // sideways momentum is converted into forward speed, fading out as the angle becomes excessive
  if (vF > 20.f) vF += kDriftKeep * K * (1.f - clampf(excess, 0.f, 1.f)) * (vL0 * vL0 - vL * vL) / (2.f * vF);
  // excessive sideways sliding scrubs speed progressively
  if (K > 0.f && vF > 0.f) {
    float sc = (kSlipBase * clampf(beta * (1.f / kSlipSweet), 0.f, 1.f) + kSlipScrub * excess) * K * dt;
    if (sc > 0.5f) sc = 0.5f;
    vF -= vF * sc; vL -= vL * sc;
  }

  // ---- longitudinal ----
  if (k.up) vF += kAccel * dt;
  if (k.dn) vF -= kBrake * (1.f - 0.8f * K) * dt;
  if (k.hand && !holding && vF > 0.f) vF -= kBrake * 0.35f * dt;   // plain handbrake scrubs speed
  vF -= vF * kDrag * (1.f - (1.f - kDriftDragMul) * K) * dt;
  if (!k.up && !k.dn) {
    const float r = kRoll * dt;
    if (vF > r) vF -= r; else if (vF < -r) vF += r; else vF = 0.f;
  }
  if (vF >  kMaxSpeed)   vF =  kMaxSpeed;
  if (vF < -kMaxReverse) vF = -kMaxReverse;
  {   // the TRUE speed (forward + sideways) can never exceed top speed: a slide is not a speed boost
    const float m2 = vF * vF + vL * vL;
    if (m2 > kMaxSpeed * kMaxSpeed) { const float f = kMaxSpeed / sqrtf(m2); vF *= f; vL *= f; }
  }

  // ---- steering (sign of vF flips it naturally in reverse) ----
  float sf = vF / 55.f;
  if (sf > 1.f) sf = 1.f; else if (sf < -1.f) sf = -1.f;
  float topFade = 1.f - 0.30f * (vF / kMaxSpeed);
  if (topFade < 0.7f) topFade = 0.7f;
  float sm = 1.f + (kDriftSteerMul - 1.f) * K;
  if (K > 0.f && beta > 0.f) {
    if (counter) sm *= 1.f + (kCounterBoost - 1.f) * K;                               // countersteer bites harder
    else         sm *= 1.f - kDeepSteerCut * clampf(excess, 0.f, 1.f);                // pushing deeper loses authority
  }
  c.head += c.steer * kSteer * sf * topFade * sm * dt;
  while (c.head >= 256.f) c.head -= 256.f;
  while (c.head <   0.f)  c.head += 256.f;

  // ---- reassemble velocity, integrate position ----
  c.vx = hx * vF - hy * vL;
  c.vy = hy * vF + hx * vL;
  c.x += c.vx * dt;
  c.y += c.vy * dt;

  // ---- track boundary: never leave the road; lose speed and slide along it ----
  locateCar(c);
  c.wall = false; c.wallHit = 0.f;
  const float over = fabsx(c.lat) - gWallLim;
  if (over > 0.f) {
    const float sgn = (c.lat > 0.f) ? 1.f : -1.f;
    const float nx = -gSY[c.sj] * sgn, ny = gSX[c.sj] * sgn;     // outward normal
    c.x -= nx * over; c.y -= ny * over;                          // back onto the road
    c.lat = sgn * gWallLim;
    const float vn = c.vx * nx + c.vy * ny;                      // outward speed
    c.wall = true;
    if (vn > 0.f) {
      c.vx -= nx * vn * 1.1f; c.vy -= ny * vn * 1.1f;            // stop pushing out (tiny bounce)
      c.wallHit = vn;
      if (vn > 6.f) {                                            // impact: substantial speed loss
        float loss = vn * (1.f / 110.f);
        if (loss < 0.15f) loss = 0.15f; else if (loss > kWallLoss) loss = kWallLoss;
        c.vx *= 1.f - loss; c.vy *= 1.f - loss;
      }
    }
    float sd = kWallDrag * dt; if (sd > 0.5f) sd = 0.5f;         // scrape drag
    c.vx -= c.vx * sd; c.vy -= c.vy * sd;
    // nose swings along the wall so the car slides instead of sticking
    const float sp2 = c.vx * c.vx + c.vy * c.vy;
    if (sp2 > 25.f * 25.f && (c.vx * hx + c.vy * hy) > 0.f) {
      const float e = wrapB(atan2B(c.vy, c.vx) - c.head);
      const float lim = kWallAlign * dt;
      c.head += clampf(e, -lim, lim);
      while (c.head >= 256.f) c.head -= 256.f;
      while (c.head <   0.f)  c.head += 256.f;
    }
  }

  // refresh heading-frame velocity for the caller (after wall response)
  {
    const uint8_t h2 = (uint8_t)(int32_t)c.head;
    const float ax = cosB(h2) * (1.f / 32768.f), ay = sinB(h2) * (1.f / 32768.f);
    c.vF = c.vx * ax + c.vy * ay;
    c.vL = -c.vx * ay + c.vy * ax;
    c.speed = sqrtf(c.vF * c.vF + c.vL * c.vL);                 // true speed
  }

  // ---- lap counting: crossing start/finish forward (+1) or backward (-1) ----
  if (c.prevSeg > (gN * 3) / 4 && c.seg < gN / 4)      ++c.lap;
  else if (c.prevSeg < gN / 4 && c.seg > (gN * 3) / 4) --c.lap;
}

void stepPlayer(const fb::Input& in, float dt, uint32_t dtUs) {
  const uint32_t dtMs = dtUs / 1000;
  Ctl k;
  k.up   = (in.held & fb::KEY_UP)   != 0;
  k.dn   = (in.held & fb::KEY_DOWN) != 0;
  k.hand = kXIsHandbrake ? ((in.held & fb::KEY_X) != 0)
                         : ((in.held & (fb::KEY_UP | fb::KEY_DOWN)) == (fb::KEY_UP | fb::KEY_DOWN));
  if (!kXIsHandbrake && k.hand) { k.up = false; k.dn = false; }
  // LEFT = +1 (counter-clockwise = left when facing +heading)
  k.steer = (float)(((in.held & fb::KEY_LEFT) ? 1 : 0) - ((in.held & fb::KEY_RIGHT) ? 1 : 0));

  driveCar(gP, k, dt);

  const uint8_t hb = (uint8_t)(int32_t)gP.head;
  const float hx = cosB(hb) * (1.f / 32768.f);
  const float hy = sinB(hb) * (1.f / 32768.f);
  const float vF = gP.vF, vL = gP.vL;

  // ---- camera: look along the MOMENTUM (velocity) direction, not the bonnet ----
  // target = heading + blend * confidence * (velocity angle - heading), where the
  // offset is capped, and confidence fades to 0 at low speed and when reversing
  // (the velocity direction is meaningless / backwards there). The camera then
  // eases toward the target with a capped rotation rate: no snapping, no flips.
  float target = gP.head;
  const float sp2 = gP.vx * gP.vx + gP.vy * gP.vy;
  if (vF > 0.f && sp2 > kCamSpd0 * kCamSpd0) {
    float wS = (sp2 - kCamSpd0 * kCamSpd0) / (kCamSpd1 * kCamSpd1 - kCamSpd0 * kCamSpd0);
    if (wS > 1.f) wS = 1.f;
    float wF = vF * (1.f / kCamFwdFull);
    if (wF > 1.f) wF = 1.f;
    float off = wrapB(atan2B(gP.vy, gP.vx) - gP.head);   // signed look offset: + = momentum is left of the bonnet
    if (off >  kCamMaxSlip) off =  kCamMaxSlip;
    else if (off < -kCamMaxSlip) off = -kCamMaxSlip;
    target = gP.head + kCamVelBlend * wS * wF * off;
  }
  float cd = wrapB(target - gCamA);
  float ck = kCamRate * dt; if (ck > 1.f) ck = 1.f;
  float cs = cd * ck;
  const float cLim = kCamMaxRate * dt;
  if (cs > cLim) cs = cLim; else if (cs < -cLim) cs = -cLim;
  gCamA += cs;
  while (gCamA >= 256.f) gCamA -= 256.f;
  while (gCamA <   0.f)  gCamA += 256.f;

  // ---- lap bookkeeping (player) ----
  if (gP.lap > gLapHigh) {
    gLapHigh = gP.lap;
    if (gP.lap >= 2) {                           // a full lap was completed
      if (gBestLapMs == 0 || gLapMs < gBestLapMs) gBestLapMs = gLapMs;
    }
    gLapMs = 0;
  }
  if (gP.lap > (int)kLaps && !gP.done) { gP.done = true; gP.finishMs = gRaceMs; }

  // ---- drift scoring + effects ----
  if (gP.drift) {
    gDriftAcc += (vL < 0 ? -vL : vL) * dt * 6.f;
    gSpawnUs += dtUs;
    if (gSpawnUs >= 45000UL) {
      gSpawnUs = 0;
      spawnPuff(gP.x - hx * 5.f, gP.y - hy * 5.f, hb);   // rear axle smoke
    }
    gTrailUs += dtUs;
    if (gTrailUs >= 35000UL) {
      gTrailUs = 0;
      const uint8_t w = gTrHead;
      gTrLX[w] = gP.x - 3.f * hy - 4.f * hx;  gTrLY[w] = gP.y + 3.f * hx - 4.f * hy;
      gTrRX[w] = gP.x + 3.f * hy - 4.f * hx;  gTrRY[w] = gP.y - 3.f * hx - 4.f * hy;
      gTrAge[w] = 0;
      gTrHead = (uint8_t)((w + 1) % kTrailN);
    }
  } else {
    gSpawnUs = 0; gTrailUs = 0;
  }

  // ---- wall-scrape rumble + dust ----
  if (gP.wall && gP.speed > 40.f) {
    gRumble = (int8_t)((rnd() & 1) ? 1 : -1);
    gSpawnUs += dtUs;
    if (gSpawnUs >= 60000UL) { gSpawnUs = 0; spawnPuff(gP.x - hx * 4.f, gP.y - hy * 4.f, hb); }
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
// AI driver. Light: one lookahead aim point, a 6-sample braking envelope from
// the precomputed gVSafe table, and a loop over <= 3 other cars. It drives the
// SAME physics as the player (smoothed steering, same grip/drift model).
// ---------------------------------------------------------------------------
inline void dec16(uint16_t& t, uint32_t ms) { t = (t > ms) ? (uint16_t)(t - ms) : (uint16_t)0; }

void aiThink(Car& c, Ctl& k, int me, uint32_t dtMs, float dt) {
  k.up = k.dn = k.hand = false; k.steer = 0.f;
  if (c.goMs) { dec16(c.goMs, dtMs); return; }
  dec16(c.driftCool, dtMs); dec16(c.mistakeMs, dtMs); dec16(c.noiseMs, dtMs); dec16(c.recMs, dtMs);

  const float v  = c.vF;
  const float av = fabsx(v);
  const uint8_t hb = (uint8_t)(int32_t)c.head;
  const float hx = cosB(hb) * (1.f / 32768.f), hy = sinB(hb) * (1.f / 32768.f);

  // ---- stuck / wrong-way detection ----
  if (av < 14.f) { if (c.stuckMs < 5000) c.stuckMs = (uint16_t)(c.stuckMs + dtMs); } else c.stuckMs = 0;
  const float eT = wrapB((float)gTTan[(c.seg + 2) % gN] - c.head);
  if (fabsx(eT) > 85.f) { if (c.wrongMs < 5000) c.wrongMs = (uint16_t)(c.wrongMs + dtMs); } else c.wrongMs = 0;
  if (c.wall && av < 25.f) { if (c.wallMs < 5000) c.wallMs = (uint16_t)(c.wallMs + dtMs); } else c.wallMs = 0;
  if (c.recMs == 0 && (c.stuckMs > 600 || c.wrongMs > 500 || c.wallMs > 450)) {
    c.recMs = 3500; c.recMode = 0; c.recSw = 0; c.stuckMs = c.wrongMs = c.wallMs = 0;
    c.drift = false;
  }

  // ---- recovery: reverse off the wall and/or turn round, then rejoin ----
  // Steering is chosen from the ACTUAL direction of motion (it flips in reverse).
  if (c.recMs) {
    const float e  = wrapB((float)gTTan[(c.seg + 3) % gN] - c.head);
    const float ae = fabsx(e);
    if (c.recMode == 0) c.recMode = (ae > 40.f && (fabsx(c.lat) > gWallLim * 0.6f || av < 6.f)) ? 1 : 2;
    if (c.recSw < 5000) c.recSw = (uint16_t)(c.recSw + dtMs);
    // K-turn: swap direction whenever a wall stops us and the nose is still wrong
    if (c.recSw > 350 && c.wall && ae > 30.f) { c.recMode = (c.recMode == 1) ? 2 : 1; c.recSw = 0; }
    if (c.recMode == 1 && ae < 30.f) c.recMode = 2;
    if (c.recMode == 2 && ae < 30.f && v > 12.f) { c.recMs = 0; c.recMode = 0; }
    else {
      const float dir = (v < -2.f) ? -1.f : 1.f;
      k.steer = ((e > 0.f) ? 1.f : -1.f) * dir;
      if (c.recMode == 1) k.dn = true;
      else                k.up = (v < 55.f);
      return;
    }
  }

  // ---- lookahead aim ----
  int la = (int)((20.f + 0.22f * av) / gSp);
  if (la < 2) la = 2; else if (la > 10) la = 10;
  const int ti = (c.seg + la) % gN;
  const float curA = (float)gTCur[ti];                           // + = left bend ahead

  // ---- other cars: follow / overtake ----
  const float laneMax = gWallLim * 0.7f;
  float laneT = c.laneBias + clampf(curA * 0.2f, -gWallLim * 0.5f, gWallLim * 0.5f);     // slightly cut the inside of bends
  float capV  = 1e9f;
  const bool bendy = fabsx(curA) > 20.f;
  const float look = 46.f + 0.25f * av;
  for (int j = 0; j < gCarsN; ++j) {
    if (j == me) continue;
    const Car& o = gCars[j];
    const float dx = o.x - c.x, dy = o.y - c.y;
    const float f = dx * hx + dy * hy;
    if (f < -12.f || f > look) continue;
    const float l = -dx * hy + dy * hx;
    if (fabsx(l) > 14.f) continue;
    if (!bendy || f < 24.f) {
      float s = (o.lat > c.lat) ? -1.f : 1.f;                    // pass on the open side
      float tgt = clampf(o.lat + s * gWallLim, -laneMax, laneMax);
      if (fabsx(tgt - o.lat) < gWallLim * 0.65f) { s = -s; tgt = clampf(o.lat + s * gWallLim, -laneMax, laneMax); }
      laneT = tgt;
    }
    if (f > 0.f && f < 26.f && fabsx(l) < 8.f) {                 // right behind: do not ram
      const float cap = o.vF + 8.f;
      if (cap < capV) capV = cap;
    }
  }
  {
    const float step = 18.f * dt;
    c.lane += clampf(laneT - c.lane, -step, step);
  }

  // ---- speed plan: brake early enough for the slowest bend ahead ----
  const bool canDrift = c.driftLike > 0.f && c.driftCool == 0;
  const float boost = (c.driftLike > 0.f) ? (1.15f + 0.15f * c.driftLike) : 1.f;
  static const uint8_t kProbe[6] = { 2, 4, 7, 10, 14, 19 };
  float cap2 = 1e9f;
  for (int p = 0; p < 6; ++p) {
    const int ix = (c.seg + kProbe[p]) % gN;
    const float lim = (float)gVSafe[ix] * c.pace * boost * (c.mistakeMs ? 1.15f : 1.f);
    const float d = (float)kProbe[p] * gSp;
    const float vv = lim * lim + 2.f * 110.f * d;               // braking envelope
    if (vv < cap2) cap2 = vv;
  }
  float vT = sqrtf(cap2);
  const float vTop = kMaxSpeed * c.pace;
  if (vT > vTop) vT = vTop;
  if (vT > capV) vT = capV;
  if (c.mistakeMs == 0 && (rnd() % 4000) == 0 && c.pace < 0.95f) c.mistakeMs = 800;   // now and then: over-cook a bend

  // ---- drift decision ----
  if (c.drift) {
    c.driftMs = (uint16_t)(c.driftMs + dtMs);
    if (fabsx(curA) < 11.f || c.vF < 55.f || c.driftMs > 2600) { c.driftCool = 900; c.driftMs = 0; c.drift = false; }
  }
  bool wantDrift = false;
  if (canDrift && av > 85.f && fabsx(curA) > 26.f && capV > 1e8f && c.driftMs < 2600) wantDrift = true;
  if (c.drift && fabsx(curA) >= 11.f && c.vF >= 55.f && c.driftMs < 2600) wantDrift = true;
  k.hand = wantDrift;

  // ---- steering: aim the VELOCITY at the target, heading leads it ----
  const float tx = gTX[ti] - gSY[ti] * c.lane;
  const float ty = gTY[ti] + gSX[ti] * c.lane;
  const float aim = atan2B(ty - c.y, tx - c.x);
  float desired = aim;
  if (av > 35.f && v > 0.f) {
    const float velA = atan2B(c.vy, c.vx);
    const float eV = wrapB(aim - velA);
    const float smax = wantDrift ? 38.f : 20.f;
    desired = velA + clampf(eV * 1.3f, -smax, smax);
  }
  if (c.noiseMs == 0) { c.noiseMs = (uint16_t)(400 + rnd() % 500); c.noiseT = ((float)(rnd() % 200) - 100.f) * 0.0006f; }
  c.noise += (c.noiseT - c.noise) * 3.f * dt;
  float st = wrapB(desired - c.head) * (1.f / 12.f) + c.noise;
  if (wantDrift && fabsx(st) < 0.5f) st = (curA > 0.f) ? 0.5f : -0.5f;   // keep the slide going
  k.steer = clampf(st, -1.f, 1.f);

  // ---- throttle / brake ----
  if (v < vT - 3.f) k.up = true;
  else if (v > vT + 10.f && !wantDrift) k.dn = true;
  else if (v > vT + 25.f) { /* drifting: just lift */ }
}

// Car-car contact: soft push-apart with a little speed loss (rare: AI avoids)
void collideCars() {
  for (int i = 0; i < gCarsN; ++i) {
    for (int j = i + 1; j < gCarsN; ++j) {
      Car& a = gCars[i]; Car& b = gCars[j];
      const float dx = b.x - a.x, dy = b.y - a.y;
      const float d2 = dx * dx + dy * dy;
      if (d2 >= kCarR2 || d2 < 1e-4f) continue;
      const float d = sqrtf(d2);
      const float nx = dx / d, ny = dy / d;
      const float push = (11.f - d) * 0.5f;
      a.x -= nx * push; a.y -= ny * push;
      b.x += nx * push; b.y += ny * push;
      const float rel = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny;   // < 0 = closing
      if (rel < 0.f) {
        const float imp = -rel * 0.55f;
        a.vx -= nx * imp; a.vy -= ny * imp;
        b.vx += nx * imp; b.vy += ny * imp;
        a.vx *= 0.97f; a.vy *= 0.97f; b.vx *= 0.97f; b.vy *= 0.97f;
      }
    }
  }
}

void stepAi(float dt, uint32_t dtUs) {
  const uint32_t dtMs = dtUs / 1000;
  for (int i = 1; i < gCarsN; ++i) {
    Car& c = gCars[i];
    Ctl k;
    aiThink(c, k, i, dtMs, dt);
    driveCar(c, k, dt);
    if (c.lap > (int)kLaps && !c.done) { c.done = true; c.finishMs = gRaceMs; }
    // drift smoke for AI cars that are near the player (visual only)
    if (c.drift) {
      c.fxMs = (uint16_t)(c.fxMs + dtMs);
      if (c.fxMs >= 90) {
        c.fxMs = 0;
        const float dx = c.x - gP.x, dy = c.y - gP.y;
        if (dx * dx + dy * dy < 110.f * 110.f) {
          const uint8_t hb = (uint8_t)(int32_t)c.head;
          spawnPuff(c.x - cosB(hb) * (5.f / 32768.f), c.y - sinB(hb) * (5.f / 32768.f), hb);
        }
      }
    } else c.fxMs = 0;
  }
  collideCars();
}

void stepRace(const fb::Input& in, float dt, uint32_t dtUs) {
  stepPlayer(in, dt, dtUs);
  if (gMulti) stepAi(dt, dtUs);
}

// ---------------------------------------------------------------------------
// init / update
// ---------------------------------------------------------------------------
void gameInit() {
  buildTables();
  gRng = micros() ^ 0x9E3779B9UL;   // micros(): allowed for seeding only
  if (!gRng) gRng = 1;
  gTrackSel  = 0;
  gMulti     = false;
  buildTrack(gTrackSel);
  gState     = ST_TITLE;
  gTitleMs   = 0;
  gSelHoldUs = 0;
  gMenuIdx   = 0;
  gCdUs      = 0;
  startRace();
  // NOTE: gBestTotalMs deliberately NOT reset -> session bests survive re-entry.
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
      if (in.pressed & fb::KEY_SELECT) { gState = ST_MODE; gMenuIdx = gMulti ? 1 : 0; }
      break;

    case ST_MODE:
      if (in.pressed & (fb::KEY_UP | fb::KEY_DOWN)) gMenuIdx ^= 1;
      if (in.pressed & fb::KEY_LEFT) gState = ST_TITLE;
      if (in.pressed & fb::KEY_SELECT) {
        gMulti = (gMenuIdx == 1);
        gState = ST_TRACK;
      }
      break;

    case ST_TRACK: {
      uint8_t sel = gTrackSel;
      if (in.pressed & fb::KEY_UP)   sel = (uint8_t)((sel + kTrackCount - 1) % kTrackCount);
      if (in.pressed & fb::KEY_DOWN) sel = (uint8_t)((sel + 1) % kTrackCount);
      if (sel != gTrackSel) { gTrackSel = sel; buildTrack(gTrackSel); }
      if (in.pressed & fb::KEY_LEFT) gState = ST_MODE;
      if (in.pressed & fb::KEY_SELECT) {
        startRace();
        gCdUs  = 0;
        gState = ST_COUNT;
      }
      break;
    }

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
      } else if (gP.lap > (int)kLaps) {
        gState = ST_RESULT;
        gMenuIdx = 0;
        if (gMulti) gPlace = calcPlace(0);
        if (gBestTotalMs[gTrackSel] == 0 || gRaceMs < gBestTotalMs[gTrackSel]) gBestTotalMs[gTrackSel] = gRaceMs;
      }
      break;

    case ST_PAUSE:
      if (in.pressed & (fb::KEY_UP | fb::KEY_DOWN))
        gMenuIdx = (uint8_t)((gMenuIdx + ((in.pressed & fb::KEY_DOWN) ? 1 : 3)) % 4);
      if (in.pressed & fb::KEY_SELECT) {
        if      (gMenuIdx == 0) gState = ST_RACE;
        else if (gMenuIdx == 1) { startRace(); gCdUs = 0; gState = ST_COUNT; }
        else if (gMenuIdx == 2) { gState = ST_TRACK; }
        else                    return fb::GAME_EXIT;
      }
      break;

    case ST_RESULT:
      if (in.pressed & (fb::KEY_LEFT | fb::KEY_UP))    gMenuIdx = (uint8_t)((gMenuIdx + 2) % 3);
      if (in.pressed & (fb::KEY_RIGHT | fb::KEY_DOWN)) gMenuIdx = (uint8_t)((gMenuIdx + 1) % 3);
      if (in.pressed & fb::KEY_SELECT) {
        if      (gMenuIdx == 0) { startRace(); gCdUs = 0; gState = ST_COUNT; }
        else if (gMenuIdx == 1) { gState = ST_TRACK; }
        else                    return fb::GAME_EXIT;
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

// Cohen-Sutherland clip to a rectangle (U8g2's unsigned coords must never
// receive a negative value). The world uses the PLAYABLE rect (128 x kPlayH,
// everything above the HUD strip); the minimap uses its own box.
inline uint8_t clipCode(int16_t x, int16_t y, int16_t xm, int16_t ym, int16_t xM, int16_t yM) {
  return (uint8_t)((x < xm ? 1 : 0) | (x > xM ? 2 : 0) | (y < ym ? 4 : 0) | (y > yM ? 8 : 0));
}

bool clipSegR(int16_t& x0, int16_t& y0, int16_t& x1, int16_t& y1,
              int16_t xm, int16_t ym, int16_t xM, int16_t yM) {
  uint8_t c0 = clipCode(x0, y0, xm, ym, xM, yM), c1 = clipCode(x1, y1, xm, ym, xM, yM);
  for (int it = 0; it < 8; ++it) {
    if (!(c0 | c1)) return true;
    if (c0 & c1)    return false;
    const uint8_t c = c0 ? c0 : c1;
    int16_t x, y;
    if      (c & 8) { x = (int16_t)(x0 + (int16_t)(((int32_t)(x1 - x0) * (yM - y0)) / (y1 - y0))); y = yM; }
    else if (c & 4) { x = (int16_t)(x0 + (int16_t)(((int32_t)(x1 - x0) * (ym - y0)) / (y1 - y0))); y = ym; }
    else if (c & 2) { y = (int16_t)(y0 + (int16_t)(((int32_t)(y1 - y0) * (xM - x0)) / (x1 - x0))); x = xM; }
    else            { y = (int16_t)(y0 + (int16_t)(((int32_t)(y1 - y0) * (xm - x0)) / (x1 - x0))); x = xm; }
    if (c == c0) { x0 = x; y0 = y; c0 = clipCode(x0, y0, xm, ym, xM, yM); }
    else         { x1 = x; y1 = y; c1 = clipCode(x1, y1, xm, ym, xM, yM); }
  }
  return false;
}

void seg(U8G2& d, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  if (clipSegR(x0, y0, x1, y1, 0, 0, (int16_t)(fb::SCREEN_W - 1), (int16_t)(kPlayH - 1)))
    d.drawLine((u8g2_uint_t)x0, (u8g2_uint_t)y0, (u8g2_uint_t)x1, (u8g2_uint_t)y1);
}

// dotted line (every 2nd pixel), clipped to the playable area
void dotSeg(U8G2& d, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  const int16_t dx = (int16_t)(x1 - x0), dy = (int16_t)(y1 - y0);
  const int16_t adx = (int16_t)(dx < 0 ? -dx : dx), ady = (int16_t)(dy < 0 ? -dy : dy);
  const int16_t n = (adx > ady) ? adx : ady;
  if (n == 0) return;
  for (int16_t j = 0; j <= n; j += 2) {
    const int16_t x = (int16_t)(x0 + (int16_t)(((int32_t)dx * j) / n));
    const int16_t y = (int16_t)(y0 + (int16_t)(((int32_t)dy * j) / n));
    if (x >= 0 && x < (int16_t)fb::SCREEN_W && y >= 0 && y < kPlayH) d.drawPixel((u8g2_uint_t)x, (u8g2_uint_t)y);
  }
}

// world point -> screen; returns false when outside the coarse view radius
inline bool xform(float wx, float wy, float cosC, float sinC, int16_t& sx, int16_t& sy) {
  const float rx = wx - gP.x, ry = wy - gP.y;
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

// AI silhouettes (all clipped to the playable area through seg()):
//   0 RACER: slim body + front/rear wings   1 BOX: hollow hull with two bars
//   2 KITE : filled diamond with a dark centre stripe
void drawAiCar(U8G2& d, int16_t cx0, int16_t cy0, uint8_t scrA, uint8_t style) {
  const int32_t s = sinB(scrA), c = cosB(scrA);
  #define TX(px, py) ((int16_t)(cx0 + (int16_t)(((px) * s + (py) * c) >> 15)))
  #define TY(px, py) ((int16_t)(cy0 + (int16_t)(((px) * c - (py) * s) >> 15)))
  #define L(ax, ay, bx, by) seg(d, TX(ax, ay), TY(ax, ay), TX(bx, by), TY(bx, by))
  d.setDrawColor(1);
  if (style == 0) {
    // slim body drawn as three thick lines + wings
    L(0, 8, 0, -5); L(1, 6, 1, -5); L(-1, 6, -1, -5); L(2, 2, 2, -4); L(-2, 2, -2, -4);
    L(-5, 7, 5, 7); L(-5, -6, 5, -6); L(-5, -5, 5, -5);
  } else if (style == 1) {
    L(-4, -6, 4, -6); L(4, -6, 4, 4); L(4, 4, 2, 7); L(2, 7, -2, 7); L(-2, 7, -4, 4); L(-4, 4, -4, -6);
    L(-3, 1, 3, 1); L(-3, -3, 3, -3);
  } else {
    // filled diamond built from clipped chords (never touches U8g2 with off-screen triangle points)
    for (int yy = -6; yy <= 8; ++yy) {
      const int w = (yy >= -1) ? (4 * (8 - yy) + 4) / 9 : (4 * (yy + 6) + 2) / 5;
      L(-w, yy, w, yy);
    }
    d.setDrawColor(0);
    L(0, 5, 0, -3);
    d.setDrawColor(1);
  }
  #undef L
  #undef TY
  #undef TX
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

  const int first = gP.seg - kVisBack;
  for (int k = 0; k < kVisN; ++k) {
    int i = (first + k) % gN;
    if (i < 0) i += gN;
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
    ex[k] = (int16_t)(sx + nxc * (float)gRoadHalf); ey[k] = (int16_t)(sy - nyc * (float)gRoadHalf);
    fx[k] = (int16_t)(sx - nxc * (float)gRoadHalf); fy[k] = (int16_t)(sy + nyc * (float)gRoadHalf);
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
    // kerb: a bordered, striped band OUTSIDE the road edge on the side the bend turns toward. The border is a
    // solid line at the outer limit (it follows the thickness, so it tapers as the kerb fades in and out);
    // inside it the stripe lines are drawn in short on/off pieces. Purely visual: the drivable width and the
    // wall limit are untouched.
    for (int sideLeft = 1; sideLeft >= 0; --sideLeft) {
      const uint8_t* kb = sideLeft ? gKerbL : gKerbR;
      const int ea = kb[ti[k - 1]], eb = kb[ti[k]];
      if (ea == 0 && eb == 0) continue;
      const int16_t* bxp = sideLeft ? ex : fx;
      const int16_t* byp = sideLeft ? ey : fy;
      const int sg = sideLeft ? 1 : -1;
      // border (t = 1 is the existing outer edge pixel line, so thickness E puts the border at t = 1 + E)
      seg(d, (int16_t)(bxp[k - 1] + sg * (1 + ea) * ox[k - 1]), (int16_t)(byp[k - 1] + sg * (1 + ea) * oy[k - 1]),
             (int16_t)(bxp[k] + sg * (1 + eb) * ox[k]),         (int16_t)(byp[k] + sg * (1 + eb) * oy[k]));
      const int m = (ea < eb) ? ea : eb;
      for (int t = 2; t <= m; ++t) {                       // stripes between the road edge and the border
        const int16_t x0 = (int16_t)(bxp[k - 1] + sg * t * ox[k - 1]), y0 = (int16_t)(byp[k - 1] + sg * t * oy[k - 1]);
        const int16_t x1 = (int16_t)(bxp[k] + sg * t * ox[k]),         y1 = (int16_t)(byp[k] + sg * t * oy[k]);
        const int16_t dx = (int16_t)(x1 - x0), dy = (int16_t)(y1 - y0);
        seg(d, x0, y0, (int16_t)(x0 + dx / 4), (int16_t)(y0 + dy / 4));
        seg(d, (int16_t)(x0 + dx / 2), (int16_t)(y0 + dy / 2), (int16_t)(x0 + (dx * 3) / 4), (int16_t)(y0 + (dy * 3) / 4));
      }
    }
    // centre dashes (6 points on / 6 off), left out around a corner arrow so the arrow reads cleanly
    const uint8_t mk = gMark[i];
    if ((i % 12) < 6 && !(mk & 3) && !(gMark[ti[k - 1]] & 3)) seg(d, mx[k - 1], my[k - 1], mx[k], my[k]);
    // predictive corner arrows (painted on the road, world-anchored): the arrow is the road tangent at
    // this point turned toward the coming bend, so it rotates with the track and shows left vs right
    if ((mk & 3) && mx[k] >= 8 && mx[k] <= (int16_t)(fb::SCREEN_W - 8)
                 && my[k] >= 6 && my[k] <= (int16_t)(kPlayH - 6)) {
      static const uint8_t kTurnBrads[4] = { 0, 28, 38, 48 };      // how far the arrow leans: gentle / medium / sharp
      const uint8_t lvl = (uint8_t)(mk & 3);
      const uint8_t lean = (mk & 4) ? kTurnBrads[lvl] : (uint8_t)(256 - kTurnBrads[lvl]);
      drawArrow(d, mx[k], my[k], (uint8_t)(gTTan[i] - C + lean), false);
    }
    // drift-initiation zone: a dotted line across the road, clearly unlike the solid road arrows
    if (mk & 8) {
      const int16_t ax = (int16_t)(mx[k] + ((ex[k] - mx[k]) * 3) / 4), ay = (int16_t)(my[k] + ((ey[k] - my[k]) * 3) / 4);
      const int16_t bx = (int16_t)(mx[k] + ((fx[k] - mx[k]) * 3) / 4), by = (int16_t)(my[k] + ((fy[k] - my[k]) * 3) / 4);
      dotSeg(d, ax, ay, bx, by);
    }
    // trees, alternating sides
    if (i % 22 == 11) {
      const bool left = ((i / 22) & 1) == 0;
      const int16_t bx = left ? ex[k] : fx[k], by = left ? ey[k] : fy[k];
      int16_t txx = (int16_t)(mx[k] + (int16_t)(((bx - mx[k]) * 19) / 16));
      int16_t tyy = (int16_t)(my[k] + (int16_t)(((by - my[k]) * 19) / 16));
      const int kbt = left ? gKerbL[i] : gKerbR[i];      // keep trees clear of a kerb on their side
      if (kbt > 0) {
        const int push = 2 + kbt;
        txx = (int16_t)(txx + (left ? push * ox[k] : -push * ox[k]));
        tyy = (int16_t)(tyy + (left ? push * oy[k] : -push * oy[k]));
      }
      if (txx >= 2 && txx <= (int16_t)(fb::SCREEN_W - 3) && tyy >= 2 && tyy <= (int16_t)(kPlayH - 3))
        d.drawCircle((u8g2_uint_t)txx, (u8g2_uint_t)tyy, 2);
    }
    // start/finish checker line
    if (i == 0) {
      const int16_t tdx = (int16_t)(mx[k] - mx[k - 1]), tdy = (int16_t)(my[k] - my[k - 1]);
      for (int m = -7; m <= 7; m += 2) {                      // 8 steps across the road, any width
        int16_t bx = (int16_t)(mx[k] + (int16_t)(((ex[k] - mx[k]) * m) / 8));
        int16_t by = (int16_t)(my[k] + (int16_t)(((ey[k] - my[k]) * m) / 8));
        if ((((m + 7) / 2) & 1)) { bx = (int16_t)(bx + tdx / 4); by = (int16_t)(by + tdy / 4); }
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

  // AI opponents (only when near the screen)
  for (int i3 = 1; i3 < gCarsN; ++i3) {
    const Car& o = gCars[i3];
    int16_t sx, sy;
    if (!xform(o.x, o.y, cosC, sinC, sx, sy)) continue;
    if (sx < -10 || sx > (int16_t)(fb::SCREEN_W + 10) || sy < -10 || sy > (int16_t)(kPlayH + 10)) continue;
    const uint8_t scrA = (uint8_t)((uint8_t)(int32_t)o.head - C);
    drawAiCar(d, sx, sy, scrA, o.style);
  }
}

void renderCar(U8G2& d) {
  const uint8_t C    = (uint8_t)(int32_t)(gCamA - 64.f);
  const uint8_t scrA = (uint8_t)((uint8_t)(int32_t)gP.head - C);
  drawCarShape(d, (int16_t)(kCarX0 + gRumble), (int16_t)(kCarY0 + gRumble), scrA);
}

// Compact monochrome minimap (bottom-left, above the HUD). Same orientation as
// the main view (camera direction = up), so bends appear exactly as they will
// on screen. Track ahead = solid line, behind = dotted; start line = tick;
// opponents = 2x2 dots; the player = arrow showing the true heading.
void renderMinimap(U8G2& d) {
  const int16_t xM = (int16_t)(kMapW - 2), yM = (int16_t)(kPlayH - 1), ym = (int16_t)(kMapY + 1);
  d.setDrawColor(0);
  d.drawBox(0, (u8g2_uint_t)kMapY, (u8g2_uint_t)kMapW, (u8g2_uint_t)kMapH);
  d.setDrawColor(1);
  d.drawHLine(0, (u8g2_uint_t)kMapY, (u8g2_uint_t)kMapW);
  d.drawVLine((u8g2_uint_t)(kMapW - 1), (u8g2_uint_t)kMapY, (u8g2_uint_t)kMapH);

  const uint8_t C    = (uint8_t)(int32_t)(gCamA - 64.f);
  const float   cosC = cosB(C) * (1.f / 32768.f);
  const float   sinC = sinB(C) * (1.f / 32768.f);

  int16_t px0 = 0, py0 = 0;
  for (int k = -7; k <= 24; ++k) {
    int i = (gP.seg + k) % gN; if (i < 0) i += gN;
    const float rx = gTX[i] - gP.x, ry = gTY[i] - gP.y;
    const int16_t sx = (int16_t)(kMapCx + (rx * cosC + ry * sinC) * kMapScale);
    const int16_t sy = (int16_t)(kMapCy - (-rx * sinC + ry * cosC) * kMapScale);
    if (k > -7 && (k > 0 || (k & 1))) {            // behind the car: every other segment = dotted
      int16_t a0 = px0, b0 = py0, a1 = sx, b1 = sy;
      if (clipSegR(a0, b0, a1, b1, 0, ym, xM, yM))
        d.drawLine((u8g2_uint_t)a0, (u8g2_uint_t)b0, (u8g2_uint_t)a1, (u8g2_uint_t)b1);
    }
    if (i == 0 && sx >= 1 && sx <= xM - 1 && sy >= ym + 1 && sy <= yM - 1) {   // start/finish tick
      d.drawPixel((u8g2_uint_t)(sx - 1), (u8g2_uint_t)(sy - 1));
      d.drawPixel((u8g2_uint_t)(sx + 1), (u8g2_uint_t)(sy + 1));
    }
    px0 = sx; py0 = sy;
  }
  // opponents
  for (int i = 1; i < gCarsN; ++i) {
    const float rx = gCars[i].x - gP.x, ry = gCars[i].y - gP.y;
    const int16_t sx = (int16_t)(kMapCx + (rx * cosC + ry * sinC) * kMapScale);
    const int16_t sy = (int16_t)(kMapCy - (-rx * sinC + ry * cosC) * kMapScale);
    if (sx >= 1 && sx <= xM - 1 && sy >= ym + 1 && sy <= yM - 1)
      d.drawBox((u8g2_uint_t)sx, (u8g2_uint_t)sy, 2, 2);
  }
  // player arrow (true heading relative to the map's "up")
  const uint8_t a  = (uint8_t)((uint8_t)(int32_t)gP.head - C);
  const int32_t s  = sinB(a), c = cosB(a);
  const int16_t tx = (int16_t)(kMapCx + ((3 * c) >> 15)), ty = (int16_t)(kMapCy - ((3 * s) >> 15));
  const int16_t bx = (int16_t)(kMapCx - ((2 * c) >> 15)), by = (int16_t)(kMapCy + ((2 * s) >> 15));
  const int16_t qx = (int16_t)((2 * s) >> 15), qy = (int16_t)((2 * c) >> 15);   // perpendicular
  d.drawLine((u8g2_uint_t)tx, (u8g2_uint_t)ty, (u8g2_uint_t)(bx + qx), (u8g2_uint_t)(by + qy));
  d.drawLine((u8g2_uint_t)tx, (u8g2_uint_t)ty, (u8g2_uint_t)(bx - qx), (u8g2_uint_t)(by - qy));
  d.drawPixel((u8g2_uint_t)kMapCx, (u8g2_uint_t)kMapCy);
}

// Next-corner indicator for the HUD: the nearest corner entry ahead of the car, shown only inside a
// look-ahead of ~kHudLookS seconds of travel. Direction = chevron side, severity = 1..3 chevrons.
bool nextCorner(int& dir, int& sev) {
  const int ci = gNextCor[gP.seg];
  if (ci < 0) return false;
  const Corner& c = gCor[ci];
  const float dist = (float)(((int)c.entry - (int)gP.seg + gN) % gN) * gSp;
  float spd = gP.vF; if (spd < 60.f) spd = 60.f;
  if (dist > clampf(kHudLookS * spd, kHudLookMin, kHudLookMax)) return false;
  dir = c.dir; sev = c.sev;
  return true;
}

// 3x5 px chevrons, 4 px apart, rows 57..61 (inside the 5x7 text band of the HUD strip)
void drawChevrons(U8G2& d, int16_t x0, int dir, int n) {
  for (int k = 0; k < n; ++k) {
    const int16_t x = (int16_t)(x0 + 4 * k);
    for (int r = 0; r < 5; ++r) {
      const int off = (r < 3) ? r : (4 - r);              // 0,1,2,1,0
      const int16_t px = (int16_t)(dir > 0 ? (x + 2 - off) : (x + off));
      d.drawPixel((u8g2_uint_t)px, (u8g2_uint_t)(57 + r));
    }
  }
}

void fmtTime(char* b, uint32_t ms) {
  snprintf(b, 14, "%lu:%02lu.%lu", (unsigned long)(ms / 60000UL),
           (unsigned long)((ms / 1000UL) % 60UL), (unsigned long)((ms / 100UL) % 10UL));
}

// One compact HUD strip at the very bottom: [ LAP x/3 | NNN KMH | corner | m:ss.t ].
// Multiplayer: [ P2/4 L1/3 | NNNKMH | corner | m:ss.t ]. 'corner' = 1-3 chevrons for the next bend.
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
  int lap = gP.lap;
  if (lap > (int)kLaps) lap = (int)kLaps;
  if (lap < 1) lap = 1;
  if (gMulti) {
    snprintf(buf, sizeof(buf), "P%d/%d L%d/%d", (int)calcPlace(0), gCarsN, lap, (int)kLaps);
    d.drawStr(1, by, buf);
    snprintf(buf, sizeof(buf), "%3dKMH", (int)(gP.speed * kKmhScale));
    d.drawStr(50, by, buf);
  } else {
    snprintf(buf, sizeof(buf), "LAP %d/%d", lap, (int)kLaps);
    d.drawStr(2, by, buf);                                              // left
    snprintf(buf, sizeof(buf), "%3d KMH", (int)(gP.speed * kKmhScale)); // fixed width -> no jitter
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(buf)) / 2 - 2), by, buf);  // centre
  }
  {
    int cd, cs;
    if (nextCorner(cd, cs)) drawChevrons(d, 83, cd, cs + 1);                    // next corner, between speed and time
  }
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
  d.drawStr(2, 41, "UP GAS  DOWN BRAKE/REV");
  d.drawStr(2, 49, "L/R STEER  X DRIFT");
  if (((gTitleMs / 500UL) & 1UL)) {
    const char* p = "PRESS SELECT";
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(p)) / 2), 61, p);
  } else {
    const char* p = "SOLO OR VS AI  6 TRACKS";
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(p)) / 2), 61, p);
  }
}

void renderMode(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(1);
  d.setFont(u8g2_font_6x13B_tr);
  const char* t = "GAME MODE";
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(t)) / 2), 13, t);
  d.drawHLine(20, 17, fb::SCREEN_W - 40);
  d.setFont(u8g2_font_5x7_tr);
  static const char* opt[2]  = { "SINGLEPLAYER", "MULTIPLAYER" };
  static const char* desc[2] = { "SOLO TIME ATTACK", "RACE 3 AI OPPONENTS" };
  for (uint8_t i = 0; i < 2; ++i) {
    const u8g2_uint_t y = (u8g2_uint_t)(32 + i * 12);
    d.drawStr(30, y, opt[i]);
    if (i == gMenuIdx) { d.drawStr(20, y, ">"); }
  }
  const char* ds = desc[gMenuIdx];
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(ds)) / 2), 49, ds);
  d.drawStr(2, 61, "UP/DN PICK  SELECT OK");
}

void renderTrackMenu(U8G2& d) {
  char buf[16];
  d.setDrawColor(1);
  d.setFontMode(1);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(2, 7, "SELECT TRACK");
  d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(gMulti ? "MULTI" : "SOLO")), 7, gMulti ? "MULTI" : "SOLO");
  d.drawHLine(0, 9, fb::SCREEN_W);
  for (uint8_t i = 0; i < kTrackCount; ++i) {
    const u8g2_uint_t y = (u8g2_uint_t)(18 + i * 8);
    snprintf(buf, sizeof(buf), "%d %s", (int)(i + 1), kTracks[i].name);
    d.drawStr(8, y, buf);
    if (i == gTrackSel) d.drawStr(1, y, ">");
  }
  // preview of the selected layout (gT* already hold the selected track)
  const int16_t bx = 64, by = 12, bw = 62, bh = 34;
  d.drawFrame((u8g2_uint_t)bx, (u8g2_uint_t)by, (u8g2_uint_t)bw, (u8g2_uint_t)bh);
  const float w = gBX1 - gBX0, h = gBY1 - gBY0;
  float sc = (float)(bw - 8) / w;
  const float sc2 = (float)(bh - 8) / h;
  if (sc2 < sc) sc = sc2;
  const float cx = (gBX0 + gBX1) * 0.5f, cy = (gBY0 + gBY1) * 0.5f;
  const float mx = (float)bx + bw * 0.5f, my = (float)by + bh * 0.5f;
  int16_t lx = 0, ly = 0;
  for (int i = 0; i <= gN; ++i) {
    const int k = i % gN;
    const int16_t sx = (int16_t)(mx + (gTX[k] - cx) * sc);
    const int16_t sy = (int16_t)(my - (gTY[k] - cy) * sc);
    if (i) d.drawLine((u8g2_uint_t)lx, (u8g2_uint_t)ly, (u8g2_uint_t)sx, (u8g2_uint_t)sy);
    lx = sx; ly = sy;
  }
  const char* lv = kTracks[gTrackSel].level;
  d.drawStr((u8g2_uint_t)(bx + (bw - d.getStrWidth(lv)) / 2), 55, lv);
  if (gBestTotalMs[gTrackSel]) {
    fmtTime(buf, gBestTotalMs[gTrackSel]);
    d.drawStr((u8g2_uint_t)(bx + (bw - d.getStrWidth(buf)) / 2), 63, buf);
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
  d.drawBox(22, 4, 84, 56);
  d.setDrawColor(1);
  d.drawFrame(22, 4, 84, 56);
  d.setFontMode(1);
  d.setFont(u8g2_font_6x13B_tr);
  const char* t = "PAUSED";
  d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(t)) / 2), 18, t);
  static const char* opt[4] = { "RESUME", "RESTART RACE", "TRACK MENU", "QUIT TO MENU" };
  d.setFont(u8g2_font_5x7_tr);
  for (uint8_t i = 0; i < 4; ++i) {
    d.drawStr(34, (u8g2_uint_t)(29 + i * 8), opt[i]);
    if (i == gMenuIdx) d.drawStr(27, (u8g2_uint_t)(29 + i * 8), ">");
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
  if (gMulti) {
    snprintf(buf, sizeof(buf), "%d/%d", (int)gPlace, gCarsN);
    d.drawStr(2, 50, "POSITION"); d.drawStr((u8g2_uint_t)(fb::SCREEN_W - 2 - d.getStrWidth(buf)), 50, buf);
  } else if (gBestTotalMs[gTrackSel] == gRaceMs) {
    const char* nb = "NEW BEST!";
    d.drawStr((u8g2_uint_t)((fb::SCREEN_W - d.getStrWidth(nb)) / 2), 50, nb);
  }
  static const char* opt[3] = { "AGAIN", "TRACKS", "QUIT" };
  static const uint8_t ox[3] = { 8, 48, 96 };
  for (uint8_t i = 0; i < 3; ++i) {
    d.drawStr(ox[i], 61, opt[i]);
    if (i == gMenuIdx) d.drawStr((u8g2_uint_t)(ox[i] - 6), 61, ">");
  }
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
    case ST_MODE:
      renderMode(d);
      break;
    case ST_TRACK:
      renderTrackMenu(d);
      break;
    case ST_COUNT:
      renderWorld(d);
      renderCar(d);
      renderMinimap(d);
      renderHud(d);
      renderCountdown(d);
      break;
    case ST_RACE:
      renderWorld(d);
      renderCar(d);
      renderMinimap(d);
      renderHud(d);
      break;
    case ST_PAUSE:
      renderWorld(d);
      renderCar(d);
      renderMinimap(d);
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
