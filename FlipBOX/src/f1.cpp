// src/Game1.cpp  --  F1 Racing  (flipBOX)
//
// Pseudo-3D F1 time-trial racer for the 128x64 monochrome OLED.
//   X            : accelerate (button on D4 / GPIO4)
//   UP / DOWN    : shift up / shift down (3 gears)
//   LEFT / RIGHT : steer
//   Hold SELECT ~1 s (title, countdown or race) to quit to the launcher;
//   the results screen also offers an explicit RETRY / QUIT menu.
//
// Gearbox: each gear has its own top speed (250 / 350 / 400 km/h), launch
// acceleration, steering authority and grip. Low gears accelerate hard and
// steer sharply; high gears are fast but lose grip, so a hairpin taken at
// speed in 3rd gear slides straight off the road, while the same corner in
// 1st is a smooth turn. There is no brake: lift off X and downshift (engine
// braking pulls the car down to the lower gear's top speed).
//
// The road is a classic OutRun-style scanline renderer: one precomputed
// perspective table maps each road scanline to a world distance, and the
// road centre per scanline is found by double-integrating the track
// curvature from the car out to the horizon. All physics is Q16.16
// fixed-point driven by Frame::dtUs, so the feel is identical at any FPS.
// The top-centre track map is rasterised once in init() from the same
// curvature table that drives the road, so it always matches the circuit.
//
// Exports fbGame_F1Racing (already registered in main.cpp's FB_GAME_LIST).

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

#include <stdio.h>   // snprintf (integer formats only)
#include <math.h>    // sinf/cosf: used ONLY in init() to rasterise the track map

namespace {

// ---------------------------------------------------------------------------
// Fixed-point helpers (Q16.16).  Q() turns a readable decimal into Q16.16 at
// compile time, so every tuning constant below can be edited as a plain number.
// ---------------------------------------------------------------------------
constexpr int32_t Q(float v) { return (int32_t)(v * 65536.0f + 0.5f); }
inline int32_t mulQ16(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 16); }
inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

// -- gearbox: one entry per gear (index 0 = 1st gear) --
constexpr int kGearCount = 3;
constexpr int32_t kGearTopQ16[kGearCount]   = { Q(250.0f), Q(350.0f), Q(400.0f) };  // km/h  top speed in this gear
constexpr int32_t kGearAccelQ16[kGearCount] = { Q(100.0f), Q(70.0f),  Q(60.0f)  };  // km/h/s launch acceleration (tapers toward top speed)
constexpr int32_t kGearSteerQ16[kGearCount] = { Q(2.4f),   Q(1.8f),   Q(1.3f)   };  // road-widths/s steering at full speed
constexpr int32_t kGearGripQ16[kGearCount]  = { Q(0.60f),  Q(0.30f),  Q(0.18f)  };  // road-widths/s of cornering load tolerated before sliding
constexpr int32_t kShiftPointQ16[kGearCount - 1] = { Q(188.0f), Q(250.0f) };         // km/h where the NEXT gear pulls harder (shift-light)
constexpr int32_t kAccelTaperQ16 = Q(0.80f);     // accel *= (1 - 0.80 * v / gearTop)
constexpr uint32_t kShiftCutUs   = 120000UL;     // throttle cut while the gear changes, us

// -- car physics (speeds in Q16.16 km/h, rates per second) --
constexpr int32_t kMaxSpeedQ16   = kGearTopQ16[kGearCount - 1];   // overall top speed (400 km/h)
constexpr int     kMaxKmh        = (int)(kMaxSpeedQ16 >> 16);
constexpr int32_t kDragBaseQ16   = Q(8.0f);      // km/h/s coasting (X released): rolling resistance
constexpr int32_t kDragAeroQ16   = Q(60.0f);     // km/h/s extra coasting drag at 400 km/h (grows with v^2)
constexpr int32_t kEngBrakeQ16   = Q(60.0f);     // km/h/s engine braking when above the gear's top speed
constexpr int32_t kEngBrakeGainQ16 = Q(1.5f);    // extra km/h/s per km/h over the gear's top speed
constexpr int32_t kOffDecelQ16   = Q(350.0f);    // extra deceleration while off-road
constexpr int32_t kOffMaxQ16     = Q(100.0f);    // off-road speed cap
constexpr int32_t kSteerMinQ16   = Q(0.35f);     // fraction of steering kept at v = 0
constexpr int32_t kSteerSpanQ16  = 65536 - kSteerMinQ16;
constexpr int32_t kPlayerXMaxQ16 = Q(1.85f);     // furthest the car may wander off the road
constexpr int32_t kOffEdgeQ16    = 65536;        // 1.00: |playerX| beyond this = off the tarmac

// -- cornering / sliding --
// Cornering load = (curve/128) * (speed/400)^2 * kCurvePush  [road-widths/s].
// Steering can always fight that load; once it exceeds the gear's grip the
// car starts to slide: the excess is multiplied (kSlideMul), steering weakens
// (kSlideSteer) and the tyres scrub speed (kSlideScrub).
constexpr int32_t kCurvePushQ16  = Q(1.40f);
constexpr int32_t kSlideMulQ16   = Q(5.00f);     // extra outward push per unit of excess load
constexpr int32_t kSlideSteerQ16 = Q(0.45f);     // steering authority kept while sliding
constexpr int32_t kSlideScrubQ16 = Q(250.0f);    // km/h/s speed scrubbed per unit of excess load
constexpr int32_t kSlideOnQ16    = Q(0.02f);     // excess load that lights the SLIDE warning

// -- pseudo-3D projection --
constexpr int kHorizonY = 24;                            // y of the horizon line
constexpr int kRowsN    = fb::SCREEN_H - 1 - kHorizonY;  // road scanlines (39: y = 25..63)
constexpr int kProjC    = 250;                           // z[metres] = kProjC / (y - kHorizonY)
constexpr int kRoadWNum = 98;                            // half-width px = (y-kHorizonY) * kRoadWNum/64
constexpr int kCurveDiv = 58000;                         // curve accumulator -> px (smaller = sharper bends)

// -- track --
constexpr int kCurveShift  = 3;                // curve table resolution: one entry per 8 m
constexpr int kCurveRampM  = 28;               // curvature blend-in length at segment starts
constexpr int kBandShift   = 4;                // ground stripe period: 16 m
constexpr int kDashShift   = 3;                // centre dashes: 8 m on / 8 m off

// -- flow / HUD --
constexpr uint32_t kCountStepUs = 800000UL;    // countdown step (3, 2, 1)
constexpr uint32_t kGoShowUs    = 900000UL;    // "GO!" overlay time once racing
constexpr uint32_t kQuitHoldUs  = 1000000UL;   // SELECT hold time that quits to the launcher

// ---------------------------------------------------------------------------
// Track: a hand-designed 4.8 km circuit (clockwise) of straights, sweepers,
// chicane-like pairs and two hairpins.  Edit freely: lap length, the table
// size and the track map all derive from this list automatically.
// curve: +right / -left, -127..127.  ~35 sweeper, ~65 medium, ~95 tight,
// 127 hairpin.  Net turning should be mostly one way so the map closes into
// a loop (the map is auto-corrected, but wildly unbalanced lists look odd).
// ---------------------------------------------------------------------------
struct TrackSeg { int16_t len; int8_t curve; };

constexpr TrackSeg kSegs[] = {
  { 420,    0 },   // start/finish straight
  { 100,  100 },   // T1  tight right
  { 233,    0 },
  { 149,  -65 },   // T2  medium left
  { 138,    0 },
  {  90,   60 },   // T3  medium right
  { 116,    0 },
  { 105,  127 },   // T4  HAIRPIN right -- downshift!
  { 689,    0 },   // back straight (3rd gear, flat out)
  { 182,  -35 },   // T5  fast left sweeper
  { 123,    0 },
  { 129,   95 },   // T6  tight right
  { 119,    0 },
  { 106,  -90 },   // T7  tight left
  { 220,    0 },
  { 105, -127 },   // T8  HAIRPIN left
  { 200,    0 },
  { 111,   90 },   // T9  tight right
  { 142,    0 },
  {  80,   60 },   // T10 medium right
  {  84,    0 },
  { 332,   30 },   // T11 long right sweeper
  { 260,    0 },
  { 130,   70 },   // T12 medium right
  {  80,    0 },
  { 100,  -45 },   // T13 gentle left
  { 273,    0 },   // run back to the line
};
constexpr int kSegCount = (int)(sizeof(kSegs) / sizeof(kSegs[0]));

constexpr int32_t segLenSum(int n) { return n <= 0 ? 0 : segLenSum(n - 1) + kSegs[n - 1].len; }
constexpr int32_t kTrackLen    = segLenSum(kSegCount);                        // one lap, metres
constexpr int     kCurveTblLen = (int)((kTrackLen + (1 << kCurveShift) - 1) >> kCurveShift);

// ---------------------------------------------------------------------------
// Track map (top-centre HUD): frame, inner bitmap and player marker geometry.
// ---------------------------------------------------------------------------
constexpr int kMapX      = 42;                     // frame top-left
constexpr int kMapY      = 0;
constexpr int kMapW      = 44;                     // frame size
constexpr int kMapH      = 23;
constexpr int kMapIW     = kMapW - 2;              // inner bitmap size
constexpr int kMapIH     = kMapH - 2;
constexpr int kMapStride = (kMapIW + 7) >> 3;      // bytes per bitmap row
constexpr int kMapPad    = 2;                      // keeps the 5x5 marker halo inside the frame
constexpr float kMapRadPerUnit = 0.000218f;        // fallback heading change per (curve unit * metre)

// ---------------------------------------------------------------------------
// Tables and state
// ---------------------------------------------------------------------------
enum State : uint8_t { ST_TITLE, ST_COUNT, ST_PLAY, ST_RESULT };

constexpr uint8_t kFlagBand  = 1u << 0;   // ground/rumble stripe row
constexpr uint8_t kFlagDash  = 1u << 1;   // centre dash row
constexpr uint8_t kFlagStart = 1u << 2;   // start/finish line row

int8_t  gCurveTbl[kCurveTblLen];   // flattened, smoothed curvature per 8 m
int16_t gRowZ[kRowsN];             // constant: metres ahead per scanline
int16_t gRowHW[kRowsN];            // constant: road half-width px per scanline
int16_t gRowCX[kRowsN];            // per-frame: road centre x per scanline
uint8_t gRowFlags[kRowsN];         // per-frame: kFlag* per scanline

uint8_t gMapBits[kMapStride * kMapIH];   // XBM of the circuit outline (built in init)
uint8_t gMapPx[kCurveTblLen];            // map pixel of each 8 m track cell
uint8_t gMapPy[kCurveTblLen];

State    gState;
int32_t  gSpeedQ;        // Q16.16 km/h
int32_t  gPlayerXQ;      // Q16.16 road-widths: 0 = centre, +/-1.0 = road edge
int32_t  gDistQ;         // Q16.16 metres covered this race
int32_t  gLeanQ;         // Q16.16 px, cosmetic wing lean while steering
uint32_t gRaceTimeUs;
uint32_t gStateUs;
uint32_t gAnimMs;        // cosmetic clock (blink / jitter)
uint32_t gSelHoldUs;     // SELECT hold timer for quit
bool     gSelHold;       // TITLE: a SELECT press is currently in progress
bool     gOffroad;
bool     gSliding;       // cornering load above the gear's grip
bool     gShake;         // cosmetic: car judders (off-road or sliding)
uint8_t  gGear;          // 0..kGearCount-1
uint32_t gShiftUs;       // remaining shift throttle-cut time
uint8_t  gRevPx;         // rev bar fill, 0..kRevH
bool     gRevHot;        // shift-light: next gear would pull harder
uint8_t  gMapDotX;       // player marker, inner-bitmap coordinates
uint8_t  gMapDotY;
uint8_t  gMenuSel;       // RESULT cursor: 0 = RETRY, 1 = QUIT
bool     gNewBest;
uint32_t gFinalTimeUs;

// Session-persistent best lap: intentionally NOT reset by gameInit() so it
// survives leaving and re-entering the game (cleared only at power cycle).
uint32_t gBestTimeUs = 0;

// Cached HUD strings (re-formatted only when the value changes).
char     gSpeedStr[8];
int      gSpeedCache;
char     gTimeStr[16];
uint32_t gTimeCacheTenths;

const char* const kGearTxt[kGearCount] = { "1", "2", "3" };

// ---------------------------------------------------------------------------
// Track / projection preparation
// ---------------------------------------------------------------------------

// Flatten the segment list into one curvature sample per 8 metres, blending
// each segment's curvature in over kCurveRampM so bends never snap on.
void buildCurveTable() {
  int segIdx     = 0;
  int segBegin   = 0;
  int segEnd     = kSegs[0].len;
  int prevCurve  = kSegs[kSegCount - 1].curve;   // circuit wraps: seg before first is last
  for (int i = 0; i < kCurveTblLen; ++i) {
    const int d = (i << kCurveShift) + 4;        // sample at the entry's centre
    while (d >= segEnd && segIdx < kSegCount - 1) {
      prevCurve = kSegs[segIdx].curve;
      ++segIdx;
      segBegin  = segEnd;
      segEnd   += kSegs[segIdx].len;
    }
    const int cur = kSegs[segIdx].curve;
    const int len = kSegs[segIdx].len;
    int ramp = kCurveRampM;
    if (ramp > len / 2) ramp = len / 2;
    int c = cur;
    const int localD = d - segBegin;
    if (ramp > 0 && localD < ramp) {
      c = prevCurve + (int)(((int32_t)(cur - prevCurve) * localD) / ramp);
    }
    gCurveTbl[i] = (int8_t)c;
  }
}

inline int sampleCurve(int32_t posM) {
  int32_t p = posM % kTrackLen;
  return gCurveTbl[p >> kCurveShift];
}

// Per-frame: for every road scanline compute the road centre x and the
// ground-stripe / dash / start-line flags. Done in update() so render()
// only draws (spec 10.4).
//
// Classic pseudo-3D curve math: walking from the nearest scanline to the
// horizon, curvature is integrated twice (cv = sum of curve*dz, cx = sum of
// cv*dz), giving the lateral displacement of the road at that distance; the
// perspective-scaled result is the on-screen centre shift.
void updateProjection() {
  const int32_t posMod = (gDistQ >> 16) % kTrackLen;
  int32_t cv = 0;
  int32_t cx = 0;
  for (int i = kRowsN - 1; i >= 0; --i) {        // bottom (near) -> horizon (far)
    const int32_t z  = gRowZ[i];
    const int32_t dz = (i == kRowsN - 1) ? z : (z - gRowZ[i + 1]);
    int32_t dist = posMod + z;
    if (dist >= kTrackLen) dist -= kTrackLen;
    const int32_t c = gCurveTbl[dist >> kCurveShift];
    cv += c * dz;
    cx += cv * dz;
    const int32_t hw = gRowHW[i];
    const int32_t offPx     = (cx * hw) / kCurveDiv;          // bend displacement
    const int32_t playerOff = (gPlayerXQ * hw) >> 16;         // road slides under the car
    int32_t sx = (int32_t)(fb::SCREEN_W / 2) - playerOff + offPx;
    sx = clampI32(sx, -80, (int32_t)fb::SCREEN_W + 80);       // drawing clamps again
    gRowCX[i] = (int16_t)sx;
    uint8_t fl = 0;
    if ((dist >> kBandShift) & 1)          fl |= kFlagBand;
    if (((dist >> kDashShift) & 1) && hw >= 3) fl |= kFlagDash;
    if (dist < 8)                          fl |= kFlagStart;
    gRowFlags[i] = fl;
  }
}

// ---------------------------------------------------------------------------
// Track map: walk the curvature table once to get the circuit outline, close
// the loop, fit it to the map box and rasterise it into gMapBits (init only;
// float + sinf/cosf are fine here, never per frame).
// ---------------------------------------------------------------------------
struct MapWalker { float x, y, a; };

inline void mapStep(MapWalker& w, int i, float radPerUnit) {
  constexpr float kStepM = (float)(1 << kCurveShift);
  w.a += (float)gCurveTbl[i] * kStepM * radPerUnit;
  w.x += cosf(w.a) * kStepM;
  w.y += sinf(w.a) * kStepM;
}

inline void mapSet(int x, int y) {
  if (x < 0 || y < 0 || x >= kMapIW || y >= kMapIH) return;
  gMapBits[y * kMapStride + (x >> 3)] |= (uint8_t)(1u << (x & 7));   // XBM: LSB = leftmost
}

void buildMap() {
  for (int i = 0; i < kMapStride * kMapIH; ++i) gMapBits[i] = 0;

  // Heading scale: make one lap turn exactly 360 degrees (any track whose
  // turns are mostly one-handed), so the outline always closes cleanly.
  int32_t sum = 0;
  for (int i = 0; i < kCurveTblLen; ++i) sum += gCurveTbl[i];
  float k = kMapRadPerUnit;
  const float netRad = (float)sum * (float)(1 << kCurveShift) * kMapRadPerUnit;
  if (netRad > 1.5708f || netRad < -1.5708f) {
    k = 6.2831853f / ((float)sum * (float)(1 << kCurveShift));
  }

  // Pass A: where does the walk end up?  Any gap to the start is spread
  // linearly along the lap (pass B/C) so the loop is exactly closed.
  MapWalker w = { 0.0f, 0.0f, 0.0f };
  for (int i = 0; i < kCurveTblLen; ++i) mapStep(w, i, k);
  const float rx = w.x / (float)kCurveTblLen;
  const float ry = w.y / (float)kCurveTblLen;

  // Pass B: bounding box.
  float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
  w.x = 0.0f; w.y = 0.0f; w.a = 0.0f;
  for (int i = 0; i < kCurveTblLen; ++i) {
    mapStep(w, i, k);
    const float px = w.x - rx * (float)(i + 1);
    const float py = w.y - ry * (float)(i + 1);
    if (px < minx) minx = px;
    if (px > maxx) maxx = px;
    if (py < miny) miny = py;
    if (py > maxy) maxy = py;
  }
  float spanX = maxx - minx;
  float spanY = maxy - miny;
  if (spanX < 1.0f) spanX = 1.0f;
  if (spanY < 1.0f) spanY = 1.0f;
  const float availW = (float)(kMapIW - 2 * kMapPad - 1);
  const float availH = (float)(kMapIH - 2 * kMapPad - 1);
  float scale = availW / spanX;
  if (availH / spanY < scale) scale = availH / spanY;
  const float offX = (float)kMapPad + (availW - spanX * scale) * 0.5f;
  const float offY = (float)kMapPad + (availH - spanY * scale) * 0.5f;

  // Pass C: rasterise. Steps are well under a pixel, so plotting every cell
  // gives a gap-free outline; the per-cell pixel is kept for the marker.
  w.x = 0.0f; w.y = 0.0f; w.a = 0.0f;
  for (int i = 0; i < kCurveTblLen; ++i) {
    mapStep(w, i, k);
    const float px = w.x - rx * (float)(i + 1);
    const float py = w.y - ry * (float)(i + 1);
    const int ix = clampI32((int)((px - minx) * scale + offX + 0.5f), kMapPad, kMapIW - 1 - kMapPad);
    const int iy = clampI32((int)((py - miny) * scale + offY + 0.5f), kMapPad, kMapIH - 1 - kMapPad);
    gMapPx[i] = (uint8_t)ix;
    gMapPy[i] = (uint8_t)iy;
    mapSet(ix, iy);
  }

  // Start/finish tick: the lap closes on the last cell and the start straight
  // runs west->east, so a short vertical bar marks the line.
  const int sx = gMapPx[kCurveTblLen - 1];
  const int sy = gMapPy[kCurveTblLen - 1];
  for (int dy = -2; dy <= 2; ++dy) mapSet(sx, sy + dy);
}

// Marker position for the current distance (cheap lookup, per frame).
void updateMapDot() {
  const int32_t p = (gDistQ >> 16) % kTrackLen;
  const int idx = (int)(p >> kCurveShift);
  gMapDotX = gMapPx[idx];
  gMapDotY = gMapPy[idx];
}

// ---------------------------------------------------------------------------
// Small text helpers
// ---------------------------------------------------------------------------
void formatTime(char* buf, uint32_t us) {
  const uint32_t m = us / 60000000UL;
  const uint32_t s = (us / 1000000UL) % 60UL;
  const uint32_t t = (us / 100000UL) % 10UL;
  snprintf(buf, 16, "%lu:%02lu.%lu", (unsigned long)m, (unsigned long)s, (unsigned long)t);
}

void updateHudStrings() {
  const int kmh = (int)(gSpeedQ >> 16);
  if (kmh != gSpeedCache) {
    gSpeedCache = kmh;
    snprintf(gSpeedStr, sizeof(gSpeedStr), "%d", kmh);
  }
  const uint32_t tenths = gRaceTimeUs / 100000UL;
  if (tenths != gTimeCacheTenths) {
    gTimeCacheTenths = tenths;
    formatTime(gTimeStr, gRaceTimeUs);
  }
}

// ---------------------------------------------------------------------------
// Gearbox
// ---------------------------------------------------------------------------
constexpr int kRevH = 15;   // rev bar inner height, px

// UP = shift up, DOWN = shift down (edge-triggered). A shift briefly cuts the
// throttle, so changing gear needlessly costs time.
void handleShift(const fb::Input& in, uint32_t dtUs) {
  gShiftUs = (gShiftUs > dtUs) ? (gShiftUs - dtUs) : 0;
  if ((in.pressed & fb::KEY_UP) && gGear < kGearCount - 1) {
    ++gGear;
    gShiftUs = kShiftCutUs;
  }
  if ((in.pressed & fb::KEY_DOWN) && gGear > 0) {
    --gGear;
    gShiftUs = kShiftCutUs;
  }
}

// Rev bar + shift light for the current gear (HUD data, computed in update()).
void updateRev() {
  const int32_t top = kGearTopQ16[gGear];
  int32_t ratio = (int32_t)(((int64_t)gSpeedQ << 16) / top);   // v / gear top, Q16
  ratio = clampI32(ratio, 0, 65536);
  gRevPx  = (uint8_t)((ratio * kRevH) >> 16);
  gRevHot = (gGear < kGearCount - 1) && (gSpeedQ >= kShiftPointQ16[gGear]);
}

// ---------------------------------------------------------------------------
// Race lifecycle
// ---------------------------------------------------------------------------
void resetRace() {
  gSpeedQ   = 0;
  gPlayerXQ = 0;
  gDistQ    = 0;
  gLeanQ    = 0;
  gRaceTimeUs = 0;
  gStateUs    = 0;
  gSelHoldUs  = 0;
  gSelHold    = false;
  gOffroad    = false;
  gSliding    = false;
  gShake      = false;
  gGear       = 0;
  gShiftUs    = 0;
  gRevPx      = 0;
  gRevHot     = false;
  gMenuSel    = 0;
  gNewBest    = false;
  gFinalTimeUs = 0;
  gSpeedCache = -1;
  gTimeCacheTenths = 0xFFFFFFFFUL;
  updateMapDot();
}

void gameInit() {
  buildCurveTable();
  buildMap();
  for (int i = 0; i < kRowsN; ++i) {
    const int dy = i + 1;                      // y = kHorizonY + dy
    gRowZ[i]  = (int16_t)(kProjC / dy);        // metres ahead of the car
    gRowHW[i] = (int16_t)((dy * kRoadWNum) >> 6);
  }
  resetRace();
  gState  = ST_TITLE;
  gAnimMs = 0;
  updateProjection();                          // valid road from the first frame
  // gBestTimeUs deliberately survives re-entry (session best).
}

// ---------------------------------------------------------------------------
// update() -- logic only
// ---------------------------------------------------------------------------
fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  gAnimMs += f.dtUs / 1000UL;
  const int32_t dt16 = (int32_t)((f.dtUs << 16) / 1000000UL);   // Q16.16 seconds

  switch (gState) {

    case ST_TITLE:
      // Tap SELECT to start; hold it ~1 s to quit. The hold timer makes the
      // two gestures unambiguous even though both begin with the same press.
      if (in.pressed & fb::KEY_SELECT) { gSelHold = true; gSelHoldUs = 0; }
      if (gSelHold) {
        if (in.held & fb::KEY_SELECT) {
          gSelHoldUs += f.dtUs;
          if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
        } else {
          gSelHold = false;
          if (gSelHoldUs < kQuitHoldUs) { resetRace(); gState = ST_COUNT; }
        }
      }
      break;

    case ST_COUNT:
      gStateUs += f.dtUs;
      gSpeedQ = 0;                             // cars held on the grid
      handleShift(in, f.dtUs);                 // pick a launch gear during the countdown
      updateRev();
      updateProjection();                      // static road, start line visible
      updateHudStrings();
      if (in.held & fb::KEY_SELECT) {
        gSelHoldUs += f.dtUs;
        if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
      } else {
        gSelHoldUs = 0;
      }
      if (gStateUs >= 3UL * kCountStepUs) {    // after "3 2 1" -> GO
        gState = ST_PLAY;
        gStateUs = 0;
        gRaceTimeUs = 0;
        gSelHoldUs = 0;
      }
      break;

    case ST_PLAY: {
      // --- quit hold (SELECT is otherwise unused while racing) ---
      if (in.held & fb::KEY_SELECT) {
        gSelHoldUs += f.dtUs;
        if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
      } else {
        gSelHoldUs = 0;
      }

      // --- gear shifting ---
      handleShift(in, f.dtUs);
      const int32_t topQ = kGearTopQ16[gGear];

      // --- off-road state from the current lateral position ---
      gOffroad = (gPlayerXQ > kOffEdgeQ16) || (gPlayerXQ < -kOffEdgeQ16);

      // --- engine braking: above the gear's top speed (after a downshift)
      //     the engine drags the car back down to that top speed ---
      if (gSpeedQ > topQ) {
        const int32_t over = gSpeedQ - topQ;
        gSpeedQ -= mulQ16(kEngBrakeQ16 + mulQ16(kEngBrakeGainQ16, over), dt16);
        if (gSpeedQ < topQ) gSpeedQ = topQ;
      }

      // --- throttle (X) / coasting ---
      const bool gas = (in.held & fb::KEY_X) != 0;
      if (gas) {
        if (gShiftUs == 0 && gSpeedQ < topQ) {
          const int32_t ratio = (int32_t)(((int64_t)gSpeedQ << 16) / topQ);   // v / gear top
          const int32_t a = mulQ16(kGearAccelQ16[gGear], 65536 - mulQ16(kAccelTaperQ16, ratio));
          gSpeedQ += mulQ16(a, dt16);
          if (gSpeedQ > topQ) gSpeedQ = topQ;
        }
      } else {
        const int32_t snC = gSpeedQ / kMaxKmh;                              // v / vmax, Q16
        const int32_t drag = kDragBaseQ16 + mulQ16(kDragAeroQ16, mulQ16(snC, snC));
        gSpeedQ -= mulQ16(drag, dt16);
      }
      if (gOffroad) {
        // Strong bleed while above the off-road cap; only mild resistance
        // below it so the car can always crawl back onto the tarmac.
        if (gSpeedQ > kOffMaxQ16) gSpeedQ -= mulQ16(kOffDecelQ16, dt16);
        else                      gSpeedQ -= mulQ16(kDragBaseQ16 * 2, dt16);
      }

      // --- cornering load vs this gear's grip ---
      const int32_t sn = gSpeedQ / kMaxKmh;                                 // v / vmax, Q16
      const int32_t curveNow = sampleCurve(gDistQ >> 16);                   // -127..127
      const int32_t loadSigned = mulQ16(mulQ16(curveNow << 9, mulQ16(sn, sn)), kCurvePushQ16);
      const int32_t load = loadSigned < 0 ? -loadSigned : loadSigned;
      int32_t excess = load - kGearGripQ16[gGear];
      if (excess < 0) excess = 0;
      gSliding = excess > kSlideOnQ16;
      int32_t push = loadSigned;                                            // outward push, road-widths/s
      if (excess > 0) {
        const int32_t extra = mulQ16(excess, kSlideMulQ16);
        push += (loadSigned < 0) ? -extra : extra;
        gSpeedQ -= mulQ16(mulQ16(excess, kSlideScrubQ16), dt16);            // tyre scrub
      }
      gSpeedQ = clampI32(gSpeedQ, 0, kMaxSpeedQ16);

      // --- steering: authority depends on gear and speed, drops while sliding ---
      int steer = 0;
      if (in.held & fb::KEY_LEFT)  steer -= 1;
      if (in.held & fb::KEY_RIGHT) steer += 1;
      int32_t sRate = mulQ16(kGearSteerQ16[gGear], kSteerMinQ16 + mulQ16(kSteerSpanQ16, sn));
      if (gSliding) sRate = mulQ16(sRate, kSlideSteerQ16);
      gPlayerXQ += (int32_t)steer * mulQ16(sRate, dt16);
      gPlayerXQ -= mulQ16(push, dt16);                                      // bends push the car outward
      gPlayerXQ = clampI32(gPlayerXQ, -kPlayerXMaxQ16, kPlayerXMaxQ16);

      // --- distance / time ---
      constexpr int32_t kKmhToMsQ16 = 18204;                                // 1/3.6 in Q16.16
      gDistQ      += mulQ16(mulQ16(gSpeedQ, kKmhToMsQ16), dt16);
      gRaceTimeUs += f.dtUs;

      // --- cosmetic steering lean (exponential approach, frame-rate safe) ---
      const int32_t leanTgtQ = (int32_t)steer * (2L << 16);
      gLeanQ += mulQ16(leanTgtQ - gLeanQ, dt16 * 6);
      gShake = gSliding || (gOffroad && (gSpeedQ > (10L << 16)));

      // --- HUD prep ---
      updateRev();
      updateMapDot();
      updateHudStrings();
      updateProjection();

      // --- lap complete ---
      if ((gDistQ >> 16) >= kTrackLen) {
        gFinalTimeUs = gRaceTimeUs;
        gNewBest = (gBestTimeUs == 0) || (gRaceTimeUs < gBestTimeUs);
        if (gNewBest) gBestTimeUs = gRaceTimeUs;
        gMenuSel = 0;
        gStateUs = 0;
        gState = ST_RESULT;
      }
      break;
    }

    case ST_RESULT:
      if (in.pressed & (fb::KEY_UP | fb::KEY_DOWN | fb::KEY_LEFT | fb::KEY_RIGHT)) {
        gMenuSel ^= 1u;
      }
      if (in.pressed & fb::KEY_SELECT) {
        if (gMenuSel == 0) {
          resetRace();
          gState = ST_COUNT;
        } else {
          return fb::GAME_EXIT;
        }
      }
      break;
  }
  return fb::GAME_RUN;
}

// ---------------------------------------------------------------------------
// render() -- drawing only. All geometry is clamped to the 128x64 screen
// before any U8g2 call (U8g2 coordinates are 8-bit unsigned by default).
// ---------------------------------------------------------------------------
void hline(U8G2& d, int x, int y, int w) {
  if (w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x >= (int)fb::SCREEN_W) return;
  if (x + w > (int)fb::SCREEN_W) w = (int)fb::SCREEN_W - x;
  if (w <= 0) return;
  d.drawHLine((u8g2_uint_t)x, (u8g2_uint_t)y, (u8g2_uint_t)w);
}

void box(U8G2& d, int x, int y, int w, int h) {
  if (w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= (int)fb::SCREEN_W || y >= (int)fb::SCREEN_H) return;
  if (x + w > (int)fb::SCREEN_W) w = (int)fb::SCREEN_W - x;
  if (y + h > (int)fb::SCREEN_H) h = (int)fb::SCREEN_H - y;
  if (w <= 0 || h <= 0) return;
  d.drawBox((u8g2_uint_t)x, (u8g2_uint_t)y, (u8g2_uint_t)w, (u8g2_uint_t)h);
}

void frameBox(U8G2& d, int x, int y, int w, int h) {
  d.drawFrame((u8g2_uint_t)x, (u8g2_uint_t)y, (u8g2_uint_t)w, (u8g2_uint_t)h);
}

void centerStr(U8G2& d, int y, const char* s) {
  int x = ((int)fb::SCREEN_W - (int)d.getStrWidth(s)) / 2;
  if (x < 0) x = 0;
  d.drawStr((u8g2_uint_t)x, (u8g2_uint_t)y, s);
}

// Rear-view F1 car: rear wing + endplates, stepped bodywork, big rear tyres.
// Drawn with a dark halo so it never blends into the road graphics.
void drawCar(U8G2& d, int cx, int top, int lean, bool jitter) {
  if (jitter) top += (int)((gAnimMs >> 5) & 1u);
  const int lt = lean;        // wing leans most
  const int lm = lean >> 1;   // upper body leans a little
  d.setDrawColor(0);          // halo pass
  box(d, cx - 14 + lt, top - 1, 28, 5);
  box(d, cx - 14 + lt, top - 1, 4, 7);
  box(d, cx + 10 + lt, top - 1, 4, 7);
  box(d, cx - 4 + lm, top + 2, 8, 8);
  box(d, cx - 5, top + 8, 10, 6);
  box(d, cx - 13, top + 6, 7, 8);
  box(d, cx + 6, top + 6, 7, 8);
  d.setDrawColor(1);          // lit pass
  box(d, cx - 13 + lt, top, 26, 3);            // rear wing
  box(d, cx - 13 + lt, top, 2, 5);             // endplates
  box(d, cx + 11 + lt, top, 2, 5);
  box(d, cx - 3 + lm, top + 3, 6, 6);          // engine cover / cockpit
  box(d, cx - 4, top + 9, 8, 4);               // diffuser / gearbox
  box(d, cx - 12, top + 7, 5, 6);              // rear tyres
  box(d, cx + 7, top + 7, 5, 6);
  hline(d, cx - 8, top + 10, 4);               // suspension arms
  hline(d, cx + 4, top + 10, 4);
}

void renderWorld(U8G2& d) {
  d.setDrawColor(1);
  d.drawHLine(0, (u8g2_uint_t)kHorizonY, fb::SCREEN_W);   // horizon anchors the road

  // road scanlines, horizon -> bottom
  for (int i = 0; i < kRowsN; ++i) {
    const int y  = kHorizonY + 1 + i;
    const int hw = (int)gRowHW[i];
    const int cx = (int)gRowCX[i];
    const int ew = hw / 12 + 1;          // edge line width (grows toward viewer)
    const int rw = hw / 10 + 1;          // rumble strip width
    const int l  = cx - hw;              // road spans [l, r)
    const int r  = cx + hw;
    const uint8_t fl = gRowFlags[i];
    if (fl & kFlagStart) {
      hline(d, l, y, r - l);             // start/finish line across the road
    }
    if (fl & kFlagBand) {
      hline(d, 0, y, l - rw);            // grass stripe (perspective-compressed)
      hline(d, r + rw, y, (int)fb::SCREEN_W - r - rw);
      hline(d, l - rw, y, rw);           // rumble strips
      hline(d, r, y, rw);
    }
    hline(d, l, y, ew);                  // road edges (always lit)
    hline(d, r - ew, y, ew);
    if (fl & kFlagDash) {
      int dw = hw / 10 + 1;
      if (dw > 6) dw = 6;
      hline(d, cx - (dw >> 1), y, dw);   // centre dashes scroll with speed
    }
  }

  // player car: fixed near bottom-centre, the road slides beneath it
  drawCar(d, fb::SCREEN_W / 2, 47, (int)(gLeanQ >> 16), gShake);
}

// Top-centre circuit map with the player marker (dark halo keeps it readable
// on the outline).
void renderMap(U8G2& d) {
  d.setDrawColor(1);
  frameBox(d, kMapX, kMapY, kMapW, kMapH);
  d.setBitmapMode(0);
  d.drawXBM((u8g2_uint_t)(kMapX + 1), (u8g2_uint_t)(kMapY + 1),
            (u8g2_uint_t)kMapIW, (u8g2_uint_t)kMapIH, gMapBits);
  const int mx = kMapX + 1 + (int)gMapDotX;
  const int my = kMapY + 1 + (int)gMapDotY;
  d.setDrawColor(0);
  box(d, mx - 2, my - 2, 5, 5);
  d.setDrawColor(1);
  box(d, mx - 1, my - 1, 3, 3);
}

// Gear number + rev bar, bottom-right (erased panel so it never blends into
// the road). The digit inverts for the duration of a shift.
void renderGearPanel(U8G2& d) {
  const int px = 100, py = 40, pw = 27, ph = 23;
  d.setDrawColor(0);
  box(d, px, py, pw, ph);
  d.setDrawColor(1);
  frameBox(d, px, py, pw, ph);

  const int bx = px + 3, by = py + 3, bh = ph - 6;          // rev bar: frame 5 x 17, inner 3 x 15
  frameBox(d, bx, by, 5, bh);
  const bool lit = !gRevHot || ((gAnimMs >> 6) & 1u);       // blink = shift now
  if (gRevPx > 0 && lit) box(d, bx + 1, by + bh - 1 - (int)gRevPx, 3, (int)gRevPx);

  d.setFont(u8g2_font_logisoso16_tn);
  d.setFontMode(1);
  const int gx = px + 11;
  const int gy = py + ph - 4;
  if (gShiftUs > 0) {
    box(d, gx - 2, py + 2, 15, ph - 4);
    d.setDrawColor(0);
    d.drawStr((u8g2_uint_t)gx, (u8g2_uint_t)gy, kGearTxt[gGear]);
    d.setDrawColor(1);
  } else {
    d.drawStr((u8g2_uint_t)gx, (u8g2_uint_t)gy, kGearTxt[gGear]);
  }
  d.setFontMode(0);
}

void renderSlideWarning(U8G2& d) {
  if (!((gAnimMs >> 6) & 1u)) return;                        // blink
  d.setDrawColor(0);
  box(d, 1, 54, 29, 9);
  d.setDrawColor(1);
  frameBox(d, 1, 54, 29, 9);
  d.setFont(u8g2_font_5x7_tr);
  d.setFontMode(1);
  d.drawStr(4, 61, "SLIDE");
  d.setFontMode(0);
}

void renderHud(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(0);                      // solid text: readable over the road
  d.setFont(u8g2_font_logisoso16_tn);    // big speed, top-left
  d.drawStr(2, 16, gSpeedStr);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(2, 23, "KM/H");
  d.setFont(u8g2_font_6x13_tr);          // race time top-right, BELOW the
  const int w = (int)d.getStrWidth(gTimeStr);   // launcher's FPS-box zone
  d.drawStr((u8g2_uint_t)((int)fb::SCREEN_W - 2 - w), 21, gTimeStr);
  renderMap(d);
  renderGearPanel(d);
  if (gSliding) renderSlideWarning(d);
}

// Small "hold SELECT to quit" progress bar (bottom-centre).
void drawQuitBar(U8G2& d) {
  const int x = 40, y = 57, w = 48, h = 6;
  d.setDrawColor(0);
  box(d, x - 1, y - 1, w + 2, h + 2);
  d.setDrawColor(1);
  frameBox(d, x, y, w, h);
  const int fw = (int)((gSelHoldUs * (uint32_t)(w - 2)) / kQuitHoldUs);
  if (fw > 0) d.drawBox((u8g2_uint_t)(x + 1), (u8g2_uint_t)(y + 1),
                        (u8g2_uint_t)fw, (u8g2_uint_t)(h - 4));
}

void renderTitle(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(0);
  for (int x = 0; x < (int)fb::SCREEN_W; x += 8) {     // chequered strip
    d.drawBox((u8g2_uint_t)x, 0, 4, 4);
    d.drawBox((u8g2_uint_t)(x + 4), 4, 4, 4);
  }
  d.setFont(u8g2_font_7x13B_tr);
  centerStr(d, 19, "F1 RACING");
  d.setFont(u8g2_font_5x7_tr);
  centerStr(d, 28, "flipBOX TIME TRIAL");
  drawCar(d, fb::SCREEN_W / 2, 31, 0, false);
  d.setFont(u8g2_font_5x7_tr);
  centerStr(d, 52, "X=GAS  UP/DN=GEAR");
  if (((gAnimMs / 600UL) & 1UL) || gSelHold) {
    d.setFont(u8g2_font_6x10_tr);
    centerStr(d, 61, "PRESS SELECT");
  } else {
    d.setFont(u8g2_font_5x7_tr);
    centerStr(d, 61, "HOLD SELECT: QUIT");
  }
  if (gSelHold && gSelHoldUs > 0) drawQuitBar(d);
}

void renderCountdown(U8G2& d) {
  const uint32_t phase = gStateUs / kCountStepUs;      // 0,1,2
  const char* txt = (phase == 0) ? "3" : (phase == 1) ? "2" : "1";
  d.setDrawColor(0);
  box(d, 52, 26, 24, 28);
  d.setDrawColor(1);
  d.drawFrame(52, 26, 24, 28);
  d.setFont(u8g2_font_logisoso24_tn);
  d.setFontMode(1);
  const int w = (int)d.getStrWidth(txt);
  d.drawStr((u8g2_uint_t)(64 - w / 2), 50, txt);
}

void renderGo(U8G2& d) {
  d.setDrawColor(0);
  box(d, 50, 28, 28, 16);
  d.setDrawColor(1);
  d.drawFrame(50, 28, 28, 16);
  d.setFont(u8g2_font_helvB10_tr);
  d.setFontMode(1);
  const int w = (int)d.getStrWidth("GO!");
  d.drawStr((u8g2_uint_t)(64 - w / 2), 41, "GO!");
}

void menuItem(U8G2& d, int x, int y, const char* s, bool sel) {
  const int w = (int)d.getStrWidth(s);
  if (sel) {
    d.drawBox((u8g2_uint_t)(x - 3), (u8g2_uint_t)(y - 9),
              (u8g2_uint_t)(w + 6), 11);
    d.setDrawColor(0);
    d.drawStr((u8g2_uint_t)x, (u8g2_uint_t)y, s);
    d.setDrawColor(1);
  } else {
    d.drawStr((u8g2_uint_t)x, (u8g2_uint_t)y, s);
  }
}

void renderResult(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(0);
  d.setFont(u8g2_font_7x13B_tr);
  centerStr(d, 15, "FINISH!");
  char tbuf[16];
  char buf[28];
  d.setFont(u8g2_font_6x13_tr);
  formatTime(tbuf, gFinalTimeUs);
  snprintf(buf, sizeof(buf), "TIME %s", tbuf);
  d.drawStr(4, 31, buf);
  if (gBestTimeUs != 0) formatTime(tbuf, gBestTimeUs);
  else                  snprintf(tbuf, sizeof(tbuf), "--:--.-");
  snprintf(buf, sizeof(buf), "BEST %s", tbuf);
  d.drawStr(4, 44, buf);
  if (gNewBest) {
    d.setFont(u8g2_font_5x7_tr);
    d.drawStr(96, 44, "NEW!");
    d.setFont(u8g2_font_6x13_tr);
  }
  menuItem(d, 8, 57, "RETRY", gMenuSel == 0);
  menuItem(d, 76, 57, "QUIT", gMenuSel == 1);
}

void gameRender(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(0);
  switch (gState) {
    case ST_TITLE:
      renderTitle(d);
      break;
    case ST_COUNT:
      renderWorld(d);
      renderHud(d);
      renderCountdown(d);
      if (gSelHoldUs > 0) drawQuitBar(d);
      break;
    case ST_PLAY:
      renderWorld(d);
      renderHud(d);
      if (gRaceTimeUs < kGoShowUs) renderGo(d);
      if (gSelHoldUs > 0) drawQuitBar(d);
      break;
    case ST_RESULT:
      renderResult(d);
      break;
  }
}

}  // namespace

FB_GAME(F1Racing, gameInit, gameUpdate, gameRender, nullptr);
