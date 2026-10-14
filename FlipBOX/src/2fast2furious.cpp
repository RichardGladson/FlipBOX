// src/2fast2furious.cpp  --  2 Fast 2 Furious  (flipBOX)
//
// Pseudo-3D drift racer for the 128x64 monochrome OLED.
//   UP           : throttle (holds / feeds a slide)
//   DOWN         : brake (also helps the tyres bite again)
//   LEFT / RIGHT : steer -- and COUNTERSTEER (steering against the slide)
//   X            : handbrake (D4 / GPIO4): press = yaw kick, hold = rear grip cut
//   Hold SELECT ~1 s (title, countdown or run) to quit to the launcher;
//   the results screen also offers an explicit RETRY / QUIT menu.
//
// Drift model (all Q16.16 fixed-point, integrated with Frame::dtUs):
//   phi   the car's PATH: travel direction relative to the road  [deg]
//   beta  SLIP ANGLE: nose relative to travel direction          [deg]  (the drift)
//   Both are first-order and anchored, so with no keys held on a straight the
//   car straightens out by itself (nothing integrates away):
//     phi'  = kPath*(phiTarget - phi) - roadTurn*(1 - assist*grip) + kTurnIn*(1 - grip)*beta
//             phiTarget = lock * steer * grip     (grip steering = steady sideways speed)
//     beta' = -kAlign*grip*beta + drive
//             drive = handbrake / power-over tail push + steering into (or against)
//                     an established slide - line balance; tapered to zero near 75 deg
//   Rear grip is cut by the handbrake, by throttle + full lock at speed, and
//   (feedback) by a big slip angle -- which is what keeps a slide alive. Lifting
//   off, braking and countersteering give grip back and straighten the car.
//
// Scoring: points accrue while |beta| is in the drift window, weighted by
// speed, an angle bell (best ~30-45 deg, zero past ~75 deg) and smoothness.
// A chain survives a short grace window, so left<->right transitions link
// instead of resetting; the points bank when the chain ends. Spinning out or
// hitting the barrier forfeits the unbanked points.
//
// The road is the classic OutRun-style scanline renderer from the F1 game: one
// precomputed perspective table, curvature integrated twice per frame, and a
// track map rasterised once in init() from the same curvature table.
//
// Exports fbGame_TwoFast2Furious (add one line to FB_GAME_LIST in main.cpp).

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
#include <math.h>    // sinf/cosf: used ONLY in init() (track map, sine table)

namespace {

// ---------------------------------------------------------------------------
// Fixed-point helpers (Q16.16).  Q() turns a readable decimal into Q16.16 at
// compile time, so every tuning constant below can be edited as a plain number.
// Angles are Q16.16 DEGREES, speeds Q16.16 km/h, rates are per second.
// (>> on negative values is an arithmetic shift on GCC/Xtensa.)
// ---------------------------------------------------------------------------
constexpr int32_t Q(float v) { return (int32_t)(v * 65536.0f + (v < 0.0f ? -0.5f : 0.5f)); }
inline int32_t mulQ16(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 16); }
inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int32_t absI32(int32_t v) { return v < 0 ? -v : v; }
// Exponential-approach factor for "x += (target - x) * rate * dt"; clamped to
// 1.0 so a long frame can never overshoot the target (frame-rate safe).
// Clamped ratio num/den as Q16.16 in [0,1] (64-bit intermediate; multiply, not
// shift, so negative numerators are well-defined).
inline int32_t ratioQ16(int32_t num, int32_t den) {
  return clampI32((int32_t)(((int64_t)num * 65536) / den), 0, 65536);
}
inline int32_t stepFactor(int32_t rateQ, int32_t dt16) { return clampI32(mulQ16(rateQ, dt16), 0, 65536); }
inline int32_t approach(int32_t cur, int32_t tgt, int32_t rateQ, int32_t dt16) {
  return cur + mulQ16(tgt - cur, stepFactor(rateQ, dt16));
}

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

// -- speed --
constexpr int32_t kTopSpeedQ   = Q(190.0f);    // km/h overall top speed
constexpr int32_t kAccelQ      = Q(42.0f);     // km/h/s at standstill (UP)
constexpr int32_t kAccelTaperQ = Q(0.72f);     // accel *= 1 - taper * v/top
constexpr int32_t kBrakeQ      = Q(130.0f);    // km/h/s while DOWN is held
constexpr int32_t kDragBaseQ   = Q(9.0f);      // km/h/s coasting (rolling resistance)
constexpr int32_t kDragAeroQ   = Q(55.0f);     // km/h/s extra coasting drag at top speed (grows with v^2)
constexpr int32_t kScrubStartQ = Q(6.0f);      // deg of slip before tyres scrub speed
constexpr int32_t kScrubPerDegQ= Q(0.50f);     // km/h/s scrubbed per degree of slip beyond that
constexpr int32_t kOffDecelQ   = Q(300.0f);    // km/h/s extra deceleration off-road
constexpr int32_t kOffMaxQ     = Q(90.0f);     // km/h off-road speed cap
constexpr int32_t kKmhToMsQ    = 18204;        // 1/3.6 in Q16.16

// -- road geometry (physics <-> renderer) --
constexpr int     kRoadHalfM   = 8;            // metres from road centre to edge (power of two: cheap divide)
constexpr int32_t kRoadTurnQ   = Q(0.0125f);   // deg of road heading change per (curve unit * metre); matches the map
constexpr int32_t kPlayerXMaxQ = Q(1.85f);     // soft barrier (road-widths from centre)
constexpr int32_t kOffEdgeQ    = 65536;        // |x| beyond this = off the tarmac
constexpr int32_t kWallKeepQ   = Q(0.60f);     // fraction of speed kept after a barrier hit
constexpr uint32_t kWallCoolUs = 500000UL;     // minimum time between barrier hits

// -- steering input and PATH (travel direction relative to the road) --
// Grip steering sets a TARGET path angle, so a held key gives a steady sideways
// speed (like the original game) and releasing it lets the car straighten.
// Sensitivity retune (v2): roughly 2x lock, 3x path chase, faster steer ramp,
// less fade at speed so the car answers the wheel immediately.
constexpr int32_t kSteerRiseQ  = Q(18.0f);     // 1/s   steering position builds toward +-1
constexpr int32_t kSteerFallQ  = Q(16.0f);     // 1/s   and returns to centre
constexpr int32_t kPhiLockQ    = Q(28.0f);     // deg   path angle at full lock (low speed, full grip)
constexpr int32_t kPhiFadeQ    = Q(0.25f);     // lock lost at top speed (less fade)
constexpr int32_t kPathRateQ   = Q(9.0f);      // 1/s   path angle chases its target (anchors it to the road)
constexpr int32_t kRoadAssistQ = Q(0.55f);     // share of the road's turning that grip cancels for you
constexpr int32_t kSlideTurnQ  = Q(0.90f);     // 1/s   a sliding car's path turns toward its nose (x lost grip)
constexpr int32_t kPhMaxQ      = Q(75.0f);     // deg   path-angle clamp (keeps cos > 0)

// -- SLIP ANGLE beta (nose relative to travel direction): the drift itself --
// Sensitivity retune: higher drive, lower gate (authority from first degree),
// stronger countersteer / line balance / taper so higher gain stays controllable.
constexpr int32_t kAlignQ      = Q(5.0f);      // 1/s   tyres realign the nose (x grip): slow when sliding = momentum
constexpr int32_t kAlignRecQ   = Q(10.0f);     // 1/s   spin recovery realignment
constexpr int32_t kSlipSteerQ  = Q(52.0f);     // deg/s steering into an established slide holds / widens it
constexpr int32_t kCounterQ    = Q(2.2f);      // countersteer authority multiplier
constexpr int32_t kGateLoQ     = Q(0.0f);      // deg   steering works from the first degree of slip
constexpr int32_t kGateSpanQ   = Q(6.0f);      // deg   ... fully from lo+span
constexpr int32_t kGateFloorQ  = Q(0.55f);     // steering authority floor on a car that has lost grip (see stepCar)
constexpr int32_t kLineBalQ    = Q(18.0f);     // deg/s of slip trimmed per deg of path error while sliding (holds the line)
constexpr int32_t kPowDriveQ   = Q(55.0f);     // deg/s power-over (throttle + lock at speed) pushes the tail out
constexpr int32_t kTaperEndQ   = Q(75.0f);     // deg   drives taper to zero here so a slide can't run away ...
constexpr int32_t kTaperSpanQ  = Q(35.0f);     // deg   ... starting this far below it
constexpr int32_t kBetaMaxQ    = Q(85.0f);     // deg   slip-angle clamp

// -- rear grip (1.0 = full) --
constexpr int32_t kGripMinQ    = Q(0.16f);
constexpr int32_t kGripHbQ     = Q(0.70f);     // grip removed by a fully applied handbrake
constexpr int32_t kGripPowerQ  = Q(0.55f);     // grip removed by throttle + full lock at high speed
constexpr int32_t kPowerV0Q    = Q(45.0f);     // km/h power-over starts (lower threshold)
constexpr int32_t kPowerV1Q    = Q(100.0f);    // km/h power-over fully developed
constexpr int32_t kPowerStQ    = Q(0.35f);     // steering position above which power-over bites
constexpr int32_t kGripSlideQ  = Q(0.40f);     // grip removed by a large slip angle (keeps a slide alive)
constexpr int32_t kSlideStartQ = Q(12.0f);     // deg
constexpr int32_t kSlideSpanQ  = Q(28.0f);     // deg over which the slide feedback builds
constexpr int32_t kGripLiftQ   = Q(0.22f);     // grip returned when the throttle is released
constexpr int32_t kGripBrakeQ  = Q(0.18f);     // extra grip while braking
constexpr int32_t kGripFallQ   = Q(14.0f);     // 1/s   grip is lost quickly ...
constexpr int32_t kGripRiseQ   = Q(3.0f);      // 1/s   ... and regained slowly (momentum)

// -- handbrake (X) --
constexpr int32_t  kHbRiseQ     = Q(16.0f);    // 1/s
constexpr int32_t  kHbFallQ     = Q(5.0f);     // 1/s
constexpr uint32_t kHbMaxUs     = 700000UL;    // longest effective hold; release and press again for more
constexpr int32_t  kHbMinSpeedQ = Q(40.0f);    // km/h below which the handbrake does nothing
constexpr int32_t  kHbKickQ     = Q(22.0f);    // deg slip-angle kick on press (toward steering / corner)
constexpr int32_t  kHbDriveQ    = Q(130.0f);   // deg/s tail push while held (x handbrake level)
constexpr int32_t  kHbScrubQ    = Q(28.0f);    // km/h/s scrubbed at full handbrake

// -- spin-out / recovery --
constexpr int32_t  kSpinDegQ    = Q(78.0f);    // |slip| above this ...
constexpr uint32_t kSpinUs      = 350000UL;    // ... for this long = spin-out
constexpr uint32_t kRecoverUs   = 800000UL;    // assisted recovery time
constexpr int32_t  kRecoverAccelQ = Q(0.30f);  // throttle strength while recovering
constexpr int32_t  kRecoverDragQ  = Q(80.0f);  // km/h/s extra drag while recovering

// -- drift scoring --
constexpr int32_t  kDriftOnQ    = Q(13.0f);    // deg  slip angle that starts a drift
constexpr int32_t  kDriftOffQ   = Q(8.0f);     // deg  slip angle that ends it (hysteresis)
constexpr int32_t  kDriftMinSpeedQ = Q(45.0f); // km/h
constexpr uint32_t kDriftValidUs= 250000UL;    // slide must persist this long to count
constexpr uint32_t kGraceUs     = 1500000UL;   // chain survives this long without a drift
constexpr uint32_t kMultStepUs  = 1600000UL;   // sustained drift time per multiplier step
constexpr uint32_t kMultMax     = 6;
constexpr int32_t  kAfLoQ       = Q(13.0f);    // angle bell: 0 at lo, 1 at midLo ...
constexpr int32_t  kAfMidLoQ    = Q(30.0f);
constexpr int32_t  kAfMidHiQ    = Q(46.0f);    // ... 1 until midHi, 0 again at hi
constexpr int32_t  kAfHiQ       = Q(75.0f);
constexpr int32_t  kScoreRateQ  = Q(0.90f);    // points/s per km/h at full angle factor, x1 multiplier
constexpr int32_t  kRateSmoothQ = Q(10.0f);    // 1/s  smoothing of the slip-rate (smoothness) measure
constexpr int32_t  kCtlCalmQ    = Q(35.0f);    // deg/s slip change that still counts as smooth
constexpr int32_t  kCtlWildQ    = Q(120.0f);   // deg/s slip change that earns the minimum
constexpr int32_t  kCtlMinQ     = Q(0.45f);    // minimum smoothness factor
constexpr uint32_t kLinkBonus   = 150;         // points x multiplier for a left<->right link
constexpr uint32_t kPendCapQ8   = 99999UL * 256UL;   // pending pool cap (Q24.8 points)
constexpr uint32_t kScoreCap    = 999999UL;

// -- camera --
// Stronger horizon swing so drifts read clearly; lower follow so the car's
// own yaw (sprite) dominates the visual angle.
constexpr int32_t kCamFollowQ   = Q(0.28f);    // share of the car's yaw the view follows
constexpr int32_t kCamMaxQ      = Q(50.0f);    // deg
constexpr int32_t kCamRateQ     = Q(10.0f);    // 1/s
constexpr int32_t kShearDiv     = kRoadHalfM << 16;   // projection: px per metre = hw / kRoadHalfM

// -- visual road scroll (stripes / dashes only; physics & map use real distance) --
constexpr int32_t kScrollMulQ   = Q(3.0f);     // 3x visual scroll rate

// -- pseudo-3D projection --
constexpr int kHorizonY = 24;                            // y of the horizon line
constexpr int kRowsN    = fb::SCREEN_H - 1 - kHorizonY;  // road scanlines (39: y = 25..63)
constexpr int kProjC    = 250;                           // z[metres] = kProjC / (y - kHorizonY)
constexpr int kRoadWNum = 98;                            // half-width px = (y-kHorizonY) * kRoadWNum/64
constexpr int kCurveDiv = 58000;                         // curve accumulator -> px (smaller = sharper bends)

// -- track --
constexpr int kCurveShift  = 3;                // curve table resolution: one entry per 8 m
constexpr int kCurveRampM  = 40;               // curvature blend-in length at segment starts
constexpr int kBandShift   = 4;                // ground stripe period: 16 m
constexpr int kDashShift   = 3;                // centre dashes: 8 m on / 8 m off

// -- flow / HUD --
constexpr uint32_t kMaxStepUs   = 50000UL;     // clamp for dt (the launcher clamps too)
constexpr uint32_t kSplitStepUs = 25000UL;     // frames longer than this are integrated in 2 sub-steps
constexpr uint32_t kCountStepUs = 800000UL;    // countdown step (3, 2, 1)
constexpr uint32_t kGoShowUs    = 900000UL;    // "GO!" overlay time once running
constexpr uint32_t kQuitHoldUs  = 1000000UL;   // SELECT hold time that quits to the launcher
constexpr uint32_t kMsgUs       = 800000UL;    // centre message time
constexpr uint32_t kHbFlashUs   = 220000UL;    // handbrake ring effect

// ---------------------------------------------------------------------------
// Track: a hand-designed ~3.9 km drift course (clockwise): long sweepers for
// sustained slides, a left-right S, an esses chicane, two hairpins (brake +
// handbrake) and short straights to rebuild speed. Edit freely: the lap
// length, the table size and the track map all derive from this list.
// curve: +right / -left, -127..127.  ~35-50 sweeper, ~60-65 medium, ~118 hairpin.
// ---------------------------------------------------------------------------
struct TrackSeg { int16_t len; int8_t curve; };

constexpr TrackSeg kSegs[] = {
  { 360,    0 },   // start/finish straight
  { 240,   42 },   // T1  long right sweeper
  { 110,    0 },
  { 120,  -55 },   // T2  medium left
  {  35,    0 },
  { 130,   60 },   // T3  medium right (S with T2)
  { 160,    0 },
  { 100,  118 },   // T4  HAIRPIN right -- brake, handbrake, countersteer
  { 420,    0 },   // back straight
  { 280,  -36 },   // T5  long left sweeper
  {  70,    0 },
  {  90,   65 },   // T6  esses: right
  {  28,    0 },
  {  90,  -65 },   // T7  esses: left
  {  28,    0 },
  {  90,   65 },   // T8  esses: right
  {  28,    0 },
  {  90,  -65 },   // T9  esses: left
  { 140,    0 },
  { 100, -118 },   // T10 HAIRPIN left
  { 200,    0 },
  { 330,   48 },   // T11 long right sweeper
  { 150,    0 },
  { 180,   58 },   // T12 medium-long right
  { 330,    0 },   // run back to the line
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
constexpr float kMapRadPerUnit = 0.000218f;        // heading change per (curve unit * metre) = kRoadTurnQ in radians

// ---------------------------------------------------------------------------
// Car sprite: R34 Skyline GT-R, pre-rasterised at init into monochrome bitmaps
// at several yaw angles. Left/right are mirrored at init so runtime is two
// XBM draws (silhouette then body). Smoke / rings use the stored rear-wheel
// pixel positions for the current angle.
// ---------------------------------------------------------------------------
constexpr int kCarCX  = fb::SCREEN_W / 2;
constexpr int kCarTop = 46;
constexpr int kCarAngN = 9;                    // 0, 7.5, 15, ... 60 deg
constexpr int kCarBmpW = 40;                   // bitmap width (even)
constexpr int kCarBmpH = 18;                   // bitmap height
constexpr int kCarBmpStride = (kCarBmpW + 7) >> 3;
constexpr int kCarBmpBytes = kCarBmpStride * kCarBmpH;
// Angles in degrees for the table (yaw of nose relative to camera).
constexpr int8_t kCarAngDeg[kCarAngN] = { 0, 8, 15, 22, 30, 38, 45, 52, 60 };

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
int16_t gSinTbl[91];               // sin(0..90 deg) in Q15, built in init

uint8_t gMapBits[kMapStride * kMapIH];   // XBM of the circuit outline (built in init)
uint8_t gMapPx[kCurveTblLen];            // map pixel of each 8 m track cell
uint8_t gMapPy[kCurveTblLen];

struct Puff { int32_t x, y; int32_t life; int8_t vx, vy; };   // x,y in 1/256 px; life in us
constexpr int kPuffN = 10;
constexpr uint32_t kPuffIntervalUs = 45000UL;
Puff     gPuffs[kPuffN];
uint8_t  gPuffNext;
uint32_t gPuffAccUs;
uint32_t gRng;

State    gState;
// vehicle
int32_t  gSpeedQ;        // Q16.16 km/h
int32_t  gPlayerXQ;      // Q16.16 road-widths: 0 = centre, +/-1.0 = road edge (+ = right)
int32_t  gDistQ;         // Q16.16 metres of road progress this run
int32_t  gScrollDistQ;   // Q16.16 metres of visual scroll (3x, for stripes/dashes only)
int32_t  gPhQ;           // Q16.16 deg  travel direction vs road (the car's path)
int32_t  gBetaQ;         // Q16.16 deg  slip angle: nose relative to travel (+ = nose right) -- the drift angle
int32_t  gStQ;           // Q16.16 steering position -1..1
int32_t  gHbQ;           // Q16.16 handbrake level 0..1
int32_t  gGripQ;         // Q16.16 rear grip 0..1
int32_t  gCamQ;          // Q16.16 deg smoothed camera yaw
int32_t  gShearQ;        // Q16.16 per-metre road shear derived from gCamQ
int32_t  gLeanQ;         // Q16.16 px cosmetic body roll
uint32_t gHbHeldUs;
uint32_t gHbFlashUs;
uint32_t gSpinUs;
uint32_t gRecoverUs;
uint32_t gWallCoolUs;
bool     gWallHit;
bool     gOffroad;
bool     gSlipFx;        // tyres smoking / marking
uint8_t  gCarAng;        // 0..kCarAngN-1 sprite angle index
int8_t   gCarDir;        // +1 nose right, -1 nose left
bool     gBraking;       // DOWN held -> brighter brake lamps
// Pre-rendered car bitmaps: [angle][0=sil / 1=body], mirrored copies for left.
uint8_t  gCarSilR[kCarAngN][kCarBmpBytes];
uint8_t  gCarBodyR[kCarAngN][kCarBmpBytes];
uint8_t  gCarSilL[kCarAngN][kCarBmpBytes];
uint8_t  gCarBodyL[kCarAngN][kCarBmpBytes];
// Rear-wheel centres in bitmap space (for smoke / rings), right-side poses.
int8_t   gCarWheelX[kCarAngN][2];   // [ang][0=far, 1=near]
int8_t   gCarWheelY[kCarAngN];
// scoring
int32_t  gPrevBetaQ;
int32_t  gBetaRateQ;     // smoothed |d beta/dt|, deg/s
bool     gDrifting;
bool     gChain;
int8_t   gLastDir;
uint32_t gDriftHoldUs;
uint32_t gGraceUs;
uint32_t gMult;
uint32_t gMultUs;
uint32_t gPendQ8;        // unbanked points, Q24.8
uint32_t gScore;
uint32_t gBestChain;
uint32_t gMaxMult;
uint32_t gMaxAngle;      // deg
// flow
uint32_t gRunUs;
uint32_t gStateUs;
uint32_t gAnimMs;        // cosmetic clock (blink / jitter)
uint32_t gSelHoldUs;     // SELECT hold timer for quit
bool     gSelHold;       // TITLE: a SELECT press is currently in progress
uint8_t  gMapDotX;       // player marker, inner-bitmap coordinates
uint8_t  gMapDotY;
uint8_t  gMenuSel;       // RESULT cursor: 0 = RETRY, 1 = QUIT
bool     gNewBest;
uint32_t gMsgUs;
uint8_t  gBarPx;         // combo bar fill 0..kBarW
bool     gBarBlink;

// Session-persistent best score: intentionally NOT reset by gameInit() so it
// survives leaving and re-entering the game (cleared only at power cycle).
uint32_t gBestScore = 0;

// Cached HUD strings (re-formatted only when the value changes).
constexpr int kBarW = 19;
char     gSpeedStr[12];
int      gSpeedCache;
char     gScoreStr[12];
uint32_t gScoreCache;
char     gPendStr[14];
int32_t  gPendCache;
char     gMultStr[2];                    // single digit 1..kMultMax
uint32_t gMultCache;
char     gAngStr[12];
int      gAngCache;
char     gMsgStr[24];

// ---------------------------------------------------------------------------
// Small math helpers
// ---------------------------------------------------------------------------
uint32_t rnd() {                       // xorshift32
  uint32_t x = gRng;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  gRng = x;
  return x;
}

// sin of an angle in Q16.16 degrees (|angle| <= 90) -> Q16.16, table + lerp.
int32_t sinDegQ(int32_t degQ) {
  const int32_t a = absI32(degQ);
  const int32_t i = a >> 16;
  int32_t s;
  if (i >= 90) {
    s = 65535;
  } else {
    const int32_t fr = a & 0xFFFF;
    const int32_t s0 = gSinTbl[i];
    const int32_t s1 = gSinTbl[i + 1];
    s = (s0 + (((s1 - s0) * fr) >> 16)) * 2;     // Q15 -> Q16
  }
  return degQ < 0 ? -s : s;
}
inline int32_t cosDegQ(int32_t degQ) {
  int32_t r = (90 << 16) - absI32(degQ);
  if (r < 0) r = 0;
  return sinDegQ(r);
}

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
// perspective-scaled result is the on-screen centre shift. The drift camera
// adds one more term: the view follows part of the car's yaw, so the road
// swings sideways in proportion to distance (shear = z * tan(yaw) * px/metre).
void updateProjection() {
  const int32_t posMod = (gDistQ >> 16) % kTrackLen;
  // Visual scroll uses a separate 3x counter so stripes/dashes race while
  // curvature, lap distance and the start line stay on real distance.
  const int32_t scrollMod = (gScrollDistQ >> 16) % kTrackLen;
  int32_t cv = 0;
  int32_t cx = 0;
  for (int i = kRowsN - 1; i >= 0; --i) {        // bottom (near) -> horizon (far)
    const int32_t z  = gRowZ[i];
    const int32_t dz = (i == kRowsN - 1) ? z : (z - gRowZ[i + 1]);
    int32_t dist = posMod + z;
    if (dist >= kTrackLen) dist -= kTrackLen;
    int32_t sdist = scrollMod + z;
    if (sdist >= kTrackLen) sdist -= kTrackLen;
    const int32_t c = gCurveTbl[dist >> kCurveShift];
    cv += c * dz;
    cx += cv * dz;
    const int32_t hw = gRowHW[i];
    const int32_t offPx     = (cx * hw) / kCurveDiv;          // bend displacement
    const int32_t playerOff = (gPlayerXQ * hw) >> 16;         // road slides under the car
    const int32_t shearPx   = (gShearQ * z * hw) / kShearDiv; // camera yaw (<= ~4.5e8: fits int32)
    int32_t sx = (int32_t)(fb::SCREEN_W / 2) - playerOff + offPx - shearPx;
    sx = clampI32(sx, -80, (int32_t)fb::SCREEN_W + 80);       // drawing clamps again
    gRowCX[i] = (int16_t)sx;
    uint8_t fl = 0;
    if ((sdist >> kBandShift) & 1)          fl |= kFlagBand;
    if (((sdist >> kDashShift) & 1) && hw >= 3) fl |= kFlagDash;
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
// HUD data (formatted in update(), only when a value changes)
// ---------------------------------------------------------------------------
void showMsg(const char* s) {
  snprintf(gMsgStr, sizeof(gMsgStr), "%s", s);
  gMsgUs = kMsgUs;
}

void updateHudStrings() {
  const int kmh = (int)(gSpeedQ >> 16);
  if (kmh != gSpeedCache) {
    gSpeedCache = kmh;
    snprintf(gSpeedStr, sizeof(gSpeedStr), "%d", kmh);
  }
  if (gScore != gScoreCache) {
    gScoreCache = gScore;
    snprintf(gScoreStr, sizeof(gScoreStr), "%lu", (unsigned long)gScore);
  }
  const int32_t pend = (int32_t)(gPendQ8 >> 8);
  if (pend != gPendCache) {
    gPendCache = pend;
    snprintf(gPendStr, sizeof(gPendStr), "+%ld", (long)pend);
  }
  if (gMult != gMultCache) {
    gMultCache = gMult;
    gMultStr[0] = (char)('0' + (int)(gMult > 9 ? 9 : gMult));
    gMultStr[1] = '\0';
  }
  const int ang = (int)(absI32(gBetaQ) >> 16);
  if (ang != gAngCache) {
    gAngCache = ang;
    snprintf(gAngStr, sizeof(gAngStr), "%d", ang);
  }
  if (gDrifting) {
    gBarPx = (gMult >= kMultMax) ? (uint8_t)kBarW : (uint8_t)((gMultUs * (uint32_t)kBarW) / kMultStepUs);
    gBarBlink = false;
  } else if (gChain) {
    gBarPx = (uint8_t)((gGraceUs * (uint32_t)kBarW) / kGraceUs);
    gBarBlink = gGraceUs < (kGraceUs * 35UL) / 100UL;
  } else {
    gBarPx = 0;
    gBarBlink = false;
  }
}

// ---------------------------------------------------------------------------
// Smoke / dust puffs: tiny fixed pool, deterministic PRNG, no allocation.
// ---------------------------------------------------------------------------
void spawnPuff(int x, int y, int dir) {
  Puff& p = gPuffs[gPuffNext];
  gPuffNext = (uint8_t)((gPuffNext + 1) % kPuffN);
  p.x = (int32_t)x * 256;
  p.y = (int32_t)y * 256;
  p.vx = (int8_t)(dir * (int)(6 + (rnd() & 15)));      // px/s, outward
  p.vy = (int8_t)((int)(rnd() & 15) - 12);             // px/s, mostly rising
  p.life = (int32_t)(260000UL + (rnd() & 0x3FFFFUL));  // 260..520 ms
}

// Screen positions of the two rear tyre centres for the current sprite pose.
inline void tyreXs(int& farX, int& nearX) {
  const int ang = (int)gCarAng;
  const int dir = gCarDir >= 0 ? 1 : -1;
  // Bitmaps are centred on kCarBmpW/2; wheel coords are relative to bitmap origin.
  const int baseX = kCarCX - kCarBmpW / 2;
  const int baseY = kCarTop;
  if (dir >= 0) {
    farX  = baseX + gCarWheelX[ang][0];
    nearX = baseX + gCarWheelX[ang][1];
  } else {
    // Mirrored: X flips around bitmap centre
    farX  = baseX + (kCarBmpW - 1 - gCarWheelX[ang][0]);
    nearX = baseX + (kCarBmpW - 1 - gCarWheelX[ang][1]);
  }
  (void)baseY;  // Y used only for rings / smoke Y
}
inline int tyreY() {
  return kCarTop + gCarWheelY[gCarAng];
}

void updatePuffs(uint32_t dtUs) {
  for (int i = 0; i < kPuffN; ++i) {
    Puff& p = gPuffs[i];
    if (p.life <= 0) continue;
    p.life -= (int32_t)dtUs;
    p.x += ((int32_t)p.vx * (int32_t)dtUs) / 3906;       // px/s -> 1/256 px per dt
    p.y += ((int32_t)p.vy * (int32_t)dtUs) / 3906;
  }
  if (gSlipFx) {
    gPuffAccUs += dtUs;
    int guard = 3;
    while (gPuffAccUs >= kPuffIntervalUs && guard-- > 0) {
      gPuffAccUs -= kPuffIntervalUs;
      int fx, nx;
      tyreXs(fx, nx);
      const int dir = gCarDir >= 0 ? 1 : -1;
      const int ty = tyreY();
      spawnPuff(fx, ty, -dir);
      spawnPuff(nx, ty, dir);
    }
    if (gPuffAccUs > kPuffIntervalUs) gPuffAccUs = kPuffIntervalUs;
  } else {
    gPuffAccUs = 0;
  }
}

// ---------------------------------------------------------------------------
// Scoring
// ---------------------------------------------------------------------------
void bankChain() {
  const uint32_t pts = gPendQ8 >> 8;
  gScore += pts;
  if (gScore > kScoreCap) gScore = kScoreCap;
  if (pts > gBestChain) gBestChain = pts;
  gPendQ8 = 0;
  gChain  = false;
  gMult   = 1;
  gMultUs = 0;
  gGraceUs = 0;
}

void breakChain(const char* why) {         // forfeits the unbanked points
  gPendQ8 = 0;
  gChain  = false;
  gMult   = 1;
  gMultUs = 0;
  gGraceUs = 0;
  gDrifting = false;
  gDriftHoldUs = 0;
  showMsg(why);
}

// Angle bell: 0 below kAfLo, ramps to 1 at kAfMidLo, flat to kAfMidHi, back to
// 0 at kAfHi. Over-rotating therefore never outscores a clean, held slide.
int32_t angleFactor(int32_t a) {
  if (a <= kAfLoQ || a >= kAfHiQ) return 0;
  if (a < kAfMidLoQ) return (int32_t)(((int64_t)(a - kAfLoQ) << 16) / (kAfMidLoQ - kAfLoQ));
  if (a <= kAfMidHiQ) return 65536;
  return (int32_t)(((int64_t)(kAfHiQ - a) << 16) / (kAfHiQ - kAfMidHiQ));
}

// Smoothness: 1.0 while the slip angle changes calmly, down to kCtlMinQ when
// it is thrashing about.
int32_t smoothFactor(int32_t rate) {
  if (rate <= kCtlCalmQ) return 65536;
  if (rate >= kCtlWildQ) return kCtlMinQ;
  return 65536 - (int32_t)(((int64_t)(rate - kCtlCalmQ) * (65536 - kCtlMinQ)) / (kCtlWildQ - kCtlCalmQ));
}

void updateScoring(uint32_t dtUs, int32_t dt16) {
  const int32_t betaQ = gBetaQ;
  const int32_t absB  = absI32(betaQ);

  if (dt16 > 0) {                                      // smoothness measure
    int64_t r = ((int64_t)absI32(betaQ - gPrevBetaQ) << 16) / dt16;
    if (r > (int64_t)Q(2000.0f)) r = Q(2000.0f);
    gBetaRateQ = approach(gBetaRateQ, (int32_t)r, kRateSmoothQ, dt16);
  }
  gPrevBetaQ = betaQ;

  // spin-out: slip beyond the limit for too long
  if (gRecoverUs == 0) {
    if (absB > kSpinDegQ) {
      gSpinUs += dtUs;
      if (gSpinUs >= kSpinUs) {
        gSpinUs = 0;
        gRecoverUs = kRecoverUs;
        breakChain("SPIN OUT");
      }
    } else {
      gSpinUs = (gSpinUs > dtUs) ? (gSpinUs - dtUs) : 0;
    }
  }
  if (gWallHit) {
    gWallHit = false;
    breakChain("WALL HIT");
  }

  // drift detection with hysteresis (on at 13 deg, off at 8) and a short
  // validation time, so numerical flicker never toggles it
  const bool inHold = gDriftHoldUs > 0;
  const bool cand = !gOffroad && gRecoverUs == 0 && gSpeedQ >= kDriftMinSpeedQ &&
                    absB >= (inHold ? kDriftOffQ : kDriftOnQ);
  if (cand) {
    if (gDriftHoldUs < kDriftValidUs) gDriftHoldUs += (dtUs > 0 ? dtUs : 1);
  } else {
    gDriftHoldUs = 0;
  }
  const bool valid = cand && gDriftHoldUs >= kDriftValidUs;

  if (valid && !gDrifting) {                           // a drift (re)starts
    const int8_t dir = betaQ > 0 ? 1 : -1;
    if (gChain) {
      if (dir != gLastDir) {                           // left<->right link
        if (gMult < kMultMax) ++gMult;
        if (gMult > gMaxMult) gMaxMult = gMult;
        gMultUs = 0;
        gPendQ8 += (kLinkBonus * gMult) << 8;
        if (gPendQ8 > kPendCapQ8) gPendQ8 = kPendCapQ8;
        char b[24];
        snprintf(b, sizeof(b), "LINK x%lu", (unsigned long)gMult);
        showMsg(b);
      }
    } else {
      gChain = true;
      gMult = 1;
      gMultUs = 0;
    }
    gLastDir = dir;
  }
  gDrifting = valid;

  if (gDrifting) {
    // points/s = speed * angle bell * smoothness * multiplier   (Q16 math)
    int32_t x = mulQ16(gSpeedQ, kScoreRateQ);
    x = mulQ16(x, angleFactor(absB));
    x = mulQ16(x, smoothFactor(gBetaRateQ));
    const int64_t inc = (int64_t)mulQ16((int32_t)((int64_t)x * (int64_t)gMult), dt16) >> 8;   // Q16 pts -> Q8
    gPendQ8 += (uint32_t)inc;
    if (gPendQ8 > kPendCapQ8) gPendQ8 = kPendCapQ8;
    gMultUs += dtUs;
    if (gMultUs >= kMultStepUs) {
      gMultUs = 0;
      if (gMult < kMultMax) ++gMult;
      if (gMult > gMaxMult) gMaxMult = gMult;
    }
    gGraceUs = kGraceUs;                               // chain stays alive while sliding
    const uint32_t deg = (uint32_t)(absB >> 16);
    if (deg > gMaxAngle) gMaxAngle = deg;
  } else if (gChain) {
    gGraceUs = (gGraceUs > dtUs) ? (gGraceUs - dtUs) : 0;
    if (gGraceUs == 0) bankChain();
  }
}

// ---------------------------------------------------------------------------
// Vehicle physics. One call integrates one (sub-)step of dt16 seconds.
//
// Two angles, both first-order and both anchored, so nothing can integrate
// away while no key is held:
//   phi  (gPhQ)   travel direction vs road. Sideways speed = v*sin(phi).
//                 phi' = kPath*(phiTarget - phi) - roadTurn*(1 - assist*grip)
//                        + kSlideTurn*(1 - grip)*beta
//                 phiTarget = lock * steer * grip: grip steering sets a path angle.
//   beta (gBetaQ) slip angle, nose vs travel (what you see and score).
//                 beta' = -kAlign*grip*beta + drive
//                 drive = tail push (handbrake, power-over)
//                       + steering into/against the slide (once beta > ~6 deg)
//                       - line balance (path heading inside the drift trims the angle)
//                 tapered to zero by ~75 deg so it can't run away.
// Low grip (handbrake, power-over, a big angle) makes the realignment term
// small, so beta persists; lifting off / braking restore grip and it decays.
// ---------------------------------------------------------------------------
void stepCar(int32_t dt16, bool thr, bool brk, int steerIn, bool hbOn, int32_t curveNow) {
  const bool rec = gRecoverUs > 0;
  const int32_t speedFrac = (int32_t)(((int64_t)gSpeedQ << 16) / kTopSpeedQ);   // v / top, Q16
  const int32_t vmsQ = mulQ16(gSpeedQ, kKmhToMsQ);                               // m/s

  // steering position and handbrake level ease toward their targets
  const int32_t stTgt = rec ? 0 : ((int32_t)steerIn * 65536);
  gStQ = approach(gStQ, stTgt, (steerIn == 0 || rec) ? kSteerFallQ : kSteerRiseQ, dt16);
  gHbQ = approach(gHbQ, hbOn ? 65536 : 0, hbOn ? kHbRiseQ : kHbFallQ, dt16);
  const int32_t absSt = absI32(gStQ);
  const int32_t beta  = gBetaQ;
  const int32_t absB  = absI32(beta);

  // --- speed: throttle / brake / coast, tyre scrub, handbrake drag ---
  int32_t v = gSpeedQ;
  if (brk) {
    v -= mulQ16(kBrakeQ, dt16);
  } else if (thr) {
    int32_t a = mulQ16(kAccelQ, 65536 - mulQ16(kAccelTaperQ, speedFrac));
    if (rec) a = mulQ16(a, kRecoverAccelQ);
    v += mulQ16(a, dt16);
  } else {
    v -= mulQ16(kDragBaseQ + mulQ16(kDragAeroQ, mulQ16(speedFrac, speedFrac)), dt16);
  }
  if (absB > kScrubStartQ) v -= mulQ16(mulQ16(kScrubPerDegQ, absB - kScrubStartQ), dt16);
  v -= mulQ16(mulQ16(kHbScrubQ, gHbQ), dt16);
  if (rec) v -= mulQ16(kRecoverDragQ, dt16);
  if (gOffroad) {
    if (v > kOffMaxQ) v -= mulQ16(kOffDecelQ, dt16);
    else              v -= mulQ16(kDragBaseQ * 2, dt16);
  }
  gSpeedQ = clampI32(v, 0, kTopSpeedQ);

  // --- road turn rate under the car, deg/s ---
  const int32_t rhoQ = (int32_t)((int64_t)curveNow * mulQ16(vmsQ, kRoadTurnQ));

  // --- rear grip target ---
  int32_t pw = 0;                                      // power-over level 0..1
  int32_t gT = 65536 - mulQ16(kGripHbQ, gHbQ);
  if (thr && !brk && !rec) {                           // throttle + lock at speed
    const int32_t vf = ratioQ16(gSpeedQ - kPowerV0Q, kPowerV1Q - kPowerV0Q);
    const int32_t sf = ratioQ16(absSt - kPowerStQ, 65536 - kPowerStQ);
    pw = mulQ16(vf, sf);
    gT -= mulQ16(kGripPowerQ, pw);
  }
  const int32_t sl = ratioQ16(absB - kSlideStartQ, kSlideSpanQ);
  gT -= mulQ16(kGripSlideQ, sl);                       // big angle -> less grip: the slide persists
  if (!thr) gT += kGripLiftQ;                          // lift off: tyres bite again
  if (brk)  gT += kGripBrakeQ;
  gT = rec ? 65536 : clampI32(gT, kGripMinQ, 65536);
  gGripQ = approach(gGripQ, gT, gT < gGripQ ? kGripFallQ : kGripRiseQ, dt16);

  // --- slip angle: tail push + steering into / against the slide ---
  int32_t dirE;                                        // side the tail is pushed toward
  if (steerIn != 0 && !rec) dirE = steerIn;            // raw input: reacts at once, not via the smoothed position
  else if (absB > Q(2.0f))  dirE = beta > 0 ? 1 : -1;
  else                      dirE = curveNow > 0 ? 1 : (curveNow < 0 ? -1 : 0);
  int32_t D = dirE * (mulQ16(kHbDriveQ, gHbQ) + mulQ16(kPowDriveQ, pw));
  int32_t gate = ratioQ16(absB - kGateLoQ, kGateSpanQ);
  const bool counter = (gStQ > 0 && beta < 0) || (gStQ < 0 && beta > 0);   // steering against the slide
  // Steering authority never drops to zero on a car that has lost grip: against
  // the slide it keeps a fixed floor (so it can pull the car through zero), into
  // it the floor scales with lost grip (so a flick can start the opposite slide).
  const int32_t gateFloor = counter ? kGateFloorQ : mulQ16(kGateFloorQ, 65536 - gGripQ);
  if (gate < gateFloor) gate = gateFloor;
  int32_t ds = mulQ16(mulQ16(kSlipSteerQ, gStQ), gate);
  if (counter) ds = mulQ16(ds, kCounterQ);
  D += ds;
  // line balance: if the path heads toward the inside of the drift (phi on the
  // same side as beta) trim the angle, if it heads outward add some -- this is
  // what keeps a held slide on the road instead of cutting the corner.
  if (beta != 0) {
    const int32_t bal = mulQ16(mulQ16(mulQ16(kLineBalQ, 65536 - gGripQ), gPhQ), ratioQ16(absB - Q(4.0f), Q(8.0f)));
    D -= bal;                                          // same sign logic for left and right drifts
  }
  if ((D > 0 && beta > 0) || (D < 0 && beta < 0)) {    // widening the angle: taper toward ~75 deg
    D = mulQ16(D, ratioQ16(kTaperEndQ - absB, kTaperSpanQ));
  }
  const int32_t align = mulQ16(rec ? kAlignRecQ : kAlignQ, gGripQ);
  gBetaQ = clampI32(beta + mulQ16(D - mulQ16(align, beta), dt16), -kBetaMaxQ, kBetaMaxQ);

  // --- path: steering target, road push (partly cancelled by grip), slide turn-in ---
  const int32_t phiMax = mulQ16(kPhiLockQ, 65536 - mulQ16(kPhiFadeQ, speedFrac));
  const int32_t phiT   = mulQ16(mulQ16(phiMax, gStQ), gGripQ);
  const int32_t rhoE   = mulQ16(rhoQ, 65536 - mulQ16(kRoadAssistQ, gGripQ));
  const int32_t turnIn = mulQ16(mulQ16(kSlideTurnQ, 65536 - gGripQ), beta);
  gPhQ += mulQ16(mulQ16(kPathRateQ, phiT - gPhQ) - rhoE + turnIn, dt16);
  gPhQ = clampI32(gPhQ, -kPhMaxQ, kPhMaxQ);

  // --- position: sideways motion = v*sin(phi), progress = v*cos(phi) ---
  gPlayerXQ += mulQ16(mulQ16(vmsQ, sinDegQ(gPhQ)), dt16) / kRoadHalfM;
  const int32_t dProg = mulQ16(mulQ16(vmsQ, cosDegQ(gPhQ)), dt16);
  gDistQ       += dProg;
  gScrollDistQ += mulQ16(dProg, kScrollMulQ);   // 3x visual scroll for stripes/dashes

  // --- soft barrier ---
  if (gPlayerXQ > kPlayerXMaxQ || gPlayerXQ < -kPlayerXMaxQ) {
    const bool outward = (gPlayerXQ > 0) == (gPhQ > 0);
    gPlayerXQ = clampI32(gPlayerXQ, -kPlayerXMaxQ, kPlayerXMaxQ);
    if (outward) {
      gPhQ = 0;
      if (gWallCoolUs == 0 && gSpeedQ > Q(25.0f)) {
        gWallHit = true;
        gWallCoolUs = kWallCoolUs;
        gSpeedQ = mulQ16(gSpeedQ, kWallKeepQ);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Run lifecycle
// ---------------------------------------------------------------------------
void resetRun() {
  gSpeedQ = 0; gPlayerXQ = 0; gDistQ = 0; gScrollDistQ = 0; gLeanQ = 0;
  gPhQ = 0; gBetaQ = 0; gStQ = 0; gHbQ = 0;
  gGripQ = 65536; gCamQ = 0; gShearQ = 0;
  gHbHeldUs = 0; gHbFlashUs = 0; gSpinUs = 0; gRecoverUs = 0; gWallCoolUs = 0;
  gWallHit = false; gOffroad = false; gSlipFx = false;
  gCarAng = 0; gCarDir = 1; gBraking = false;
  gPrevBetaQ = 0; gBetaRateQ = 0;
  gDrifting = false; gChain = false; gLastDir = 1; gDriftHoldUs = 0; gGraceUs = 0;
  gMult = 1; gMultUs = 0; gPendQ8 = 0; gScore = 0;
  gBestChain = 0; gMaxMult = 1; gMaxAngle = 0;
  gRunUs = 0; gStateUs = 0; gSelHoldUs = 0; gSelHold = false;
  gMenuSel = 0; gNewBest = false;
  gMsgUs = 0; gMsgStr[0] = '\0';
  gBarPx = 0; gBarBlink = false;
  for (int i = 0; i < kPuffN; ++i) gPuffs[i].life = 0;
  gPuffNext = 0; gPuffAccUs = 0;
  gSpeedCache = -1; gScoreCache = 0xFFFFFFFFUL; gPendCache = -1;
  gMultCache = 0xFFFFFFFFUL; gAngCache = -1;
  updateMapDot();
  updateHudStrings();
}

// ---------------------------------------------------------------------------
// R34 Skyline GT-R sprite rasteriser (init only, float OK).
// Parametric rear 3/4 model: boxy body, flared arches, four round tail lamps,
// rear wing on posts, twin exhaust tips. Projected from a camera behind and
// slightly above, then stored as monochrome XBM (silhouette + body).
// ---------------------------------------------------------------------------
inline void carSet(uint8_t* bits, int x, int y) {
  if (x < 0 || y < 0 || x >= kCarBmpW || y >= kCarBmpH) return;
  bits[y * kCarBmpStride + (x >> 3)] |= (uint8_t)(1u << (x & 7));
}
inline void carClear(uint8_t* bits, int x, int y) {
  if (x < 0 || y < 0 || x >= kCarBmpW || y >= kCarBmpH) return;
  bits[y * kCarBmpStride + (x >> 3)] &= (uint8_t)~(1u << (x & 7));
}
inline bool carGet(const uint8_t* bits, int x, int y) {
  if (x < 0 || y < 0 || x >= kCarBmpW || y >= kCarBmpH) return false;
  return (bits[y * kCarBmpStride + (x >> 3)] & (1u << (x & 7))) != 0;
}

void buildCarSprites() {
  const float halfW = 1.05f;
  const float halfH = 0.55f;
  const float bodyL = 2.40f;
  const float cabinH = 0.95f;
  const float wingH = 1.15f;
  const float wingW = 1.15f;
  const float archW = 1.25f;
  const float wheelY = -0.15f;

  for (int ai = 0; ai < kCarAngN; ++ai) {
    const float yaw = (float)kCarAngDeg[ai] * 0.017453293f;
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float camZ = -4.2f, camY = 1.8f;
    const float foc = 28.0f;

    uint8_t sil[kCarBmpBytes];
    uint8_t body[kCarBmpBytes];
    for (int i = 0; i < kCarBmpBytes; ++i) { sil[i] = 0; body[i] = 0; }

    auto project = [&](float x, float y, float z, int& ox, int& oy) {
      float rx = x * cy + z * sy;
      float rz = -x * sy + z * cy;
      float ry = y;
      float dx = rx;
      float dy = ry - camY;
      float dz = rz - camZ;
      if (dz < 0.4f) dz = 0.4f;
      ox = (int)(kCarBmpW / 2 + foc * dx / dz + 0.5f);
      oy = (int)(kCarBmpH / 2 + foc * dy / dz + 0.5f);
    };

    auto fillQuad = [&](uint8_t* dst, float ax, float ay, float az,
                        float bx, float by, float bz,
                        float cx, float cy_, float cz,
                        float dx, float dy, float dz, bool solid) {
      int pa[4][2];
      project(ax, ay, az, pa[0][0], pa[0][1]);
      project(bx, by, bz, pa[1][0], pa[1][1]);
      project(cx, cy_, cz, pa[2][0], pa[2][1]);
      project(dx, dy, dz, pa[3][0], pa[3][1]);
      int minx = pa[0][0], maxx = pa[0][0], miny = pa[0][1], maxy = pa[0][1];
      for (int i = 1; i < 4; ++i) {
        if (pa[i][0] < minx) minx = pa[i][0];
        if (pa[i][0] > maxx) maxx = pa[i][0];
        if (pa[i][1] < miny) miny = pa[i][1];
        if (pa[i][1] > maxy) maxy = pa[i][1];
      }
      minx = clampI32(minx, 0, kCarBmpW - 1);
      maxx = clampI32(maxx, 0, kCarBmpW - 1);
      miny = clampI32(miny, 0, kCarBmpH - 1);
      maxy = clampI32(maxy, 0, kCarBmpH - 1);
      for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
          bool inside = true;
          for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            int cross = (pa[j][0] - pa[i][0]) * (y - pa[i][1]) - (pa[j][1] - pa[i][1]) * (x - pa[i][0]);
            if (cross < 0) { inside = false; break; }
          }
          if (inside) {
            carSet(sil, x, y);
            if (solid) carSet(dst, x, y);
          }
        }
      }
    };

    // Rear face
    fillQuad(body, -halfW, -halfH, 0, halfW, -halfH, 0, halfW, halfH, 0, -halfW, halfH, 0, true);

    // Side face (dithered)
    {
      int pa[4][2];
      project( halfW, -halfH, 0, pa[0][0], pa[0][1]);
      project( halfW * 0.7f, -halfH, bodyL, pa[1][0], pa[1][1]);
      project( halfW * 0.7f,  halfH, bodyL, pa[2][0], pa[2][1]);
      project( halfW,  halfH, 0, pa[3][0], pa[3][1]);
      int minx = pa[0][0], maxx = pa[0][0], miny = pa[0][1], maxy = pa[0][1];
      for (int i = 1; i < 4; ++i) {
        if (pa[i][0] < minx) minx = pa[i][0];
        if (pa[i][0] > maxx) maxx = pa[i][0];
        if (pa[i][1] < miny) miny = pa[i][1];
        if (pa[i][1] > maxy) maxy = pa[i][1];
      }
      minx = clampI32(minx, 0, kCarBmpW - 1);
      maxx = clampI32(maxx, 0, kCarBmpW - 1);
      miny = clampI32(miny, 0, kCarBmpH - 1);
      maxy = clampI32(maxy, 0, kCarBmpH - 1);
      for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
          bool inside = true;
          for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            int cross = (pa[j][0] - pa[i][0]) * (y - pa[i][1]) - (pa[j][1] - pa[i][1]) * (x - pa[i][0]);
            if (cross < 0) { inside = false; break; }
          }
          if (inside) {
            carSet(sil, x, y);
            if (((x + y) & 1) == 0) carSet(body, x, y);
          }
        }
      }
    }

    // Cabin
    fillQuad(body, -halfW * 0.75f, halfH, 0.15f, halfW * 0.75f, halfH, 0.15f,
             halfW * 0.55f, cabinH, 0.9f, -halfW * 0.55f, cabinH, 0.9f, true);
    // Glass cut-out
    {
      int pa[4][2];
      project(-halfW * 0.65f, halfH + 0.08f, 0.25f, pa[0][0], pa[0][1]);
      project( halfW * 0.65f, halfH + 0.08f, 0.25f, pa[1][0], pa[1][1]);
      project( halfW * 0.50f, cabinH - 0.05f, 0.85f, pa[2][0], pa[2][1]);
      project(-halfW * 0.50f, cabinH - 0.05f, 0.85f, pa[3][0], pa[3][1]);
      int minx = pa[0][0], maxx = pa[0][0], miny = pa[0][1], maxy = pa[0][1];
      for (int i = 1; i < 4; ++i) {
        if (pa[i][0] < minx) minx = pa[i][0];
        if (pa[i][0] > maxx) maxx = pa[i][0];
        if (pa[i][1] < miny) miny = pa[i][1];
        if (pa[i][1] > maxy) maxy = pa[i][1];
      }
      for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
          bool inside = true;
          for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            int cross = (pa[j][0] - pa[i][0]) * (y - pa[i][1]) - (pa[j][1] - pa[i][1]) * (x - pa[i][0]);
            if (cross < 0) { inside = false; break; }
          }
          if (inside) carClear(body, x, y);
        }
      }
    }

    // Rear wing blade
    fillQuad(body, -wingW, wingH - 0.08f, -0.05f, wingW, wingH - 0.08f, -0.05f,
             wingW, wingH, -0.05f, -wingW, wingH, -0.05f, true);
    // Posts
    for (float sx : {-0.55f, 0.55f}) {
      int x0, y0, x1, y1;
      project(sx, halfH, 0.05f, x0, y0);
      project(sx, wingH - 0.08f, -0.02f, x1, y1);
      int steps = absI32(y1 - y0) + absI32(x1 - x0) + 1;
      for (int s = 0; s <= steps; ++s) {
        int px = x0 + (x1 - x0) * s / steps;
        int py = y0 + (y1 - y0) * s / steps;
        carSet(sil, px, py); carSet(body, px, py);
        carSet(sil, px + 1, py); carSet(body, px + 1, py);
      }
    }

    // Four round tail lamps
    const float lampXs[4] = {-0.72f, -0.38f, 0.38f, 0.72f};
    for (int li = 0; li < 4; ++li) {
      int lx, ly;
      project(lampXs[li], 0.15f, 0.02f, lx, ly);
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx)
          if (dx * dx + dy * dy <= 4) carClear(body, lx + dx, ly + dy);
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx)
          if (dx * dx + dy * dy <= 4 && dx * dx + dy * dy >= 2) {
            carSet(sil, lx + dx, ly + dy);
            carSet(body, lx + dx, ly + dy);
          }
    }

    // Twin exhaust tips
    for (float ex : {-0.25f, 0.25f}) {
      int exx, exy;
      project(ex, -halfH + 0.08f, -0.02f, exx, exy);
      carSet(sil, exx, exy); carSet(sil, exx + 1, exy);
      carClear(body, exx, exy); carClear(body, exx + 1, exy);
    }

    // Rear wheels
    int wlx, wly, wrx, wry;
    project(-archW + 0.15f, wheelY, 0.15f, wlx, wly);
    project( archW - 0.15f, wheelY, 0.15f, wrx, wry);
    gCarWheelX[ai][0] = (int8_t)wlx;
    gCarWheelX[ai][1] = (int8_t)wrx;
    gCarWheelY[ai] = (int8_t)((wly + wry) / 2);
    for (int wi = 0; wi < 2; ++wi) {
      int wx = (wi == 0) ? wlx : wrx;
      int wy = (wi == 0) ? wly : wry;
      for (int dy = -3; dy <= 3; ++dy)
        for (int dx = -3; dx <= 3; ++dx) {
          if (dx * dx + dy * dy <= 9) {
            carSet(sil, wx + dx, wy + dy);
            if (dx * dx + dy * dy <= 4) carClear(body, wx + dx, wy + dy);
            else carSet(body, wx + dx, wy + dy);
          }
        }
    }

    // Front wheel (3/4 views)
    if (ai >= 2) {
      int fwx, fwy;
      project(halfW * 0.55f, wheelY, bodyL * 0.85f, fwx, fwy);
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx)
          if (dx * dx + dy * dy <= 5) {
            carSet(sil, fwx + dx, fwy + dy);
            if (dx * dx + dy * dy <= 2) carClear(body, fwx + dx, fwy + dy);
            else carSet(body, fwx + dx, fwy + dy);
          }
    }

    // Outline expand
    uint8_t outline[kCarBmpBytes];
    for (int i = 0; i < kCarBmpBytes; ++i) outline[i] = 0;
    for (int y = 0; y < kCarBmpH; ++y) {
      for (int x = 0; x < kCarBmpW; ++x) {
        if (!carGet(sil, x, y)) continue;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx)
            carSet(outline, x + dx, y + dy);
      }
    }
    for (int i = 0; i < kCarBmpBytes; ++i) {
      gCarSilR[ai][i] = outline[i];
      gCarBodyR[ai][i] = body[i];
      gCarSilL[ai][i] = 0;
      gCarBodyL[ai][i] = 0;
    }
    // Mirror left
    for (int y = 0; y < kCarBmpH; ++y) {
      for (int x = 0; x < kCarBmpW; ++x) {
        int mx = kCarBmpW - 1 - x;
        if (carGet(outline, x, y)) carSet(gCarSilL[ai], mx, y);
        if (carGet(body, x, y))    carSet(gCarBodyL[ai], mx, y);
      }
    }
  }
}

void gameInit() {
  for (int i = 0; i <= 90; ++i)                        // sine table (init-only float)
    gSinTbl[i] = (int16_t)(sinf((float)i * 0.017453293f) * 32767.0f + 0.5f);
  buildCurveTable();
  buildMap();
  buildCarSprites();
  for (int i = 0; i < kRowsN; ++i) {
    const int dy = i + 1;                              // y = kHorizonY + dy
    gRowZ[i]  = (int16_t)(kProjC / dy);                // metres ahead of the car
    gRowHW[i] = (int16_t)((dy * kRoadWNum) >> 6);
  }
  gRng = (uint32_t)micros() ^ 0x2F2F5A17UL;            // micros(): seeding only
  if (gRng == 0) gRng = 0x2F2F5A17UL;
  gState  = ST_TITLE;
  gAnimMs = 0;
  resetRun();
  updateProjection();                                  // valid road from the first frame
  // gBestScore deliberately survives re-entry (session best).
}

// ---------------------------------------------------------------------------
// update() -- logic only
// ---------------------------------------------------------------------------
fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  const uint32_t dtUs = (f.dtUs > kMaxStepUs) ? kMaxStepUs : f.dtUs;
  gAnimMs += dtUs / 1000UL;
  const int32_t dt16 = (int32_t)((dtUs << 16) / 1000000UL);   // Q16.16 seconds

  switch (gState) {

    case ST_TITLE:
      // Tap SELECT to start; hold it ~1 s to quit. The hold timer makes the
      // two gestures unambiguous even though both begin with the same press.
      if (in.pressed & fb::KEY_SELECT) { gSelHold = true; gSelHoldUs = 0; }
      if (gSelHold) {
        if (in.held & fb::KEY_SELECT) {
          gSelHoldUs += dtUs;
          if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
        } else {
          gSelHold = false;
          if (gSelHoldUs < kQuitHoldUs) { resetRun(); gState = ST_COUNT; }
        }
      }
      break;

    case ST_COUNT:
      gStateUs += dtUs;
      gSpeedQ = 0;                                     // car held on the grid
      updateProjection();                              // static road, start line visible
      updateHudStrings();
      if (in.held & fb::KEY_SELECT) {
        gSelHoldUs += dtUs;
        if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
      } else {
        gSelHoldUs = 0;
      }
      if (gStateUs >= 3UL * kCountStepUs) {            // after "3 2 1" -> GO
        gState = ST_PLAY;
        gStateUs = 0;
        gRunUs = 0;
        gSelHoldUs = 0;
      }
      break;

    case ST_PLAY: {
      // --- quit hold (SELECT is otherwise unused while driving) ---
      if (in.held & fb::KEY_SELECT) {
        gSelHoldUs += dtUs;
        if (gSelHoldUs >= kQuitHoldUs) return fb::GAME_EXIT;
      } else {
        gSelHoldUs = 0;
      }
      gRunUs += dtUs;

      // --- timers ---
      gRecoverUs  = (gRecoverUs  > dtUs) ? (gRecoverUs  - dtUs) : 0;
      gWallCoolUs = (gWallCoolUs > dtUs) ? (gWallCoolUs - dtUs) : 0;
      gHbFlashUs  = (gHbFlashUs  > dtUs) ? (gHbFlashUs  - dtUs) : 0;
      gMsgUs      = (gMsgUs      > dtUs) ? (gMsgUs      - dtUs) : 0;

      // --- controls: UP throttle, DOWN brake, L/R steer, X handbrake ---
      const bool thr = (in.held & fb::KEY_UP) != 0;
      const bool brk = (in.held & fb::KEY_DOWN) != 0;
      int steer = 0;
      if (in.held & fb::KEY_LEFT)  steer -= 1;
      if (in.held & fb::KEY_RIGHT) steer += 1;
      const bool hbHeld = (in.held & fb::KEY_X) != 0;
      if (hbHeld) gHbHeldUs += dtUs; else gHbHeldUs = 0;
      const bool hbOn = hbHeld && gHbHeldUs < kHbMaxUs && gSpeedQ >= kHbMinSpeedQ && gRecoverUs == 0;

      gOffroad = (gPlayerXQ > kOffEdgeQ) || (gPlayerXQ < -kOffEdgeQ);
      const int32_t curveNow = sampleCurve(gDistQ >> 16);          // -127..127

      // handbrake press: yaw kick toward the steering direction, or into the
      // upcoming corner when no steering is held
      if ((in.pressed & fb::KEY_X) && hbOn) {
        const int dir = steer != 0 ? steer : (curveNow > 0 ? 1 : (curveNow < 0 ? -1 : 0));
        gBetaQ = clampI32(gBetaQ + dir * kHbKickQ, -kBetaMaxQ, kBetaMaxQ);
        gHbFlashUs = kHbFlashUs;
        int fx, nx;
        tyreXs(fx, nx);
        const int ty = tyreY();
        for (int i = 0; i < 3; ++i) { spawnPuff(fx, ty, -1); spawnPuff(nx, ty, 1); }
      }

      // --- physics: 1 step, or 2 if the frame was long (bounded cost) ---
      const int n = (dtUs > kSplitStepUs) ? 2 : 1;
      const int32_t dtS = dt16 / n;
      for (int k = 0; k < n; ++k) stepCar(dtS, thr, brk, steer, hbOn, curveNow);

      // --- scoring, camera, effects ---
      updateScoring(dtUs, dt16);

      gCamQ = approach(gCamQ, clampI32(gPhQ + gBetaQ, -kCamMaxQ, kCamMaxQ), kCamRateQ, dt16);   // view follows the nose
      gShearQ = mulQ16(sinDegQ(gCamQ), kCamFollowQ);
      gLeanQ  = approach(gLeanQ, gStQ * 2, Q(8.0f), dt16);

      const int32_t absB = absI32(gBetaQ);
      // Map absolute slip angle to sprite table (0..60 deg -> 0..kCarAngN-1)
      int ang = (int)((absB >> 16) * (kCarAngN - 1) / 60);
      if (ang > kCarAngN - 1) ang = kCarAngN - 1;
      if (ang < 0) ang = 0;
      gCarAng = (uint8_t)ang;
      if (gBetaQ > Q(1.0f)) gCarDir = 1; else if (gBetaQ < -Q(1.0f)) gCarDir = -1;
      gBraking = brk;
      gSlipFx = (!gOffroad && absB >= Q(10.0f) && gSpeedQ >= Q(25.0f)) || (gOffroad && gSpeedQ >= Q(20.0f));
      updatePuffs(dtUs);

      updateMapDot();
      updateHudStrings();
      updateProjection();

      // --- lap complete: bank what is pending and show the results ---
      if ((gDistQ >> 16) >= kTrackLen) {
        bankChain();
        gNewBest = gScore > gBestScore;
        if (gNewBest) gBestScore = gScore;
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
          resetRun();
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

// Box in "nose-side" coordinates: xo is measured from the car centre toward
// the side the nose points to (dir = +1 right, -1 left), so one drawing routine
// serves both drift directions.
inline void dbox(U8G2& d, int cx, int dir, int xo, int w, int y, int h) {
  box(d, dir >= 0 ? cx + xo : cx - xo - w, y, w, h);
}

// R34 GT-R: two XBM draws (black silhouette then white body). Angle from
// slip; left/right from mirrored tables. Optional 1 px vertical jitter when
// off-road / smoking. Brake lamps are part of the body bitmap; when braking
// we overdraw brighter dots on the four lamp centres.
void drawCar(U8G2& d, int cx, int top, int ang, int dir, bool jitter) {
  if (jitter) top += (int)((gAnimMs >> 5) & 1u);
  if (ang < 0) ang = 0;
  if (ang >= kCarAngN) ang = kCarAngN - 1;
  const int bx = cx - kCarBmpW / 2;
  const uint8_t* sil  = (dir >= 0) ? gCarSilR[ang]  : gCarSilL[ang];
  const uint8_t* body = (dir >= 0) ? gCarBodyR[ang] : gCarBodyL[ang];
  d.setBitmapMode(0);
  d.setDrawColor(0);
  d.drawXBM((u8g2_uint_t)bx, (u8g2_uint_t)top,
            (u8g2_uint_t)kCarBmpW, (u8g2_uint_t)kCarBmpH, sil);
  d.setDrawColor(1);
  d.drawXBM((u8g2_uint_t)bx, (u8g2_uint_t)top,
            (u8g2_uint_t)kCarBmpW, (u8g2_uint_t)kCarBmpH, body);
  // Brake lamps: four bright pixels near the rear face when braking
  if (gBraking) {
    // Approximate lamp positions from the 0-deg pose (close enough at all angles)
    const int ly = top + gCarWheelY[0] - 4;
    const int centres[4] = { -9, -4, 4, 9 };
    for (int i = 0; i < 4; ++i) {
      int lx = cx + ((dir >= 0) ? centres[i] : -centres[i]);
      d.drawPixel((u8g2_uint_t)lx, (u8g2_uint_t)ly);
      d.drawPixel((u8g2_uint_t)(lx + 1), (u8g2_uint_t)ly);
    }
  }
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

  // skid marks under the rear tyres (flicker reads as dashed marks)
  const int ang = (int)gCarAng;
  const int dir = (int)gCarDir;
  if (gSlipFx && ((gAnimMs >> 6) & 1u)) {
    int fx, nx;
    tyreXs(fx, nx);
    const int ty = tyreY() + 2;
    box(d, fx - 1, ty, 3, 2);
    box(d, nx - 1, ty, 3, 2);
  }

  // smoke / dust puffs
  for (int i = 0; i < kPuffN; ++i) {
    const Puff& p = gPuffs[i];
    if (p.life <= 0) continue;
    const int px = (int)(p.x >> 8);
    const int py = (int)(p.y >> 8);
    if (px < 0 || px >= (int)fb::SCREEN_W || py < 0 || py >= (int)fb::SCREEN_H) continue;
    if (p.life > 300000) box(d, px, py, 2, 2);
    else                 d.drawPixel((u8g2_uint_t)px, (u8g2_uint_t)py);
  }

  // player car: fixed near bottom-centre, the road slides beneath it
  drawCar(d, kCarCX, kCarTop, ang, dir, gSlipFx && gOffroad);

  // handbrake rings at the rear tyres
  if (gHbFlashUs > 0) {
    int fx, nx;
    tyreXs(fx, nx);
    const int ty = tyreY();
    const int r = 2 + (int)((kHbFlashUs - gHbFlashUs) / 70000UL);
    d.drawCircle((u8g2_uint_t)fx, (u8g2_uint_t)ty, (u8g2_uint_t)r);
    d.drawCircle((u8g2_uint_t)nx, (u8g2_uint_t)ty, (u8g2_uint_t)r);
  }
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

// Bottom-left: combo multiplier and the combo-stability bar (fills while the
// next multiplier step builds; drains, then blinks, while the chain is only
// being held alive by the grace window).
void renderComboPanel(U8G2& d) {
  const int px = 0, py = 40, pw = 27, ph = 23;
  d.setDrawColor(0);
  box(d, px, py, pw, ph);
  d.setDrawColor(1);
  frameBox(d, px, py, pw, ph);
  d.setFontMode(1);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(4, 56, "x");
  d.setFont(u8g2_font_logisoso16_tn);
  d.drawStr(11, 56, gMultStr);
  frameBox(d, 3, 58, kBarW + 2, 4);
  if (gBarPx > 0 && (!gBarBlink || ((gAnimMs >> 6) & 1u))) box(d, 4, 59, (int)gBarPx, 2);
  d.setFontMode(0);
}

// Bottom-right: slip-angle gauge. The marker moves with the signed angle
// (right = nose right); ticks mark the 30 and 45 degree scoring sweet spot.
void renderAnglePanel(U8G2& d) {
  const int px = 100, py = 40, pw = 27, ph = 23;
  d.setDrawColor(0);
  box(d, px, py, pw, ph);
  d.setDrawColor(1);
  frameBox(d, px, py, pw, ph);
  d.setFontMode(1);
  d.setFont(u8g2_font_5x7_tr);
  const bool nearSpin = absI32(gBetaQ) > Q(60.0f);
  if (nearSpin) {
    if ((gAnimMs >> 6) & 1u) d.drawStr(px + 4, 47, "SPIN!");
  } else {
    d.drawStr(px + 6, 47, "ANG");
  }
  const int cx = px + 13;
  hline(d, cx - 10, 53, 21);                           // axis
  d.drawVLine((u8g2_uint_t)(cx - 10), 50, 5);
  d.drawVLine((u8g2_uint_t)cx, 50, 5);
  d.drawVLine((u8g2_uint_t)(cx + 10), 50, 5);
  d.drawVLine((u8g2_uint_t)(cx - 5), 52, 3);           // 30 deg
  d.drawVLine((u8g2_uint_t)(cx + 5), 52, 3);
  int off = (int)((gBetaQ >> 16) * 10 / 60);
  off = clampI32(off, -10, 10);
  box(d, cx + off - 1, 49, 3, 6);
  const int w = (int)d.getStrWidth(gAngStr);
  d.drawStr((u8g2_uint_t)(cx - w / 2), 61, gAngStr);
  d.setFontMode(0);
}

void renderHud(U8G2& d) {
  d.setDrawColor(1);
  d.setFontMode(0);                      // solid text: readable over the road
  d.setFont(u8g2_font_logisoso16_tn);    // big speed, top-left
  d.drawStr(2, 16, gSpeedStr);
  d.setFont(u8g2_font_5x7_tr);
  d.drawStr(2, 23, "KM/H");
  d.setFont(u8g2_font_6x10_tr);          // banked score top-right, BELOW the
  int w = (int)d.getStrWidth(gScoreStr); // launcher's FPS-box zone
  d.drawStr((u8g2_uint_t)((int)fb::SCREEN_W - 2 - w), 20, gScoreStr);
  if (gChain && gPendQ8 > 0) {           // unbanked points of the current chain
    w = (int)d.getStrWidth(gPendStr);
    d.drawStr((u8g2_uint_t)((int)fb::SCREEN_W - 2 - w), 35, gPendStr);
  }
  renderMap(d);
  renderComboPanel(d);
  renderAnglePanel(d);
}

// Centre message ("LINK x3", "SPIN OUT", "WALL HIT").
void renderMsg(U8G2& d) {
  d.setFont(u8g2_font_6x10_tr);
  d.setFontMode(1);
  const int w = (int)d.getStrWidth(gMsgStr);
  const int x = ((int)fb::SCREEN_W - w) / 2;
  d.setDrawColor(0);
  box(d, x - 3, 28, w + 6, 12);
  d.setDrawColor(1);
  frameBox(d, x - 3, 28, w + 6, 12);
  d.drawStr((u8g2_uint_t)x, 37, gMsgStr);
  d.setFontMode(0);
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
  centerStr(d, 19, "2 FAST 2 FURIOUS");
  d.setFont(u8g2_font_5x7_tr);
  centerStr(d, 27, "flipBOX DRIFT SIM");
  drawCar(d, fb::SCREEN_W / 2, 28, 4, ((gAnimMs / 1400UL) & 1UL) ? -1 : 1, false);   // sliding both ways
  d.setFont(u8g2_font_5x7_tr);
  centerStr(d, 49, "UP=GAS  DOWN=BRAKE");
  centerStr(d, 56, "X=HANDBRAKE  L/R=STEER");
  if (((gAnimMs / 600UL) & 1UL) || gSelHold) {
    centerStr(d, 63, "PRESS SELECT");
  } else {
    centerStr(d, 63, "HOLD SELECT: QUIT");
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
  centerStr(d, 11, "RUN COMPLETE");
  char buf[48];
  d.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "SCORE %lu", (unsigned long)gScore);
  d.drawStr(4, 23, buf);
  snprintf(buf, sizeof(buf), "CHAIN %lu", (unsigned long)gBestChain);
  d.drawStr(4, 33, buf);
  snprintf(buf, sizeof(buf), "COMBO x%lu  ANG %lu", (unsigned long)gMaxMult, (unsigned long)gMaxAngle);
  d.drawStr(4, 43, buf);
  d.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "SESSION BEST %lu", (unsigned long)gBestScore);
  d.drawStr(4, 52, buf);
  if (gNewBest) d.drawStr(100, 52, "NEW!");
  d.setFont(u8g2_font_6x10_tr);
  menuItem(d, 8, 62, "RETRY", gMenuSel == 0);
  menuItem(d, 76, 62, "QUIT", gMenuSel == 1);
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
      if (gRunUs < kGoShowUs) renderGo(d);
      else if (gMsgUs > 0)    renderMsg(d);
      if (gSelHoldUs > 0) drawQuitBar(d);
      break;
    case ST_RESULT:
      renderResult(d);
      break;
  }
}

}  // namespace

FB_GAME(TwoFast2Furious, gameInit, gameUpdate, gameRender, nullptr);
