/*
 * ============================================================================
 *  flipBOX  --  Launcher  (main.cpp, Wave 1)
 *  ESP32-WROOM-32 | Arduino | PlatformIO | U8g2 | SSD1306 128x64 I2C | 6 keys
 * ============================================================================
 *
 *  FLOW
 *     boot -> game-selection menu -> selected game -> (game exits) -> menu
 *
 *  ADDING A GAME  (three steps, one line in this file)
 *     1. Create src/NewGame.cpp.
 *     2. Paste the "FLIPBOX GAME CONTRACT" block below (between its BEGIN/END
 *        markers) at the top of that file and implement:
 *            static void           gameInit();                       // optional
 *            static fb::Status     gameUpdate(const fb::Input&, const fb::Frame&);
 *            static void           gameRender(U8G2& d);
 *            static void           gameShutdown();                   // optional
 *        then end the file with:
 *            FB_GAME(NewGame, gameInit, gameUpdate, gameRender, gameShutdown);
 *        (pass nullptr for init / shutdown if unused).
 *     3. Add ONE line to FB_GAME_LIST below:   X(NewGame, "Menu Label")
 *
 *  The launcher links against each game through a weak symbol, so this file
 *  compiles and runs even while a game's .cpp does not exist yet; selecting a
 *  missing game shows "NOT INSTALLED".
 *
 *  PER-FRAME ORDER IN A GAME:   input poll -> update() -> clear -> render()
 *                               -> present (asynchronous, see below)
 *
 *  PERFORMANCE NOTES
 *     * No delay(), no heap allocation, no String, no framework object churn.
 *     * Frame rate is NOT capped: the loop runs as fast as the OLED bus allows.
 *     * The 1 KB frame is pushed to the OLED by a small task on core 0 while
 *       core 1 already computes the next frame (set FB_ASYNC_DISPLAY 0 to fall
 *       back to a plain, synchronous sendBuffer()).
 *     * FB_FAST_FLUSH: that task streams the frame as a few long I2C writes
 *       (window set once, horizontal addressing) instead of U8g2's tile-by-tile
 *       transfers.  Set it to 0 to use the stock U8g2 tile path.
 *     * FB_LATE_LATCH: the game loop waits until the OLED link is nearly idle
 *       and only THEN samples input / runs update() / render(), so the frame
 *       shown reflects the newest possible button state.  Throughput is kept
 *       because the wait is sized from the measured update+render time.
 *     * Buttons are read straight from the GPIO input registers, additionally
 *       sampled every 1 ms by an esp_timer callback.  Every accepted edge is
 *       time-stamped and latched, so taps shorter than a frame are never lost
 *       and games can ask for the exact press time (see "LAUNCHER EXTRAS").
 *     * The menu only redraws when something changed.
 * ============================================================================
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_timer.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================================================
//  1. BUILD OPTIONS (override with -D in platformio.ini if you like)
// ============================================================================
#ifndef FB_I2C_HZ
#define FB_I2C_HZ 1000000UL     // OLED bus clock. 400000 = safe default of most
                                // modules; 1000000+ = faster (needs decent
                                // pull-ups and short wires). Lower it if the
                                // screen shows noise/garbage.
#endif
#ifndef FB_ASYNC_DISPLAY
#define FB_ASYNC_DISPLAY 1      // 1 = push frames to the OLED from core 0
#endif
#ifndef FB_SHOW_FPS
#define FB_SHOW_FPS 0           // 1 = small FPS counter (top-right) inside games
#endif
#ifndef FB_DEBOUNCE_US
#define FB_DEBOUNCE_US 5000UL   // lock-out after a button edge (first edge is
                                // accepted with zero latency)
#endif
#ifndef FB_MAX_DT_US
#define FB_MAX_DT_US 50000UL    // frame delta handed to games is clamped to this
#endif
#ifndef FB_FAST_FLUSH
#define FB_FAST_FLUSH 1         // 1 = stream the frame as long I2C writes from the
                                // flush task (needs FB_ASYNC_DISPLAY 1);
                                // 0 = stock U8g2 tile transfers
#endif
#ifndef FB_I2C_CHUNK
#define FB_I2C_CHUNK 256        // data bytes per I2C write in the fast flush
                                // (power of two <= 1024; reduced automatically
                                // if the Wire buffer cannot grow that far)
#endif
#ifndef FB_LATE_LATCH
#define FB_LATE_LATCH 1         // 1 = start each frame as late as possible so the
                                // newest input is shown (needs FB_ASYNC_DISPLAY 1)
#endif
#ifndef FB_LATCH_MARGIN_US
#define FB_LATCH_MARGIN_US 400UL // safety slack added to the measured frame work time
#endif
#ifndef FB_PANEL_LATENCY_US
#define FB_PANEL_LATENCY_US 5000UL // average delay between the last byte reaching the
                                // OLED and the pixels lighting up (~half a panel
                                // scan). Only used for the latency estimate that
                                // games can query.
#endif
#ifndef FB_OLED_OSC
#define FB_OLED_OSC 0xF0        // SSD1306 display clock (cmd 0xD5): 0xF0 = fastest
                                // panel refresh (less scan-out delay / tearing).
                                // 0x80 = controller default, 0 = do not touch.
                                // If the picture flickers or looks dim, set 0.
#endif

// ============================================================================
//  2. GAME REGISTRY  --  ONE line per game:  X(symbol, "menu label")
//     'symbol' must match the id used in that game's FB_GAME(...) line.
// ============================================================================
#define FB_GAME_LIST(X)                                    \
  X(F1Racing,       "F1 Racing")                           \
  X(FastAndFurious, "Fast and Furious")                    \
  X(TokyoDrift,     "Tokyo Drift")

// ============================================================================
//  3. FLIPBOX GAME CONTRACT v1
//     Paste everything between BEGIN and END, unchanged, at the top of every
//     game .cpp (plus the two #includes). Identical copies are ODR-safe.
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
constexpr uint8_t KEY_X      = 1u << 5;

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
//
//  Render rules: the launcher clears the buffer and sets draw colour 1, solid
//  font mode, bitmap mode 0 on entry. Set fonts/colours you need inside
//  render() each frame -- do not assume they survive between frames (the debug
//  FPS overlay may touch them). Never call sendBuffer()/clearBuffer() yourself.

// ============================================================================
//  LAUNCHER EXTRAS (optional; NOT part of contract v1, so games that do not use
//  them are unaffected).  A game that wants them declares the same prototypes
//  inside its own  namespace fb { ... }  block.
//
//     keyPressedAtUs(KEY_x)  micros() timestamp of the most recent accepted
//                            press edge of ONE key (+-1 ms accurate).
//     frameStartUs()         micros() timestamp this frame's input was sampled
//                            at; the Frame::dtUs clock is based on it.
//     displayLatencyUs()     estimated delay from frameStartUs() until what the
//                            frame draws becomes visible on the panel.
// ============================================================================
namespace fb {
uint32_t keyPressedAtUs(uint8_t keyMask);
uint32_t frameStartUs();
uint32_t displayLatencyUs();
}

// Weak references: a missing game .cpp resolves to nullptr instead of a link error.
#define FB_DECLARE_WEAK(id, label) extern const fb::Game fbGame_##id __attribute__((weak));
FB_GAME_LIST(FB_DECLARE_WEAK)

namespace {

// ============================================================================
//  4. HARDWARE  (existing wiring plus owner-configured X button on D4/GPIO4)
// ============================================================================
constexpr uint8_t PIN_SDA    = 21;
constexpr uint8_t PIN_SCL    = 22;
constexpr uint8_t PIN_UP     = 26;
constexpr uint8_t PIN_DOWN   = 33;
constexpr uint8_t PIN_LEFT   = 25;
constexpr uint8_t PIN_RIGHT  = 27;
constexpr uint8_t PIN_SELECT = 32;
constexpr uint8_t PIN_X      = 4;  // Board pin D4 (ESP32 GPIO4)
// Buttons: INPUT_PULLUP, pressed = LOW.

U8G2_SSD1306_128X64_NONAME_F_HW_I2C gDisp(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);

// ============================================================================
//  5. DISPLAY PRESENTATION
// ============================================================================
// Flush timing, shared between the game loop (core 1) and the flush task
// (core 0).  All are single aligned 32-bit words, so plain volatile access is
// atomic enough.
volatile uint32_t gFlushDurUs   = 12000;   // smoothed duration of one OLED transfer
volatile uint32_t gFlushStartUs = 0;       // micros() when the latest transfer began
volatile bool     gFlushBusy    = false;   // a transfer is in flight
uint32_t          gWorkUs       = 3000;    // smoothed update+render time of a frame
uint32_t          gFrameNowUs   = 0;       // micros() the current frame was started at

#if FB_ASYNC_DISPLAY
uint8_t           gShadow[1024] __attribute__((aligned(4)));   // frame being sent
SemaphoreHandle_t gFlushDone = nullptr;                        // given when OLED link is idle
TaskHandle_t      gFlushTaskHandle = nullptr;

#if FB_FAST_FLUSH
uint16_t gFlushChunk = 32;      // data bytes per I2C write (set in setup())
bool     gNeedWindow = true;    // (re)send the column/page window before the next frame

// Streams gShadow to the SSD1306 in a few long writes.  The controller is put
// in horizontal addressing mode with the window covering the whole screen, so
// 1024 data bytes fill it and the address pointer wraps back to the origin by
// itself; the window is only re-sent after a failed transfer.
bool fastFlush() {
  uint8_t a8 = gDisp.getU8x8()->i2c_address;       // 8-bit form, 0x78 for 0x3C
  if (a8 == 255) a8 = 0x78;
  const uint8_t addr = (uint8_t)(a8 >> 1);

  if (gNeedWindow) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x00);                      // control byte: command stream
    Wire.write((uint8_t)0x20); Wire.write((uint8_t)0x00);                     // horizontal addressing
    Wire.write((uint8_t)0x21); Wire.write((uint8_t)0x00); Wire.write((uint8_t)127);  // columns 0..127
    Wire.write((uint8_t)0x22); Wire.write((uint8_t)0x00); Wire.write((uint8_t)7);    // pages   0..7
    if (Wire.endTransmission() != 0) return false;
    gNeedWindow = false;
  }
  for (uint16_t off = 0; off < sizeof(gShadow); off = (uint16_t)(off + gFlushChunk)) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x40);                      // control byte: data stream
    Wire.write(gShadow + off, (size_t)gFlushChunk);
    if (Wire.endTransmission() != 0) {
      gNeedWindow = true;                           // pointer position unknown now
      return false;
    }
  }
  return true;
}
#endif  // FB_FAST_FLUSH

void flushTask(void*) {
#if !FB_FAST_FLUSH
  u8x8_t* const u8x8 = gDisp.getU8x8();
#endif
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if FB_FAST_FLUSH
    fastFlush();
#else
    for (uint8_t row = 0; row < 8; ++row) {
      u8x8_DrawTile(u8x8, 0, row, 16, gShadow + (row << 7));
    }
#endif
    uint32_t d = micros() - gFlushStartUs;
    if (d > 60000UL) d = 60000UL;                   // ignore stalls / error timeouts
    gFlushDurUs = (uint32_t)((gFlushDurUs * 3UL + d) >> 2);
    gFlushBusy  = false;
    xSemaphoreGive(gFlushDone);
  }
}
#endif  // FB_ASYNC_DISPLAY

// Hands the finished frame to the OLED. With async display this returns as soon
// as the 1 KB snapshot is taken; it only waits if the previous frame is still
// travelling over I2C (that wait is the natural, hardware-limited frame pacing).
inline void presentFrame() {
#if FB_ASYNC_DISPLAY
  xSemaphoreTake(gFlushDone, portMAX_DELAY);
  memcpy(gShadow, gDisp.getBufferPtr(), sizeof(gShadow));
  gFlushStartUs = micros();
  gFlushBusy    = true;
  xTaskNotifyGive(gFlushTaskHandle);
#else
  const uint32_t t0 = micros();
  gDisp.sendBuffer();
  uint32_t d = micros() - t0;
  if (d > 60000UL) d = 60000UL;
  gFlushDurUs = (uint32_t)((gFlushDurUs * 3UL + d) >> 2);
#endif
}

// Late latch: block until the transfer in flight is about to finish, minus the
// time update()+render() are expected to take.  The next frame is then ready
// exactly when the link frees up (no throughput loss), but its input was
// sampled as late as possible, so it is roughly one transfer "fresher" than
// sampling right after the previous frame was handed over.
inline void waitForLatch() {
#if FB_ASYNC_DISPLAY && FB_LATE_LATCH
  const uint32_t workEst = gWorkUs + (gWorkUs >> 2) + FB_LATCH_MARGIN_US;
  for (;;) {
    if (!gFlushBusy) return;
    const int32_t remain = (int32_t)(gFlushStartUs + gFlushDurUs - workEst - micros());
    if (remain <= 0 || remain > 100000) return;     // due, or timing data is bogus
    if (remain > 1500 && portTICK_PERIOD_MS == 1) vTaskDelay(1);   // sleep coarse, spin the rest
  }
#endif
}

// ============================================================================
//  6. INPUT  (6 buttons, direct register read, zero-latency debounce + gating)
// ============================================================================
// The raw pins are sampled (a) by pollInput() once per frame and (b) by a 1 kHz
// esp_timer callback.  Both run the same debouncer under one spinlock, and every
// accepted edge is LATCHED (pressed / released bits + press time-stamp).  A
// frame therefore sees every edge since the previous frame, even a tap that was
// pressed and released entirely between two frames.
constexpr uint8_t kKeyCount = 6;

constexpr bool pinHigh(uint32_t lo, uint32_t hi, uint8_t pin) {
  return (pin < 32) ? (((lo >> pin) & 1u) != 0u) : (((hi >> (pin - 32)) & 1u) != 0u);
}

inline uint8_t readRaw() {
  const uint32_t lo = REG_READ(GPIO_IN_REG);    // GPIO 0..31
  const uint32_t hi = REG_READ(GPIO_IN1_REG);   // GPIO 32..39
  return (uint8_t)((pinHigh(lo, hi, PIN_UP)     ? 0 : fb::KEY_UP)    |
                   (pinHigh(lo, hi, PIN_DOWN)   ? 0 : fb::KEY_DOWN)  |
                   (pinHigh(lo, hi, PIN_LEFT)   ? 0 : fb::KEY_LEFT)  |
                   (pinHigh(lo, hi, PIN_RIGHT)  ? 0 : fb::KEY_RIGHT) |
                   (pinHigh(lo, hi, PIN_SELECT) ? 0 : fb::KEY_SELECT) |
                   (pinHigh(lo, hi, PIN_X)      ? 0 : fb::KEY_X));
}

fb::Input gIn = {0, 0, 0};

// ---- shared with the 1 kHz sampler (protected by gInMux) -------------------
portMUX_TYPE      gInMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t           gSt      = 0;                 // debounced state
uint8_t           gPressL  = 0;                 // press   edges since the last pollInput()
uint8_t           gRelL    = 0;                 // release edges since the last pollInput()
uint32_t          gChangeUs[kKeyCount] = {0, 0, 0, 0, 0, 0};
volatile uint32_t gPressUs[kKeyCount]  = {0, 0, 0, 0, 0, 0};   // time of latest press edge

// ---- game-loop side only ----------------------------------------------------
uint8_t           gBlocked = 0;                 // keys ignored until physically released
uint8_t           gPrevCur = 0;                 // debounced state seen by the previous poll

// The first edge of a key is accepted immediately; further changes of that same
// key are ignored for FB_DEBOUNCE_US, which swallows contact bounce.
// Caller must hold gInMux.
inline void sampleLocked(uint32_t nowUs) {
  const uint8_t diff = (uint8_t)(readRaw() ^ gSt);
  if (!diff) return;
  const uint8_t raw = (uint8_t)(gSt ^ diff);
  for (uint8_t i = 0; i < kKeyCount; ++i) {
    const uint8_t bit = (uint8_t)(1u << i);
    if ((diff & bit) && (uint32_t)(nowUs - gChangeUs[i]) >= FB_DEBOUNCE_US) {
      gSt ^= bit;
      gChangeUs[i] = nowUs;
      if (raw & bit) { gPressL |= bit; gPressUs[i] = nowUs; }
      else           { gRelL   |= bit; }
    }
  }
}

void inputTimerCb(void*) {
  portENTER_CRITICAL(&gInMux);
  sampleLocked(micros());
  portEXIT_CRITICAL(&gInMux);
}

void inputInit() {
  pinMode(PIN_UP,     INPUT_PULLUP);
  pinMode(PIN_DOWN,   INPUT_PULLUP);
  pinMode(PIN_LEFT,   INPUT_PULLUP);
  pinMode(PIN_RIGHT,  INPUT_PULLUP);
  pinMode(PIN_SELECT, INPUT_PULLUP);
  pinMode(PIN_X,      INPUT_PULLUP);
  gSt      = readRaw();
  gPrevCur = gSt;
  gBlocked = gSt;                       // a key held at power-on does nothing

  esp_timer_create_args_t args = {};
  args.callback = inputTimerCb;
  args.name     = "fbKeys";
  esp_timer_handle_t h = nullptr;
  if (esp_timer_create(&args, &h) == ESP_OK) esp_timer_start_periodic(h, 1000);
}

// Blocks the given keys until they are released (input gating).
inline void gateInput(uint8_t mask)  { gBlocked |= mask; }
inline void gateHeldInputs()         { gBlocked |= gPrevCur; }

// Called once per frame: takes a fresh sample, then hands the game everything
// that happened since the previous call.
inline void pollInput(uint32_t nowUs) {
  uint8_t cur, pL, rL;
  portENTER_CRITICAL(&gInMux);
  sampleLocked(nowUs);
  cur = gSt;
  pL  = gPressL;
  rL  = gRelL;
  gPressL = 0;
  gRelL   = 0;
  portEXIT_CRITICAL(&gInMux);

  const uint8_t wasBlocked = gBlocked;
  gBlocked &= (uint8_t)(cur & ~rL);                         // released (even briefly) -> unblocked
  gIn.held     = (uint8_t)(cur & ~gBlocked);
  gIn.pressed  = (uint8_t)(pL  & ~gBlocked);
  gIn.released = (uint8_t)(rL  & ~wasBlocked);
  gPrevCur = cur;
}

// ============================================================================
//  7. GAME TABLE
// ============================================================================
struct Entry {
  const char*     label;
  const fb::Game* game;        // nullptr when the game's .cpp is not in src/
};

#define FB_TABLE_ROW(id, label) { label, &fbGame_##id },
const Entry kGames[] = { FB_GAME_LIST(FB_TABLE_ROW) };

constexpr uint8_t kGameCount = (uint8_t)(sizeof(kGames) / sizeof(kGames[0]));
static_assert(kGameCount >= 1, "FB_GAME_LIST needs at least one entry");

// ============================================================================
//  8. LAUNCHER / MENU STATE
// ============================================================================
constexpr uint8_t kVisibleRows = 3;

uint8_t  gSel       = 0;            // highlighted entry (kept when returning from a game)
uint8_t  gTop       = 0;            // first visible entry
bool     gMenuDirty = true;
uint32_t gMsgUntil  = 0;            // "NOT INSTALLED" popup deadline (ms), 0 = hidden

const fb::Game* gActive = nullptr;  // nullptr = menu

// Game timing
uint32_t gLastUs    = 0;
uint32_t gFrameIdx  = 0;
uint32_t gFpsT0     = 0;
uint32_t gFpsFrames = 0;
uint16_t gFps       = 0;
fb::Frame gFrame    = {0, 0, 0, 0};

inline char digitChar(uint8_t n) { return (n < 10) ? (char)('0' + n) : '+'; }

void drawMenu() {
  gDisp.clearBuffer();
  gDisp.setFontMode(1);
  gDisp.setDrawColor(1);

  // Header: title, "n/m" counter, rule.
  gDisp.setFont(u8g2_font_7x13B_tr);
  gDisp.drawStr(2, 11, "flipBOX");
  gDisp.setFont(u8g2_font_5x7_tr);
  gDisp.drawGlyph(111, 11, digitChar((uint8_t)(gSel + 1)));
  gDisp.drawGlyph(116, 11, '/');
  gDisp.drawGlyph(121, 11, digitChar(kGameCount));
  gDisp.drawHLine(0, 14, 128);

  // List: rows 15 px apart starting at y=17, selected row is an inverted bar.
  const bool    scroll = kGameCount > kVisibleRows;
  const uint8_t listW  = scroll ? 124 : 128;
  for (uint8_t r = 0; r < kVisibleRows; ++r) {
    const uint8_t idx = (uint8_t)(gTop + r);
    if (idx >= kGameCount) break;
    const uint8_t y   = (uint8_t)(17 + 15 * r);
    const bool    sel = (idx == gSel);
    if (sel) {
      gDisp.drawBox(0, y, listW, 14);
      gDisp.setDrawColor(0);
      gDisp.setFont(u8g2_font_6x13B_tr);
    } else {
      gDisp.setDrawColor(1);
      gDisp.setFont(u8g2_font_6x13_tr);
    }
    gDisp.drawGlyph(4, y + 11, digitChar((uint8_t)(idx + 1)));
    gDisp.drawStr(16, y + 11, kGames[idx].label);
    gDisp.setDrawColor(1);
  }

  // Scroll bar (only when the list does not fit).
  if (scroll) {
    constexpr uint8_t trackY = 17, trackH = 44;
    uint8_t thumbH = (uint8_t)((trackH * kVisibleRows) / kGameCount);
    if (thumbH < 5) thumbH = 5;
    constexpr uint8_t range = (kGameCount > kVisibleRows) ? (uint8_t)(kGameCount - kVisibleRows) : (uint8_t)1;
    const uint8_t thumbY = (uint8_t)(trackY + ((trackH - thumbH) * gTop) / range);
    gDisp.drawVLine(127, trackY, trackH);
    gDisp.drawBox(125, thumbY, 3, thumbH);
  }

  // Popup for a registered game whose .cpp has not been added yet.
  if (gMsgUntil) {
    gDisp.setDrawColor(0);
    gDisp.drawBox(14, 20, 100, 24);
    gDisp.setDrawColor(1);
    gDisp.drawFrame(14, 20, 100, 24);
    gDisp.drawFrame(15, 21, 98, 22);
    gDisp.setFont(u8g2_font_6x13B_tr);
    gDisp.drawStr(25, 37, "NOT INSTALLED");
  }
}

void menuMove(int8_t dir) {
  if (dir < 0) gSel = gSel ? (uint8_t)(gSel - 1) : (uint8_t)(kGameCount - 1);
  else         gSel = (uint8_t)((gSel + 1 >= kGameCount) ? 0 : gSel + 1);
  if (gSel < gTop)                          gTop = gSel;
  else if (gSel >= gTop + kVisibleRows)     gTop = (uint8_t)(gSel - kVisibleRows + 1);
  gMsgUntil  = 0;
  gMenuDirty = true;
}

void enterGame(const fb::Game* g) {
  gActive = g;
  gateHeldInputs();                     // SELECT (etc.) stays dead for the game until released

  gDisp.setDrawColor(1);                // predictable draw state for the game
  gDisp.setFontMode(0);
  gDisp.setBitmapMode(0);
  gDisp.setFontDirection(0);
  gDisp.setFontPosBaseline();

  gFrameIdx  = 0;
  gFpsFrames = 0;
  gFps       = 0;
  gLastUs    = micros();
  gFpsT0     = gLastUs;
  if (g->init) g->init();
}

void leaveGame() {
  if (gActive->shutdown) gActive->shutdown();
  gActive = nullptr;
  gateHeldInputs();                     // the key that exited must not act in the menu
  gMsgUntil  = 0;
  gMenuDirty = true;
}

void runMenu() {
  const uint8_t p = gIn.pressed;
  if (p) {
    if (p & (fb::KEY_UP | fb::KEY_LEFT)) {
      menuMove(-1);
      gateInput(p & (fb::KEY_UP | fb::KEY_LEFT));
    } else if (p & (fb::KEY_DOWN | fb::KEY_RIGHT)) {
      menuMove(+1);
      gateInput(p & (fb::KEY_DOWN | fb::KEY_RIGHT));
    } else if (p & fb::KEY_SELECT) {
      gateInput(fb::KEY_SELECT);
      const fb::Game* g = kGames[gSel].game;
      if (g && g->update && g->render) {
        enterGame(g);
        return;
      }
      gMsgUntil = millis() + 1200u;
      if (gMsgUntil == 0) gMsgUntil = 1;
      gMenuDirty = true;
    }
  }

  if (gMsgUntil && (int32_t)(millis() - gMsgUntil) >= 0) {
    gMsgUntil  = 0;
    gMenuDirty = true;
  }

  if (gMenuDirty) {
    gMenuDirty = false;
    drawMenu();
    presentFrame();
  }
}

#if FB_SHOW_FPS
void drawFpsOverlay() {
  const uint8_t* const prevFont = gDisp.getU8g2()->font;
  char buf[8];
  snprintf(buf, sizeof(buf), "%u", (unsigned)gFps);
  gDisp.setFont(u8g2_font_5x7_tr);
  const uint8_t w = (uint8_t)gDisp.getStrWidth(buf);
  gDisp.setDrawColor(0);
  gDisp.drawBox(128 - w - 3, 0, w + 3, 9);
  gDisp.setDrawColor(1);
  gDisp.drawStr(128 - w - 1, 8, buf);
  gDisp.setFont(prevFont);
}
#endif

inline void runGame(uint32_t nowUs) {
  gFrameNowUs = nowUs;
  uint32_t dt = nowUs - gLastUs;
  gLastUs = nowUs;
  if (dt > FB_MAX_DT_US) dt = FB_MAX_DT_US;

  gFrame.nowMs = millis();
  gFrame.dtUs  = dt;
  gFrame.index = gFrameIdx++;
  gFrame.fps   = gFps;

  if (gActive->update(gIn, gFrame) == fb::GAME_EXIT) {
    leaveGame();
    return;
  }

  gDisp.clearBuffer();
  gActive->render(gDisp);
#if FB_SHOW_FPS
  drawFpsOverlay();
#endif

  uint32_t work = micros() - nowUs;                 // input sample .. render done
  if (work > 50000UL) work = 50000UL;
  gWorkUs = (uint32_t)(gWorkUs - (gWorkUs >> 3) + (work >> 3));
  presentFrame();

  ++gFpsFrames;
  const uint32_t el = nowUs - gFpsT0;
  if (el >= 500000UL) {
    gFps       = (uint16_t)((gFpsFrames * 1000000UL) / el);
    gFpsFrames = 0;
    gFpsT0     = nowUs;
  }
}

}  // namespace

// ============================================================================
//  LAUNCHER EXTRAS (see prototypes near the top of this file)
// ============================================================================
namespace fb {

uint32_t keyPressedAtUs(uint8_t keyMask) {
  for (uint8_t i = 0; i < kKeyCount; ++i)
    if (keyMask & (1u << i)) return gPressUs[i];
  return 0;
}

uint32_t frameStartUs() { return gFrameNowUs; }

uint32_t displayLatencyUs() {
  uint32_t l = gWorkUs + (gWorkUs >> 2) + FB_LATCH_MARGIN_US   // sample -> frame handed over
             + gFlushDurUs                                      // frame -> last byte on the panel
             + FB_PANEL_LATENCY_US;                             // panel scan-out
  if (l > 100000UL) l = 100000UL;
  return l;
}

}  // namespace fb

// ============================================================================
//  9. ARDUINO ENTRY POINTS
// ============================================================================
void setup() {
  inputInit();

#if FB_ASYNC_DISPLAY && FB_FAST_FLUSH
  // The Wire buffer must hold control byte + one chunk; it can only be resized
  // before Wire.begin(), which gDisp.begin() performs.
  {
    uint16_t chunk = FB_I2C_CHUNK;
    if (chunk > 1024) chunk = 1024;
    size_t got = Wire.setBufferSize((size_t)chunk + 1);
    if (got == 0) got = 128;                                    // library default
    while (chunk > 16 && (size_t)chunk + 1 > got) chunk = (uint16_t)(chunk >> 1);
    gFlushChunk = chunk;
  }
#endif

  gDisp.begin();
  gDisp.setBusClock(FB_I2C_HZ);         // after begin(): applied to every transfer
  Wire.setClock(FB_I2C_HZ);             // also for the raw Wire writes of the fast flush

#if FB_OLED_OSC
  {
    u8x8_t* const u8x8 = gDisp.getU8x8();
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, 0x0D5);      // set display clock divide ratio / oscillator frequency
    u8x8_cad_SendArg(u8x8, (uint8_t)FB_OLED_OSC);
    u8x8_cad_EndTransfer(u8x8);
  }
#endif

  gDisp.clearBuffer();

#if FB_ASYNC_DISPLAY
  gFlushDone = xSemaphoreCreateBinary();
  xSemaphoreGive(gFlushDone);           // link starts idle
  xTaskCreatePinnedToCore(flushTask, "fbOLED", 4096, nullptr, 2, &gFlushTaskHandle, 0);
#endif

  gMenuDirty = true;                    // boots straight into the menu
}

void loop() {
  if (gActive) waitForLatch();          // game frames start just-in-time (no-op in the menu)
  const uint32_t now = micros();
  pollInput(now);
  if (gActive) runGame(now);
  else         runMenu();
}
