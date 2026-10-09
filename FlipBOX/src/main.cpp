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
 *     * Buttons are read straight from the GPIO input registers.
 *     * The menu only redraws when something changed.
 * ============================================================================
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================================================
//  1. BUILD OPTIONS (override with -D in platformio.ini if you like)
// ============================================================================
#ifndef FB_I2C_HZ
#define FB_I2C_HZ 800000UL      // OLED bus clock. 400000 = safe default of most
                                // modules; 800000+ = faster. Lower it if the
                                // screen shows noise/garbage.
#endif
#ifndef FB_ASYNC_DISPLAY
#define FB_ASYNC_DISPLAY 1      // 1 = push frames to the OLED from core 0
#endif
#ifndef FB_SHOW_FPS
#define FB_SHOW_FPS 0           // 1 = small FPS counter (top-right) inside games
#endif
#ifndef FB_DEBOUNCE_US
#define FB_DEBOUNCE_US 8000UL   // lock-out after a button edge (first edge is
                                // accepted with zero latency)
#endif
#ifndef FB_MAX_DT_US
#define FB_MAX_DT_US 50000UL    // frame delta handed to games is clamped to this
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
#if FB_ASYNC_DISPLAY
uint8_t           gShadow[1024] __attribute__((aligned(4)));   // frame being sent
SemaphoreHandle_t gFlushDone = nullptr;                        // given when OLED link is idle

void flushTask(void*) {
  u8x8_t* const u8x8 = gDisp.getU8x8();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (uint8_t row = 0; row < 8; ++row) {
      u8x8_DrawTile(u8x8, 0, row, 16, gShadow + (row << 7));
    }
    xSemaphoreGive(gFlushDone);
  }
}
TaskHandle_t gFlushTaskHandle = nullptr;
#endif

// Hands the finished frame to the OLED. With async display this returns as soon
// as the 1 KB snapshot is taken; it only waits if the previous frame is still
// travelling over I2C (that wait is the natural, hardware-limited frame pacing).
inline void presentFrame() {
#if FB_ASYNC_DISPLAY
  xSemaphoreTake(gFlushDone, portMAX_DELAY);
  memcpy(gShadow, gDisp.getBufferPtr(), sizeof(gShadow));
  xTaskNotifyGive(gFlushTaskHandle);
#else
  gDisp.sendBuffer();
#endif
}

// ============================================================================
//  6. INPUT  (6 buttons, direct register read, zero-latency debounce + gating)
// ============================================================================
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
uint8_t   gStable  = 0;                 // debounced state
uint8_t   gBlocked = 0;                 // keys ignored until physically released
uint32_t  gChangeUs[kKeyCount] = {0, 0, 0, 0, 0, 0};

void inputInit() {
  pinMode(PIN_UP,     INPUT_PULLUP);
  pinMode(PIN_DOWN,   INPUT_PULLUP);
  pinMode(PIN_LEFT,   INPUT_PULLUP);
  pinMode(PIN_RIGHT,  INPUT_PULLUP);
  pinMode(PIN_SELECT, INPUT_PULLUP);
  pinMode(PIN_X,      INPUT_PULLUP);
  gStable  = readRaw();
  gBlocked = gStable;                   // a key held at power-on does nothing
}

// Blocks the given keys until they are released (input gating).
inline void gateInput(uint8_t mask)  { gBlocked |= mask; }
inline void gateHeldInputs()         { gBlocked |= gStable; }

// The first edge of a key is accepted immediately; further changes of that same
// key are ignored for FB_DEBOUNCE_US, which swallows contact bounce.
inline void pollInput(uint32_t nowUs) {
  const uint8_t prev = gStable;
  const uint8_t diff = readRaw() ^ prev;
  if (diff) {
    for (uint8_t i = 0; i < kKeyCount; ++i) {
      const uint8_t bit = (uint8_t)(1u << i);
      if ((diff & bit) && (uint32_t)(nowUs - gChangeUs[i]) >= FB_DEBOUNCE_US) {
        gStable ^= bit;
        gChangeUs[i] = nowUs;
      }
    }
  }
  const uint8_t cur        = gStable;
  const uint8_t wasBlocked = gBlocked;
  gBlocked &= cur;                                          // released -> unblocked
  gIn.held     = (uint8_t)(cur & ~gBlocked);
  gIn.pressed  = (uint8_t)((cur & ~prev) & ~gBlocked);
  gIn.released = (uint8_t)((prev & ~cur) & ~wasBlocked);
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
//  9. ARDUINO ENTRY POINTS
// ============================================================================
void setup() {
  inputInit();

  gDisp.begin();
  gDisp.setBusClock(FB_I2C_HZ);         // after begin(): applied to every transfer
  gDisp.clearBuffer();

#if FB_ASYNC_DISPLAY
  gFlushDone = xSemaphoreCreateBinary();
  xSemaphoreGive(gFlushDone);           // link starts idle
  xTaskCreatePinnedToCore(flushTask, "fbOLED", 4096, nullptr, 2, &gFlushTaskHandle, 0);
#endif

  gMenuDirty = true;                    // boots straight into the menu
}

void loop() {
  const uint32_t now = micros();
  pollInput(now);
  if (gActive) runGame(now);
  else         runMenu();
}
