/*
 * ============================================================================
 *  flipBOX  --  Launcher  (main.cpp, Wave 1)  +  Options / Settings
 *  ESP32-WROOM-32 | Arduino | PlatformIO | U8g2 | SSD1306 128x64 I2C | 6 keys
 * ============================================================================
 *
 *  FLOW
 *     boot -> [password gate if enabled] -> game-selection menu
 *           -> (hold UP 1s) -> options menu
 *           -> selected game -> (game exits) -> menu
 *
 *  OPTIONS (hold UP 1s on main menu; hold DOWN 1s to leave options)
 *     1. Brightness   – 1 % … 100 % (never fully black)
 *     2. Password     – require UP-DOWN-UP-DOWN on every boot
 *     3. Airplane     – cosmetic toggle only
 *     4. Troll Mode   – screen becomes cracky / flaky; hold SELECT 4 s to kill
 *     5 / 6. (blank – hoverable, not selectable)
 *
 *  All settings live in NVS (Preferences) so they survive power cycles.
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
#include <Preferences.h>

// ============================================================================
//  1. BUILD OPTIONS (override with -D in platformio.ini if you like)
// ============================================================================
#ifndef FB_I2C_HZ
#define FB_I2C_HZ 1000000UL
#endif
#ifndef FB_ASYNC_DISPLAY
#define FB_ASYNC_DISPLAY 1
#endif
#ifndef FB_SHOW_FPS
#define FB_SHOW_FPS 0
#endif
#ifndef FB_DEBOUNCE_US
#define FB_DEBOUNCE_US 5000UL
#endif
#ifndef FB_MAX_DT_US
#define FB_MAX_DT_US 50000UL
#endif
#ifndef FB_FAST_FLUSH
#define FB_FAST_FLUSH 1
#endif
#ifndef FB_I2C_CHUNK
#define FB_I2C_CHUNK 256
#endif
#ifndef FB_LATE_LATCH
#define FB_LATE_LATCH 1
#endif
#ifndef FB_LATCH_MARGIN_US
#define FB_LATCH_MARGIN_US 400UL
#endif
#ifndef FB_PANEL_LATENCY_US
#define FB_PANEL_LATENCY_US 5000UL
#endif
#ifndef FB_OLED_OSC
#define FB_OLED_OSC 0xF0
#endif

// ============================================================================
//  2. GAME REGISTRY  --  ONE line per game:  X(symbol, "menu label")
// ============================================================================
#define FB_GAME_LIST(X)                                    \
  X(F1Racing,       "F1 Racing")                           \
  X(FastAndFurious, "Fast and Furious")                    \
  X(TokyoDrift,     "Tokyo Drift")                         \
  X(TwoFast2Furious, "2 Fast 2 Furious") \
  X(HillClimb, "Hill Climb Racing") \
  X(MiamiRun, "Miami Run")

// ============================================================================
//  3. FLIPBOX GAME CONTRACT v1
// ============================================================================
// ----- FLIPBOX GAME CONTRACT v1 : BEGIN ------------------------------------
#include <Arduino.h>
#include <U8g2lib.h>

namespace fb {

constexpr uint8_t SCREEN_W = 128;
constexpr uint8_t SCREEN_H = 64;

constexpr uint8_t KEY_UP     = 1u << 0;
constexpr uint8_t KEY_DOWN   = 1u << 1;
constexpr uint8_t KEY_LEFT   = 1u << 2;
constexpr uint8_t KEY_RIGHT  = 1u << 3;
constexpr uint8_t KEY_SELECT = 1u << 4;
constexpr uint8_t KEY_X      = 1u << 5;

struct Input {
  uint8_t held;
  uint8_t pressed;
  uint8_t released;
};

struct Frame {
  uint32_t nowMs;
  uint32_t dtUs;
  uint32_t index;
  uint16_t fps;
};

enum Status : uint8_t { GAME_RUN = 0, GAME_EXIT = 1 };

struct Game {
  void   (*init)();
  Status (*update)(const Input& in, const Frame& f);
  void   (*render)(U8G2& d);
  void   (*shutdown)();
};

}  // namespace fb

#define FB_GAME(id, initFn, updateFn, renderFn, shutdownFn)                    \
  extern const fb::Game fbGame_##id;                                           \
  const fb::Game fbGame_##id = { initFn, updateFn, renderFn, shutdownFn }
// ----- FLIPBOX GAME CONTRACT v1 : END --------------------------------------

namespace fb {
uint32_t keyPressedAtUs(uint8_t keyMask);
uint32_t frameStartUs();
uint32_t displayLatencyUs();
}

// Weak references
#define FB_DECLARE_WEAK(id, label) extern const fb::Game fbGame_##id __attribute__((weak));
FB_GAME_LIST(FB_DECLARE_WEAK)

namespace {

// ============================================================================
//  4. HARDWARE
// ============================================================================
constexpr uint8_t PIN_SDA    = 21;
constexpr uint8_t PIN_SCL    = 22;
constexpr uint8_t PIN_UP     = 26;
constexpr uint8_t PIN_DOWN   = 33;
constexpr uint8_t PIN_LEFT   = 25;
constexpr uint8_t PIN_RIGHT  = 27;
constexpr uint8_t PIN_SELECT = 32;
constexpr uint8_t PIN_X      = 4;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C gDisp(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);

// ============================================================================
//  5. DISPLAY PRESENTATION
// ============================================================================
volatile uint32_t gFlushDurUs   = 12000;
volatile uint32_t gFlushStartUs = 0;
volatile bool     gFlushBusy    = false;
uint32_t          gWorkUs       = 3000;
uint32_t          gFrameNowUs   = 0;

#if FB_ASYNC_DISPLAY
uint8_t           gShadow[1024] __attribute__((aligned(4)));
SemaphoreHandle_t gFlushDone = nullptr;
TaskHandle_t      gFlushTaskHandle = nullptr;

#if FB_FAST_FLUSH
uint16_t gFlushChunk = 32;
bool     gNeedWindow = true;

bool fastFlush() {
  uint8_t a8 = gDisp.getU8x8()->i2c_address;
  if (a8 == 255) a8 = 0x78;
  const uint8_t addr = (uint8_t)(a8 >> 1);

  if (gNeedWindow) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x20); Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x21); Wire.write((uint8_t)0x00); Wire.write((uint8_t)127);
    Wire.write((uint8_t)0x22); Wire.write((uint8_t)0x00); Wire.write((uint8_t)7);
    if (Wire.endTransmission() != 0) return false;
    gNeedWindow = false;
  }
  for (uint16_t off = 0; off < sizeof(gShadow); off = (uint16_t)(off + gFlushChunk)) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x40);
    Wire.write(gShadow + off, (size_t)gFlushChunk);
    if (Wire.endTransmission() != 0) {
      gNeedWindow = true;
      return false;
    }
  }
  return true;
}
#endif

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
    if (d > 60000UL) d = 60000UL;
    gFlushDurUs = (uint32_t)((gFlushDurUs * 3UL + d) >> 2);
    gFlushBusy  = false;
    xSemaphoreGive(gFlushDone);
  }
}
#endif

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

inline void waitForLatch() {
#if FB_ASYNC_DISPLAY && FB_LATE_LATCH
  const uint32_t workEst = gWorkUs + (gWorkUs >> 2) + FB_LATCH_MARGIN_US;
  for (;;) {
    if (!gFlushBusy) return;
    const int32_t remain = (int32_t)(gFlushStartUs + gFlushDurUs - workEst - micros());
    if (remain <= 0 || remain > 100000) return;
    if (remain > 1500 && portTICK_PERIOD_MS == 1) vTaskDelay(1);
  }
#endif
}

// ============================================================================
//  6. INPUT
// ============================================================================
constexpr uint8_t kKeyCount = 6;

constexpr bool pinHigh(uint32_t lo, uint32_t hi, uint8_t pin) {
  return (pin < 32) ? (((lo >> pin) & 1u) != 0u) : (((hi >> (pin - 32)) & 1u) != 0u);
}

inline uint8_t readRaw() {
  const uint32_t lo = REG_READ(GPIO_IN_REG);
  const uint32_t hi = REG_READ(GPIO_IN1_REG);
  return (uint8_t)((pinHigh(lo, hi, PIN_UP)     ? 0 : fb::KEY_UP)    |
                   (pinHigh(lo, hi, PIN_DOWN)   ? 0 : fb::KEY_DOWN)  |
                   (pinHigh(lo, hi, PIN_LEFT)   ? 0 : fb::KEY_LEFT)  |
                   (pinHigh(lo, hi, PIN_RIGHT)  ? 0 : fb::KEY_RIGHT) |
                   (pinHigh(lo, hi, PIN_SELECT) ? 0 : fb::KEY_SELECT) |
                   (pinHigh(lo, hi, PIN_X)      ? 0 : fb::KEY_X));
}

fb::Input gIn = {0, 0, 0};

portMUX_TYPE      gInMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t           gSt      = 0;
uint8_t           gPressL  = 0;
uint8_t           gRelL    = 0;
uint32_t          gChangeUs[kKeyCount] = {0, 0, 0, 0, 0, 0};
volatile uint32_t gPressUs[kKeyCount]  = {0, 0, 0, 0, 0, 0};

uint8_t           gBlocked = 0;
uint8_t           gPrevCur = 0;

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
  gBlocked = gSt;

  esp_timer_create_args_t args = {};
  args.callback = inputTimerCb;
  args.name     = "fbKeys";
  esp_timer_handle_t h = nullptr;
  if (esp_timer_create(&args, &h) == ESP_OK) esp_timer_start_periodic(h, 1000);
}

inline void gateInput(uint8_t mask)  { gBlocked |= mask; }
inline void gateHeldInputs()         { gBlocked |= gPrevCur; }

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
  gBlocked &= (uint8_t)(cur & ~rL);
  gIn.held     = (uint8_t)(cur & ~gBlocked);
  gIn.pressed  = (uint8_t)(pL  & ~gBlocked);
  gIn.released = (uint8_t)(rL  & ~wasBlocked);
  gPrevCur = cur;
}

// ============================================================================
//  7. SETTINGS  (NVS / Preferences)
// ============================================================================
Preferences gPrefs;

struct Settings {
  uint8_t brightness;      // 1 … 100
  bool    passwordOn;
  bool    airplane;
  bool    troll;
};

Settings gCfg = { 80, false, false, false };

void applyBrightness() {
  // Map 1-100 → 8-255 so the panel never goes fully black
  uint8_t c = (uint8_t)(8 + ((uint16_t)(gCfg.brightness - 1) * 247) / 99);
  gDisp.setContrast(c);
}

void loadSettings() {
  gPrefs.begin("flipbox", true);
  gCfg.brightness  = gPrefs.getUChar("bri", 80);
  if (gCfg.brightness < 1)  gCfg.brightness = 1;
  if (gCfg.brightness > 100) gCfg.brightness = 100;
  gCfg.passwordOn  = gPrefs.getBool("pwd", false);
  gCfg.airplane    = gPrefs.getBool("air", false);
  gCfg.troll       = gPrefs.getBool("trl", false);
  gPrefs.end();
}

void saveSettings() {
  gPrefs.begin("flipbox", false);
  gPrefs.putUChar("bri", gCfg.brightness);
  gPrefs.putBool ("pwd", gCfg.passwordOn);
  gPrefs.putBool ("air", gCfg.airplane);
  gPrefs.putBool ("trl", gCfg.troll);
  gPrefs.end();
}

// ============================================================================
//  8. UI STATE MACHINE
// ============================================================================
enum UiMode : uint8_t {
  UI_PASSWORD = 0,   // boot gate
  UI_MENU,
  UI_SETTINGS,
  UI_BRIGHTNESS,
  UI_GAME
};

UiMode   gMode          = UI_MENU;
uint8_t  gSel           = 0;
uint8_t  gTop           = 0;
bool     gMenuDirty     = true;
uint32_t gMsgUntil      = 0;

const fb::Game* gActive = nullptr;

// Hold tracking for long-press actions
uint32_t gHoldStartMs   = 0;     // when the relevant button first went down
uint8_t  gHoldKey       = 0;     // which key we are currently tracking
bool     gHoldArmed     = false; // true while a qualifying hold is in progress

// Settings grid
constexpr uint8_t kOptCount = 6;
uint8_t  gOptSel        = 0;     // 0-5
bool     gSettingsDirty = true;

// Open / close animation for the settings panel (0 = closed, 255 = fully open)
uint8_t  gSetAnim       = 0;
bool     gSetAnimIn     = true;  // true = opening, false = closing

// Brightness editor
uint8_t  gBriTemp       = 80;

// Password sequence: UP DOWN UP DOWN
constexpr uint8_t kPwdLen = 4;
const uint8_t kPwdSeq[kPwdLen] = {
  fb::KEY_UP, fb::KEY_DOWN, fb::KEY_UP, fb::KEY_DOWN
};
uint8_t  gPwdIdx        = 0;
bool     gPwdWrong      = false;
uint32_t gPwdWrongUntil = 0;

// Troll-mode disable hold
uint32_t gTrollHoldStart = 0;
bool     gTrollHoldArmed = false;

// Simple PRNG for troll visuals (xorshift)
uint32_t gRng = 0xA5A5C3C3u;
inline uint32_t rnd() {
  gRng ^= gRng << 13;
  gRng ^= gRng >> 17;
  gRng ^= gRng << 5;
  return gRng;
}

// ============================================================================
//  9. GAME TABLE
// ============================================================================
struct Entry {
  const char*     label;
  const fb::Game* game;
};

#define FB_TABLE_ROW(id, label) { label, &fbGame_##id },
const Entry kGames[] = { FB_GAME_LIST(FB_TABLE_ROW) };
constexpr uint8_t kGameCount = (uint8_t)(sizeof(kGames) / sizeof(kGames[0]));
static_assert(kGameCount >= 1, "FB_GAME_LIST needs at least one entry");

constexpr uint8_t kVisibleRows = 3;

// Game timing
uint32_t gLastUs    = 0;
uint32_t gFrameIdx  = 0;
uint32_t gFpsT0     = 0;
uint32_t gFpsFrames = 0;
uint16_t gFps       = 0;
fb::Frame gFrame    = {0, 0, 0, 0};

inline char digitChar(uint8_t n) { return (n < 10) ? (char)('0' + n) : '+'; }

// ============================================================================
// 10. DRAWING HELPERS (premium monochrome UI)
// ============================================================================
void drawRoundedBox(int x, int y, int w, int h, int r) {
  // Soft rounded rectangle (filled)
  gDisp.drawBox(x + r, y, w - 2 * r, h);
  gDisp.drawBox(x, y + r, w, h - 2 * r);
  gDisp.drawDisc(x + r, y + r, r);
  gDisp.drawDisc(x + w - r - 1, y + r, r);
  gDisp.drawDisc(x + r, y + h - r - 1, r);
  gDisp.drawDisc(x + w - r - 1, y + h - r - 1, r);
}

void drawRoundedFrame(int x, int y, int w, int h, int r) {
  gDisp.drawHLine(x + r, y, w - 2 * r);
  gDisp.drawHLine(x + r, y + h - 1, w - 2 * r);
  gDisp.drawVLine(x, y + r, h - 2 * r);
  gDisp.drawVLine(x + w - 1, y + r, h - 2 * r);
  // corner arcs approximated with pixels
  gDisp.drawPixel(x + 1, y + 1);
  gDisp.drawPixel(x + w - 2, y + 1);
  gDisp.drawPixel(x + 1, y + h - 2);
  gDisp.drawPixel(x + w - 2, y + h - 2);
  gDisp.drawPixel(x + 2, y);
  gDisp.drawPixel(x + w - 3, y);
  gDisp.drawPixel(x + 2, y + h - 1);
  gDisp.drawPixel(x + w - 3, y + h - 1);
  gDisp.drawPixel(x, y + 2);
  gDisp.drawPixel(x + w - 1, y + 2);
  gDisp.drawPixel(x, y + h - 3);
  gDisp.drawPixel(x + w - 1, y + h - 3);
}

// Progress bar for long-press feedback
void drawHoldBar(uint8_t progress /*0-100*/, int y = 60) {
  const int barW = 100;
  const int barX = (128 - barW) / 2;
  gDisp.setDrawColor(1);
  gDisp.drawFrame(barX, y, barW, 4);
  int fill = (barW - 2) * progress / 100;
  if (fill > 0) gDisp.drawBox(barX + 1, y + 1, fill, 2);
}

// ============================================================================
// 11. TROLL MODE OVERLAY  (“virus” visual noise)
// ============================================================================
void applyTrollOverlay() {
  if (!gCfg.troll) return;

  // Random horizontal / vertical cracks
  for (int i = 0; i < 6; ++i) {
    int x0 = (int)(rnd() % 128);
    int y0 = (int)(rnd() % 64);
    int len = 8 + (int)(rnd() % 24);
    if (rnd() & 1) {
      gDisp.drawHLine(x0, y0, len);
    } else {
      gDisp.drawVLine(x0, y0, len);
    }
  }

  // Scattered blobs / “flakes”
  for (int i = 0; i < 12; ++i) {
    int x = (int)(rnd() % 128);
    int y = (int)(rnd() % 64);
    int r = 1 + (int)(rnd() % 3);
    if (rnd() & 1) gDisp.drawDisc(x, y, r);
    else           gDisp.drawCircle(x, y, r);
  }

  // Occasional full-width glitch lines
  if ((rnd() & 0x1F) < 4) {
    int y = (int)(rnd() % 64);
    gDisp.drawHLine(0, y, 128);
    if (y + 1 < 64) gDisp.drawHLine(0, y + 1, 128);
  }

  // Soft “blur” by random pixel inversion in a few bands
  for (int b = 0; b < 3; ++b) {
    int y = (int)(rnd() % 56);
    for (int x = 0; x < 128; x += 2 + (int)(rnd() % 3)) {
      // invert a single pixel (read-modify via draw)
      // U8g2 has no getPixel, so we just sprinkle extra noise
      gDisp.drawPixel(x, y + (int)(rnd() % 8));
    }
  }
}

// ============================================================================
// 12. MENU DRAWING
// ============================================================================
void drawMenu() {
  gDisp.clearBuffer();
  gDisp.setFontMode(1);
  gDisp.setDrawColor(1);

  // Header
  gDisp.setFont(u8g2_font_7x13B_tr);
  gDisp.drawStr(2, 11, "flipBOX");
  gDisp.setFont(u8g2_font_5x7_tr);
  gDisp.drawGlyph(111, 11, digitChar((uint8_t)(gSel + 1)));
  gDisp.drawGlyph(116, 11, '/');
  gDisp.drawGlyph(121, 11, digitChar(kGameCount));
  gDisp.drawHLine(0, 14, 128);

  // Airplane indicator (tiny plane icon when active)
  if (gCfg.airplane) {
    gDisp.setFont(u8g2_font_5x7_tr);
    gDisp.drawStr(78, 11, "AIR");
  }

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

  if (scroll) {
    constexpr uint8_t trackY = 17, trackH = 44;
    uint8_t thumbH = (uint8_t)((trackH * kVisibleRows) / kGameCount);
    if (thumbH < 5) thumbH = 5;
    constexpr uint8_t range = (kGameCount > kVisibleRows) ? (uint8_t)(kGameCount - kVisibleRows) : (uint8_t)1;
    const uint8_t thumbY = (uint8_t)(trackY + ((trackH - thumbH) * gTop) / range);
    gDisp.drawVLine(127, trackY, trackH);
    gDisp.drawBox(125, thumbY, 3, thumbH);
  }

  if (gMsgUntil) {
    gDisp.setDrawColor(0);
    gDisp.drawBox(14, 20, 100, 24);
    gDisp.setDrawColor(1);
    gDisp.drawFrame(14, 20, 100, 24);
    gDisp.drawFrame(15, 21, 98, 22);
    gDisp.setFont(u8g2_font_6x13B_tr);
    gDisp.drawStr(25, 37, "NOT INSTALLED");
  }

  // Hold-UP progress bar (entering settings) – only after 500 ms
  if (gHoldArmed && gHoldKey == fb::KEY_UP && gMode == UI_MENU) {
    uint32_t held = millis() - gHoldStartMs;
    if (held >= 500) {
      if (held > 1000) held = 1000;
      // map 500→1000 ms into 0→100 %
      drawHoldBar((uint8_t)((held - 500) * 100 / 500));
    }
  }

  // Troll-kill progress bar
  if (gTrollHoldArmed && gCfg.troll) {
    uint32_t held = millis() - gTrollHoldStart;
    if (held > 4000) held = 4000;
    drawHoldBar((uint8_t)(held / 40), 54);
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

// ============================================================================
// 13. SETTINGS SCREEN  (pure GUI 2×3 icon grid – no text)
// ============================================================================
const bool kOptSelectable[kOptCount] = {
  true, true, true, true, false, false
};

// Evenly spaced centres for 128×64 with no labels
void optCentre(uint8_t idx, int& cx, int& cy) {
  const int col = idx % 3;
  const int row = idx / 3;
  cx = 24 + col * 40;   // 24, 64, 104
  cy = 18 + row * 28;   // 18, 46
}

// Classic trollface simplified for ~14 px icon
void drawTrollFace(int cx, int cy) {
  // Head outline
  gDisp.drawCircle(cx, cy, 7);
  // Jaw bulge
  gDisp.drawPixel(cx - 6, cy + 4);
  gDisp.drawPixel(cx + 6, cy + 4);
  gDisp.drawPixel(cx - 5, cy + 5);
  gDisp.drawPixel(cx + 5, cy + 5);

  // Eyes (narrow slits)
  gDisp.drawHLine(cx - 5, cy - 2, 3);
  gDisp.drawHLine(cx + 2, cy - 2, 3);
  gDisp.drawPixel(cx - 4, cy - 3);
  gDisp.drawPixel(cx + 3, cy - 3);

  // Nose / cheeks
  gDisp.drawPixel(cx - 1, cy);
  gDisp.drawPixel(cx + 1, cy);

  // Huge grin
  gDisp.drawHLine(cx - 5, cy + 2, 11);
  // teeth
  gDisp.drawPixel(cx - 4, cy + 3);
  gDisp.drawPixel(cx - 2, cy + 3);
  gDisp.drawPixel(cx,     cy + 3);
  gDisp.drawPixel(cx + 2, cy + 3);
  gDisp.drawPixel(cx + 4, cy + 3);
  // lower lip
  gDisp.drawHLine(cx - 4, cy + 4, 9);
  // mouth corners
  gDisp.drawPixel(cx - 6, cy + 1);
  gDisp.drawPixel(cx + 6, cy + 1);
  gDisp.drawPixel(cx - 5, cy + 1);
  gDisp.drawPixel(cx + 5, cy + 1);
}

void drawOptIcon(uint8_t idx, bool selected, int animOffsetY) {
  int cx, cy;
  optCentre(idx, cx, cy);
  cy += animOffsetY;

  const bool blank = !kOptSelectable[idx];
  const int  r     = 13;

  // Selection ring
  gDisp.setDrawColor(1);
  if (selected) {
    // solid outer disc + hollow centre = strong focus
    gDisp.drawDisc(cx, cy, r);
    gDisp.setDrawColor(0);
    gDisp.drawDisc(cx, cy, r - 3);
    gDisp.setDrawColor(1);
  } else if (!blank) {
    gDisp.drawCircle(cx, cy, r);
    gDisp.drawCircle(cx, cy, r - 1);
  } else {
    gDisp.drawCircle(cx, cy, r - 1);
  }

  // Glyph
  if (blank) {
    gDisp.drawHLine(cx - 3, cy, 6);
  } else {
    switch (idx) {
      case 0: { // brightness – sun
        gDisp.drawDisc(cx, cy, 3);
        gDisp.drawVLine(cx,     cy - 9, 3);
        gDisp.drawVLine(cx,     cy + 6, 3);
        gDisp.drawHLine(cx - 9, cy,     3);
        gDisp.drawHLine(cx + 6, cy,     3);
        gDisp.drawPixel(cx - 6, cy - 6); gDisp.drawPixel(cx - 7, cy - 7);
        gDisp.drawPixel(cx + 6, cy - 6); gDisp.drawPixel(cx + 7, cy - 7);
        gDisp.drawPixel(cx - 6, cy + 6); gDisp.drawPixel(cx - 7, cy + 7);
        gDisp.drawPixel(cx + 6, cy + 6); gDisp.drawPixel(cx + 7, cy + 7);
        break;
      }
      case 1: { // password – lock
        gDisp.drawFrame(cx - 5, cy - 1, 10, 7);
        gDisp.drawBox(cx - 2, cy + 1, 4, 3);
        gDisp.drawCircle(cx, cy - 4, 3);
        gDisp.setDrawColor(0);
        gDisp.drawDisc(cx, cy - 4, 1);
        gDisp.setDrawColor(1);
        break;
      }
      case 2: { // airplane
        gDisp.drawHLine(cx - 7, cy, 14);
        gDisp.drawHLine(cx - 7, cy + 1, 14);
        gDisp.drawLine(cx - 1, cy, cx - 5, cy - 5);
        gDisp.drawLine(cx,     cy, cx - 4, cy - 5);
        gDisp.drawLine(cx - 1, cy, cx - 5, cy + 5);
        gDisp.drawLine(cx,     cy, cx - 4, cy + 5);
        gDisp.drawLine(cx + 5, cy, cx + 7, cy - 3);
        gDisp.drawLine(cx + 5, cy, cx + 7, cy + 3);
        break;
      }
      case 3: { // trollface
        drawTrollFace(cx, cy);
        break;
      }
    }
  }

  // ON indicator – filled dot at top-right of the circle
  bool isOn = false;
  if (idx == 1) isOn = gCfg.passwordOn;
  if (idx == 2) isOn = gCfg.airplane;
  if (idx == 3) isOn = gCfg.troll;
  if (isOn) {
    gDisp.setDrawColor(1);
    gDisp.drawDisc(cx + r - 2, cy - r + 2, 3);
    if (selected) {
      gDisp.setDrawColor(0);
      gDisp.drawDisc(cx + r - 2, cy - r + 2, 1);
      gDisp.setDrawColor(1);
    }
  }
}

void drawSettings() {
  gDisp.clearBuffer();
  gDisp.setFontMode(1);
  gDisp.setDrawColor(1);

  // Slide-in animation
  int offsetY = 0;
  if (gSetAnim < 255) {
    offsetY = (int)((255 - gSetAnim) * 36 / 255);
  }

  // Pure icon grid – no header, no footer text
  for (uint8_t i = 0; i < kOptCount; ++i) {
    drawOptIcon(i, i == gOptSel, offsetY);
  }

  // Hold-DOWN progress while leaving – only after 500 ms
  if (gHoldArmed && gHoldKey == fb::KEY_DOWN && gMode == UI_SETTINGS) {
    uint32_t held = millis() - gHoldStartMs;
    if (held >= 500) {
      if (held > 1000) held = 1000;
      // map 500→1000 ms into 0→100 %
      drawHoldBar((uint8_t)((held - 500) * 100 / 500), 60);
    }
  }

  // Troll-kill progress
  if (gTrollHoldArmed && gCfg.troll) {
    uint32_t held = millis() - gTrollHoldStart;
    if (held > 4000) held = 4000;
    drawHoldBar((uint8_t)(held / 40), 60);
  }
}

// ============================================================================
// 14. BRIGHTNESS EDITOR
// ============================================================================
void drawBrightness() {
  gDisp.clearBuffer();
  gDisp.setFontMode(1);
  gDisp.setDrawColor(1);

  gDisp.setFont(u8g2_font_7x13B_tr);
  gDisp.drawStr(28, 12, "BRIGHTNESS");
  gDisp.drawHLine(0, 15, 128);

  // Big percentage
  char buf[8];
  snprintf(buf, sizeof(buf), "%u%%", (unsigned)gBriTemp);
  gDisp.setFont(u8g2_font_logisoso16_tr);
  int tw = gDisp.getStrWidth(buf);
  gDisp.drawStr((128 - tw) / 2, 40, buf);

  // Slider track
  const int trackX = 14, trackW = 100, trackY = 48;
  gDisp.drawFrame(trackX, trackY, trackW, 8);
  int fill = (trackW - 2) * gBriTemp / 100;
  if (fill > 0) gDisp.drawBox(trackX + 1, trackY + 1, fill, 6);

  // Tick marks at 25/50/75
  for (int p = 25; p <= 75; p += 25) {
    int x = trackX + (trackW * p / 100);
    gDisp.drawVLine(x, trackY - 2, 2);
  }

  gDisp.setFont(u8g2_font_4x6_tr);
  gDisp.drawStr(2, 63, "UP/DN adjust  SEL save  X cancel");
}

// ============================================================================
// 15. PASSWORD GATE
// ============================================================================
void drawPassword() {
  gDisp.clearBuffer();
  gDisp.setFontMode(1);
  gDisp.setDrawColor(1);

  gDisp.setFont(u8g2_font_7x13B_tr);
  gDisp.drawStr(22, 14, "ENTER CODE");
  gDisp.drawHLine(0, 17, 128);

  // Four dots / filled when entered
  for (uint8_t i = 0; i < kPwdLen; ++i) {
    int x = 28 + i * 20;
    if (i < gPwdIdx) gDisp.drawDisc(x, 34, 5);
    else             gDisp.drawCircle(x, 34, 5);
  }

  if (gPwdWrong) {
    gDisp.setFont(u8g2_font_6x13B_tr);
    gDisp.drawStr(34, 54, "WRONG");
  } else {
    gDisp.setFont(u8g2_font_5x7_tr);
    gDisp.drawStr(18, 54, "UP  DN  UP  DN");
  }
}

// ============================================================================
// 16. GAME ENTER / LEAVE
// ============================================================================
void enterGame(const fb::Game* g) {
  gActive = g;
  gMode   = UI_GAME;
  gateHeldInputs();

  gDisp.setDrawColor(1);
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
  if (gActive && gActive->shutdown) gActive->shutdown();
  gActive = nullptr;
  gMode   = UI_MENU;
  gateHeldInputs();
  gMsgUntil  = 0;
  gMenuDirty = true;
}

// ============================================================================
// 17. MODE RUNNERS
// ============================================================================
void runPassword() {
  if (gPwdWrong && (int32_t)(millis() - gPwdWrongUntil) >= 0) {
    gPwdWrong = false;
    gPwdIdx   = 0;
  }

  const uint8_t p = gIn.pressed;
  if (p && !gPwdWrong) {
    uint8_t expected = kPwdSeq[gPwdIdx];
    if (p & expected) {
      gPwdIdx++;
      if (gPwdIdx >= kPwdLen) {
        gMode = UI_MENU;
        gMenuDirty = true;
        gateHeldInputs();
      }
    } else if (p & (fb::KEY_UP | fb::KEY_DOWN | fb::KEY_LEFT | fb::KEY_RIGHT | fb::KEY_SELECT | fb::KEY_X)) {
      gPwdWrong = true;
      gPwdWrongUntil = millis() + 800;
      gPwdIdx = 0;
    }
    gateInput(p);
  }

  drawPassword();
  applyTrollOverlay();
  presentFrame();
}

void runMenu() {
  // Global troll-kill (hold SELECT 4 s) even while on the menu
  if (gCfg.troll) {
    if (gIn.held & fb::KEY_SELECT) {
      if (!gTrollHoldArmed) {
        gTrollHoldArmed = true;
        gTrollHoldStart = millis();
        gMenuDirty = true;
      } else {
        uint32_t held = millis() - gTrollHoldStart;
        if (held >= 4000) {
          gCfg.troll = false;
          saveSettings();
          gTrollHoldArmed = false;
          gateInput(fb::KEY_SELECT);
          gMenuDirty = true;
        } else {
          gMenuDirty = true;
        }
      }
    } else {
      if (gTrollHoldArmed) { gTrollHoldArmed = false; gMenuDirty = true; }
    }
  }

  // --- long-press UP to open settings (bar only after 500 ms) ---
  if (gIn.held & fb::KEY_UP) {
    if (!gHoldArmed || gHoldKey != fb::KEY_UP) {
      gHoldArmed   = true;
      gHoldKey     = fb::KEY_UP;
      gHoldStartMs = millis();
    } else {
      uint32_t held = millis() - gHoldStartMs;
      if (held >= 1000) {
        // open settings
        gHoldArmed     = false;
        gMode          = UI_SETTINGS;
        gOptSel        = 0;
        gSetAnim       = 0;
        gSetAnimIn     = true;
        gSettingsDirty = true;
        gateInput(fb::KEY_UP);
        return;
      }
      // redraw only once the bar is visible (>= 500 ms)
      if (held >= 500) gMenuDirty = true;
    }
  } else {
    if (gHoldArmed && gHoldKey == fb::KEY_UP) {
      gHoldArmed = false;
      gMenuDirty = true;   // clear bar if it was showing
    }
  }

  // Short-press navigation always works (not blocked by hold tracking)
  const uint8_t p = gIn.pressed;
  if (p) {
    if (p & (fb::KEY_UP | fb::KEY_LEFT)) {
      menuMove(-1);
      // do NOT gate UP here – allow the hold timer to keep running
      if (p & fb::KEY_LEFT) gateInput(fb::KEY_LEFT);
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
    applyTrollOverlay();
    presentFrame();
  }
}

void runSettings() {
  // Global troll-kill also works here
  if (gCfg.troll) {
    if (gIn.held & fb::KEY_SELECT) {
      if (!gTrollHoldArmed) {
        gTrollHoldArmed = true;
        gTrollHoldStart = millis();
        gSettingsDirty = true;
      } else if (millis() - gTrollHoldStart >= 4000) {
        gCfg.troll = false;
        saveSettings();
        gTrollHoldArmed = false;
        gateInput(fb::KEY_SELECT);
        gSettingsDirty = true;
      } else {
        gSettingsDirty = true;
      }
    } else if (gTrollHoldArmed) {
      gTrollHoldArmed = false;
      gSettingsDirty = true;
    }
  }

  // Animation
  if (gSetAnimIn && gSetAnim < 255) {
    gSetAnim = (uint8_t)min(255, (int)gSetAnim + 18);
    gSettingsDirty = true;
  } else if (!gSetAnimIn && gSetAnim > 0) {
    gSetAnim = (uint8_t)max(0, (int)gSetAnim - 22);
    gSettingsDirty = true;
    if (gSetAnim == 0) {
      gMode = UI_MENU;
      gMenuDirty = true;
      gateHeldInputs();
      return;
    }
  }

  // Long-press DOWN to exit (bar only after 500 ms)
  if (gIn.held & fb::KEY_DOWN) {
    if (!gHoldArmed || gHoldKey != fb::KEY_DOWN) {
      gHoldArmed   = true;
      gHoldKey     = fb::KEY_DOWN;
      gHoldStartMs = millis();
    } else {
      uint32_t held = millis() - gHoldStartMs;
      if (held >= 1000) {
        gHoldArmed = false;
        gSetAnimIn = false;          // start close animation
        gSettingsDirty = true;
        gateInput(fb::KEY_DOWN);
      } else if (held >= 500) {
        gSettingsDirty = true;       // redraw bar once visible
      }
    }
  } else {
    if (gHoldArmed && gHoldKey == fb::KEY_DOWN) {
      gHoldArmed = false;
      gSettingsDirty = true;         // clear bar if it was showing
    }
  }

  // Short-press navigation always works (not blocked by hold tracking)
  const uint8_t p = gIn.pressed;
  if (p && gSetAnimIn) {
    if (p & fb::KEY_LEFT) {
      if (gOptSel % 3 > 0) gOptSel--;
      else gOptSel = (uint8_t)(gOptSel + 2);
      gSettingsDirty = true;
      gateInput(fb::KEY_LEFT);
    } else if (p & fb::KEY_RIGHT) {
      if (gOptSel % 3 < 2) gOptSel++;
      else gOptSel = (uint8_t)(gOptSel - 2);
      gSettingsDirty = true;
      gateInput(fb::KEY_RIGHT);
    } else if (p & fb::KEY_UP) {
      if (gOptSel >= 3) gOptSel = (uint8_t)(gOptSel - 3);
      else gOptSel = (uint8_t)(gOptSel + 3);
      gSettingsDirty = true;
      gateInput(fb::KEY_UP);
    } else if (p & fb::KEY_DOWN) {
      // short press moves; long press is tracked above
      if (gOptSel < 3) gOptSel = (uint8_t)(gOptSel + 3);
      else gOptSel = (uint8_t)(gOptSel - 3);
      gSettingsDirty = true;
      // do NOT gate DOWN – allow the hold timer to keep running
    } else if (p & fb::KEY_SELECT) {
      gateInput(fb::KEY_SELECT);
      if (kOptSelectable[gOptSel]) {
        switch (gOptSel) {
          case 0: // brightness
            gBriTemp = gCfg.brightness;
            gMode = UI_BRIGHTNESS;
            break;
          case 1: // password toggle
            gCfg.passwordOn = !gCfg.passwordOn;
            saveSettings();
            gSettingsDirty = true;
            break;
          case 2: // airplane
            gCfg.airplane = !gCfg.airplane;
            saveSettings();
            gSettingsDirty = true;
            break;
          case 3: // troll
            gCfg.troll = !gCfg.troll;
            saveSettings();
            gSettingsDirty = true;
            break;
        }
      }
    }
  }

  if (gSettingsDirty || gSetAnim < 255) {
    gSettingsDirty = false;
    drawSettings();
    applyTrollOverlay();
    presentFrame();
  }
}

void runBrightness() {
  const uint8_t p = gIn.pressed;
  bool dirty = false;

  if (p & fb::KEY_UP) {
    if (gBriTemp < 100) { gBriTemp++; dirty = true; }
    gateInput(fb::KEY_UP);
  }
  if (p & fb::KEY_DOWN) {
    if (gBriTemp > 1) { gBriTemp--; dirty = true; }
    gateInput(fb::KEY_DOWN);
  }
  // also support held for faster scrub
  static uint32_t lastRepeat = 0;
  if ((gIn.held & (fb::KEY_UP | fb::KEY_DOWN)) && (millis() - lastRepeat > 40)) {
    lastRepeat = millis();
    if ((gIn.held & fb::KEY_UP) && gBriTemp < 100) { gBriTemp++; dirty = true; }
    if ((gIn.held & fb::KEY_DOWN) && gBriTemp > 1) { gBriTemp--; dirty = true; }
  }

  if (p & fb::KEY_SELECT) {
    gCfg.brightness = gBriTemp;
    applyBrightness();
    saveSettings();
    gMode = UI_SETTINGS;
    gSettingsDirty = true;
    gateInput(fb::KEY_SELECT);
    return;
  }
  if (p & fb::KEY_X) {
    gMode = UI_SETTINGS;
    gSettingsDirty = true;
    gateInput(fb::KEY_X);
    return;
  }

  // live preview
  uint8_t c = (uint8_t)(8 + ((uint16_t)(gBriTemp - 1) * 247) / 99);
  gDisp.setContrast(c);

  drawBrightness();
  applyTrollOverlay();
  presentFrame();
  (void)dirty;
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
  // Troll-mode kill: hold SELECT 4 seconds
  if (gCfg.troll) {
    if (gIn.held & fb::KEY_SELECT) {
      if (!gTrollHoldArmed) {
        gTrollHoldArmed = true;
        gTrollHoldStart = millis();
      } else if (millis() - gTrollHoldStart >= 4000) {
        gCfg.troll = false;
        saveSettings();
        gTrollHoldArmed = false;
        gateInput(fb::KEY_SELECT);
      }
    } else {
      gTrollHoldArmed = false;
    }
  }

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
  applyTrollOverlay();

  // progress bar while killing troll
  if (gTrollHoldArmed) {
    uint32_t held = millis() - gTrollHoldStart;
    if (held > 4000) held = 4000;
    drawHoldBar((uint8_t)(held / 40), 58);
  }

  uint32_t work = micros() - nowUs;
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
//  LAUNCHER EXTRAS
// ============================================================================
namespace fb {

uint32_t keyPressedAtUs(uint8_t keyMask) {
  for (uint8_t i = 0; i < kKeyCount; ++i)
    if (keyMask & (1u << i)) return gPressUs[i];
  return 0;
}

uint32_t frameStartUs() { return gFrameNowUs; }

uint32_t displayLatencyUs() {
  uint32_t l = gWorkUs + (gWorkUs >> 2) + FB_LATCH_MARGIN_US
             + gFlushDurUs
             + FB_PANEL_LATENCY_US;
  if (l > 100000UL) l = 100000UL;
  return l;
}

}  // namespace fb

// ============================================================================
//  9. ARDUINO ENTRY POINTS
// ============================================================================
void setup() {
  inputInit();
  loadSettings();

#if FB_ASYNC_DISPLAY && FB_FAST_FLUSH
  {
    uint16_t chunk = FB_I2C_CHUNK;
    if (chunk > 1024) chunk = 1024;
    size_t got = Wire.setBufferSize((size_t)chunk + 1);
    if (got == 0) got = 128;
    while (chunk > 16 && (size_t)chunk + 1 > got) chunk = (uint16_t)(chunk >> 1);
    gFlushChunk = chunk;
  }
#endif

  gDisp.begin();
  gDisp.setBusClock(FB_I2C_HZ);
  Wire.setClock(FB_I2C_HZ);

#if FB_OLED_OSC
  {
    u8x8_t* const u8x8 = gDisp.getU8x8();
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, 0x0D5);
    u8x8_cad_SendArg(u8x8, (uint8_t)FB_OLED_OSC);
    u8x8_cad_EndTransfer(u8x8);
  }
#endif

  applyBrightness();
  gDisp.clearBuffer();

#if FB_ASYNC_DISPLAY
  gFlushDone = xSemaphoreCreateBinary();
  xSemaphoreGive(gFlushDone);
  xTaskCreatePinnedToCore(flushTask, "fbOLED", 4096, nullptr, 2, &gFlushTaskHandle, 0);
#endif

  // Boot into password gate or main menu
  if (gCfg.passwordOn) {
    gMode = UI_PASSWORD;
    gPwdIdx = 0;
  } else {
    gMode = UI_MENU;
    gMenuDirty = true;
  }
}

void loop() {
  if (gMode == UI_GAME) waitForLatch();
  const uint32_t now = micros();
  pollInput(now);

  switch (gMode) {
    case UI_PASSWORD:   runPassword();   break;
    case UI_MENU:       runMenu();       break;
    case UI_SETTINGS:   runSettings();   break;
    case UI_BRIGHTNESS: runBrightness(); break;
    case UI_GAME:       runGame(now);    break;
  }
}
