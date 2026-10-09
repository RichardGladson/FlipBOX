# flipBOX — Game Development Reference

A human-readable reference for developing games for flipBOX. This document contains the hardware, interface, integration, rendering, timing, and gameplay parameters. It is a technical reference, not an AI prompt.

## 1. Platform parameters

| Parameter | Specification |
|---|---|
| Device | `flipBOX` |
| MCU | ESP32-WROOM-32, dual-core Xtensa LX6 |
| CPU frequency | 240 MHz |
| Build system | PlatformIO, `board = esp32dev`, Arduino framework |
| Language | C++11 |
| Display | 0.96-inch SSD1306-class monochrome OLED |
| Resolution | 128 × 64 pixels |
| Display library | U8g2 (`olikraus/U8g2`) |
| U8g2 mode | Full-buffer, `U8G2_SSD1306_128X64_NONAME_F_HW_I2C` |
| Display rotation | `U8G2_R0` |
| Display reset | `U8X8_PIN_NONE` |
| I²C address | Typically `0x3C` (8-bit form `0x78`) |
| Default I²C clock | 800 kHz; owner may lower to 400 kHz if needed |
| Input | Six buttons: UP, DOWN, LEFT, RIGHT, SELECT, X |
| Audio / radios / storage | Not used by the game system |
| Extra libraries | Not permitted; U8g2 and the Arduino/ESP32 core only |

The OLED is monochrome: `1` means lit and `0` means dark. The origin is the top-left corner; valid coordinates are `x = 0..127`, `y = 0..63`.

## 2. Hardware pinout

The launcher owns all hardware pins. Game code uses the input masks and the provided `U8G2&`; it must not access GPIO or initialize hardware.

### OLED

| OLED pin | ESP32 pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3V3 |
| GND | GND |

### Buttons

All buttons are active-LOW, configured with internal pull-ups, and wired between the listed pin and GND. The launcher converts the physical input to logical masks, where a set bit means pressed.

| Button | Game mask | Bit | ESP32 pin |
|---|---|---:|---|
| UP | `fb::KEY_UP` | 0 | GPIO 26 |
| DOWN | `fb::KEY_DOWN` | 1 | GPIO 33 |
| LEFT | `fb::KEY_LEFT` | 2 | GPIO 25 |
| RIGHT | `fb::KEY_RIGHT` | 3 | GPIO 27 |
| SELECT / START | `fb::KEY_SELECT` | 4 | GPIO 32 |
| X | `fb::KEY_X` | 5 | D4 / GPIO 4 |

The X button is on **D4 / GPIO 4**, not GPIO 8. LEFT=25 and RIGHT=27 are to be verified by the owner; if swapped, only the launcher constants change, not game code. Do not assume any physical button arrangement beyond the names above.

## 3. Game interface: Contract v1

Every game is one self-contained `.cpp` file in `src/`. The following contract block is copied verbatim into each game file. Do not edit or reformat it.

```cpp
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
```

### Required exported interface

- `init()` is optional and runs once every time the game is entered.
- `update(const fb::Input&, const fb::Frame&)` is required; it advances game logic and returns `fb::GAME_RUN` or `fb::GAME_EXIT`.
- `render(U8G2&)` is required; it draws the current state only.
- `shutdown()` is optional and runs once on exit.
- Put all helpers, constants, state, and game functions in an anonymous namespace or give them internal linkage.
- The only exported global symbol should be `fbGame_<id>`, created by `FB_GAME`.
- The final line of the file must be `FB_GAME(<id>, initFn, updateFn, renderFn, shutdownFn);`. The ID must match the launcher registry exactly, including case.

## 4. Input semantics

| Field | Meaning |
|---|---|
| `in.held` | Buttons currently down |
| `in.pressed` | Buttons that transitioned down since the previous update; one-update edge |
| `in.released` | Buttons that transitioned up since the previous update; one-update edge |
| `f.nowMs` | `millis()` at the start of the frame |
| `f.dtUs` | Time since previous frame, clamped to 50,000 µs |
| `f.index` | Frame index, starting at zero on entry |
| `f.fps` | Measured FPS, refreshed every 500 ms; initially zero |

The launcher polls input every iteration and applies an 8 ms per-button debounce lockout after an accepted edge. Multiple buttons may be held simultaneously. Keys held during boot, menu actions, game entry, or game exit are gated until physically released; the game will not see those keys or their release events.

Use `held` for continuous actions (steering, acceleration, braking) and `pressed` for one-shot/menu actions (confirm, pause, nitro activation). Menu-like screens must react to `pressed` edges to prevent repeated actions when a button is held. Do not read input in `render()`.

## 5. Timing and performance parameters

- Frame rate is uncapped and varies with game workload and OLED transfer; do not assume a fixed FPS. A rough range is 30–85 FPS.
- Motion, acceleration, braking, friction, drift, countdowns, and timers must be driven by `f.dtUs` or `f.nowMs`, not frame counts.
- Use sub-pixel fixed-point positions (for example Q16.16 or 24.8) so movement remains smooth at high FPS.
- `f.dtUs` is capped at 50 ms. Guard divisions against zero.
- Avoid `double`, heap allocation, `String`, STL containers, blocking calls, per-pixel trigonometry/root functions, and expensive work in inner loops.
- Precompute reusable trigonometry, curve, or projection tables in `init()` or at compile time.
- Keep `update()` + `render()` at approximately 10 ms or less where practical.
- Keep static RAM around 16 KB or less per game and flash around 100–150 KB or less per game. Keep stack usage small; the loop task has an 8 KB stack.
- Write C++11-compatible code and compile warning-free with `-Wall -Wextra`.
- Use fixed-size arrays. Do not use `new`, `malloc`, `std::vector`, `std::string`, `std::function`, or other dynamic allocation.
- Never use `delay()`, `delayMicroseconds()`, busy-wait loops, blocking I/O, or `yield()` in a game.

## 6. Rendering parameters

- The launcher clears the full U8g2 buffer before `render()` and presents the frame afterward.
- `render()` draws only. It must not mutate gameplay state, read input, or call display-transfer or display-initialization functions.
- Never call `sendBuffer()`, `clearBuffer()`, `updateDisplay*()`, `begin()`, `setBusClock()`, `setPowerSave()`, `setContrast()`, `setFlipMode()`, `setDisplayRotation()`, `getU8x8()`, or use `Wire`.
- Set the font, draw colour, font mode, and bitmap mode needed by each frame inside `render()`; do not rely on state carrying over.
- Use `drawStr()` with fixed `char` buffers and `snprintf` for dynamic text; avoid `String` and `%f`.
- Direct framebuffer access through `getBufferPtr()` is allowed inside `render()` for performance. Buffer layout: 8 pages × 128 columns; byte index `(y >> 3) * 128 + x`; bit `(y & 7)` is the pixel position. Bounds-check every direct write.
- Keep all geometry inside 128×64 and use signed types for calculations that can go negative.
- Prioritize high contrast, large recognizable shapes, readable HUD text, and uncluttered layouts. Use at least 6×10 text for important information; 5×7 is for secondary details only.
- Avoid large filled areas unless intentional. Keep 1–2 px margins for text and make overlapping UI/graphics clearly distinguishable.
- A finished game should include an appropriate title/intro, readable gameplay HUD, results/game-over screen, restart flow, and a visible way to return to the launcher.
- Use the device name `flipBOX` wherever the device name appears.

## 7. Lifecycle and game-state parameters

Typical flow:

`TITLE/INTRO → SELECTION (if needed) → COUNTDOWN → PLAYING → PAUSED (if needed) → RESULTS/GAME OVER → RESTART or EXIT`

- `init()` must reset every mutable state variable on each entry, initialize tables, and seed the PRNG. C++ static initializers run only once at boot.
- `update()` handles input, simulation, and state transitions. It must not draw.
- `render()` draws the current state. It must not advance gameplay.
- A game must provide an explicit return-to-launcher path by returning `fb::GAME_EXIT` from `update()`. The launcher has no global exit button or button combo.
- Recommended exit design: show restart/quit options on results, game-over, or pause screens, and/or allow SELECT to be held for about one second with a visible progress cue. Tell the player how to exit on screen.
- Avoid large local arrays, deep recursion, and unaligned multi-byte memory access.

## 8. Game file and registration parameters

Project layout:

```text
platformio.ini
src/
  main.cpp
  Game1.cpp
  Game2.cpp
  Game3.cpp
  <AnyNewGame>.cpp
```

PlatformIO compiles every `.cpp` in `src/` automatically. Do not modify `platformio.ini`, create helper files, add headers, or couple games together.

### Current registry

| Menu label | Registry ID | Planned file | Exported symbol |
|---|---|---|---|
| F1 Racing | `F1Racing` | `Game1.cpp` | `fbGame_F1Racing` |
| Fast and Furious | `FastAndFurious` | `Game2.cpp` | `fbGame_FastAndFurious` |
| Tokyo Drift | `TokyoDrift` | `Game3.cpp` | `fbGame_TokyoDrift` |

These three IDs are already registered. For a new game, add one `X(<id>, "<Menu Label>")` entry to `FB_GAME_LIST` in `main.cpp`; all entries except the last need a trailing backslash. IDs must be unique valid C identifiers and match `FB_GAME(...)` exactly. Menu labels must be unique ASCII strings of at most 18 characters (16 or fewer recommended).

### Launcher behavior

- Boots directly into the `flipBOX` game menu.
- UP/LEFT selects the previous item; DOWN/RIGHT selects the next item; navigation wraps.
- SELECT launches the highlighted game.
- The selected game remains highlighted when returning to the menu.
- A registered game that is missing or has a null `update`/`render` pointer displays `NOT INSTALLED`.
- The menu redraws only when something changes.

## 9. Hard scope limits

Games must not add or use:

- Audio, speakers, or buzzers
- Wi-Fi, Bluetooth/BLE, NRF24, or networking
- SD cards, external storage, EEPROM/NVS/Preferences/SPIFFS/LittleFS persistence
- Sensors or additional peripherals
- Extra libraries
- Additional buttons or GPIO access
- Changes to the launcher or contract
- Additional files or placeholder/TODO gameplay

A future feature requiring a new input, audio, or peripheral interface requires an explicit contract/launcher change; do not anticipate it with unused code.

## 10. Existing game design parameters

These are the currently planned game concepts. Their distinct mechanics should be preserved.

### F1 Racing — `F1Racing` / `Game1.cpp`

- Pseudo-3D racing: the road approaches a mostly stationary player car.
- LEFT/RIGHT steers; UP accelerates; DOWN brakes/decelerates.
- No drifting.
- The road curves left and right.
- Core challenge: speed, steering accuracy, and anticipation.

### Fast and Furious — `FastAndFurious` / `Game2.cpp`

- Straight-line drag racing with two cars.
- Player chooses Nissan Skyline GTR or Dodge Charger.
- Skyline has higher acceleration; Charger has higher top speed.
- Timing, acceleration, and nitro are the core mechanics.
- Opponent operates independently; opponent speed must not depend on player speed.
- Two independent nitro controls. Each nitro gives +30 km/h for 3 seconds; both active together give +60 km/h.
- Nitro is a one-time switch, not an action that requires holding the button.
- Both NO2 bars remain visible.

### Tokyo Drift — `TokyoDrift` / `Game3.cpp`

- True top-down racing, distinct from the pseudo-3D F1 game.
- Momentum-based arcade driving: acceleration, steering, turning, inertia, lateral movement, and controlled drifting.
- Pressing LEFT/RIGHT while pressing DOWN initiates a drift.
- Drifting must be controllable and rewarding.
- The car remains approximately centered while the world/track scrolls.
- The top-down car must be clearly visible and visibly tilt while drifting.

## 11. Relevant U8g2 fonts

Use only font names known to exist. Fonts already used by the launcher:

- `u8g2_font_5x7_tr`
- `u8g2_font_6x13_tr`
- `u8g2_font_6x13B_tr`
- `u8g2_font_7x13B_tr`

Other known options include `u8g2_font_6x10_tr`, `u8g2_font_6x12_tr`, `u8g2_font_7x13_tr`, `u8g2_font_helvB08_tr`, `u8g2_font_helvB10_tr`, `u8g2_font_logisoso16_tn`, `u8g2_font_logisoso20_tn`, and `u8g2_font_logisoso24_tn`.

## 12. Troubleshooting reference

| Symptom | Likely cause / fix |
|---|---|
| `NOT INSTALLED` | Check source file location, registry ID spelling/case, `FB_GAME` symbol, and non-null `update`/`render`; clean-build if needed. |
| Multiple definition of a symbol | Give helpers and state internal linkage using an anonymous namespace or `static`. |
| `setup()` / `loop()` redefinition | Remove them; the launcher owns both. |
| Contract redefinition/mismatch | Include the contract exactly once and verbatim. |
| Cannot exit game | Implement a `GAME_EXIT` path and show the exit control on screen. |
| Game starts with an action already triggered | Use `pressed` edges for menu-like actions and account for input gating. |
| State carries over on re-entry | Reset mutable state in `init()`. |
| Movement changes with FPS | Drive motion and timers using `dtUs` / `nowMs`. |
| Movement jitters at high FPS | Use sub-pixel fixed-point positions. |
| Alignment exception | Avoid unaligned pointer casts; use `memcpy` or aligned data. |
| Stack overflow/watchdog reset | Reduce local arrays and recursion; keep large arrays static. |
| OLED noise | Owner may lower I²C clock with `-DFB_I2C_HZ=400000`. |
| FPS is low | Reduce per-pixel work, trig in hot paths, and large fills; precompute tables and use spans/direct buffer writes. |
| Rendering changes after overlay | Set font, colour, and drawing modes inside every `render()`. |

## 13. Owner-side build options

These are launcher/build options, not per-game APIs:

| Flag | Default | Meaning |
|---|---:|---|
| `FB_I2C_HZ` | `800000UL` | OLED bus clock |
| `FB_ASYNC_DISPLAY` | `1` | Asynchronous display transfer |
| `FB_SHOW_FPS` | `0` | Draw FPS overlay in top-right |
| `FB_DEBOUNCE_US` | `8000UL` | Debounce lockout per key |
| `FB_MAX_DT_US` | `50000UL` | Maximum frame delta |

A game must work regardless of these values. The FPS overlay occupies roughly the top-right 9 pixels; do not place critical HUD information there.
