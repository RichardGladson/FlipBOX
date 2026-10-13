# flipBOX — AI Game Generation Specification

**Document version:** 1.2 — aligned to the supplied `main.cpp` Wave 1 and **Game Contract v1** (with the X-button mask), including asynchronous fast flush and late-latched input.
**Purpose:** This is the AI-facing prompt and complete technical specification for generating one compatible flipBOX game. Provide this file to the AI along with the game request template at the end. It contains the required output format, implementation rules, hardware and launcher contract, and reusable game-generation requirements.

---

## PINOUT — QUICK REFERENCE (display and all buttons)

The original NyanBOX/jbohack assignments remain fixed, and the owner has added the X button on **D4 / ESP32 GPIO4**. **Never invent or change any assignment documented here.** The launcher (`main.cpp`) owns every pin below; **a game never references a GPIO number** and only sees the `fb::KEY_*` masks and the `U8G2&` it is handed.

### OLED display (I²C, SSD1306-class, 128×64, driven by U8g2 hardware I²C)
| OLED module pin | ESP32 pin | Notes |
|---|---|---|
| **SDA** | **GPIO 21** | Default ESP32 I²C0 data line. |
| **SCL** | **GPIO 22** | Default ESP32 I²C0 clock line. |
| VCC | 3V3 | Standard module supply (typical 0.96" modules; not configured in software). |
| GND | GND | Common ground. |

- I²C address: U8g2 default for this module (0x3C, i.e. 0x78 in 8-bit form).
- Bus clock: 1 MHz by default (`FB_I2C_HZ`; test lower settings if the screen shows noise).
- OLED modules normally carry their own I²C pull-up resistors; none are added in software.
- Reset pin: **not used** (`U8X8_PIN_NONE`).
- U8g2 object: `U8G2_SSD1306_128X64_NONAME_F_HW_I2C gDisp(U8G2_R0, U8X8_PIN_NONE, /*SCL*/ 22, /*SDA*/ 21);`

### Buttons (six, active-LOW, internal pull-ups)
| Button | `fb::` mask | Bit | ESP32 pin | Wiring |
|---|---|---|---|---|
| **UP** | `fb::KEY_UP` | 0 | **GPIO 26** | one side to the GPIO, other side to GND |
| **DOWN** | `fb::KEY_DOWN` | 1 | **GPIO 33** | one side to the GPIO, other side to GND |
| **LEFT** | `fb::KEY_LEFT` | 2 | **GPIO 25** | one side to the GPIO, other side to GND |
| **RIGHT** | `fb::KEY_RIGHT` | 3 | **GPIO 27** | one side to the GPIO, other side to GND |
| **SELECT / START** | `fb::KEY_SELECT` | 4 | **GPIO 32** | one side to the GPIO, other side to GND |
| **X** | `fb::KEY_X` | 5 | **D4 / GPIO 4** | one side to D4/GPIO4, other side to GND |

- Configured as `INPUT_PULLUP`; **pressed = pin reads LOW**, released = HIGH. The launcher inverts this, so in game code a set bit always means "pressed/down".
- The five original GPIOs (25, 26, 27, 32, 33) are read directly from the ESP32 GPIO input registers; X uses **D4 / GPIO 4**, read via `GPIO_IN_REG`.
- SELECT is the confirm/START button in menus.
- The physical arrangement of the buttons on the case is not specified here; do not assume a layout beyond the six names above.
- **Whenever the user refers to the X button, it means the physical button wired to board pin D4 (ESP32 GPIO4), not GPIO8.** Configure it as `INPUT_PULLUP` and wire the switch between D4/GPIO4 and GND.

### Wiring diagram
```
                 ESP32-WROOM-32 (esp32dev)
              ┌───────────────────────────┐
  OLED SDA ───┤ GPIO 21                   │
  OLED SCL ───┤ GPIO 22                   │
  OLED VCC ───┤ 3V3                       │
  OLED GND ───┤ GND                       │
              │                           │
  UP     ─o/o─┤ GPIO 26   ┐               │
  DOWN   ─o/o─┤ GPIO 33   │  other side   │
  LEFT   ─o/o─┤ GPIO 25   ├─ of every ────┤ GND
  RIGHT  ─o/o─┤ GPIO 27   │  button       │
  SELECT ─o/o─┤ GPIO 32                   │
  X      ─o/o─┤ D4 / GPIO 4               │
              └───────────────────────────┘
        (internal pull-ups enabled in firmware; pressed = LOW)
```

### Status of each assignment (be honest about certainty)
- **Confirmed existing assignments:** OLED on SDA 21 / SCL 22; original buttons use GPIOs {25, 26, 27, 32, 33}; **UP = 26, DOWN = 33, SELECT = 32**. The added X button is configured by the owner to **D4 / GPIO4** (not GPIO8).
- **To verify by the owner:** the **LEFT = 25 / RIGHT = 27** split is inferred from NyanBOX's layout. If the owner's existing wiring has these two swapped, the **only** change needed is the `PIN_LEFT` / `PIN_RIGHT` constants in `main.cpp`. **Game code is unaffected either way**, because games only ever use `fb::KEY_LEFT` / `fb::KEY_RIGHT`.
- Any other pins used by the original NyanBOX hardware (e.g. radio modules) are **not used by flipBOX games and must not be touched** (§12).

---

## 0. HOW TO READ THIS DOCUMENT

### 0.1 Authority order (highest first)
1. **The user's game request** (the "Game Request" that accompanies this file) decides *what the game is*: rules, feel, controls, visuals, difficulty.
2. **The Game Contract (§5) and the launcher behaviour (§4, §6, §7, Appendix A)** decide *how the game plugs in*. These are fixed. They are not negotiable and must never be altered by a game.
3. **The rest of this document** (hardware, performance, display rules, structure rules) decides *how the game must be built*.
4. If the user *also* pastes the real `main.cpp` and it disagrees with this document, **`main.cpp` wins**; state the discrepancy to the user in one short note and continue.
5. If the Game Request conflicts with a hard rule here (for example, "use Wi-Fi leaderboard"), do not silently break the rule: say so in one sentence and offer the closest compliant design.

### 0.2 Keywords
**MUST / MUST NOT / NEVER / ALWAYS** = hard requirement. **SHOULD / SHOULD NOT** = strong default; deviate only with a reason you state. **MAY** = optional.

### 0.3 What the LLM is being asked to do
Write **exactly one game** as **exactly one file**: `src/<GameFile>.cpp`. Optionally tell the user the **one registry line** to add to `main.cpp` (only needed for a game that is not already registered, see §11). Do **not** write `main.cpp`, do **not** write other games, do **not** create headers or helper files, do **not** change `platformio.ini`.

If the Game Request is ambiguous, make sensible, clearly stated assumptions and build the game. Ask a question **only** if something essential is impossible to guess, and then ask **one** question.

---

## 1. NON-NEGOTIABLE RULES (TL;DR CHECKLIST)

A game that violates any of these is wrong, regardless of how good it looks.

1. **One game = one `.cpp` file** inside `src/`. Self-contained. No shared headers, no shared helper files, no dependencies on other games.
2. **Paste the Game Contract v1 block verbatim** (§5.1) at the top of the file. **Never edit it.**
3. **End the file with one `FB_GAME(<id>, init, update, render, shutdown);` line** (§5.4). The `<id>` must match the registry exactly (case-sensitive) or the launcher silently shows "NOT INSTALLED".
4. **The game MUST NOT define `setup()`, `loop()` or `main()`**, and MUST NOT initialise any hardware.
5. **Everything except the single exported `fbGame_<id>` symbol MUST have internal linkage** (put it in an anonymous namespace or mark it `static`). Otherwise two games with a variable of the same name will fail to link.
6. **`update()` does logic only. `render()` does drawing only.** Never draw in `update()`. Never mutate game state in `render()` (except trivially cached render-only data).
7. **Never call `sendBuffer()`, `clearBuffer()`, `updateDisplay*()`, `begin()`, `setBusClock()`, `setPowerSave()`, `setContrast()`, `setFlipMode()`, `setDisplayRotation()` or `getU8x8()`.** The launcher owns the display.
8. **Never use GPIO, `pinMode`, `digitalRead`, `Wire`, `SPI`, `Serial` (in final code)**. Input comes **only** from the `fb::Input` struct passed to `update()`.
9. **Never use `delay()`, `delayMicroseconds()`, busy-wait loops, or any blocking call.** Everything is non-blocking and driven by the frame loop.
10. **Never allocate heap memory** (`new`, `malloc`, `String`, `std::string`, `std::vector`, `std::function`, etc.). Fixed-size static arrays only.
11. **`init()` MUST fully reset every piece of game state**, because the player can leave the game and re-enter it without rebooting. C++ static initialisers run only once, at boot.
12. **The game MUST provide a way for the player to return to the launcher** (return `fb::GAME_EXIT` from `update()`). The launcher has **no** global "exit" button or combo (§10.3).
13. **All motion/physics MUST be frame-rate independent** (driven by `Frame::dtUs`). The frame rate is uncapped and varies.
14. **No floating-point `double`, no per-pixel `sinf/cosf/sqrtf`, no `float` printf in hot paths.** Integer / fixed-point math and precomputed tables (§9).
15. **Nothing may be drawn outside 128×64**, and nothing in the UI may blend into something else (§8.6).
16. **Code MUST compile warning-free** with `-Wall -Wextra` under C++11 (§3.3).
17. **No audio, Wi-Fi, Bluetooth, NRF24, networking, external storage, extra peripherals, or extra libraries** (§12).
18. **The result must look and feel like a real retro handheld game**, not a debug demo (§8.6, §10).

---

## 2. THE PRODUCT AND THE HARDWARE

**flipBOX** is a retro handheld-style game system. The launcher boots straight into a game-selection menu; the player picks a game, plays it, and the game returns to the menu when it exits.

### 2.1 Hardware (fixed, do not change)
| Item | Value |
|---|---|
| MCU | ESP32-WROOM-32 (dual-core Xtensa LX6, 240 MHz, hardware single-precision FPU) |
| Board / framework | `board = esp32dev`, Arduino framework, PlatformIO, `platform = espressif32` |
| CPU frequency | 240 MHz |
| Display | 0.96" monochrome OLED, **128×64**, I²C, SSD1306-class, driven by **U8g2** (`olikraus/U8g2`) |
| Display object | `U8G2_SSD1306_128X64_NONAME_F_HW_I2C` — **full-buffer mode**, 1024-byte frame buffer, rotation `U8G2_R0` |
| Input | **Exactly six** physical buttons: UP, DOWN, LEFT, RIGHT, SELECT, X. X is wired to board pin **D4 / ESP32 GPIO4** (not GPIO8). No other controls exist and none may be assumed. |
| Audio | None. |
| Other | No SD card, no radios in use, no sensors in use. |

The OLED is **monochrome**: a pixel is either lit (colour `1`) or dark (colour `0`). There is no grey. UI clarity is extremely important.

### 2.2 Pin assignment (information only — games NEVER touch pins)
The full pinout (display and all six buttons, with wiring diagram and certainty status) is in the **PINOUT — QUICK REFERENCE** section at the very top of this document. The launcher owns all GPIO; games only see `fb::KEY_*` masks and the `U8G2&`. The wiring is the existing NyanBOX/jbohack pinout. **Never invent or change GPIO assignments.**

### 2.3 Performance priority
**Maximum FPS, input responsiveness and modularity are the top priorities.** The frame rate is **not** artificially capped (not 30, not 60). Games render as fast as the hardware and the OLED link realistically allow, and games prioritise raw performance over abstraction.

---

## 3. SOFTWARE ENVIRONMENT AND BUILD CONSTRAINTS

### 3.1 Project layout
```
platformio.ini            (do not modify)
src/
  main.cpp                (the launcher — do not modify, except one registry line, §11)
  Game1.cpp               (planned: an example game)
  Game2.cpp               (planned: an example game)
  Game3.cpp               (planned: an example game)
  <AnyNewGame>.cpp        (future games)
```
PlatformIO compiles **every** `.cpp` in `src/` automatically and links them into one firmware. The only coupling between the launcher and a game is the contract in §5.

### 3.2 Libraries
Only **U8g2** (already a dependency) plus the Arduino/ESP32 core. **Do not add libraries.** Standard C/C++ headers that need no heap (`<stdint.h>`, `<string.h>`, `<stdio.h>` for `snprintf`, `<math.h>` only for compile-time/init-time use) are fine.

### 3.3 Language level and warnings
Different arduino-esp32 versions compile with different standards (`gnu++11` on 2.x, newer on 3.x). **Write C++11-compatible code**:
- No aggregate initialisation of structs that have default member initialisers.
- `constexpr` functions must be a single `return` expression.
- No `inline` variables, no structured bindings, no digit separators, no generic lambdas, no `if constexpr`.
- Code must be correct under both `-Os` (Arduino default) and `-O2`.
- Compile **warning-free** with `-Wall -Wextra` (mark intentionally unused parameters with `(void)param;` or leave them unnamed; do not leave unused functions — pass `nullptr` to `FB_GAME` instead of defining an unused `shutdown`).

### 3.3.1 Alignment
Xtensa faults on unaligned 16/32-bit memory access. **Never** cast a `uint8_t*` into a `uint16_t*`/`uint32_t*` that may be unaligned. Use `memcpy` or aligned types. Be careful with packed structs.

---

## 4. ARCHITECTURE AND LIFECYCLE

```
ESP32 boot
  └─ setup()      launcher: input pins, OLED begin, I²C clock, display flush task (core 0)
  └─ loop()       launcher: runs forever on core 1
        ├─ pollInput()                       (every iteration, always)
        ├─ if no game active  →  runMenu()  (draws only when something changed)
        └─ if game active     →  runGame()
              ├─ compute Frame (dt, index, fps)
              ├─ game.update(input, frame)   → GAME_EXIT? → shutdown(), back to menu
              ├─ display.clearBuffer()
              ├─ game.render(display)
              ├─ [optional debug FPS overlay]
              └─ presentFrame()               (hand the 1 KB frame to the OLED)
```

### 4.1 Exact sequence when a game starts
1. In the menu the player presses SELECT on an installed game.
2. The launcher blocks SELECT until released and calls `enterGame(game)`:
   1. All currently-held buttons are **gated** (invisible to the game until physically released).
   2. Draw state is reset: draw colour `1`, font mode `0` (solid), bitmap mode `0`, font direction `0`, font position = baseline.
   3. Frame counters reset: `Frame::index = 0`, `fps = 0`, timing origin = now.
   4. `init()` is called **if it is non-null**.
3. On the **next** loop iterations, input is polled and `update()` then `render()` run every iteration.
4. Until the game's first frame is presented, the OLED still shows the menu. **`init()` cannot draw** (it has no display reference); first pixels come from the first `render()`.

A game is launchable only if the symbol exists **and** its `update` and `render` pointers are non-null. `init` and `shutdown` are optional.

### 4.2 Exact sequence when a game exits
1. `update()` returns `fb::GAME_EXIT`.
2. **`render()` is NOT called for that frame.**
3. `shutdown()` is called if non-null.
4. All currently-held buttons are gated (so the key that triggered the exit cannot act in the menu).
5. The menu is redrawn immediately, with the same game still highlighted.

### 4.3 Core / task model
- `update()` and `render()` run on **core 1**, inside the Arduino `loopTask` (default stack **8 KB** — keep local variables small; no large local arrays; no deep recursion).
- Core **0** runs a small launcher task that pushes the finished frame to the OLED over I²C.
- Games **MUST NOT** touch `Wire`/I²C, and **SHOULD NOT** create FreeRTOS tasks. (If a design genuinely benefits from using core 0 for non-display work, justify it explicitly; such a task must never touch the display or I²C, must not run the launcher's flush task out of CPU time, and must be stopped in `shutdown()`.)

### 4.4 Display pipeline (important for performance reasoning)
`render()` draws into U8g2's internal 1024-byte buffer. `presentFrame()` then:
1. waits **only if** the previous frame is still being transmitted,
2. copies the 1024-byte buffer into a shadow buffer (~microseconds),
3. wakes the core-0 task, which transmits the shadow over I²C **while core 1 already computes the next frame**.

Consequences:
- FPS = bounded by `max(game work per frame, I²C transmit time)`, **not their sum**.
- I²C transmit time for a full 1 KB frame is roughly **9–14 ms at 1 MHz**, depending on transfer overhead and module behaviour. These are estimates, not guarantees; measure on the actual hardware. Treat these as estimates.
- **Roughly the first ~10 ms of `update()+render()` per frame is "free"** (hidden behind the transfer). Beyond that, every extra millisecond lowers FPS.
- The frame you draw appears on the glass roughly one transfer later; that is normal.

---

## 5. THE GAME CONTRACT (v1)

### 5.1 The block to paste verbatim at the top of the game file
Copy **everything between the BEGIN and END markers, unchanged** (it already includes the two `#include`s it needs). Do **not** reformat, rename, reorder or "improve" it. Identical copies in every game file are what makes the build safe (one-definition rule).

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

**Contract-change rule:** if the LLM believes the contract needs a new field or function (for example a new button, sound, or per-frame display info), it MUST NOT modify the block. Instead it must say so to the user in one short note, because changing the contract requires changing `main.cpp` **and every other game file** at the same time.

### 5.2 Member-by-member semantics

**`fb::SCREEN_W` / `SCREEN_H`** — 128 / 64. Valid pixel coordinates: `x ∈ [0,127]`, `y ∈ [0,63]`, origin top-left, +x right, +y down.

**`fb::Input`** (see §6):
- `held`: bitmask of keys currently down.
- `pressed`: keys that went down since the previous `update()` (edge, true for exactly one update).
- `released`: keys that went up since the previous `update()` (edge, true for exactly one update).
- Test with `if (in.pressed & fb::KEY_SELECT)`.

**`fb::Frame`** (see §7):
- `nowMs` = `millis()` at the start of the frame.
- `dtUs` = microseconds since the previous frame, **clamped to 50 000 µs (50 ms)**. On the very first frame it is the (small) time since `init()` returned. Practically never 0, but guard divisions anyway.
- `index` = 0 on the first `update()` after entry, +1 per frame.
- `fps` = measured frames/second, refreshed every 500 ms; **0 until the first 500 ms window completes**.

**`fb::Status`**: `GAME_RUN` = keep playing; `GAME_EXIT` = return to the launcher.

**`fb::Game`**: four function pointers.
- `init()` — optional (`nullptr` allowed). Called once on every entry, before the first `update()`. Reset all state, precompute tables, seed the PRNG.
- `update(in, f)` — **required**. Called once per loop iteration while the game is active. Returns `GAME_RUN` or `GAME_EXIT`.
- `render(d)` — **required**. Called once per iteration right after `update()` (except on the exit frame). The buffer has already been cleared (all pixels dark). `d` is the shared `U8G2` object.
- `shutdown()` — optional (`nullptr` allowed). Called once when the game exits.

### 5.3 Call-order guarantees
For each game session: `init()` → (`update()` → `render()`) repeated → final `update()` returning `GAME_EXIT` → `shutdown()`. `update()` and `render()` strictly alternate, one-to-one, on the same task. No call overlaps another. `render()` is never called without a preceding `update()` in the same iteration.

### 5.4 Registration macro
`FB_GAME(id, initFn, updateFn, renderFn, shutdownFn);` defines the exported symbol `fbGame_<id>`. It goes **at global scope**, as the **last line** of the file, **outside** any anonymous namespace (the functions it points to MAY be inside the anonymous namespace). Pass `nullptr` for `initFn` / `shutdownFn` if unused. Example shape:

```cpp
FB_GAME(an example game, gameInit, gameUpdate, gameRender, nullptr);
```

---

## 6. INPUT SYSTEM (what the game actually sees)

### 6.1 Properties
- Six buttons: masks `KEY_UP`, `KEY_DOWN`, `KEY_LEFT`, `KEY_RIGHT`, `KEY_SELECT`, `KEY_X` (bits 0–5). `KEY_X` is the physical button wired to D4 / ESP32 GPIO4, not GPIO8.
- Read straight from the GPIO input registers once per loop iteration, immediately before `update()`. Reading costs well under a microsecond.
- **Edge-latched debounce:** the first accepted edge is recorded immediately; subsequent changes of the same button are ignored for `FB_DEBOUNCE_US` (currently 5 ms) to suppress contact bounce. So there is no added input lag.
- The launcher also samples inputs on a 1 ms `esp_timer` callback and latches accepted press/release edges between game updates, so short taps are less likely to be lost. `waitForLatch()` delays the game-frame start until close to the end of the current OLED transfer, allowing the next input sample to be fresher.
- Multiple buttons can be held at once. Opposite directions (LEFT+RIGHT) can both be down; decide how the game resolves that.

### 6.2 Input gating (key blocking)
Gating means a key is **invisible** (not in `held`, `pressed` or `released`) until it has been **physically released**. The launcher gates:
- at **power-on**: any key held during boot,
- at **menu actions**: the key that was just used,
- at **game entry**: every key still held when the game starts (so the SELECT that launched the game does not instantly act inside it),
- at **game exit**: every key still held when the game returns to the menu.

Design consequences for the game:
- A key held at entry only "exists" for the game after the player releases it and presses it again. Do not require a key to be held from frame 0.
- The `released` event of a gated key is suppressed too (the game never sees a release for a press it never saw).
- The **game itself** should apply the same discipline to its own non-gameplay screens (title, car select, pause, game over): act on `pressed` edges, never on `held`, for menu-like actions, so holding a button cannot cause double selections.

### 6.3 Using the input
- Continuous controls (steer, accelerate, brake): use `held`.
- One-shot actions (nitro toggle, confirm, pause): use `pressed`.
- Charge/hold actions: count time yourself by accumulating `Frame::dtUs` while the bit is set in `held`. The launcher provides no per-button hold timers.
- Do not read input in `render()`.

---

## 7. TIMING

### 7.1 Frame rate is variable
There is no FPS cap and no fixed timestep. FPS depends on the game's cost and the OLED link. Expect anywhere from ~30 to ~85 FPS. **Never assume a constant frame time.** Never tie speed to the number of frames.

### 7.2 Time-based movement
Use `f.dtUs`. Recommended pattern (all-integer, fixed-point):

```cpp
// dt as 16.16 fixed-point seconds; dtUs <= 50000 so (dtUs << 16) cannot overflow uint32_t
uint32_t dt16 = (f.dtUs << 16) / 1000000UL;      // 1.0 s == 65536
// speed in 16.16 units/second  ->  distance this frame in 16.16 units:
int32_t step = (int32_t)(((int64_t)speed16 * dt16) >> 16);
```
- **Keep positions in sub-pixel fixed-point** (e.g. 16.16 or 24.8). At high FPS, `speed × dt` is often far less than one pixel; integer pixel positions would stall movement.
- Acceleration, braking, friction, drag, drift, nitro timers, countdowns: all expressed per second and integrated with `dt`.
- A per-frame division is fine; **avoid divisions inside per-pixel/per-scanline loops** (precompute reciprocals/tables).
- If the game needs deterministic physics (e.g. drag-race timing), you MAY use a fixed-step accumulator (`acc += dtUs; while (acc >= STEP_US) { step(); acc -= STEP_US; }`) with a cap on iterations per frame. State which approach is used.
- A frame is clamped to 50 ms; if the game ever runs below 20 FPS, game time simply runs slower than real time — do not try to compensate.
- Real-time timers (3-second nitro, countdown) MUST be accumulated from `dtUs` (or compared with `nowMs`), not counted in frames.

### 7.3 Measuring performance
`Frame::fps` is available for debug HUDs. The owner can also enable a launcher-side FPS overlay with the build flag `-DFB_SHOW_FPS=1`; when on, a small black box with the FPS number is drawn in the **top-right corner** (about `(textWidth+3) × 9` px at y=0..8) over the finished frame, and the launcher restores font and draw colour 1 afterwards. Do not put critical information in that corner of the screen.

### 7.4 Random numbers
Use a tiny integer PRNG (e.g. xorshift32) stored in static state, seeded in `init()` from `f`-independent sources such as `micros()` (allowed here only for seeding) mixed with a constant, so each run differs. Do not use `random()` from Arduino in hot paths.

---

## 8. DISPLAY AND RENDERING

### 8.1 What `render(U8G2& d)` receives
- `d` is the shared full-buffer U8g2 object (base class `U8G2`), 128×64, monochrome, white/lit = colour `1`.
- The buffer is **already cleared to dark** before every `render()`. Do not call `clearBuffer()`; if the background must stay dark, simply do not fill it. Avoid large filled background fills (§8.6).
- Draw state at **game entry**: draw colour 1, font mode 0 (solid), bitmap mode 0, font direction 0, font position baseline. **Do not rely on any draw state surviving between frames** (the optional debug overlay may change the font). Set the font, colour and modes you need **inside `render()`**, every frame. Setting a font is just a pointer assignment — it is cheap.
- `render()` MUST NOT call `sendBuffer()` / `clearBuffer()` / `updateDisplay*()`; the launcher presents the frame.
- `render()` MUST NOT change anything except the buffer and U8g2 draw state.

### 8.2 U8g2 drawing facts that matter
- Coordinates: origin top-left. Text `drawStr(x, y, s)`: `y` is the **baseline** under the default font position.
- Solid font mode (`setFontMode(0)`) paints a background box in colour 0 behind each glyph; **transparent mode (`setFontMode(1)`)** draws only the glyph pixels (use it on top of graphics).
- Draw colours: `0` = dark, `1` = lit, `2` = XOR.
- Inverted text on a bar: `d.setDrawColor(1); d.drawBox(...); d.setDrawColor(0); d.drawStr(...)` (with transparent font mode), then set colour back to `1`.
- Fast primitives: `drawHLine`, `drawVLine`, `drawBox`, `drawFrame`, `drawPixel` (per-call overhead, use sparingly in bulk), `drawLine`, `drawTriangle` (filled), `drawDisc`, `drawCircle`, `drawRBox`, `drawRFrame`, `drawXBM`.
- Bitmaps: `drawXBM(x, y, w, h, bits)` — XBM format (row-major, each row padded to whole bytes, **least-significant bit = leftmost pixel**). `static const uint8_t` data is stored in flash and works directly on ESP32 (`PROGMEM` is a no-op). Use `setBitmapMode(1)` for transparent sprites.
- The U8g2 draw functions clip to the screen, but clipped work is still work: do not draw things that are off-screen.
- `U8G2` also supports `print`; prefer `drawStr` with a fixed `char` buffer filled by `snprintf` (no `String`, no `%f`).

### 8.2.1 Rendering and responsiveness priorities
The OLED transfer is often the throughput bottleneck, not game logic. Optimise in this order and verify each change with measurements:

1. **Measure before tuning.** Build with `-DFB_SHOW_FPS=1`; compare `FB_I2C_HZ` at 400000, 800000 and 1000000 on the actual module. If FPS tracks bus speed, the link is the limit. Record FPS and screen stability rather than assuming a theoretical ceiling.
2. **Keep the asynchronous fast flush enabled.** The current launcher already snapshots the full 1024-byte frame and streams it in chunks using a preconfigured horizontal-addressing window (`FB_ASYNC_DISPLAY=1`, `FB_FAST_FLUSH=1`). Do not reimplement or bypass this pipeline in a game. If you change chunk sizes, validate that the Wire buffer supports them and check for I²C errors or visual corruption.
3. **Treat 1 MHz as a testable setting, not a universal guarantee.** OLED modules, pull-ups and wiring vary. Use short wires and adequate pull-ups; if noise, missing pixels or instability appears, reduce the clock (for example to 800 kHz or 400 kHz). Never trade correctness for a nominal FPS number.
4. **Optimise drawing calls after the bus path is measured.** Avoid per-pixel `drawPixel()` loops for filled shapes. Prefer `drawBox`, `drawHLine`, `drawVLine`, spans, `drawXBM()` sprites and precomputed bitmaps. For repeated sprites, prepare the required orientations/scales in `init()` or compile-time data instead of rebuilding them pixel by pixel every frame.
5. **Use direct buffer writes only for proven hot paths.** They can accelerate scanline fills, but bypass U8g2 clipping. Keep writes bounds-checked, follow the page layout in §8.3, and compare visual output against the normal primitives.
6. **Sample input late; don't add frame waits.** The launcher already calls `waitForLatch()` before sampling input and updating/rendering, using measured work and transfer timing to reduce stale-input latency without intentionally capping throughput. Games must not add `delay()`, busy waits, or their own display synchronisation.
7. **Make controls feel immediate.** Apply control changes on the first relevant update, avoid overly slow camera/steering easing, and use short input buffers where the game design benefits from them. Never discard a legitimate short press merely because an animation or cooldown is active unless that is an explicit gameplay rule.
8. **Use real timestamps for timing-sensitive mechanics.** Optional launcher helpers `fb::keyPressedAtUs()`, `fb::frameStartUs()` and `fb::displayLatencyUs()` are declared in §4/launcher extras. If the game needs precise reaction timing, use press timestamps and account for display latency rather than measuring only frame counts. Declare these helper prototypes inside the game's own `namespace fb` block when used; do not modify the contract.
9. **Tune CPU/compiler settings only after profiling.** The board is configured for high performance, but `-O2` (with `build_unflags = -Os`) should be tested against the actual build and memory use. Do not assume compiler flags will improve a bus-bound game.
10. **Account for visual cadence.** Large world shifts per frame can look jumpy even at a high FPS. Use `dtUs`-based motion, suitable parallax speeds and spacing, and speed-dependent visual cues where appropriate.

### 8.3 Direct frame-buffer access (allowed, advanced, fast)
For scanline-heavy rendering (pseudo-3D roads, scrolling terrain) `d.getBufferPtr()` returns the raw 1024-byte buffer. Layout:

```
8 pages × 128 columns. Byte index = (y >> 3) * 128 + x
Bit within the byte = (y & 7); bit 0 is the TOP pixel of that 8-pixel column slice.
1 = lit, 0 = dark.
```
Rules: only inside `render()`; always bounds-check yourself (direct writes bypass U8g2 clipping); never keep the pointer across frames; do not touch the buffer outside 0..1023.

### 8.4 Fonts
Use U8g2 font names **only if you are certain they exist**; a wrong name is a compile error. Prefer the transparent/restricted-charset variants (`_tr`, numeric-only `_tn`) — they are smaller and faster. Fonts already used successfully by the launcher: `u8g2_font_5x7_tr`, `u8g2_font_6x13_tr`, `u8g2_font_6x13B_tr`, `u8g2_font_7x13B_tr`. Other well-known ones: `u8g2_font_6x10_tr`, `u8g2_font_6x12_tr`, `u8g2_font_7x13_tr`, `u8g2_font_helvB08_tr`, `u8g2_font_helvB10_tr`, `u8g2_font_logisoso16_tn`, `u8g2_font_logisoso20_tn`, `u8g2_font_logisoso24_tn` (large numbers for speed/time displays).

Readability: 5×7 is the **smallest** acceptable size, for secondary info only. Use ≥ 6×10 for anything the player must read quickly. Use large numeric fonts for speed, countdowns and timers.

### 8.4.1 Things a game MUST NOT do to the display
Never call: `begin()`, `setBusClock()`, `setPowerSave()`, `setContrast()`, `setFlipMode()`, `setDisplayRotation()`, `sendBuffer()`, `clearBuffer()`, `updateDisplay()`, `updateDisplayArea()`, `getU8x8()`, or anything on `Wire`. Never use U8g2 page mode (`firstPage/nextPage`).

### 8.5 Safe drawing
- Every element must stay within `x ∈ [0,127]`, `y ∈ [0,63]`. Compute positions so that nothing is accidentally drawn outside or wraps. When a sprite/text can approach an edge, clamp or clip deliberately.
- Rotated/animated shapes (tilted cars, nitro flames) must be checked at their extreme rotations/positions.
- Beware integer/unsigned underflow when computing coordinates (`uint8_t x = a - b` with `b > a` wraps to a large number). Use signed `int16_t`/`int32_t` for geometry and clamp before drawing.

### 8.6 Visual design principles for a 128×64 monochrome OLED
- **Large, recognisable, simple geometric shapes.** Strong contrast. No tiny, overly detailed sprites. No visual clutter.
- **Do not let UI elements visually blend into each other.** Separate HUD from playfield with margins, rules, or an erased (colour 0) box behind HUD text that overlaps action.
- Outline filled shapes with a 1-px gap/border when they overlap other lit shapes (draw a dark halo, then the lit shape).
- Use dithering/stipple (checkerboard, 25/50/75 % patterns) sparingly and deliberately for "grey" (e.g. grass, shadows, speed lines). Dither patterns on small shapes become noise; avoid them there.
- **Avoid large filled background areas** unless they are the point (they cost fill time and swamp the contrast).
- Keep text on one baseline grid; keep ≥ 1–2 px margin from screen edges for text.
- Think in a **pixel budget**: ~2 px minimum line thickness for anything that must be seen at a glance; ≥ 8×8 px for key objects.
- Games must look like **real retro handheld games**: title/intro screen, clear HUD, readable score/speed/time, game-over/results screen, restart flow. Never ship a "debug demonstration" look.
- The pixels are essentially square; circles stay circles.
- Use the device name **`flipBOX`** anywhere a device name appears (title screens, credits).

---

## 9. PERFORMANCE RULES

1. **Math:** integer / fixed-point (16.16, 24.8, Q15) wherever practical. `float` (hardware single-precision FPU) is acceptable for occasional use; **never `double`** (software emulation, very slow).
2. **Trigonometry and roots:** precompute **lookup tables in `init()`** or as `static const` arrays (e.g. a 256- or 1024-entry sine table in Q15). No `sinf/cosf/atan2f/sqrtf/powf` per frame per object, and never per pixel.
3. **Precalculate reusable geometry** (road scanline projection tables, rotated car outlines for N angles, perspective curves, pre-scaled strips) once in `init()` or at compile time. Static `const` tables live in flash and are read through the cache; keep hot tables small.
4. **No dynamic allocation.** No `new`/`malloc`/`String`/`std::` containers. Fixed arrays with compile-time sizes. No `static` objects with non-trivial constructors/destructors.
5. **Memory budget (targets):** game static RAM ≲ 16 KB; flash ≲ 100–150 KB per game. Keep stack usage tiny (the loop task has 8 KB total, shared with the launcher and the U8g2 call chain). No large local arrays; make big arrays `static`.
6. **Avoid expensive per-pixel rendering.** Prefer `drawHLine`/`drawVLine`/`drawBox`, spans, and precomputed `drawXBM()` sprites over `drawPixel()` loops. Use direct buffer writes (§8.3) for measured hot paths such as scanline fills, with strict bounds checks.
7. **Keep `update()` + `render()` comfortably below the measured OLED transfer time** where possible; the asynchronous pipeline overlaps game work with I²C transmission. Work beyond the transfer window can lower FPS. Profile rather than treating any single millisecond threshold as universal.
8. **Cheap per-frame text:** only re-format a number string when the number changes (cache the `char` buffer) if formatting shows up in profiling.
9. **No `delay`, no waiting loops, no `yield()` games, no blocking I/O, no `Serial` in the final code.**
10. **Branch/memory friendliness:** simple loops, small structs, arrays of structs for entities; avoid pointer chasing and virtual calls inside inner loops.
11. **Optional advanced:** hot inner functions MAY be marked `IRAM_ATTR` and hot small tables `DRAM_ATTR`; only do so if it is clearly beneficial and the memory budget allows.
12. Do **not** build an unnecessarily complicated engine. No ECS, no scene graph, no abstraction layers. Straightforward, fast, readable code.

---

## 10. HOW TO STRUCTURE A GAME

### 10.1 One internal state machine
A real game has more than "playing". Typical states (use a small `enum` and a `switch` in `update()` and `render()`):

`TITLE/INTRO → (SELECTION, e.g. choose car / difficulty) → COUNTDOWN → PLAYING → (PAUSED) → RESULTS / GAME OVER → (restart or exit)`

- Every state's screen must be readable on 128×64 and show the controls the player needs.
- Non-gameplay screens act on `pressed` edges only (§6.2).
- On entry the game should land in its first state with all variables reset (§10.2).

### 10.2 Reset everything in `init()`
The player can leave and re-enter the game any number of times without rebooting. `init()` MUST set every variable (positions, speeds, timers, scores, flags, PRNG seed, animation counters, selection cursors) to its starting value and build any tables. Do not rely on `static` initialisers for state that changes at runtime. A deliberate **session-persistent** value (e.g. best score this power cycle) may live in static storage that `init()` intentionally does not reset; say so in a comment. There is **no persistent storage** across power cycles unless the Game Request explicitly demands it (§12).

### 10.3 The exit path (mandatory)
The launcher deliberately has **no global exit button or combo** ("do not invent additional controls"); the game decides. Every game MUST let the player return to the menu by having `update()` return `fb::GAME_EXIT`. Default recommendation unless the Game Request specifies otherwise, for consistency across flipBOX games:
- On the **results / game-over / pause screen**, offer an explicit choice (restart vs. quit), confirmed with SELECT, **and/or**
- **Hold SELECT for about 1 second** at any time (accumulate time with `dtUs`) to quit, with a visible progress cue so the player knows it is working. Make sure this does not collide with gameplay inputs of that game.
Whatever scheme is used, the on-screen text must tell the player how to leave. Remember: SELECT (and any other key) held at entry is gated until released (§6.2).

### 10.4 Update / render separation
- `update()` reads input, advances time-based simulation, handles state transitions, returns the status.
- `render()` converts the current state to pixels. It may interpolate/animate cosmetic things from state but must not advance gameplay.
- Do not draw in `update()` — the buffer is cleared after it and the drawing would be lost.
- Prepare per-frame render data (e.g. projected road segments) in `update()` and consume it in `render()` if that keeps `render()` lean.

### 10.5 Namespacing and isolation
- Put all constants, state, helpers and the four game functions in an **anonymous namespace** (or `static`). The **only** global symbol is `fbGame_<id>` created by `FB_GAME`.
- Do not use `extern` to reach into another file. Do not `#include` `main.cpp` or other games. Do not create a shared header. **Duplicating a few lines of helper code in two games is correct**; coupling games is not.
- Prefer qualifying contract names (`fb::Input`, `fb::KEY_UP`) over `using namespace fb;` at global scope.
- Do not define symbols named `setup`, `loop`, `main`, or anything that could collide with the launcher (`gDisp`, `gIn`, `kGames`, …).

### 10.6 Code style requirements
- Tunable gameplay values (acceleration, top speed, steering rate, nitro strength/duration, grip, friction, difficulty) as **named `constexpr` constants at the top of the file with units in comments**, so the owner can tune the feel without hunting through code.
- Clear section headers, concise comments explaining *why*; no noise comments.
- No magic numbers for screen geometry — derive from `fb::SCREEN_W/H` or named constants.
- Fixed-point formats documented in comments (e.g. "Q16.16 world units").

---

## 11. FILES AND REGISTRATION

### 11.1 Workflow for any game
1. Create `src/<GameFile>.cpp`.
2. Paste the contract block (§5.1) at the top.
3. Implement `init` (optional), `update`, `render`, `shutdown` (optional) inside an anonymous namespace.
4. End the file with `FB_GAME(<id>, initFn, updateFn, renderFn, shutdownFn);`
5. **Only if the game is not already registered:** add one line to `FB_GAME_LIST` in `main.cpp`.
6. Build. PlatformIO compiles and links the new file automatically; the launcher can then launch it.

### 11.2 Registering a new game
The registry in `main.cpp` is the authoritative list of menu entries. Do not assume any particular game is already registered; inspect the supplied `main.cpp` when available.

To add a game, add one `X(<id>, "<Menu Label>")` line to `FB_GAME_LIST`. Every line except the last ends with a backslash `\\`; when appending, add a trailing `\\` to the previous last line and leave the new last line without one. Example shape:

```cpp
#define FB_GAME_LIST(X) \\
  X(ExampleGame, "Example Game")
```

Rules for `<id>`: a valid C identifier, unique, and exactly equal (case-sensitive) to the `id` in `FB_GAME(...)`. The menu label must be plain ASCII 32–127, at most 18 characters (16 or fewer recommended), unique and descriptive. The menu shows three rows at a time and scrolls when more entries exist.

### 11.4 Launcher behaviour you can rely on (the player-facing side)
- Boots straight into the **flipBOX** game menu (no splash). Title "flipBOX" at the top, `n/m` counter at the right, list below, selected row drawn as an inverted bar.
- UP/LEFT = previous, DOWN/RIGHT = next (wraps around); SELECT launches. The highlight is remembered when returning from a game.
- Selecting a registered game whose `.cpp` is missing (or whose `update`/`render` pointer is null) shows a **"NOT INSTALLED"** popup for ~1.2 s.
- The menu redraws only when something changes (no animation, no wasted work).

---

## 12. SCOPE LIMITS — DO NOT ADD

No audio / buzzer / speaker support; no Wi-Fi; no Bluetooth/BLE; no NRF24 radios; no networking of any kind; no external storage (SD/SPI flash/etc.); no EEPROM/NVS/Preferences/SPIFFS/LittleFS persistence (unless the Game Request explicitly demands it, and then only with a clear note); no additional peripherals; no additional libraries; no buttons beyond UP, DOWN, LEFT, RIGHT, SELECT and X; no GPIO changes beyond the documented D4/GPIO4 X-button assignment; no `platformio.ini` changes; games must not modify the launcher or contract; no extra files; no placeholder gameplay or "TODO" game logic in the delivered file.

Sound, if the owner wants it, is a **future wave** with a contract change — do not anticipate it with dead code.

---

## 13. REQUIRED DELIVERABLE FORMAT (WHAT THE LLM MUST OUTPUT)

1. **A one-paragraph design summary:** the control scheme (which button does what on every screen, including how to exit), the game states/screens, the physics/rendering approach (time-based fixed-point, tables precomputed), and any assumptions made.
2. **The complete file `src/<GameFile>.cpp`** in a single code block — complete, real, compilable, production-quality, with the contract block pasted verbatim at the top and the `FB_GAME` line at the end. No placeholders, no "rest of the code goes here", no pseudo-code.
3. **Registry instruction:** either "already registered, no change to `main.cpp`" or the exact single line to add to `FB_GAME_LIST` (with the backslash adjustment).
4. **A short tuning guide:** which constants at the top of the file change the feel (acceleration, top speed, steering, difficulty, etc.).
5. **A short self-review** against §15 (a few lines, not a lecture), explicitly stating estimated static RAM and any known limitation.
6. Nothing else: no extra files, no other games, no changes to `main.cpp`/`platformio.ini`, no change to the contract.

---

## 14. FILE SKELETON (STRUCTURE ONLY — NOT A GAME)

This shows the required shape. It contains **no gameplay**; replace the bodies. It will not let the player leave the game until an exit path is implemented (§10.3), which is mandatory.

```cpp
// src/<GameFile>.cpp  --  <Game Title>   (flipBOX)
//
// <short description of the game and its controls>

// ----- FLIPBOX GAME CONTRACT v1 : BEGIN ------------------------------------
//  (paste the block from §5.1 here, unchanged)
// ----- FLIPBOX GAME CONTRACT v1 : END --------------------------------------

namespace {

// ---------------------------------------------------------------------------
// Tuning constants (units in comments)
// ---------------------------------------------------------------------------
// constexpr int32_t kAccel = ...;   // Q16.16 units / s^2

// ---------------------------------------------------------------------------
// State (ALL of it is reset in gameInit())
// ---------------------------------------------------------------------------
// enum State : uint8_t { ST_TITLE, ST_PLAY, ST_RESULT };
// State   gState;
// ...

// ---------------------------------------------------------------------------
// Precomputed tables (built in gameInit() or static const)
// ---------------------------------------------------------------------------

void gameInit() {
  // reset every variable, build tables, seed PRNG
}

fb::Status gameUpdate(const fb::Input& in, const fb::Frame& f) {
  (void)in; (void)f;
  // read input, advance time-based simulation using f.dtUs, handle state changes
  // return fb::GAME_EXIT when the player chooses to leave
  return fb::GAME_RUN;
}

void gameRender(U8G2& d) {
  (void)d;
  // set font/colour/mode you need, then draw. Stay inside 128x64.
}

void gameShutdown() {
  // optional cleanup (usually nothing)
}

}  // namespace

FB_GAME(<id>, gameInit, gameUpdate, gameRender, gameShutdown);
```

---

## 15. SELF-REVIEW CHECKLIST (the LLM must verify before answering)

**Integration**
- [ ] Contract block pasted verbatim, unmodified; `FB_GAME(...)` is the last line, at global scope.
- [ ] `<id>` equals the registry id exactly (case-sensitive); label ≤ 18 ASCII chars if a new line is proposed.
- [ ] No `setup()/loop()/main()`; no hardware init; no GPIO/Wire/SPI/Serial.
- [ ] Only exported global symbol is `fbGame_<id>`; everything else is in an anonymous namespace / `static`.
- [ ] No dependency on other files/games/headers.

**Lifecycle & input**
- [ ] `init()` resets *all* state; the game can be re-entered repeatedly.
- [ ] A clear exit path returns `fb::GAME_EXIT`; the screen tells the player how.
- [ ] Menu-like screens use `pressed`, not `held`; gameplay uses `held`/`pressed` appropriately.
- [ ] No assumption that a key is "down" at frame 0 (entry gating).

**Timing & performance**
- [ ] All motion/timers driven by `dtUs`; positions in sub-pixel fixed-point; correct at 30, 60 and 85 FPS.
- [ ] No `delay`, no blocking, no heap, no `String`, no `double`, no per-pixel trig/sqrt.
- [ ] Trig/curve/projection tables precomputed.
- [ ] No large stack arrays; no unaligned multi-byte pointer casts.
- [ ] `update()+render()` cost reasoned about (target ≲ ~10 ms); inner loops are tight.

**Rendering**
- [ ] `update()` draws nothing; `render()` mutates no gameplay state.
- [ ] No `clearBuffer/sendBuffer/updateDisplay/begin/setBusClock/...` calls.
- [ ] Font/colour/mode set inside `render()` each frame; transparent font mode used over graphics.
- [ ] Every pixel within 0..127 × 0..63; signed geometry; no unsigned underflow; direct buffer writes bounds-checked.
- [ ] High contrast, no blending UI, no tiny detail, no needless big fills; text ≥ 6×10 for important info.
- [ ] Looks like a real retro handheld game: title, HUD, results/restart flow.
- [ ] `flipBOX` used as the device name where one appears.

**Build**
- [ ] C++11-compatible; warning-free with `-Wall -Wextra`; no unused functions/parameters.
- [ ] Only U8g2 + Arduino core used; no new libraries; no audio/Wi-Fi/BLE/NRF/networking/storage.

---

## 16. TROUBLESHOOTING (for the human, and for the LLM when fixing errors)

| Symptom | Likely cause | Fix |
|---|---|---|
| Menu entry shows **"NOT INSTALLED"** | File not in `src/`; `FB_GAME` missing/typo; `<id>` differs from the registry id (case-sensitive); `update`/`render` is `nullptr` | Fix the id/spelling; ensure the file is compiled (clean build); ensure `update` and `render` are non-null. (A missing symbol is **not** a link error — it silently appears as NOT INSTALLED.) |
| `multiple definition of 'xyz'` | A game defined a global with external linkage that another file also defines | Put it in an anonymous namespace / `static` |
| `multiple definition of 'setup'` / `'loop'` | The game defined `setup()` or `loop()` | Remove; the launcher owns them |
| `redefinition` / contract mismatch errors | Contract block edited or duplicated twice in one file | Paste it once, verbatim |
| Game can't be left | No `GAME_EXIT` path, or exit key is gated/held | Implement §10.3; require a fresh press (or hold from after entry) |
| Game starts "pre-pressed" / skips its title screen | Acting on `held` for a menu action | Use `pressed`; remember entry gating |
| State carries over from the previous play | Relied on static initialisers | Reset everything in `init()` |
| Movement speed changes with FPS | Frame-counted instead of `dtUs`-driven | Integrate with `dtUs` (§7.2) |
| Movement stalls/jitters at high FPS | Integer pixel positions; speed×dt < 1 px | Use sub-pixel fixed-point |
| Crash / `LoadStoreAlignment` exception | Unaligned multi-byte access | Use `memcpy` or aligned data |
| Stack overflow / watchdog reset | Large local arrays or deep recursion | Make arrays `static`; avoid recursion |
| Screen noise or garbage (owner side) | I²C clock too high for the module | Build flag `-DFB_I2C_HZ=400000` |
| FPS lower than expected | `update()+render()` > ~10 ms, per-pixel `drawPixel`, float trig, big fills | Optimise per §9; use spans/direct buffer writes/tables |
| Text/graphics wrong after overlay | Relying on draw state across frames | Set font/colour/mode every `render()` |

Owner-side build flags (optional, in `platformio.ini` `build_flags`): `-DFB_I2C_HZ=<hz>` (OLED bus clock, default 1000000), `-DFB_ASYNC_DISPLAY=0|1` (default 1; 0 = plain synchronous `sendBuffer()`), `-DFB_FAST_FLUSH=0|1` (default 1; streamed I²C writes when async display is enabled), `-DFB_LATE_LATCH=0|1` (default 1), `-DFB_I2C_CHUNK=<bytes>` (default 256, reduced automatically if the Wire buffer cannot support it), `-DFB_SHOW_FPS=0|1` (default 0), `-DFB_DEBOUNCE_US=<µs>` (default 5000), `-DFB_MAX_DT_US=<µs>` (default 50000). A game must work correctly regardless of these. Optionally `-O2` (with `build_unflags = -Os`) for extra speed.

---

## APPENDIX A — LAUNCHER REFERENCE EXCERPTS (authoritative behaviour)

These are the exact launcher routines a game interacts with. Games do not copy or call them; they are here so the LLM can reason precisely about timing, gating and call order.

### A.1 Options (defaults, all overridable by `-D`)
```cpp
FB_I2C_HZ         1000000UL  // OLED bus clock
FB_ASYNC_DISPLAY  1          // frame pushed to the OLED from core 0
FB_SHOW_FPS       0          // debug FPS box (top-right) inside games
FB_DEBOUNCE_US    5000UL     // per-key lock-out after an edge
FB_MAX_DT_US      50000UL    // clamp for Frame::dtUs
```

### A.2 Input sampling, debounce and gating
The launcher samples active-LOW buttons directly from the GPIO input registers both during the game loop and from a 1 ms `esp_timer` callback. Accepted press/release edges are latched under a spinlock until `pollInput()` consumes them. The first edge is accepted immediately; later transitions on the same key are suppressed for `FB_DEBOUNCE_US`. Held keys are gated at boot, game entry and game exit until physically released.

Games must use only the `fb::Input` supplied to `update()`. Never add a second GPIO reader or input task inside a game.

### A.3 Entering and leaving a game
```cpp
void enterGame(const fb::Game* g) {
  gActive = g;
  gateHeldInputs();                     // keys still held stay dead for the game until released

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
```

### A.4 One game frame
```cpp
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
    return;                              // render() is NOT called on the exit frame
  }

  gDisp.clearBuffer();
  gActive->render(gDisp);
#if FB_SHOW_FPS
  drawFpsOverlay();
#endif
  presentFrame();

  ++gFpsFrames;
  const uint32_t el = nowUs - gFpsT0;
  if (el >= 500000UL) {                  // FPS refreshed every 500 ms
    gFps       = (uint16_t)((gFpsFrames * 1000000UL) / el);
    gFpsFrames = 0;
    gFpsT0     = nowUs;
  }
}
```

### A.5 Presenting a frame (async display)
```cpp
inline void presentFrame() {
#if FB_ASYNC_DISPLAY
  xSemaphoreTake(gFlushDone, portMAX_DELAY);                  // only waits if previous frame still in flight
  memcpy(gShadow, gDisp.getBufferPtr(), sizeof(gShadow));      // 1 KB snapshot
  xTaskNotifyGive(gFlushTaskHandle);                           // core-0 task sends it over I²C
#else
  gDisp.sendBuffer();
#endif
}
```

### A.6 Display object and loop
```cpp
U8G2_SSD1306_128X64_NONAME_F_HW_I2C gDisp(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);

void loop() {
  const uint32_t now = micros();
  pollInput(now);
  if (gActive) runGame(now);
  else         runMenu();
}
```

---

## APPENDIX B — TEMPLATE THE HUMAN FILLS IN WHEN ASKING AN LLM FOR A GAME

Paste this whole `.md` file first, then:

```
GAME REQUEST
- Game title / menu label:
- Registry id (<id>):            (e.g. ExampleGame — or "new: <id>")
- File name:                     (e.g. ExampleGame.cpp)
- Genre / camera / view:
- Core gameplay & rules:
- Controls (UP / DOWN / LEFT / RIGHT / SELECT on each screen):
- How the player exits to the launcher:   (or "use the default in §10.3")
- Win/lose/score conditions:
- Screens/states wanted (title, select, countdown, play, results, pause, ...):
- Visual style / HUD content:
- Physics feel / difficulty targets:
- Things that must NOT happen:
- Anything else:

Follow the flipBOX spec exactly. Deliver the one .cpp file in a single code block,
the registry instruction, a short tuning guide and the self-review (§15).
Do not create any other game or file.
```

*End of specification.*
