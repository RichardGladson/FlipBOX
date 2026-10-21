# Display Render Pipelines: doomBOX vs flipBOX

flipBOX and doomBOX both drive the same physical SSD1306 OLED panel, but through two fundamentally different rendering pipelines. Read this before merging any doomBOX feature into flipBOX, or vice versa — the two styles are each safe on their own, but mixing them carelessly is what causes display crashes and corruption.

## Why two different pipelines exist

doomBOX's pipeline is the simpler of the two: draw into a buffer, then block until it's on screen. It's easy to reason about and easy to write new screens for, which is why nearly every doomBOX feature module uses it.

flipBOX's pipeline exists because flipBOX has to hold a playable framerate during real-time games (F1 Racing, Fast and Furious, etc.). A synchronous, blocking display write stalls the game loop for the full I2C transfer time on *every single frame* — long enough, at typical I2C speeds, to visibly cap the framerate. flipBOX moves the actual hardware write off the main loop and onto a dedicated task, so the game loop can keep computing the next frame while the previous one is still being transferred.

## Style 1: doomBOX's synchronous pipeline

```cpp
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

u8g2.clearBuffer();
// ... draw calls ...
u8g2.sendBuffer();   // blocks here until the full I2C transfer completes
```

- One global `u8g2` object — a stock U8G2 library instance.
- `sendBuffer()` is the library's own method: it walks every page of the display, issuing the SSD1306 addressing commands and pixel data over I2C through the u8x8 HAL, synchronously. `Wire.beginTransmission/write/endTransmission` all run to completion before `sendBuffer()` returns.
- Whatever task calls `sendBuffer()` — typically Arduino's `loop()` — is frozen for the duration of the transfer. Nothing else on that task (reading input, updating state) happens until the frame has fully landed.
- Simple to write against: draw, flush, done. No task, no semaphore, no buffer management beyond what U8G2 already handles internally.

## Style 2: flipBOX's async pipeline

```cpp
gActive->render(gDisp);   // draws into gDisp's buffer only — no bus traffic yet
presentFrame();           // hands the frame off and returns immediately
```

```cpp
void presentFrame() {
  xSemaphoreTake(gFlushDone, portMAX_DELAY);
  memcpy(gShadow, gDisp.getBufferPtr(), 1024);
  xTaskNotifyGive(gFlushTaskHandle);
}
```

- One display object (`gDisp`), plus a second 1024-byte buffer, `gShadow`, used purely for transfer — the "double" in double-buffered. `gDisp`'s buffer is always the one being drawn into for the *next* frame; `gShadow` is always the one currently being sent.
- `presentFrame()` copies the just-drawn frame into `gShadow` and hands it to a dedicated FreeRTOS task, `flushTask`, running on its own CPU core. `presentFrame()` returns immediately — it does not wait for the transfer to finish.
- `flushTask` does the actual I2C write itself, via a hand-rolled `fastFlush()` that bypasses U8G2's own `sendBuffer()` entirely and talks to the panel directly with raw `Wire` calls.
- Net effect: the game loop never stalls on I2C. It draws the next frame into `gDisp` while `flushTask` is still sending the previous one from `gShadow`.

## What's actually different

| | doomBOX (synchronous) | flipBOX (async) |
|---|---|---|
| Display object | `u8g2` (stock U8G2) | `gDisp` (same class, wrapped) |
| Buffering | single buffer | double-buffered (`gDisp` + `gShadow`) |
| I2C write | `u8g2.sendBuffer()` — library method, blocking | hand-rolled `fastFlush()` on a separate task, non-blocking to the caller |
| Who touches `Wire` | whatever task calls `sendBuffer()` | only `flushTask`, pinned to its own core |
| Caller behavior | frozen until the transfer completes | returns immediately, free to keep working |
| Infrastructure needed | none beyond U8G2 itself | a FreeRTOS task, a semaphore (`gFlushDone`), a shadow buffer |
| Best suited for | menu-driven screens, scanners — anything not framerate-sensitive | real-time games — anything that needs to keep computing while a frame is in flight |

The trade-off in one line: doomBOX's style is less code and easier to reason about, at the cost of stalling the caller for every I2C transfer. flipBOX's style keeps the caller free, at the cost of a task, a semaphore, and a second buffer to manage.

## Converting doomBOX-style code to flipBOX-style

1. Stop drawing into a standalone `u8g2` object — draw into the shared `gDisp` object instead. The drawing calls themselves don't change, only the object.
2. Replace the direct `u8g2.sendBuffer()` call with a call to `presentFrame()` (or an equivalent wrapper), instead of the raw library method.
3. Make sure nothing in the ported code calls `.begin()`, `pinMode()`, or `Wire.begin()` again. The display is already initialized once, at boot; a second `begin()` can collide with whatever `flushTask` is doing mid-transfer.
4. Remember the call that used to be synchronous no longer is. Code that assumed "the screen is now on the display" the instant `sendBuffer()` returned can't assume that anymore — the frame may still be mid-transfer on `flushTask` when `presentFrame()` returns.

## Converting flipBOX-style code to doomBOX-style

Going the other direction is simpler, since it's removing infrastructure rather than adding it:

1. Call `gDisp.sendBuffer()` directly instead of `presentFrame()`. This is exactly what flipBOX's own `FB_ASYNC_DISPLAY = 0` build-time fallback does — it disables `flushTask` entirely and falls back to this.
2. Don't do this while `flushTask` is still running elsewhere in the system. Mixing a direct `sendBuffer()` call with a live `flushTask` reintroduces the exact two-writers-on-one-bus hazard this whole pipeline split exists to avoid. A screen only gets to call `sendBuffer()` directly if `flushTask` is guaranteed not to be running at the same time.

## The one rule that matters when merging

**Never let both pipelines write to the display at the same time.** Each style is internally safe on its own — the danger only shows up at the boundary between them, where a direct `sendBuffer()` call and a live `flushTask` can both try to own the I2C bus at once. Whichever direction you're converting a screen, that's the thing to get right before anything else.
