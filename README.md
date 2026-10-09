# flipBOX 🎮

**A handheld gaming platform powered by the ESP32.**

flipBOX is an ESP32-based handheld gaming project featuring a monochrome OLED display, physical controls, and a modular game system. It is designed to run lightweight arcade games directly on the device.

## 📄 Game Development Documentation

Two reference documents define how games are developed and integrated into flipBOX.

| Document                            | Description                                                                                                                               |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `flipBOX_GAME_DEVELOPMENT_GUIDE.md` | Technical reference for hardware, controls, rendering, performance, game architecture, integration, and troubleshooting.                  |
| `flipBOX_AI_GAME_SPEC.md`           | AI-facing specification covering game implementation requirements, development rules, deliverables, and a reusable game request template. |

**When developing games:**

* **Manual development:** Use `flipBOX_GAME_DEVELOPMENT_GUIDE.md` as the technical reference.
* **AI-assisted development:** Provide `flipBOX_AI_GAME_SPEC.md` alongside your specific game requirements.

## 🛠️ Hardware & Platform

* **Microcontroller:** ESP32-WROOM-32
* **Display:** 128 × 64 monochrome OLED
* **Display library:** U8g2
* **Controls:** Six-button input layout
* **Framework:** Arduino
* **Build system:** PlatformIO
* **Target CPU frequency:** 240 MHz

## 🕹️ Game System

flipBOX is intended to support multiple games through a modular architecture, allowing individual games to be developed separately and integrated into the main launcher.

Each game should follow the shared game interface and input, rendering, and performance requirements described in the development documentation.

### Planned Games

* **F1 Racing** — A pseudo-3D racing game featuring perspective, track bends, and player-controlled movement.
* **Street Racing** — A two-car racing experience inspired by arcade street-racing games.
* **Tokyo Drift** — A drifting-focused driving game concept.

Game availability depends on implementation and integration progress.

## 💻 Development Setup

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).
3. Clone or download the flipBOX repository.
4. Open the project folder in Visual Studio Code.
5. Review `platformio.ini` and verify the configured board, framework, and dependencies.
6. Build the firmware using PlatformIO.
7. Connect the ESP32 and upload the firmware.

## ⚙️ Development Principles

* Run games directly on the ESP32 without requiring a phone or external application.
* Keep game modules compatible with the shared launcher architecture.
* Optimize rendering, memory usage, and frame rate for the hardware.
* Use the established button layout and display configuration.
* Keep implementations self-contained and maintainable.
* Follow `flipBOX_GAME_DEVELOPMENT_GUIDE.md` for integration requirements.

## 📁 Project Documentation

Keep the following documentation alongside the project source:

* `README.md` — Project overview and setup.
* `flipBOX_GAME_DEVELOPMENT_GUIDE.md` — Technical game development reference.
* `flipBOX_AI_GAME_SPEC.md` — AI game generation specification.

---

**flipBOX — Pick up, power on, and play.**
