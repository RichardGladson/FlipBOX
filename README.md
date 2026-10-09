# 🎮 Flip-Up

**A modular handheld gaming console powered by ESP32.**

Flip-Up is a compact handheld gaming console built around an ESP32 microcontroller, a 0.96-inch OLED display, and seven physical buttons. It features a central game launcher designed to make adding new games simple, modular, and developer-friendly.

Rather than hardcoding every game into a single monolithic program, Flip-Up uses a launcher-based architecture that allows individual games to be developed as separate C++ source files.

## ✨ Features

* **ESP32-powered:** A compact embedded gaming platform.
* **0.96-inch OLED display:** A small monochrome display for the launcher and games.
* **Seven-button controls:** Physical input for navigating menus and playing games.
* **Centralized game launcher:** `main.cpp` manages the launcher and provides the entry point for the gaming experience.
* **Modular game architecture:** Games are implemented as separate C++ source files.
* **Simple game integration:** Register a new game in the launcher's game list and add its source file to the `src` directory.
* **Documented game interface:** A Markdown development guide describes the required C++ format for creating compatible games.

## 🕹️ How It Works

Flip-Up separates the launcher from individual game implementations.

### The launcher

The main launcher is handled by `main.cpp`. It manages the launcher functionality and maintains the list of available games.

### Individual games

Each game is implemented separately using the C++ structure defined by the project's game development guide. This keeps individual game logic independent of the launcher implementation.

### Adding games

The launcher is designed to be extended without rewriting the entire firmware for every new game.

To integrate a game:

1. Create a C++ source file that follows the required game format.
2. Place the source file in the project's `src` directory.
3. Register the game in the list maintained by `main.cpp`.
4. Build and flash the firmware to the ESP32.

The launcher can then include the newly registered game in its available games.

## 🛠️ Creating a Game

Flip-Up includes a Markdown guide explaining how to implement games compatible with its launcher.

Use that document as the reference for the required C++ structure, game interface, and integration process.

**The intended workflow is simple:** write a game, add its source file, register it with the launcher, and build the firmware.

This modular approach makes it easier to experiment with new game mechanics, maintain existing games, and expand the console over time.

## 🔧 Hardware

| Component         | Specification          |
| ----------------- | ---------------------- |
| Microcontroller   | ESP32                  |
| Display           | 0.96-inch OLED         |
| Input             | Seven physical buttons |
| Firmware          | C++                    |
| Build environment | PlatformIO             |

## 📁 Project Structure

```text
Flip-Up/
├── src/
│   ├── main.cpp
│   └── <game source files>.cpp
├── <game development guide>.md
└── platformio.ini
```

*The structure above illustrates the intended organization; consult the repository for the exact filenames and configuration.*

## 🚀 Development

Flip-Up is designed to be extended through its existing firmware architecture.

To develop or modify the project:

1. Clone the repository.
2. Open the project in Visual Studio Code with the PlatformIO extension.
3. Review `main.cpp` and the game development guide.
4. Implement or modify a game using the documented interface.
5. Register new games in the launcher's game list.
6. Build the firmware and upload it to the ESP32.

Ensure that any new game follows the project's documented interface so it integrates correctly with the launcher.

## 🎯 Project Goal

Flip-Up aims to make handheld embedded game development more accessible by combining a compact ESP32-based console with a modular, extensible software architecture.

The goal is to make expanding the game library a straightforward development task rather than requiring changes throughout the firmware.

## 📜 License

See the repository for licensing information.
